#ifndef RAYTRACER_BDPT_H
#define RAYTRACER_BDPT_H

#include "lighting/ToneMap.h"
#include "acceleration/DataStructure.h"
#include "lighting/MonteCarlo.h"
#include "Light.h"
#include "Film.h"

struct PathVertex {
    Vec3 position;
    Vec3 normal;
    Vec3 throughput;
    std::shared_ptr<Hittable> object;
    bool isDelta;
    double pdfForward;
    double pdfBackward;
    Color albedo;

    // TODO: Might also need the Geometry term in here, but not so sure yet

    PathVertex()
            : position(), normal(), throughput(1, 1, 1), object(nullptr), isDelta(false), pdfForward(1.0),
              pdfBackward(1.0), albedo(1, 1, 1) {}

    PathVertex(const Vec3 &pos, const Vec3 &norm, const Vec3 &tp, const std::shared_ptr<Hittable> &obj,
               const bool &isDelta, const double &pdfForward, const double &pdfBackward,
               const Color &alb = Color(1, 1, 1))
            : position(pos), normal(norm), throughput(tp), object(obj), isDelta(isDelta), pdfForward(pdfForward),
              pdfBackward(pdfBackward), albedo(alb) {}

    [[nodiscard]] bool isLight() const {
        return object && object->material->getType() == Emissive;
    }

    [[nodiscard]] bool isCameraLens(const Vec3 &cameraPosition, double epsilon = 1e-6) const {
        return object == nullptr && (position - cameraPosition).length() < epsilon;
    }
};

class BDPT {
public:
    MonteCarloIntegrator monteCarlo;
    int maxDepth;
    int maxPathLength = maxDepth;
    SamplingStrat samplingStrat = sampleAutoStrat;
    int debugS = -1;  // -1 means test all s values
    int debugT = -1;  // -1 means test all t values
    bool debugNoMIS = true;
    // Currently no T = 0 is available, because basically nothing is hitting the camera when using pinhole

    BDPT(int samples = 25, int depth = 10) : monteCarlo(samples), maxDepth(depth) {}

    void computeColorForRay(Ray &cameraRay,
                            const std::vector<std::shared_ptr<Hittable>> &hittables,
                            const Light &lights,
                            std::unique_ptr<DataStructure> &dataStructure, Camera &camera,
                            Film &film,
                            double sampleX,
                            double sampleY) {
        // Step 1: Generate camera path
        std::vector<PathVertex> cameraPath;
        traceCameraPath(cameraRay, hittables, dataStructure, cameraPath);

        // Step 2: Generate light path
        std::vector<PathVertex> lightPath;
        traceLightPath(lights, hittables, dataStructure, lightPath);

        for (int pathLength = 1; pathLength <= maxPathLength; pathLength++) {
            int pathVertexCount = pathLength + 1;

            for (int s = 0; s <= pathVertexCount; s++) {
                int t = pathVertexCount - s;

                // Debug purposes
                if ((debugS != -1 && s != debugS) || (debugT != -1 && t != debugT)) continue;

                // Not enough vertices
                if (s > lightPath.size() || t > cameraPath.size()) continue;

                // Cannot form path with less than 1 vertex
                if ((s == 0 && t < 2) || (t == 0 && s < 2) || (s + t) < 2) continue;

                // Light vertex cant connect to camera
                if (t == 0 && !lightPath[s - 1].isCameraLens(camera.getPosition())) continue;
                // Camera did not hit light
                if (s == 0 && !cameraPath[t - 1].isLight()) continue;

                double filmPixelX = sampleX;
                double filmPixelY = sampleY;
                bool validPixel = true;

                if (t == 0) {
                    validPixel = camera.worldToScreen(lightPath[s - 2].position, lightPath[s - 1].position, filmPixelX,
                                                      filmPixelY);
                } else if (t == 1) {
                    validPixel = camera.worldToScreen(lightPath[s - 1].position, cameraPath[0].position, filmPixelX,
                                                      filmPixelY);
                }

                if (!validPixel) continue;

                double G = 1.0;
                Color unweightedContribution = evalUnweightedContribution(lightPath, s, cameraPath, t, camera,
                                                                          hittables, dataStructure, G);

                double weight = debugNoMIS ? 1.0 / (lightPath.size() + cameraPath.size()) : 1.0 / (lightPath.size() +
                                                                                                   cameraPath.size());

                film.addLightSample(filmPixelX, filmPixelY, weight * unweightedContribution);
            }
        }
    }

private:

    void traceCameraPath(Ray &ray,
                         const std::vector<std::shared_ptr<Hittable>> &hittables,
                         std::unique_ptr<DataStructure> &dataStructure,
                         std::vector<PathVertex> &cameraPath) {
        Vec3 throughput(1, 1, 1);
        int depth = 0;

        // Store camera position as first vertex
        // Need to get the legit camera position, to not confuse if changed
        cameraPath.emplace_back(ray.origin, ray.direction, throughput, nullptr, true, 1.0, 1.0);

        // Trace the path
        while (depth < maxDepth && cameraPath.size() < maxPathLength) {
            int hitObjectIndex = -1;
            dataStructure->intersect(ray, hittables, hitObjectIndex);

            if (hitObjectIndex == -1 || ray.t >= std::numeric_limits<double>::max()) {
                break;
            }

            const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectIndex];
            const Vec3 hitPoint = ray.origin + ray.direction * ray.t;
            const Vec3 normal = hitObject->calculateNormal(hitPoint);
            const auto material = hitObject->material;
            const Color materialColor = material->shade();
            const auto materialType = material->getType();

            if (materialType == Mirror) {
                Vec3 reflectionDirection = reflect(ray.direction, normal);
                ray = Ray(hitPoint, reflectionDirection);
                depth++;
                continue;
            }

            if (materialType == Transparent) {
                bool entering = dot(ray.direction, normal) < 0;
                Vec3 outwardNormal = entering ? normal : -normal;

                double refractionRatio = entering ? (1.0 / material->getIOR()) : material->getIOR();

                // Incident angles
                double cosTheta = std::min(dot(-ray.direction, outwardNormal), 1.0);
                double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

                // Check for reflection
                bool cannotRefract = refractionRatio * sinTheta > 1.0;

                double reflectance = schlick(cosTheta, refractionRatio);

                if ((cannotRefract || monteCarlo.generateNumber() < reflectance)) {
                    // Follow reflection ray
                    Vec3 reflectionDirection = reflect(ray.direction, outwardNormal);
                    ray = Ray(hitPoint, reflectionDirection);
                } else {
                    // Follow refraction ray
                    Vec3 refractionDirection = refract(ray.direction, outwardNormal, refractionRatio);
                    ray = Ray(hitPoint, refractionDirection);
                }
                depth++;
                continue;
            }

            // Store the vertex
            cameraPath.emplace_back(hitPoint, normal, throughput, hitObject, false, 1.0, 1.0, materialColor); // TODO


            if (material->getType() == Emissive) break;
            if (depth > 3) {
                double terminationProb = std::max(0.05, 1 - vecAvg(throughput));
                if (monteCarlo.generateNumber() < terminationProb) {
                    break;
                }
                throughput /= (1 - terminationProb);
            }

            // Sample a new direction and continue
            auto [dir, brdf, pdf] = monteCarlo.sample(normal, material->shade(),
                                                      material->getType(), ray, samplingStrat,
                                                      material->getRoughness());

            if (pdf <= 0) break;

            ray = Ray(hitPoint, dir);
            throughput = throughput * (brdf / pdf);
            depth++;
        }
    }

    void traceLightPath(const Light &lights,
                        const std::vector<std::shared_ptr<Hittable>> &hittables,
                        std::unique_ptr<DataStructure> &dataStructure,
                        std::vector<PathVertex> &lightPath) {
        // Sample a point on the light
        auto [lightPosition, lightNormal, selectedLight, lightArea] = monteCarlo.sampleEmissive(lights);

        // Initial emission from light
        Color emission = selectedLight->material->getEmissionColor();
        double lightPdf = monteCarlo.evaluateLightPdf(lights, selectedLight);

        // Store light position as first vertex
        lightPath.emplace_back(lightPosition, lightNormal, emission / lightPdf, selectedLight, true, 1.0, 1.0,
                               emission);

        // Sample a direction from the light (cosine-weighted)
        Ray nullRay({0, 0, 0}, {0, 0, 0});
        Vec3 direction = monteCarlo.sampleDirection(sampleDiffuseStrat, Diffuse, lightNormal, nullRay, 0.2);
        // Check if we even need here the pdf and stuff since we deterministically choose a light source anyway???
        // Thus might not need to change the TP
        // Sample position on light: 1 / Area
        // Sample direction: cosTheta / pi
        // Emission * cosTheta (LambertsLaw) / (Sampling light posisiton * sampling direction)

        double cosTheta = dot(direction, lightNormal);
        double pdfDirection = cosTheta / M_PI;

//        Ray lightRay(lightPosition, direction);
//        Vec3 throughput = emission * cosTheta / lightPdf * pdfDirection;
        Ray lightRay(lightPosition, direction);
        const double pdfForward = pdfDirection / cosTheta;
        Vec3 throughput = lightPath[0].throughput / pdfForward;
        int depth = 0;

        // Trace the path
        while (depth < maxDepth && lightPath.size() < maxPathLength) {
            int hitObjectIndex = -1;
            dataStructure->intersect(lightRay, hittables, hitObjectIndex);

            if (hitObjectIndex == -1 || lightRay.t >= std::numeric_limits<double>::max()) {
                break;
            }

            const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectIndex];
            const Vec3 hitPoint = lightRay.origin + lightRay.direction * lightRay.t;
            const Vec3 normal = hitObject->calculateNormal(hitPoint);
            const auto material = hitObject->material;
            const Color materialColor = material->shade();
            const auto materialType = material->getType();

            if (materialType == Mirror) {
                Vec3 reflectionDirection = reflect(lightRay.direction, normal);
                lightRay = Ray(hitPoint, reflectionDirection);
                depth++;
                continue;
            }

            if (materialType == Transparent) {
                bool entering = dot(lightRay.direction, normal) < 0;
                Vec3 outwardNormal = entering ? normal : -normal;

                double refractionRatio = entering ? (1.0 / material->getIOR()) : material->getIOR();

                // Incident angles
                double cosTheta = std::min(dot(-lightRay.direction, outwardNormal), 1.0);
                double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

                // Check for reflection
                bool cannotRefract = refractionRatio * sinTheta > 1.0;

                double reflectance = schlick(cosTheta, refractionRatio);

                if ((cannotRefract || monteCarlo.generateNumber() < reflectance)) {
                    // Follow reflection ray
                    Vec3 reflectionDirection = reflect(lightRay.direction, outwardNormal);
                    lightRay = Ray(hitPoint, reflectionDirection);
                } else {
                    // Follow refraction ray
                    Vec3 refractionDirection = refract(lightRay.direction, outwardNormal, refractionRatio);
                    lightRay = Ray(hitPoint, refractionDirection);
                }
                depth++;
                continue;
            }

            // Store the vertex
            lightPath.emplace_back(hitPoint, normal, throughput, hitObject, false, 1.0, 1.0, materialColor);

            if (material->getType() == Emissive) break;
            if (depth > 3) {
                double terminationProb = std::max(0.05, 1 - vecAvg(throughput));
                if (monteCarlo.generateNumber() < terminationProb) {
                    break;
                }
                throughput /= (1 - terminationProb);
            }

            // Sample a new direction and continue
            auto [dir, brdf, pdf] = monteCarlo.sample(normal, material->shade(),
                                                      material->getType(), lightRay, samplingStrat,
                                                      material->getRoughness());

            if (pdf <= 0) break;

            lightRay = Ray(hitPoint, dir);
            throughput = throughput * (brdf / pdf);
            depth++;
        }
    }

    Color evalUnweightedContribution(const std::vector<PathVertex> &lightPath, int s,
                                     const std::vector<PathVertex> &cameraPath, int t,
                                     Camera &camera,
                                     const std::vector<std::shared_ptr<Hittable>> &hittables,
                                     std::unique_ptr<DataStructure> &dataStructure,
                                     double &G) {
        const double shadowEpsilon = 1e-4;
        Color lightContribution = (s == 0) ? Color(1, 1, 1) : lightPath[s - 1].throughput;
        Color cameraContribution = (t == 0) ? Color(1, 1, 1) : cameraPath[t - 1].throughput;
        Color connectionFactor;

        if (s == 0) {
            // Direct camera hit of light
            connectionFactor = cameraPath[t - 1].object->material->getEmissionColor();
        } else if (t == 0) {
            double importance = camera.evaluateImportance(lightPath[s - 2].position, lightPath[s - 1].position);
            connectionFactor = {importance, importance, importance};
        } else {
            const PathVertex &sEndVertex = lightPath[s - 1];
            const PathVertex &tEndVertex = cameraPath[t - 1];

            Vec3 connectVector = tEndVertex.position - sEndVertex.position;
            double distanceSquared = connectVector.lengthSquared();
            if (distanceSquared <= shadowEpsilon * shadowEpsilon) return {0, 0, 0};
            double distance = std::sqrt(distanceSquared);

            Vec3 connectDir = unitVector(connectVector);

            // Visibility test
            Ray shadowRay(sEndVertex.position, connectDir);

            // Need an extra check for the camera since with this setup my camera is not something "hittable"
            bool isConnectingToCamera = tEndVertex.isCameraLens(camera.getPosition());
            int hitIndex = -1;
            dataStructure->intersect(shadowRay, hittables, hitIndex);
            if (isConnectingToCamera) {
                // Camera connection: check for blockers
                shadowRay.t = distance - shadowEpsilon;  // Limit ray to just before camera

                dataStructure->intersect(shadowRay, hittables, hitIndex);

                if (hitIndex != -1) {
                    return {0, 0, 0};  // Something is blocking the path
                }
            } else {
                // Surface connection: check if we hit the target
                if (hitIndex == -1 || shadowRay.t < distance - shadowEpsilon) {
                    return {0, 0, 0};  // Connection failed
                }

                // Optional: Verify we hit the correct surface
                const std::shared_ptr<Hittable> &hitObject = hittables[hitIndex];
                if (hitObject != tEndVertex.object) {
                    return {0, 0, 0};  // Hit wrong surface
                }
            }


            // Evaluate light bsdf/emission
            Color fullLightContribution;
            if (s == 1) {
                // Direct Light
                double cosTheta = std::max(0.0, dot(sEndVertex.normal, connectDir));
                fullLightContribution = sEndVertex.throughput * cosTheta;
                // fullLightContribution = sEndVertex.object->material->getEmissionColor() * cosTheta;
            } else {
                // BSDF
                Vec3 lightIncomingDir = unitVector(sEndVertex.position - lightPath[s - 2].position);
                Ray lightIncomingRay(lightPath[s - 2].position, lightIncomingDir);
                fullLightContribution = monteCarlo.evaluate(sEndVertex.albedo, sEndVertex.object->material->getType(),
                                                            connectDir, sEndVertex.normal, lightIncomingRay,
                                                            sEndVertex.object->material->getRoughness());
            }

            if (fullLightContribution.lengthSquared() < 1e-10) return {0, 0, 0};

            // Evaluate camera bsdf/emission
            Color fullCameraContribution;
            if (t == 1) {
                // Camera importance
                double importance = camera.evaluateImportance(sEndVertex.position, tEndVertex.position);
                fullCameraContribution = Color{importance, importance, importance};
            } else {
                // BSDF evaluation
                Vec3 eyeIncomingDir = unitVector(tEndVertex.position - cameraPath[t - 2].position);
                Ray eyeIncomingRay(cameraPath[t - 2].position, eyeIncomingDir);
                fullCameraContribution = monteCarlo.evaluate(tEndVertex.albedo, tEndVertex.object->material->getType(),
                                                             -connectDir, tEndVertex.normal, eyeIncomingRay,
                                                             tEndVertex.object->material->getRoughness());
            }

            if (fullCameraContribution.lengthSquared() < 1e-10) return {0, 0, 0};

            // Geometry term
            double cosLight = std::max(0.0, dot(sEndVertex.normal, connectDir));
            double cosEye = std::max(0.0, dot(tEndVertex.normal, -connectDir));
            G = (cosLight * cosEye) / (distance * distance);

            if (G <= 0) return {0, 0, 0};

            connectionFactor = fullLightContribution * G * fullCameraContribution;
        }

        return lightContribution * connectionFactor * cameraContribution;
    }
};

#endif //RAYTRACER_BDPT_H
