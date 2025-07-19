#ifndef RAYTRACER_LIGHTTRACER_H
#define RAYTRACER_LIGHTTRACER_H

#include "lighting/Light.h"
#include "acceleration/DataStructure.h"
#include "lighting/MonteCarlo.h"
#include "Trace.h"
#include "Film.h"

class LightTracer {
public:
    LightTracer(int maxDepth, int numSamples)
            : mMaxDepth(maxDepth), mNumSamples(numSamples), monteCarlo(numSamples) {}

    void render(const std::vector<std::shared_ptr<Hittable>> &hittables,
                const Light &lights,
                Camera &camera,
                Film &film,
                std::unique_ptr<DataStructure> &dataStructure) {

        // For each light sample
        for (int sample = 0; sample < mNumSamples; ++sample) {
            // Trace a light path
            traceLightPath(hittables, lights, camera, film, dataStructure);
            //traceLightPathT0(hittables, lights, camera, film, dataStructure);
        }
    }

private:
    int mMaxDepth;
    int mNumSamples;
    MonteCarloIntegrator monteCarlo;

    void traceLightPathT0(const std::vector<std::shared_ptr<Hittable>> &hittables,
                          const Light &lights,
                          Camera &camera,
                          Film &film,
                          std::unique_ptr<DataStructure> &dataStructure) {

        // Sample a light source
        auto [lightPos, lightNormal, light, lightArea] = monteCarlo.sampleEmissive(lights);

        // Create initial throughput
        double lightSelectionPdf = 1.0 / lights.lights.size();
        double lightPosPdf = 1.0 / lightArea;
        Color emission = light->material->getEmissionColor();
        Color throughput = emission / (lightSelectionPdf * lightPosPdf);

        // Sample initial direction from light
        Vec3 direction = monteCarlo.sampleDirection(sampleDiffuseStrat, Diffuse, lightNormal,
                                                    Ray({0, 0, 0}, {0, 0, 0}), 0.2);

        // Create and trace ray from light
        Ray ray(lightPos, direction);

        // Trace the light path
        int pathLength = 1;
        while (pathLength < mMaxDepth) {
            int hitObjectIndex = -1;
            dataStructure->intersect(ray, hittables, hitObjectIndex);

            if (hitObjectIndex == -1 || ray.t >= std::numeric_limits<double>::max()) {
                break;
            }

            const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectIndex];
            const Vec3 hitPoint = ray.origin + ray.direction * ray.t;
            const Vec3 normal = hitObject->calculateNormal(hitPoint);
            const auto material = hitObject->material;
            const auto materialType = material->getType();

            if (camera.isLens(hitObject)) {
                double screenX, screenY;
                if (camera.worldToScreen(ray.origin, hitPoint, screenX, screenY)) {
                    film.addLightSample(screenX, screenY, throughput);
                }
                break;
            }

            if (material->getType() == Emissive) {
                break;
            }

            // Russian Roulette termination for longer paths
            if (pathLength > 3) {
                double terminationProb = std::max(0.05, 1 - vecAvg(throughput));
                if (monteCarlo.generateNumber() < terminationProb) {
                    break;
                }
                throughput /= (1 - terminationProb);
            }

            // Sample new direction and continue
            auto [dir, brdf, pdf] = monteCarlo.sample(normal, material->shade(), material->getType(),
                                                      ray, sampleAutoStrat, material->getRoughness());

            if (pdf <= 0) break;

            ray = Ray(hitPoint, dir);
            throughput = throughput * (brdf / pdf);
            pathLength++;
        }
    }

    void traceLightPath(const std::vector<std::shared_ptr<Hittable>> &hittables,
                        const Light &lights,
                        Camera &camera,
                        Film &film,
                        std::unique_ptr<DataStructure> &dataStructure) {

        CameraSample cameraSample = monteCarlo.sampleCamera(camera);

        std::vector<PathVertex> pathVertices(mMaxDepth + 1);

        // Sample a light source
        auto [lightPos, lightNormal, light, lightArea] = monteCarlo.sampleEmissive(lights);

        // Create light vertex
        Color emission = light->material->getEmissionColor();
        double lightPdf = monteCarlo.evaluateLightPdf(lights, light);

        pathVertices[0] = PathVertex(lightPos, lightNormal, emission / lightPdf, light, true, 1.0, 1.0,
                                     light->material->getEmissionColor());

        // Connect the light source directly to camera (s=0, t=1 path)
        connectToCamera(pathVertices, 0, camera, film, hittables, dataStructure, cameraSample);

        // Sample initial direction from light
        Vec3 direction = monteCarlo.sampleDirection(sampleDiffuseStrat, Diffuse, lightNormal,
                                                    Ray({0, 0, 0}, {0, 0, 0}), 0.2);

        double cosTheta = dot(direction, lightNormal);
        double pdfDirection = cosTheta / M_PI;

        // Create and trace ray from light
        Ray ray(lightPos, direction);
        const double pdfForward = pdfDirection / cosTheta;
        Vec3 throughput = pathVertices[0].throughput / pdfForward;

        // Trace the light path
        int pathLength = 1;
        while (pathLength < mMaxDepth) {
            int hitObjectIndex = -1;
            dataStructure->intersect(ray, hittables, hitObjectIndex);

            if (hitObjectIndex == -1 || ray.t >= std::numeric_limits<double>::max()) {
                break;  // Ray missed all objects
            }

            const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectIndex];
            const Vec3 hitPoint = ray.origin + ray.direction * ray.t;
            const Vec3 normal = hitObject->calculateNormal(hitPoint);
            const auto material = hitObject->material;
            const auto materialType = material->getType();

            if (materialType == Mirror) {
                // DON'T store vertex for mirrors - just continue the path
                Vec3 reflectionDirection = reflect(ray.direction, normal);
                ray = Ray(hitPoint, reflectionDirection);
                pathLength++;
                continue;
            }

            if (materialType == Transparent) {
                // DON'T store vertex for transparent materials - just continue the path
                bool entering = dot(ray.direction, normal) < 0;
                Vec3 outwardNormal = entering ? normal : -normal;
                double refractionRatio = entering ? (1.0 / material->getIOR()) : material->getIOR();

                double cosTheta = std::min(dot(-ray.direction, outwardNormal), 1.0);
                double sinTheta = sqrt(1.0 - cosTheta * cosTheta);

                bool cannotRefract = refractionRatio * sinTheta > 1.0;
                double reflectance = schlick(cosTheta, refractionRatio);

                if (cannotRefract || monteCarlo.generateNumber() < reflectance) {
                    Vec3 reflectionDirection = reflect(ray.direction, outwardNormal);
                    ray = Ray(hitPoint, reflectionDirection);
                } else {
                    Vec3 refractionDirection = refract(ray.direction, outwardNormal, refractionRatio);
                    ray = Ray(hitPoint, refractionDirection);
                }
                pathLength++;
                continue;
            }


            // Store the vertex
            pathVertices[pathLength] = PathVertex(hitPoint, normal, throughput, hitObject, true, 1.0, 1.0,
                                                  material->shade());

            // Evaluate all connections to camera (s, 1) strategies
            // It is 1, due to not checking the light itself (Would be 0)
            connectToCamera(pathVertices, pathLength, camera, film, hittables, dataStructure, cameraSample);

            pathLength += 1;

            // Break if we've hit a light source
            if (material->getType() == Emissive) {
                break;
            }

            // Russian Roulette termination for longer paths
            if (pathLength > 3) {
                double terminationProb = std::max(0.05, 1 - vecAvg(throughput));
                if (monteCarlo.generateNumber() < terminationProb) {
                    break;
                }
                throughput /= (1 - terminationProb);
            }

            // Sample new direction and continue
            auto [dir, brdf, pdf] = monteCarlo.sample(normal, material->shade(), material->getType(),
                                                      ray, sampleAutoStrat, material->getRoughness());

            if (pdf <= 0) break;

            ray = Ray(hitPoint, dir);
            throughput = throughput * (brdf / pdf);
        }
    }

    void connectToCamera(std::vector<PathVertex> &pathVertices,
                         int pathLength,
                         Camera &camera,
                         Film &film,
                         const std::vector<std::shared_ptr<Hittable>> &hittables,
                         std::unique_ptr<DataStructure> &dataStructure, const CameraSample &cameraSample) {

        PathVertex &vertex = pathVertices[pathLength];

        // Safeguard to avoid dividing by 0
        const double cameraPdf = camera.getLensRadius() > 0.0 ? 1 / camera.getLens()->area() : 1;

        Vec3 lensPoint = cameraSample.position;

        // Compute direction to camera
        Vec3 toCamera = lensPoint - vertex.position;
        double distanceToCamera = toCamera.length();
        toCamera = unitVector(toCamera);

        // Check for occlusion
        Ray shadowRay(vertex.position, toCamera);
        shadowRay.t = distanceToCamera - 1e-4;  // Set max distance to just before camera

        int hitObjectIndex = -1;
        dataStructure->intersect(shadowRay, hittables, hitObjectIndex);

        if (hitObjectIndex != -1) {
            return;  // Path is occluded
        }

        // Try to connect to camera
        double screenX, screenY;
        // More like World to Film
        if (!camera.worldToScreen(vertex.position, lensPoint, screenX, screenY)) {
            return;
        }

        // Calculate importance (We) at camera
        double importance = camera.evaluateImportance(vertex.position, lensPoint);

        if (importance <= 0) {
            return;
        }

        Vec3 previousVertexPosition;
        Vec3 incomingDir;

        if (pathLength > 0) {
            previousVertexPosition = pathVertices[pathLength - 1].position;
            incomingDir = unitVector(previousVertexPosition - vertex.position);
        } else {
            incomingDir = Vec3(0, 0, 0); // Placeholder, won't be used
        }
        // Create ray for BSDF evaluation
        Ray incomingRay(previousVertexPosition, -incomingDir);

        Color contribution;
        if (vertex.object->material->getType() == Emissive) {
            // If this is another emissive surface hit by the light path
            double cosToCamera = std::max(0.0, dot(cameraSample.normal, -toCamera));
            double G = cosToCamera / (distanceToCamera * distanceToCamera);

            contribution = vertex.throughput * G * importance * (1 / cameraPdf);
        } else {
            // For non-emissive surfaces, evaluate the material BSDF
            Color bsdf = monteCarlo.evaluate(vertex.albedo, vertex.object->material->getType(),
                                             toCamera, vertex.normal, incomingRay,
                                             vertex.object->material->getRoughness());
            // Can use different G term, but should be the same nonetheless
            double cosTheta = std::max(0.0, dot(vertex.normal, toCamera));
            double G = cosTheta / (distanceToCamera * distanceToCamera);

            contribution = vertex.throughput * bsdf * G * importance * (1 / cameraPdf);
        }

        // Add to film
        film.addLightSample(screenX, screenY, contribution);
    }
};


#endif // RAYTRACER_LIGHTTRACER_H