#ifndef RAYTRACER_TRACE_H
#define RAYTRACER_TRACE_H

#include "geometry/Hittable.h"
#include "lighting/ToneMap.h"
#include "acceleration/DataStructure.h"
#include "lighting/MonteCarlo.h"

struct Domain {
    Color color;
    double pdf;
    Vec3 sampleDir;
};

class Trace {
public:
    MonteCarloIntegrator monteCarlo;
    int maxDepth;
    SamplingStrat samplingStrat = sampleAutoStrat;

    explicit Trace(int samples = 1, int depth = 3) : monteCarlo(samples), maxDepth(depth) {}

    virtual ~Trace() = default;

    Color computeColorForRay(Ray &ray,
                             const std::vector<std::shared_ptr<Hittable>> &hittables,
                             const Light &lights,
                             int depth,
                             std::unique_ptr<DataStructure> &dataStructure) {
        Color result(0, 0, 0);
        Color throughput(1, 1, 1);
        int currentDepth = 0;
        double prevBsdfPdf = 0.0;
        bool specularBounce = true;

        // Main path tracing loop
        while (currentDepth < maxDepth) {
            // Trace ray and find intersection
            int hitObjectIndex = -1;
            dataStructure->intersect(ray, hittables, hitObjectIndex);

            // No intersection - return accumulated result
            if (hitObjectIndex == -1 || ray.t >= std::numeric_limits<double>::max()) {
                break;
            }

            //return {0.5, 0.7, 0.5};

            // Get hit object information
            const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectIndex];
            const Vec3 hitPoint = ray.origin + ray.direction * ray.t;
            const Vec3 normal = hitObject->calculateNormal(hitPoint);
            const auto material = hitObject->material;
            const auto materialType = material->getType();

            if (materialType == Emissive) {
                // First bounce -> Coloring the light correctly
                if (currentDepth == 0 || specularBounce) {
                    result += throughput * material->getEmissionColor();
                } else {
                    Vec3 P_prev = ray.origin;          // Point where BSDF sampling occurred
                    Vec3 P_light = hitPoint;         // Point on the light source hit
                    Vec3 omega_i = ray.direction;      // Direction sampled by BSDF
                    Vec3 lightNormal = normal;         // Normal on light surface at P_light

                    double lightPdf_area = monteCarlo.evaluateLightPdf(lights, hitObject); // p_select * p_area

                    double lightPdf_sa = 0.0;
                    if (lightPdf_area > 0.0) {
                        double distanceSquared = (P_light - P_prev).lengthSquared();
                        double cosY = std::max(0.0, dot(lightNormal, -omega_i)); // Cosine at light

                        if (cosY > 1e-6) {
                            lightPdf_sa = lightPdf_area * distanceSquared / cosY;
                        }
                    }

                    // Calculate MIS weight using SOLID ANGLE PDFs
                    double weight = 0.0;
                    // Check if denominator is non-zero before dividing
                    if (prevBsdfPdf + lightPdf_sa > 1e-9) {
                        weight = balanceHeuristic(prevBsdfPdf, lightPdf_sa); // Weight for the BSDF path
                    }

                    result += throughput * weight * material->getEmissionColor();

//                    // Here use ray.origin and ray.direction to calculate the hitPoint before
//                    // Important since we calculate the probability of hitting the light with the brdf sample
//                    double lightPdf = monteCarlo.evaluateLightPdf(lights, hitObject);
//
//                    double weight = balanceHeuristic(prevBsdfPdf, lightPdf);
//
//                    result += throughput * weight * material->getEmissionColor();
                }
                break;
            }

            // Russian Roulette termination
            if (currentDepth > 3) {
                double terminationProb = std::max(0.05, 1 - vecAvg(throughput));
                if (monteCarlo.generateNumber() < terminationProb) {
                    break;
                }
                throughput /= (1 - terminationProb);
            }

            bool thisIsSpecularBounce = false;

            // Handle mirror materials (perfect specular reflections)
            // Do not need to sample light if we have a perfect mirror since it probably never hits it if not directly checked
            if (materialType == Mirror) {
                Vec3 reflectionDirection = reflect(ray.direction, normal);
                ray = Ray(hitPoint, reflectionDirection);
                thisIsSpecularBounce = true;

                // Update the overall path's specular status
                // The path remains specular only if all previous bounces were also specular
                specularBounce = specularBounce && thisIsSpecularBounce;
                currentDepth++;
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

                thisIsSpecularBounce = true;  // Transparent bounces are specular
                // Update the overall path's specular status
                specularBounce = specularBounce && thisIsSpecularBounce;
                currentDepth++;
                continue;
            }

            thisIsSpecularBounce = false;

            auto [lightColor, lightPdf, lightDir] = sampleLight(ray, hitPoint, normal, material, hittables,
                                                                lights, dataStructure);
            if (lightPdf > 0) {
                result += throughput * lightColor;
            }

            auto [brdf, pdf, brdfSampleDir] = brdfSampling(ray, normal, material);

            if (pdf <= 0) {
                break;
            }

            ray = Ray(hitPoint, brdfSampleDir);

            throughput = throughput * (brdf / pdf);

            prevBsdfPdf = pdf;
            specularBounce = specularBounce && thisIsSpecularBounce;
            currentDepth++;
        }

        return result;
    }

private:
//    void checkNormalization(double pdf1, double pdf2) {
//        // Your current 50/50 custom weighting
//        double weight1 = 0.5 * pdf1 / (0.5 * pdf1 + 0.5 * pdf2);
//        double weight2 = 0.5 * pdf2 / (0.5 * pdf1 + 0.5 * pdf2);
//
//        // Balance heuristic weights
//        double balanceWeight1 = balanceHeuristic(pdf1, pdf2);
//        double balanceWeight2 = balanceHeuristic(pdf2, pdf1);
//
//        std::cout << "Custom weights sum: " << (weight1 + weight2) << std::endl;
//        std::cout << "Balance weights sum: " << (balanceWeight1 + balanceWeight2) << std::endl;
//    }

    inline double balanceHeuristic(double p1, double p2) {
        return (p1) / (p1 + p2);
    }

    inline double powerHeuristic(double p1, double p2, double beta = 2.0) {
        // Compute power heuristic weights
        double p1_pow = std::pow(p1, beta);
        double p2_pow = std::pow(p2, beta);

        return p1_pow / (p1_pow + p2_pow);
    }

    Domain sampleLight(const Ray &incomingRay, const Vec3 &hitPoint,
                       const Vec3 &normal,
                       const std::shared_ptr<Material> &material,
                       const std::vector<std::shared_ptr<Hittable>> &hittables,
                       const Light &lights,
                       std::unique_ptr<DataStructure> &dataStructure) {
        auto [lightPosition, lightNormal, light, lightArea] = monteCarlo.sampleEmissive(lights);
        auto shadowRay = lightPosition - hitPoint;
        auto sampleDir = unitVector(shadowRay);
        Ray sampleRay(hitPoint, sampleDir);

        int hitLightTemp = -1;
        dataStructure->intersect(sampleRay, hittables, hitLightTemp);

        // Check if we hit a light and it's visible
        if (hitLightTemp == -1 || sampleRay.t >= std::numeric_limits<double>::max()) {
            return {{0, 0, 0}, 0, {0, 0, 0}};
        }

        const std::shared_ptr<Hittable> &object = hittables[hitLightTemp];
        if (object->material->getType() != Emissive) {
            return {{0, 0, 0}, 0, {0, 0, 0}};
        }

        // Compute geometry term
        // Since the light is an area, we convert the integral over the hemisphere into an integral over the light source
        // This enables us to use the same solid angle
        // Basically cos theta
        auto distanceSquared = shadowRay.lengthSquared(); // Distance for attenuation
        const double cosY = std::max(0.0, dot(lightNormal, -sampleDir));
        double lightPdf = monteCarlo.evaluateLightPdf(lights, light);

        // Convert area PDF to solid angle PDF
        double lightPdf_sa = 0.0;
        if (cosY > 1e-6) { // Avoid division by zero/instability
            lightPdf_sa = lightPdf * distanceSquared / cosY;
        }




        // To importance sample, might be interesting to first get the PDF on how likely it is to sample the direction and hit the light
        // Times the PDF on the exact light point on the light source PDF aswell

        // Use here the balance heuristic since we do have an area light source and not a DeltaLight (Single Point Light without PDF values)
        // Need to use here since the weights have to sum up to 1 -> Without the balance heuristic i only would add the weight at the end
        Color brdf = monteCarlo.evaluate(material->shade(), material->getType(), sampleDir, normal, incomingRay,
                                         material->getRoughness());
        double brdfPdf = monteCarlo.calculatePdf(sampleDir, normal, material->getType(), samplingStrat, incomingRay,
                                                 material->getRoughness());
        double weight = balanceHeuristic(lightPdf_sa, brdfPdf);

        const double cosX = std::max(0.0, dot(normal, sampleDir));
        double G = (cosX * cosY) / distanceSquared;

        Color emission = object->material->getEmissionColor();
        Color contribution = (emission * brdf * G * weight) / lightPdf;

        return {contribution, lightPdf, sampleDir};
    }

    Domain brdfSampling(const Ray &incomingRay,
                        const Vec3 &normal,
                        const std::shared_ptr<Material> &material) {
        const auto materialType = material->getType();
        const auto materialRoughness = material->getRoughness();

        auto [dir, brdf, pdf] = monteCarlo.sample(normal, material->shade(),
                                                  materialType, incomingRay, samplingStrat, materialRoughness);

        return {brdf, pdf, dir};
    }
};

#endif //RAYTRACER_TRACE_H