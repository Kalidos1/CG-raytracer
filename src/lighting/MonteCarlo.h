#ifndef RAYTRACER_MONTECARLO_H
#define RAYTRACER_MONTECARLO_H

#include <random>
#include <cmath>
#include "core/Vec3.h"

struct SamplingResult {
    Vec3 direction;
    Vec3 brdf{};
    double pdf;
};

enum SamplingStrat {
    sampleAutoStrat,
    sampleDiffuseStrat,
    sampleSpecularStrat,
    sampleMixedStrat
};

struct EmissiveSample {
    Vec3 position;
    Vec3 normal;
    std::shared_ptr<Hittable> selectedLight;
    double area;
};

struct CameraSample {
    Vec3 position;
    Vec3 normal;
};

class MonteCarloIntegrator {
public:
    int numberOfSamples;

    explicit MonteCarloIntegrator(int numSamples) : numberOfSamples(numSamples) {
        std::random_device rd;
        generator = std::mt19937(rd());
        distribution = std::uniform_real_distribution<double>(0.0, 1.0);
    }

    Vec3 evaluate(Color albedo, const MaterialType &type, const Vec3 &sampleDir, const Vec3 &normal, const Ray &ray,
                  const double &roughness) {
        Vec3 wo = -ray.direction;
        double cosTheta = dot(normal, sampleDir);

        if (cosTheta <= 0.0) {
            return {0, 0, 0};
        }

        if (type == Diffuse) {
            return (albedo / M_PI) * cosTheta;
        } else if (type == Specular) {
            // For specular BRDF, compute microfacet model terms
            Vec3 halfway = unitVector(wo + sampleDir);

            // Compute all relevant dot products with safe clamping
            double NoV = std::max(dot(normal, wo), 0.001);
            double NoL = std::max(dot(normal, sampleDir), 0.001);
            double NoH = std::max(dot(normal, halfway), 0.001);
            double VoH = std::max(dot(wo, halfway), 0.001);

            // Compute BRDF terms
            double D = ggxDistribution(NoH, roughness);
            double G = ggxSmith(NoV, NoL, roughness);
            Vec3 F = fresnelSchlick(VoH, albedo);

            Vec3 specular = (D * G * F) / (4.0 * NoV * NoL);

            return specular * NoL;
        } else {  // Mixed
            // Implement mixed material BRDF here
            // Default to 50/50 mix - can be customized per material later
            double diffuseRatio = 0.5;
            double specularRatio = 1.0 - diffuseRatio;

            // Calculate diffuse component
            Vec3 diffuse = (albedo / M_PI) * cosTheta;

            // Calculate specular component (reusing code from specular case)
            Vec3 halfway = unitVector(wo + sampleDir);
            double NoV = std::max(dot(normal, wo), 0.001);
            double NoL = std::max(dot(normal, sampleDir), 0.001);
            double NoH = std::max(dot(normal, halfway), 0.001);
            double VoH = std::max(dot(wo, halfway), 0.001);
            double D = ggxDistribution(NoH, roughness);
            double G = ggxSmith(NoV, NoL, roughness);
            Vec3 F = fresnelSchlick(VoH, albedo);
            Vec3 specular = (D * G * F) / (4.0 * NoV * NoL);

            // Combine diffuse and specular components
            return diffuseRatio * diffuse + specularRatio * (specular * NoL);
        }
    }

    double calculatePdf(const Vec3 &sampleDir, const Vec3 &normal, const MaterialType &type,
                        const SamplingStrat &samplingStrat, const Ray &ray, const double &roughness) {
        double cosTheta = dot(normal, sampleDir);
        Vec3 wo = -ray.direction;
        Vec3 halfway = unitVector(wo + sampleDir);
        double NoH = std::max(dot(normal, halfway), 0.001);
        double VoH = std::max(dot(wo, halfway), 0.001);

        // If using automatic strategy, base PDF on material type
        if (samplingStrat == sampleAutoStrat) {
            if (type == Diffuse) {
                return computeCosinePdf(cosTheta);
            } else if (type == Specular) {
                return computeGGXPdf(NoH, VoH, roughness);
            } else {  // Mixed
                double diffuseRatio = 0.5;
                double specularRatio = 1.0 - diffuseRatio;
                double diffusePdf = computeCosinePdf(cosTheta);
                double specularPdf = computeGGXPdf(NoH, VoH, roughness);
                return diffuseRatio * diffusePdf + specularRatio * specularPdf;
            }
        }

        // If using specific sampling strategy, use corresponding PDF
        switch (samplingStrat) {
            case sampleDiffuseStrat:
                return computeCosinePdf(cosTheta);

            case sampleSpecularStrat:
                return computeGGXPdf(NoH, VoH, roughness);

            case sampleMixedStrat: {
                // For mixed sampling, use weighted sum of PDFs
                double diffuseRatio = 0.5;  // Can be replaced with material-specific ratio
                double specularRatio = 1.0 - diffuseRatio;
                double diffusePdf = computeCosinePdf(cosTheta);
                double specularPdf = computeGGXPdf(NoH, VoH, roughness);
                return diffuseRatio * diffusePdf + specularRatio * specularPdf;
            }

            default:
                return computeCosinePdf(cosTheta);
        }
    }


    Vec3 sampleDirection(const SamplingStrat &samplingStrat, const MaterialType &materialType, const Vec3 &normal,
                         const Ray &ray, const double &roughness) {
        Vec3 sampleDir;
        Vec3 halfway;

        switch (samplingStrat) {
            case sampleDiffuseStrat:
                sampleDir = cosineSampleHemisphere(normal);
                break;
            case sampleSpecularStrat:
                halfway = sampleGGX(normal, roughness);
                sampleDir = reflect(ray.direction, halfway);
                break;
            case sampleMixedStrat: {
                // For mixed materials, randomly choose between diffuse and specular
                double randVal = generateNumber();
                if (randVal < 0.5) {  // Can be replaced with material-specific ratio
                    sampleDir = cosineSampleHemisphere(normal);
                } else {
                    halfway = sampleGGX(normal, roughness);
                    sampleDir = reflect(ray.direction, halfway);
                }
                break;
            }
            case sampleAutoStrat:
            default:
                // Automatically choose based on material type
                if (materialType == Diffuse) {
                    sampleDir = cosineSampleHemisphere(normal);
                } else if (materialType == Specular) {
                    halfway = sampleGGX(normal, roughness);
                    sampleDir = reflect(ray.direction, halfway);
                } else {  // Mixed
                    double randVal = generateNumber();
                    if (randVal < 0.5) {  // Can be replaced with material-specific ratio
                        sampleDir = cosineSampleHemisphere(normal);
                    } else {
                        halfway = sampleGGX(normal, roughness);
                        sampleDir = reflect(ray.direction, halfway);
                    }
                }
                break;
        }

        return sampleDir;
    }

    // New method: create direction, depending on the chosen sample -> Entire scene can be diffuse sampled, or auto sampling
    // Sample method:
    // 1. Direction
    // 2. Put into Brdf evaluation with the chosen direction
    // 3. Compute the PDF for the chosen sample and brdf, etc.
    SamplingResult sample(const Vec3 &normal, const Color &albedo, const MaterialType &type, const Ray &ray,
                          const SamplingStrat &samplingStrat, const double &roughness) {
        bool applyMixedSampling = (samplingStrat == sampleMixedStrat) ||
                                  (samplingStrat == sampleAutoStrat && type == Mixed);

        if (applyMixedSampling) {
            double diffuseRatio = 0.5;  // Can be material-specific
            double specularRatio = 1.0 - diffuseRatio;

            // Choose which technique to sample with
            bool useDiffuseSampling = generateNumber() < diffuseRatio;

            // Sample direction based on chosen technique
            Vec3 sampleDir;
            if (useDiffuseSampling) {
                sampleDir = cosineSampleHemisphere(normal);
            } else {
                Vec3 halfway = sampleGGX(normal, roughness);
                sampleDir = reflect(ray.direction, halfway);
                if (dot(sampleDir, normal) <= 0) {
                    return {{0, 0, 0}, {0, 0, 0}, 0.0};
                }
            }

            // Get the full BRDF value (already includes both components)
            Vec3 brdf = evaluate(albedo, type, sampleDir, normal, ray, roughness);

            if (brdf.lengthSquared() < 1e-9 || dot(sampleDir, normal) <= 0) {
                return {sampleDir, {0, 0, 0}, 0.0};
            }

            // Calculate PDFs for both techniques
            Vec3 wo = -ray.direction;
            Vec3 halfway = unitVector(wo + sampleDir);
            double cosTheta = dot(normal, sampleDir);
            double NoH = std::max(dot(normal, halfway), 0.001);
            double VoH = std::max(dot(wo, halfway), 0.001);

            double diffusePdf = computeCosinePdf(cosTheta);
            double specularPdf = computeGGXPdf(NoH, VoH, roughness);

            double combinedPdf = diffuseRatio * diffusePdf + specularRatio * specularPdf;

            if (combinedPdf < 1e-9) {
                return {sampleDir, {0, 0, 0}, 0.0};
            }

            // Do not weight the brdf in regards to the combinedPdf since it gets double weighted if not
            // Return the weighted BRDF and the PDF we used
            return {sampleDir, brdf, combinedPdf};
        } else {
            // For other strategies, use your existing code
            Vec3 sampleDir = sampleDirection(samplingStrat, type, normal, ray, roughness);

            if (dot(sampleDir, normal) <= 0) {
                return {sampleDir, {0, 0, 0}, 0.0};
            }

            Vec3 brdf = evaluate(albedo, type, sampleDir, normal, ray, roughness);
            double pdf = calculatePdf(sampleDir, normal, type, samplingStrat, ray, roughness);

            if (pdf < 1e-9) {
                return {sampleDir, {0, 0, 0}, 0.0};
            }

            return {sampleDir, brdf, pdf};
        }
    }

    EmissiveSample sampleEmissive(const Light &lights) {
        // MIght need to make this adaptive to the light area -> The larger the area the more likely it is to actually choose it
        int lightIndex = static_cast<int>(generateNumber() * lights.lights.size());
        const auto &selectedLight = lights.lights[lightIndex];

        double u = generateNumber();
        double v = generateNumber();

        if (u + v > 1.0) {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        double w = 1.0 - u - v;

        Vec3 v0 = selectedLight->getV0();
        Vec3 v1 = selectedLight->getV1();
        Vec3 v2 = selectedLight->getV2();

        // Compute the sampled point on the triangle
        Vec3 position = u * v0 + v * v1 + w * v2;

        Vec3 normal = selectedLight->calculateNormal({0, 0, 0});
        double totalArea = selectedLight->area();
        return {position, normal, selectedLight, totalArea};
    }

    CameraSample sampleCamera(const Camera &camera) {
        double lensRadius = camera.getLensRadius();
        const Vec3 center = camera.getPosition();
        const Vec3 normal = -camera.getW();

        if (lensRadius <= 1e-6) {
            return {center, normal};
        }

        double r = sqrt(generateNumber());
        double theta = 2.0 * M_PI * generateNumber();

        const Vec3 transformedU = lensRadius * r * cos(theta) * camera.getU();
        const Vec3 transformedV = lensRadius * r * sin(theta) * camera.getV();

        Vec3 position = center + transformedU + transformedV;

        return {position, normal};
    }

    // Uniform light sampling pdf with 1 / Area
    double evaluateLightPdf(const Light &lights, const std::shared_ptr<Hittable> &hitObject) {
        // Probability of selecting this specific light from all lights
        double lightSelectionPdf = 1.0 / lights.lights.size();

        // Area PDF (uniform sampling on the light)
        double areaPdf = 1.0 / hitObject->area();

        // Final PDF combines light selection and sampling probability
        return lightSelectionPdf * areaPdf;
    }

    double generateNumber() {
        return distribution(generator);
    }

    inline double balanceHeuristic(double p1, double p2) {
        return (p1) / (p1 + p2);
    }


private:
    std::mt19937 generator;
    std::uniform_real_distribution<double> distribution;

    Vec3 uniformSampleHemisphere(const Vec3 &normal) {
        const double xi_1 = generateNumber();
        const double xi_2 = generateNumber();

        double cosTheta = 1 - xi_1;
        double sinTheta = sqrt(1 - cosTheta * cosTheta);
        double cosPhi = cos(2 * M_PI * xi_2);
        double sinPhi = sin(2 * M_PI * xi_2);

        double x = cosPhi * sinTheta;
        double y = sinPhi * sinTheta;
        double z = cosTheta;

        Vec3 sampleDir(x, y, z);
        Vec3 direction = dot(sampleDir, normal) > 0 ? sampleDir : -sampleDir;
        return direction;
    }

    static double computeUniformPdf() {
        return 1 / (2 * M_PI);
    }

    Vec3 cosineSampleHemisphere(const Vec3 &normal) {
        const double xi_1 = generateNumber();
        const double xi_2 = generateNumber();

        double cosTheta = sqrt(1 - xi_1);
        double sinTheta = sqrt(xi_1);
        double cosPhi = cos(2 * M_PI * xi_2);
        double sinPhi = sin(2 * M_PI * xi_2);

        double x = cosPhi * sinTheta;
        double y = sinPhi * sinTheta;
        double z = cosTheta;

        Vec3 up = std::abs(normal.y()) > 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
        Vec3 tangent = unitVector(cross(up, normal));
        Vec3 bitangent = cross(normal, tangent);

        Vec3 sampleDir = x * tangent + y * bitangent + z * normal;
        return sampleDir;
    }

    static double computeCosinePdf(double cosTheta) {
        return cosTheta / M_PI;
    }

    Vec3 sampleGGX(const Vec3 &normal, const double &roughness) {
        double r1 = generateNumber();
        double r2 = generateNumber();

        Vec3 B = getPerpendicularVector(normal);
        Vec3 T = cross(B, normal);
        double alpha2 = roughness * roughness;

        double cosThetaH = sqrt(std::max(0.0, (1.0 - r1) / ((alpha2 - 1.0) * r1 + 1)));
        double sinThetaH = sqrt(std::max(0.0, 1.0 - cosThetaH * cosThetaH));
        double phiH = r2 * M_PI * 2.0;

        return {T * (sinThetaH * cos(phiH)) + B * (sinThetaH * sin(phiH)) + normal * cosThetaH};
    }

    double ggxDistribution(double NoH, const double &roughness) {
        double alpha2 = roughness * roughness;
        double den = ((NoH * alpha2 - NoH) * NoH + 1.0);
        return alpha2 / (std::max(den * den, 1e-8) * M_PI);
    }

    // Actually uses Schlick term
    double ggxSmith(double NoV, double NoL, const double &roughness) {
        double k = roughness * roughness / 2;

        double gV = NoV / (NoV * (1.0 - k) + k);
        double gL = NoL / (NoL * (1.0 - k) + k);
        return gV * gL;
    }

    Vec3 fresnelSchlick(double cosTheta, const Vec3 &F0) {
        return F0 + (Vec3(1.0, 1.0, 1.0) - F0) * pow(1.0 - cosTheta, 5.0);
    }

    double computeGGXPdf(double NoH, double HoV, const double &roughness) {
        HoV = std::max(HoV, 0.001);
        return ggxDistribution(NoH, roughness) * NoH / (4.0 * HoV);
    }


};

#endif //RAYTRACER_MONTECARLO_H
