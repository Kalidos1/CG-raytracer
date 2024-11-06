#ifndef RAYTRACER_TRACE_H
#define RAYTRACER_TRACE_H

#include "geometry/Hittable.h"
#include "lighting/ToneMap.h"
#include "acceleration/DataStructure.h"

class Trace {
public:
    virtual ~Trace() = default;

    static Color computeColorForRay(Ray &ray,
                                    const std::vector<std::shared_ptr<Hittable>> &hittables,
                                    const std::vector<std::shared_ptr<Light>> &lights,
                                    int depth,
                                    std::unique_ptr<DataStructure> &dataStructure) {
        if (depth <= 0) {
            return {0.7, 0.8, 1.0};
        }

        int hitObjectTemp = -1;

        dataStructure->intersect(ray, hittables, hitObjectTemp);

        if (hitObjectTemp != -1 && ray.t < std::numeric_limits<double>::max()) {
            const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectTemp];

            // Calculate the hit point and normal of the object
            const Vec3 hitPoint = (ray.origin + ray.direction * ray.t);
            const Vec3 barycentric = hitObject->calculateBarycentricCoordinates(hitPoint);
            const Vec3 normal = hitObject->calculateNormal(hitPoint, ray);

            // TODO: Make method for Color

            Color hitColor = Color(0, 0, 0);
            Color shadeColor = Color(0, 0, 0);
            for (auto &light: lights) {
                // Check if the material is mirror, and if so, compute reflection recursively
//            if (const auto mirrorMaterial = std::dynamic_pointer_cast<Mirror>(hitObject->material)) {
//                const Vec3 reflected = reflect(unitVector(ray.direction), normal);
//                const Ray reflected_ray(hitPoint, reflected);
//                return trace(reflected_ray, hittables, lights, depth - 1);
//            }

                // Interpolate texture coordinates
                double interpolatedUV[2] = {hitObject->interpolateCoordinate1(barycentric),
                                            hitObject->interpolateCoordinate2(barycentric)};

                shadeColor = hitObject->material->shade(light, hitPoint, ray,
                                                        normal, hitObject->objectColor, interpolatedUV);


                if (isLightOccluded(light, hitPoint, hittables, ray.t, dataStructure)) {
                    shadeColor *= 0.7;
                    hitColor += shadeColor;
                } else {
                    hitColor += shadeColor;
                }
            }

            // Apply tone mapping
            std::unique_ptr<ToneMap> tone_mapper = std::make_unique<ReinhardToneMap>(4.0);
            Color finalColor = tone_mapper->apply(hitColor);

            return clamp(finalColor, 0, 1);
        }

        // Blue sky
        return {0.7, 0.8, 1.0};
    }

private:
    static bool isLightOccluded(const std::shared_ptr<Light> &light,
                                const Vec3 &hitPoint,
                                const std::vector<std::shared_ptr<Hittable>> &hittables,
                                double t,
                                std::unique_ptr<DataStructure> &dataStructure) {
        //Calculate light direction and shadow ray (From hitpoint to light source)
        const Vec3 lightDirection = unitVector(light->origin - hitPoint);
        Ray shadowRay(hitPoint, lightDirection);
        int hitObjectTemp = -1;

        dataStructure->intersect(shadowRay, hittables, hitObjectTemp);

        if (hitObjectTemp != -1 && shadowRay.t < std::numeric_limits<double>::max()) {
            const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectTemp];

            // Check if the shadow ray intersects with the object
            // If the distance from the hit point -> Intersection point is greater than hit point
            // -> Light source than we do not consider this point because it is technically behind the light
            if (hitObject->intersect(shadowRay) && shadowRay.t < (light->origin - hitPoint).length()) {
                // Check if the intersection point is on the same object
                const Vec3 intersectionPoint = shadowRay.origin + shadowRay.direction * shadowRay.t;
                if ((intersectionPoint - hitPoint).length() > 1e-6) return true;
            }
        }
        return false;
    }

};

#endif //RAYTRACER_TRACE_H
