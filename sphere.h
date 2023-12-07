#ifndef SPHERE_H
#define SPHERE_H

#include "color.h"
#include "hittable.h"
#include "material.h"

class Sphere : public Hittable {
public:
    Sphere(const point3&_center, const double _radius, const color&_sphere_color,
           const std::shared_ptr<Material>&material) : center(_center),
                                                       radius(_radius),
                                                       Hittable(_sphere_color, material) {
    }


    bool intersect(const Ray&ray, double&t) const override {
        const vec3 oc = ray.origin - center;
        const double a = dot(ray.direction, ray.direction);
        const double b = 2 * dot(oc, ray.direction);
        const double c = dot(oc, oc) - pow(radius, 2);

        const double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return false;

        // Find the nearest intersection which is in acceptable range
        const double t1 = (-b - sqrt(discriminant)) / (2.0f * a);
        const double t2 = (-b + sqrt(discriminant)) / (2.0f * a);

        if (t1 > 0.001f && t1 < t) {
            t = t1;
            return true;
        }

        if (t2 > 0.001f && t2 < t) {
            t = t2;
            return true;
        }

        return false;
    }

    [[nodiscard]] vec3 calculate_normal(const point3&hit_point, const Ray&ray) const override {
        return unit_vector(hit_point - center);
    }

    // Does not make sense to rotate or shear? a sphere so only translation for now
    void apply_model_transform(const vec3&translation, const vec3&rotation, const vec3&shear, double angle) override {
        center = center + translation;
    }

    // Not sure how much we are going to do right here
    void apply_view_transform(const vec3&translation, const vec3&rotation, double angle, const point3&cam) override {
        center = center + translation;
    }

private:
    point3 center;
    double radius;
};

#endif //SPHERE_H
