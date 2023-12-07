#ifndef HITTABLE_H
#define HITTABLE_H

#include "material.h"
#include "ray.h"

class Hittable {
public:
    virtual ~Hittable() = default;

    vec3 object_color;
    std::shared_ptr<Material> material;

    Hittable(const vec3&_color, const std::shared_ptr<Material>&_material)
        : object_color(_color), material(_material) {
    }

    virtual bool intersect(const Ray&ray, double&t) const = 0;

    [[nodiscard]] virtual vec3 calculate_normal(const point3&hit_point, const Ray&ray) const = 0;

    virtual void apply_model_transform(const vec3&translation, const vec3&rotation, const vec3&shear,
                                       double angle) = 0;

    virtual void apply_view_transform(const vec3&translation, const vec3&rotation, double angle, const point3&cam) = 0;
};

#endif //HITTABLE_H
