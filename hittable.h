#ifndef HITTABLE_H
#define HITTABLE_H

#include <memory>
#include "bounding_box.h"
#include "material.h"
#include "ray.h"

// Hittable class that defines all objects hittable by a viewing ray
class Hittable {
public:
    virtual ~Hittable() = default;

    vec3 object_color;
    std::shared_ptr<Material> material;

    Hittable(const vec3 &_color, const std::shared_ptr<Material> &_material)
            : object_color(_color), material(_material) {
    }

    [[nodiscard]] virtual bool intersect(Ray &ray) const = 0;

    [[nodiscard]] virtual vec3 calculate_normal(const point3 &hit_point, const Ray &ray) const = 0;

    [[nodiscard]] virtual point3 calculate_center() const = 0;

    virtual void apply_model_transform(const vec3 &translation, const vec3 &rotation, const vec3 &shear,
                                       double angle, const point3 &object_center) = 0;

    virtual void
    apply_view_transform(const vec3 &translation, const vec3 &rotation, double angle, const point3 &cam) = 0;

    [[nodiscard]] virtual BoundingBox get_bounding_box() const = 0;

    [[nodiscard]] virtual vec3 get_v0() const = 0;

    [[nodiscard]] virtual vec3 get_v1() const = 0;

    [[nodiscard]] virtual vec3 get_v2() const = 0;

    [[nodiscard]] virtual vec3
    calculate_barycentric_coordinates(const vec3 &p) const = 0;

    [[nodiscard]] virtual double interpolate_coordinate_1(const vec3 &barycentric) const = 0;

    [[nodiscard]] virtual double interpolate_coordinate_2(const vec3 &barycentric) const = 0;

    [[nodiscard]] virtual double area() const = 0;

    [[nodiscard]] virtual double volume() const = 0;

};
#endif //HITTABLE_H
