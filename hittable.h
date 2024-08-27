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

    vec3 objectColor;
    std::shared_ptr<Material> material;

    Hittable(const vec3 &_color, const std::shared_ptr<Material> &_material)
            : objectColor(_color), material(_material) {
    }

    [[nodiscard]] virtual bool intersect(Ray &ray) const = 0;

    [[nodiscard]] virtual vec3 calculateNormal(const point3 &hitPoint, const Ray &ray) const = 0;

    [[nodiscard]] virtual point3 calculateCenter() const = 0;

    virtual void applyModelTransform(const vec3 &translation, const vec3 &rotation, const vec3 &shear,
                                     double angle, const point3 &objectCenter) = 0;

    virtual void
    applyViewTransform(const vec3 &translation, const vec3 &rotation, double angle, const point3 &cam) = 0;

    [[nodiscard]] virtual BoundingBox getBoundingBox() const = 0;

    [[nodiscard]] virtual vec3 getV0() const = 0;

    [[nodiscard]] virtual vec3 getV1() const = 0;

    [[nodiscard]] virtual vec3 getV2() const = 0;

    [[nodiscard]] virtual vec3
    calculateBarycentricCoordinates(const vec3 &p) const = 0;

    [[nodiscard]] virtual double interpolateCoordinate1(const vec3 &barycentric) const = 0;

    [[nodiscard]] virtual double interpolateCoordinate2(const vec3 &barycentric) const = 0;

    [[nodiscard]] virtual double area() const = 0;

    [[nodiscard]] virtual double volume() const = 0;

};

#endif //HITTABLE_H
