#ifndef HITTABLE_H
#define HITTABLE_H

#include <memory>
#include "../acceleration/BoundingBox.h"
#include "../materials/Material.h"
#include "../core/Ray.h"

// Hittable class that defines all objects hittable by a viewing ray
class Hittable {
public:
    virtual ~Hittable() = default;

    Vec3 objectColor;
    std::shared_ptr<Material> material;

    Hittable(const Vec3 &_color, const std::shared_ptr<Material> &_material)
            : objectColor(_color), material(_material) {
    }

    [[nodiscard]] virtual bool intersect(Ray &ray) const = 0;

    [[nodiscard]] virtual Vec3 calculateNormal(const point3 &hitPoint, const Ray &ray) const = 0;

    [[nodiscard]] virtual point3 calculateCenter() const = 0;

    virtual void applyModelTransform(const Vec3 &translation, const Vec3 &rotation, const Vec3 &shear,
                                     double angle, const point3 &objectCenter) = 0;

    virtual void
    applyViewTransform(const Vec3 &translation, const Vec3 &rotation, double angle, const point3 &cam) = 0;

    [[nodiscard]] virtual BoundingBox getBoundingBox() const = 0;

    [[nodiscard]] virtual Vec3 getV0() const = 0;

    [[nodiscard]] virtual Vec3 getV1() const = 0;

    [[nodiscard]] virtual Vec3 getV2() const = 0;

    [[nodiscard]] virtual Vec3
    calculateBarycentricCoordinates(const Vec3 &p) const = 0;

    [[nodiscard]] virtual double interpolateCoordinate1(const Vec3 &barycentric) const = 0;

    [[nodiscard]] virtual double interpolateCoordinate2(const Vec3 &barycentric) const = 0;

    [[nodiscard]] virtual double area() const = 0;

    [[nodiscard]] virtual double volume() const = 0;

};

#endif //HITTABLE_H
