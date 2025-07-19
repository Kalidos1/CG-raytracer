#ifndef SPHERE_H
#define SPHERE_H

#include <memory>
#include "Hittable.h"

class Sphere : public Hittable {
public:
    Sphere(const point3 &_center, const double _radius, const Color &_sphere_color,
           const std::shared_ptr<Material> &material) : center(_center),
                                                        radius(_radius),
                                                        Hittable(_sphere_color, material) {
    }


    bool intersect(Ray &ray) const override {
        const Vec3 oc = ray.origin - center;
        const double a = dot(ray.direction, ray.direction);
        const double b = 2 * dot(oc, ray.direction);
        const double c = dot(oc, oc) - pow(radius, 2);

        const double discriminant = b * b - 4 * a * c;
        if (discriminant < 1e-10) return false;

        // Find the nearest intersection which is in acceptable range
        const double t1 = (-b - sqrt(discriminant)) / (2.0f * a);
        const double t2 = (-b + sqrt(discriminant)) / (2.0f * a);

        if (t1 > 0.0001f && t1 < ray.t) {
            ray.t = t1;
            return true;
        }

        if (t2 > 0.0001f && t2 < ray.t) {
            ray.t = t2;
            return true;
        }

        return false;
    }

    [[nodiscard]] Vec3 calculateNormal(const point3 &hitPoint) const override {
        return unitVector(hitPoint - center);
    }

    [[nodiscard]] point3 calculateCenter() const override {
        return center;
    }

    // Only translation, due to sphere
    void applyModelTransform(const Vec3 &translation, const Vec3 &rotation, const Vec3 &shear, double angle,
                             const point3 &objectCenter) override {
        center = center + translation;
    }

    // We subtract to simulate camera movement
    void applyViewTransform(const Vec3 &translation, const Vec3 &rotation, double angle, const point3 &cam) override {
        center = center - translation;
    }

    [[nodiscard]] BoundingBox getBoundingBox() const override {
        const Vec3 min = center - Vec3(radius, radius, radius);
        const Vec3 max = center + Vec3(radius, radius, radius);

        return {min, max};
    }

    [[nodiscard]] Vec3 getV0() const override {
        return {0, 0, 0};
    }

    [[nodiscard]] Vec3 getV1() const override {
        return {0, 0, 0};
    }

    [[nodiscard]] Vec3 getV2() const override {
        return {0, 0, 0};
    }

    [[nodiscard]] Vec3 calculateBarycentricCoordinates(const Vec3 &p) const override {
        return {0, 0, 0};
    }

    [[nodiscard]] double interpolateCoordinate1(const Vec3 &barycentric) const override {
        return 0.0;
    }

    [[nodiscard]] double interpolateCoordinate2(const Vec3 &barycentric) const override {
        return 0.0;
    }

    [[nodiscard]] double area() const override {
        return 4.0 * M_PI * radius * radius;
    }

    [[nodiscard]] double volume() const override {
        return (4.0 / 3.0) * M_PI * radius * radius * radius;
    }

private:
    point3 center;
    double radius;
};

#endif //SPHERE_H
