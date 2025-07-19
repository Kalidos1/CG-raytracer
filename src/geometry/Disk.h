#ifndef RAYTRACER_DISK_H
#define RAYTRACER_DISK_H


#include <memory>
#include "Hittable.h"

class Disk : public Hittable {
public:
    Disk(const point3 &_center, const Vec3 &_normal, const double _radius,
         const std::shared_ptr<Material> &material) : Hittable(Vec3(0, 0, 0), material), center(_center),
                                                      normal(unitVector(_normal)), radius(_radius) {
        u = getPerpendicularVector(normal);
        v = cross(normal, u);
    }


    bool intersect(Ray &ray) const override {
        // Compute plane intersection first
        double dDotN = dot(ray.direction, normal);

        // Ray is parallel to the disk plane
        if (std::abs(dDotN) < 1e-6) {
            return false;
        }

        // Compute distance to plane
        double t = dot(center - ray.origin, normal) / dDotN;

        // Intersection is behind the ray origin or beyond current closest hit
        if (t <= 0 || t >= ray.t) {
            return false;
        }

        // Compute hit point and check if it's within the disk radius
        Vec3 hitPoint = ray.origin + t * ray.direction;
        Vec3 toHit = hitPoint - center;

        // Project the vector onto the disk plane
        double radiusSq = radius * radius;
        double distSq = toHit.lengthSquared() - std::pow(dot(toHit, normal), 2);

        if (distSq > radiusSq) {
            return false;
        }

        ray.t = t;
        return true;
    }

    [[nodiscard]] Vec3 calculateNormal(const point3 &hitPoint) const override {
        return normal;
    }

    [[nodiscard]] point3 calculateCenter() const override {
        return center;
    }

    // Only translation, due to sphere
    void applyModelTransform(const Vec3 &translation, const Vec3 &rotation, const Vec3 &shear, double angle,
                             const point3 &objectCenter) override {
    }

    // We subtract to simulate camera movement
    void applyViewTransform(const Vec3 &translation, const Vec3 &rotation, double angle, const point3 &cam) override {
    }

    // Bounding Box around the disk
    [[nodiscard]] BoundingBox getBoundingBox() const override {
        Vec3 extents(radius, radius, radius);
        return {center - extents, center + extents};
    }

    [[nodiscard]] Vec3 getV0() const override { return center - radius * u - radius * v; }

    [[nodiscard]] Vec3 getV1() const override { return center + radius * u - radius * v; }

    [[nodiscard]] Vec3 getV2() const override { return center + radius * u + radius * v; }

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
        return M_PI * radius * radius;
    }

    // 0 Volume since only 2D
    [[nodiscard]] double volume() const override {
        return 0.0;
    }

private:
    Vec3 center;
    Vec3 normal;
    double radius;
    Vec3 u, v; // Local coordinate system on the disk plane
};

#endif //RAYTRACER_DISK_H
