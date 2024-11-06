#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <vector>
#include <memory>
#include <algorithm>
#include "../geometry/Hittable.h"
#include "core/Ray.h"

struct BoundingBox {
    Vec3 min;
    Vec3 max;
    double tmaxBox = std::numeric_limits<double>::infinity();

    BoundingBox() {
        int imin = std::numeric_limits<int>::min();
        int imax = std::numeric_limits<int>::max();
        min = Vec3(imax, imax, imax);
        max = Vec3(imin, imin, imin);
    }

    BoundingBox(const Vec3 &min, const Vec3 &max) : min(min), max(max) {}

    [[nodiscard]] double intersect(Ray &ray) {
        double tx1 = (min.x() - ray.origin.x()) / ray.direction.x(), tx2 =
                (max.x() - ray.origin.x()) / ray.direction.x();
        double tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
        double ty1 = (min.y() - ray.origin.y()) / ray.direction.y(), ty2 =
                (max.y() - ray.origin.y()) / ray.direction.y();
        tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
        double tz1 = (min.z() - ray.origin.z()) / ray.direction.z(), tz2 =
                (max.z() - ray.origin.z()) / ray.direction.z();
        tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));
        if (tmax >= tmin && tmin < ray.t && tmax > 0) {
            tmaxBox = tmax;
            return tmin;
        } else {
            return std::numeric_limits<double>::infinity();
        }
    }

    void grow(Vec3 vector) {
        min = ::min(min, vector);
        max = ::max(max, vector);
    }

    void grow(BoundingBox box) {
        if (box.min.x() != 1e30f) {
            grow(box.min);
            grow(box.max);
        }
    }

    [[nodiscard]] double area() const {
        Vec3 extent = max - min;
        return extent.x() * extent.y() + extent.y() * extent.z() + extent.x() * extent.z();
    }

    [[nodiscard]] BoundingBox bbUnion(const BoundingBox &b) const {
        return {
                Vec3(std::min(min.x(), b.min.x()),
                     std::min(min.y(), b.min.y()),
                     std::min(min.z(), b.min.z())),
                Vec3(std::max(max.x(), b.max.x()),
                     std::max(max.y(), b.max.y()),
                     std::max(max.z(), b.max.z()))
        };
    }

    [[nodiscard]] Vec3 center() const {
        return (min + max) / 2.0;
    }

    [[nodiscard]] double volume() const {
        Vec3 extent = max - min;
        return extent.x() * extent.y() * extent.z();
    }

    [[nodiscard]] Vec3 offset(const point3 &p) const {
        Vec3 o = p - min;
        if (max.x() > min.x()) o[0] /= max.x() - min.x();
        if (max.y() > min.y()) o[1] /= max.y() - min.y();
        if (max.z() > min.z()) o[2] /= max.z() - min.z();
        return o;
    }
};

#endif //BOUNDING_BOX_H
