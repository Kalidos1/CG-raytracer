#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <vector>
#include <memory>
#include <algorithm>
#include "hittable.h"

struct BoundingBox {
    vec3 min;
    vec3 max;

    BoundingBox() {
        int imin = std::numeric_limits<int>::min();
        int imax = std::numeric_limits<int>::max();
        min = vec3(imax, imax, imax);
        max = vec3(imin, imin, imin);
    }

    BoundingBox(const vec3 &min, const vec3 &max) : min(min), max(max) {}

    [[nodiscard]] double intersect(Ray &ray) const {
//        // AABB intersection with SLAB method
//        double tmin = -std::numeric_limits<double>::max(), tmax = std::numeric_limits<double>::max();
//
//        // Iterate over each dimension
//        for (int i = 0; i < 3; ++i) {
//            double invD = 1.0 / ray.direction[i];
//            // Compute intersection
//            double t0 = (min[i] - ray.origin[i]) * invD;
//            double t1 = (max[i] - ray.origin[i]) * invD;
//            // Check for negative ray direction
//            if (invD < 0.0)
//                std::swap(t0, t1);
//            tmin = t0 > tmin ? t0 : tmin;
//            tmax = t1 < tmax ? t1 : tmax;
//            if (tmax <= tmin)
//                return false;
//        }
//        // Check for valid intersection
//        return tmax >= 0 && tmin < ray.t;

        double tx1 = (min.x() - ray.origin.x()) / ray.direction.x(), tx2 =
                (max.x() - ray.origin.x()) / ray.direction.x();
        double tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
        double ty1 = (min.y() - ray.origin.y()) / ray.direction.y(), ty2 =
                (max.y() - ray.origin.y()) / ray.direction.y();
        tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
        double tz1 = (min.z() - ray.origin.z()) / ray.direction.z(), tz2 =
                (max.z() - ray.origin.z()) / ray.direction.z();
        tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));
        if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin; else return std::numeric_limits<double>::infinity();
    }

    void grow(vec3 vector) {
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
        vec3 extent = max - min;
        return extent.x() * extent.y() + extent.y() * extent.z() + extent.x() * extent.z();
    }

    [[nodiscard]] BoundingBox bb_union(const BoundingBox &b) const {
        return {
                vec3(std::min(min.x(), b.min.x()),
                     std::min(min.y(), b.min.y()),
                     std::min(min.z(), b.min.z())),
                vec3(std::max(max.x(), b.max.x()),
                     std::max(max.y(), b.max.y()),
                     std::max(max.z(), b.max.z()))
        };
    }

    [[nodiscard]] vec3 center() const {
        return (min + max) / 2.0;
    }

    [[nodiscard]] double volume() const {
        vec3 extent = max - min;
        return extent.x() * extent.y() * extent.z();
    }

    [[nodiscard]] vec3 offset(const point3 &p) const {
        vec3 o = p - min;
        if (max.x() > min.x()) o[0] /= max.x() - min.x();
        if (max.y() > min.y()) o[1] /= max.y() - min.y();
        if (max.z() > min.z()) o[2] /= max.z() - min.z();
        return o;
    }
};

#endif //BOUNDING_BOX_H
