#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <vector>
#include <memory>
#include <algorithm>

struct BoundingBox {
    vec3 min;
    vec3 max;

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

    // Optimized AABB intersection test
    inline double intersectAABB(const Ray &ray, const vec3 &invDir, const int dirIsNeg[3]) {
        vec3 tMin = (min - ray.origin) * invDir;
        vec3 tMax = (max - ray.origin) * invDir;

        if (dirIsNeg[0]) std::swap(tMin[0], tMax[0]);
        if (dirIsNeg[1]) std::swap(tMin[1], tMax[1]);
        if (dirIsNeg[2]) std::swap(tMin[2], tMax[2]);

        double tEnter = std::max(std::max(tMin.x(), tMin.y()), tMin.z());
        double tExit = std::min(std::min(tMax.x(), tMax.y()), tMax.z());

        if (tEnter > tExit || tExit < 0) return std::numeric_limits<double>::infinity();
        return tEnter;
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
        return BoundingBox(
                vec3(std::min(min.x(), b.min.x()),
                     std::min(min.y(), b.min.y()),
                     std::min(min.z(), b.min.z())),
                vec3(std::max(max.x(), b.max.x()),
                     std::max(max.y(), b.max.y()),
                     std::max(max.z(), b.max.z()))
        );
    }

    [[nodiscard]] vec3 center() const {
        return (min + max) / 2.0;
    }

    [[nodiscard]] double volume() const {
        vec3 extent = max - min;
        return extent.x() * extent.y() * extent.z();
    }
};

#endif //BOUNDING_BOX_H
