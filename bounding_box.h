#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <vector>
#include <memory>

struct BoundingBox {
    vec3 min;
    vec3 max;

    // TODO: Might need to check if there is a better solution
    [[nodiscard]] bool intersect(Ray &ray) const {
//        // AABB intersection with SLAB method
//
//        // We basically repeatedly clip lines in all directions to figure out if it is inside the box
//        // Check the x value of the boxes and get min and max
//        double tMin = (min.x() - ray.origin.x()) / ray.direction.x();
//        double tMax = (max.x() - ray.origin.x()) / ray.direction.x();
//
//        if (tMin > tMax) std::swap(tMin, tMax);
//
//        // Check the y value of the boxes and get min and max
//        double tYMin = (min.y() - ray.origin.y()) / ray.direction.y();
//        double tYMax = (max.y() - ray.origin.y()) / ray.direction.y();
//
//        if (tYMin > tYMax) std::swap(tYMin, tYMax);
//
//        // The x portion of is outside in relativ to the y axes
//        if (tMin > tYMax || tYMin > tMax) {
//            return false;
//        }
//
//        // Set the current x value to the min y value
//        if (tYMin > tMin) {
//            tMin = tYMin;
//        }
//        if (tYMax < tMax) {
//            tMax = tYMax;
//        }
//
//        // Check the z value of the boxes and get min and max
//        double tZMin = (min.z() - ray.origin.z()) / ray.direction.z();
//        double tZMax = (max.z() - ray.origin.z()) / ray.direction.z();
//
//        if (tZMin > tZMax) std::swap(tZMin, tZMax);
//
//        if (tMin > tZMax || tZMin > tMax) {
//            return false;
//        }
//
//        return true;

        double tmin = -std::numeric_limits<double>::max(), tmax = std::numeric_limits<double>::max();

        // Iterate over each dimension
        for (int i = 0; i < 3; ++i) {
            double invD = 1.0 / ray.direction[i];
            // Compute intersection
            double t0 = (min[i] - ray.origin[i]) * invD;
            double t1 = (max[i] - ray.origin[i]) * invD;
            // Check for negative ray direction
            if (invD < 0.0)
                std::swap(t0, t1);
            tmin = t0 > tmin ? t0 : tmin;
            tmax = t1 < tmax ? t1 : tmax;
            if (tmax <= tmin)
                return false;
        }
        // Check for valid intersection
        return tmax >= 0 && tmin < ray.t;
    }
};

#endif //BOUNDING_BOX_H
