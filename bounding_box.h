#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

struct BoundingBox {
    vec3 min;
    vec3 max;

    [[nodiscard]] bool intersect(const Ray &ray) const {
        // AABB intersection with SLAB method

        // We basically repeatedly clip lines in all directions to figure out if it is inside the box
        // Check the x value of the boxes and get min and max
        double tMin = (min.x() - ray.origin.x()) / ray.direction.x();
        double tMax = (max.x() - ray.origin.x()) / ray.direction.x();

        if (tMin > tMax) std::swap(tMin, tMax);

        // Check the y value of the boxes and get min and max
        double tYMin = (min.y() - ray.origin.y()) / ray.direction.y();
        double tYMax = (max.y() - ray.origin.y()) / ray.direction.y();

        if (tYMin > tYMax) std::swap(tYMin, tYMax);

        // The x portion of is outside in relativ to the y axes
        if (tMin > tYMax || tYMin > tMax) {
            return false;
        }

        // Set the current x value to the min y value
        if (tYMin > tMin) {
            tMin = tYMin;
        }
        if (tYMax < tMax) {
            tMax = tYMax;
        }

        // Check the z value of the boxes and get min and max
        double tZMin = (min.z() - ray.origin.z()) / ray.direction.z();
        double tZMax = (max.z() - ray.origin.z()) / ray.direction.z();

        if (tZMin > tZMax) std::swap(tZMin, tZMax);

        if (tMin > tZMax || tZMin > tMax) {
            return false;
        }

        return true;
    }

    [[nodiscard]] double min_distance(const vec3 &point) const {
        const double dx = std::max(0.0, std::max(min.x() - point.x(), point.x() - max.x()));
        const double dy = std::max(0.0, std::max(min.y() - point.y(), point.y() - max.y()));
        const double dz = std::max(0.0, std::max(min.z() - point.z(), point.z() - max.z()));

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    [[nodiscard]] bool isValid() const {
        const double epsilon = std::numeric_limits<double>::epsilon();
        return min.x() < max.x() - epsilon && min.y() < max.y() - epsilon && min.z() < max.z() - epsilon;
    }

    [[nodiscard]] vec3 calculate_center() const {
        return (min + max) / 2;
    }
};

#endif //BOUNDING_BOX_H
