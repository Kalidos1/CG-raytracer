#ifndef FLOOR_H
#define FLOOR_H
#include "hittable.h"

class Floor : public Hittable {
public:
    Floor(const double _size, const color&_color,
          const std::shared_ptr<Material>&material) : size(_size),
                                                      Hittable(_color, material) {
    }

    bool intersect(const Ray&ray, double&t) const override {
        // Size is basically the height of the floor
        const double plane_intersection = (ray.origin.y() + size) / -ray.direction.y();

        if (plane_intersection > 0.000001 && plane_intersection < t) {
            t = plane_intersection;
            return true;
        }

        return false;
    }

    [[nodiscard]] vec3 calculate_normal(const point3&hit_point) const override {
        return {0, 0, 0};
    }

    // Do not change the floor with the transforms
    void apply_model_transform(const vec3&translation, const vec3&rotation, const vec3&shear, double angle) override {
    }

    // Keep the floor at the bottom to simulate camera moving
    void apply_view_transform(const vec3&translation, const vec3&rotation, double angle, const point3&cam) override {
        size = size - translation.y();
    }

private:
    double size;
};

#endif //FLOOR_H
