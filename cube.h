#ifndef CUBE_H
#define CUBE_H

#include <vector>

#include "bounded_plane.h"
#include "hittable.h"

class Cube final : public Hittable {
public:
    Cube(const point3&_center, const color&_cube_color, double _side_length, const std::shared_ptr<Material>&_material)
        : Hittable(_cube_color, _material), center(_center), side_length(_side_length) {
        // Calculate half of the side length
        double half_side = side_length / 2.0;

        // Define the normals and points for each face of the cube
        vec3 normals[6] = {
            {1, 0, 0}, // Right face
            {-1, 0, 0}, // Left face
            {0, 1, 0}, // Top face
            {0, -1, 0}, // Bottom face
            {0, 0, 1}, // Front face
            {0, 0, -1} // Back face
        };

        point3 points[6] = {
            center + vec3(half_side, 0, 0), // Right face
            center + vec3(-half_side, 0, 0), // Left face
            center + vec3(0, half_side, 0), // Top face
            center + vec3(0, -half_side, 0), // Bottom face
            center + vec3(0, 0, half_side), // Front face
            center + vec3(0, 0, -half_side) // Back face
        };

        // Create the bounded planes for each face of the cube
        for (int i = 0; i < 6; ++i) {
            bounded_planes.emplace_back(
                std::make_shared<BoundedPlane>(points[i], normals[i], side_length, _cube_color, _material));
        }
    }

    bool intersect(const Ray&ray, double&t) const override {
        // Iterate over each face of the cube and find the closest intersection
        bool hit = false;
        double closest_t = std::numeric_limits<double>::infinity();

        for (const auto&plane: bounded_planes) {
            double current_t;
            if (plane->intersect(ray, current_t) && current_t < closest_t) {
                hit = true;
                closest_t = current_t;
            }
        }

        if (hit) {
            t = closest_t;
            return true;
        }

        return false;
    }

    [[nodiscard]] vec3 calculate_normal(const point3&hit_point, const Ray&ray) const override {
        // Find the plane with the closest intersection and return its normal
        double closest_t = std::numeric_limits<double>::infinity();
        std::shared_ptr<Hittable> closest_plane = nullptr;

        for (const auto&plane: bounded_planes) {
            double current_t;
            if (plane->intersect(ray, current_t) && current_t < closest_t) {
                closest_t = current_t;
                closest_plane = plane;
            }
        }

        if (closest_plane) {
            return closest_plane->calculate_normal(hit_point, ray);
        }

        // Default normal (this should not happen)
        return {0, 0, 0};
    }

    // Go through every plane and do the model transform
    void apply_model_transform(const vec3&translation, const vec3&rotation, const vec3&shear,
                               const double angle) override {
    }

    // Go through every plane and do the view transform
    void apply_view_transform(const vec3&translation, const vec3&rotation, double angle,
                              const point3&camera) override {
    }

private:
    point3 center;
    double side_length;
    std::vector<std::shared_ptr<Hittable>> bounded_planes;
};

#endif //CUBE_H
