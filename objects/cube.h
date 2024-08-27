#ifndef CUBE_H
#define CUBE_H

#include <vector>

#include "bounded_plane.h"
#include "hittable.h"

class Cube final : public Hittable {
public:
    Cube(const point3 &_center, const color &_cube_color, double _side_length,
         const std::shared_ptr<Material> &_material)
            : Hittable(_cube_color, _material), center(_center), side_length(_side_length) {
        // Calculate half of the side length
        double half_side = side_length / 2.0;

        // Define the normals and points for each face of the cube
        vec3 normals[6] = {
                {1,  0,  0}, // Right face
                {-1, 0,  0}, // Left face
                {0,  1,  0}, // Top face
                {0,  -1, 0}, // Bottom face
                {0,  0,  1}, // Front face
                {0,  0,  -1} // Back face
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

    bool intersect(const Ray &ray, double &t) const override {
        // Iterate over each face of the cube and find the closest intersection
        bool hit = false;
        double closest_t = std::numeric_limits<double>::infinity();

        for (const auto &plane: bounded_planes) {
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

    [[nodiscard]] vec3 calculateNormal(const point3 &hitPoint, const Ray &ray) const override {
        // Find the plane with the closest intersection and return its normal
        double closest_t = std::numeric_limits<double>::infinity();
        std::shared_ptr<Hittable> closest_plane = nullptr;

        for (const auto &plane: bounded_planes) {
            double current_t;
            if (plane->intersect(ray, current_t) && current_t < closest_t) {
                closest_t = current_t;
                closest_plane = plane;
            }
        }

        if (closest_plane) {
            return closest_plane->calculateNormal(hitPoint, ray);
        }

        // Default normal -> Does not happen, results in error
        return {0, 0, 0};
    }

    [[nodiscard]] point3 calculateCenter() const override {
        return center;
    }

    // Go through every plane and do the model transform
    void applyModelTransform(const vec3 &translation, const vec3 &rotation, const vec3 &shear,
                             const double angle, const point3 &objectCenter) override {
        for (const auto &plane: bounded_planes) {
            plane->applyModelTransform(translation, rotation, shear, angle, objectCenter);
        }
    }

    // Go through every plane and do the view transform
    void applyViewTransform(const vec3 &translation, const vec3 &rotation, double angle,
                            const point3 &camera) override {
        for (const auto &plane: bounded_planes) {
            plane->applyViewTransform(translation, rotation, angle, camera);
        }
    }

    [[nodiscard]] BoundingBox getBoundingBox() const override {
        BoundingBox combinedBox;
        bool initialized = false;


        for (const auto &plane: bounded_planes) {
            BoundingBox planeBox = plane->getBoundingBox();

            // Combine bounding boxes
            if (!initialized) {
                combinedBox = planeBox;
                initialized = true;
            } else {
                combinedBox.min = vec3(
                        std::min(combinedBox.min.x(), planeBox.min.x()),
                        std::min(combinedBox.min.y(), planeBox.min.y()),
                        std::min(combinedBox.min.z(), planeBox.min.z())
                );
                combinedBox.max = vec3(
                        std::max(combinedBox.max.x(), planeBox.max.x()),
                        std::max(combinedBox.max.y(), planeBox.max.y()),
                        std::max(combinedBox.max.z(), planeBox.max.z())
                );
            }
        }

        return combinedBox;
    }

private:
    point3 center;
    double side_length;
    std::vector<std::shared_ptr<Hittable>> bounded_planes;
};

#endif //CUBE_H
