#ifndef BOUNDED_PLANE_H
#define BOUNDED_PLANE_H

#include <memory>
#include "../vec3.h"
#include "../material.h"
#include "../triangle.h"

class BoundedPlane final : public Hittable {
public:
    BoundedPlane(const vec3 &_position, const vec3 &_normal, const double _size, const color &_plane_color,
                 const std::shared_ptr<Material> &_material)
            : Hittable(_plane_color, _material), normal(_normal), position(_position), size(_size) {
        // Calculate basis vectors for the local coordinate system of the plane
        vec3 side1, side2;
        coordinate_system(normal, side1, side2);

        // Calculate vertices for the two mirrored triangles
        vec3 v0 = position - side1 * size / 2.0 - side2 * size / 2.0;
        vec3 v1 = position + side1 * size / 2.0 - side2 * size / 2.0;
        vec3 v2 = position + side1 * size / 2.0 + side2 * size / 2.0;
        vec3 v3 = position - side1 * size / 2.0 + side2 * size / 2.0;

        // Create two triangles using the vertices
        triangle1 = std::make_shared<Triangle>(v0, v1, v2, _plane_color, _material);
        triangle2 = std::make_shared<Triangle>(v0, v2, v3, _plane_color, _material);
    }

    bool intersect(const Ray &ray, double &t) const override {
        double t1 = std::numeric_limits<double>::infinity();
        double t2 = std::numeric_limits<double>::infinity();

        // Check intersection with each triangle
        bool hit1 = triangle1->intersect(ray, t1);
        bool hit2 = triangle2->intersect(ray, t2);

        // Take the closest intersection
        if (hit1 || hit2) {
            t = std::min(t1, t2);
            return true;
        }

        return false;
    }

    [[nodiscard]] vec3 calculateNormal(const point3 &hitPoint, const Ray &ray) const override {
        // Consistent normals, use the normal of either triangle
        return triangle1->calculateNormal(hitPoint, ray);
    }

    [[nodiscard]] point3 calculateCenter() const override {
        return position;
    }


    void applyModelTransform(const vec3 &translation, const vec3 &rotation, const vec3 &shear,
                             const double angle, const point3 &objectCenter) override {
        // Apply the transformation to each triangle
        triangle1->applyModelTransform(translation, rotation, shear, angle, objectCenter);
        triangle2->applyModelTransform(translation, rotation, shear, angle, objectCenter);
    }

    void applyViewTransform(const vec3 &translation, const vec3 &rotation, double angle,
                            const point3 &camera) override {
        // Apply the transformation to each triangle
        triangle1->applyViewTransform(translation, rotation, angle, camera);
        triangle2->applyViewTransform(translation, rotation, angle, camera);
    }

    [[nodiscard]] BoundingBox getBoundingBox() const override {
        // Calculate bounding boxes for each triangle
        BoundingBox box1 = triangle1->getBoundingBox();
        BoundingBox box2 = triangle2->getBoundingBox();

        // Combine these 2 boxes into one box
        BoundingBox combinedBox;
        combinedBox.min = vec3(
                std::min({box1.min.x(), box2.min.x()}),
                std::min({box1.min.y(), box2.min.y()}),
                std::min({box1.min.z(), box2.min.z()})
        );
        combinedBox.max = vec3(
                std::max({box1.min.x(), box2.min.x()}),
                std::max({box1.min.y(), box2.min.y()}),
                std::max({box1.min.z(), box2.min.z()})
        );

        return combinedBox;
    }

private:
    vec3 normal;
    point3 position;
    double size;
    std::shared_ptr<Triangle> triangle1;
    std::shared_ptr<Triangle> triangle2;

    // Calculate a coordinate system (side1, side2) perpendicular to the given normal
    void coordinate_system(const vec3 &n, vec3 &side1, vec3 &side2) const {
        // Check which component has the largest magnitude -> Dominant axis to align coordinate system
        if (std::fabs(n.x()) > std::fabs(n.y())) {
            // Vector perpendicular to the normal along the dominant axis
            // Normalize the cross product of the normal vector with the world up vector
            const double inv_length = 1.0 / std::sqrt(n.x() * n.x() + n.z() * n.z());
            side1 = vec3(-n.z() * inv_length, 0.0, n.x() * inv_length);
        } else {
            const double inv_length = 1.0 / std::sqrt(n.y() * n.y() + n.z() * n.z());
            side1 = vec3(0.0, n.z() * inv_length, -n.y() * inv_length);
        }
        side2 = cross(n, side1);
    }
};

#endif //BOUNDED_PLANE_H
