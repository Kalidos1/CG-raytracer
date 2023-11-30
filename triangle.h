#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"
#include "mat3.h"

class Triangle : public Hittable {
public:
    Triangle(const vec3&_v0, const vec3&_v1, const vec3&_v2, const color&_triangle_color,
             const std::shared_ptr<Material>&material) : v0(_v0),
                                                         v1(_v1),
                                                         v2(_v2),
                                                         Hittable(_triangle_color, material) {
    }

    // MT (Moeller-Trumbore) intersection algorithm
    // Put the initial x,y,z triangle into u,v space and then calculate the positions of pixels inside barycentric coordinates
    bool intersect(const Ray&ray, double&t) const override {
        const vec3 edge1 = v1 - v0;
        const vec3 edge2 = v2 - v0;
        const vec3 pvec = cross(ray.direction, edge2);
        const double det = dot(edge1, pvec);

        // Check for backfacing triangle (Negative = Backfacing, Close to 0 = Misses the triangle)
        if (det < 0.0000f) return false;
        // Ray and triangle = parallel if det is close to 0
        if (fabs(det) < 0.0000f) return false;

        const double invDet = 1 / det;

        // Calculate one side of the translated triangle (Origin is also the triangle origin)
        const vec3 tvec = ray.origin - v0;
        const double u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;

        // Calculate the other edge of the translated triangle and check for intersections
        const vec3 qvec = cross(tvec, edge1);
        const double v = dot(ray.direction, qvec) * invDet;
        if (v < 0 || u + v > 1) return false;

        // If we have a intersection we calculate the pixel point
        t = dot(edge2, qvec) * invDet;

        return true;
    }

    [[nodiscard]] vec3 calculate_normal(const point3&hit_point) const override {
        const vec3 first_edge = v1 - v0;
        const vec3 second_edge = v2 - v0;
        return unit_vector(cross(first_edge, second_edge));
    }

    void apply_model_transform(const vec3&translation, const vec3&rotation, const vec3&shear,
                               const double angle) override {
        apply_translation(translation);

        apply_rotation(rotation, angle);

        apply_shear(shear);
    }


    //Make the angle value inside the rotation matrix
    // Not sure what the difference is to the model transformation except the relation to the camera
    // It might be interesting to only move the camera? But not sure
    void apply_view_transform(const vec3&translation, const vec3&rotation, double angle, const point3&camera) override {
        // Create the rotation matrix and the inverse
        const mat3 R = mat3::rotation_matrix(rotation, degrees_to_radians(angle));
        const mat3 inverse = R.inverse();

        v0 = inverse * (v0 - camera);
        v1 = inverse * (v1 - camera);
        v2 = inverse * (v2 - camera);

        apply_translation(translation);
    }


    double degrees_to_radians(double degrees) {
        return degrees * M_PI / 180.0;
    }

private:
    vec3 v0, v1, v2;

    void apply_translation(const vec3&translation) {
        v0 = v0 + translation;
        v1 = v1 + translation;
        v2 = v2 + translation;
    }

    // Fix Rotation if its below 0
    void apply_rotation(const vec3&rotation, const double angle) {
        // calculate triangle center
        const vec3 triangle_center = {
            (v0.x() + v1.x() + v2.x()) / 3.0f, (v0.y() + v1.y() + v2.y()) / 3.0f, (v0.z() + v1.z() + v2.z()) / 3.0f
        };
        // Translate triangle to origin
        apply_translation(-triangle_center);

        // Create a inverse rotation matrix to mirror rotation direction
        const mat3 R = mat3::rotation_matrix(rotation, degrees_to_radians(angle));
        const mat3 inverse = R.inverse();

        // Apply rotation matrix to each of the vectors
        v0 = inverse.operator*(v0);
        v1 = inverse.operator*(v1);
        v2 = inverse.operator*(v2);

        // Translate back to original position
        apply_translation(triangle_center);
    }

    void apply_shear(const vec3&shear) {
    }
};

#endif //TRIANGLE_H
