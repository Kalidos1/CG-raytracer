#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"
#include "mat3.h"

class Triangle : public Hittable {
public:
    Triangle(const vec3 &_v0, const vec3 &_v1, const vec3 &_v2, const color &_triangle_color,
             const std::shared_ptr<Material> &material) : v0(_v0),
                                                          v1(_v1),
                                                          v2(_v2),
                                                          Hittable(_triangle_color, material) {
    }

    // MT (Moeller-Trumbore) intersection algorithm -> https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection.html
    // Put the initial x,y,z triangle into u,v space and then calculate the positions of pixels inside barycentric coordinates
    bool intersect(const Ray &ray, double &t) const override {
        const vec3 edge1 = v1 - v0;
        const vec3 edge2 = v2 - v0;
        const vec3 pvec = cross(ray.direction, edge2);
        const double det = dot(edge1, pvec);

        // Check for backfacing triangle (Negative = Backfacing, Close to 0 = Misses the triangle)
        if (det < 1e-10) return false;
        // Ray and triangle = parallel if det is close to 0
        if (fabs(det) < 1e-10) return false;

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

    [[nodiscard]] vec3 calculate_normal(const point3 &hit_point, const Ray &ray) const override {
        const vec3 first_edge = v1 - v0;
        const vec3 second_edge = v2 - v0;
        return unit_vector(cross(first_edge, second_edge));
    }

    [[nodiscard]] point3 calculate_center() const override {
        return (v0, v1, v2) / 3.0;
    }

    void apply_model_transform(const vec3 &translation, const vec3 &rotation, const vec3 &shear,
                               const double angle, const point3 &object_center) override {
        if (angle != 0) apply_rotation(rotation, angle, object_center);
        apply_translation(translation);
        apply_shear(shear);
    }


    // Make the angle value inside the rotation matrix
    void
    apply_view_transform(const vec3 &translation, const vec3 &rotation, double angle, const point3 &camera) override {
        //Create the rotation matrix
        const mat3 R = mat3::rotation_matrix(rotation, degrees_to_radians(angle));

        // Apply rotation matrix to each of the vectors
        v0 = R.operator*(v0);
        v1 = R.operator*(v1);
        v2 = R.operator*(v2);

        // Apply the translation to the objects to simulate camera movement
        apply_translation(-translation);
    }

    [[nodiscard]] BoundingBox get_bounding_box() const override {
        // Get min value of all triangle sides
        const vec3 min(
                std::min({v0.x(), v1.x(), v2.x()}),
                std::min({v0.y(), v1.y(), v2.y()}),
                std::min({v0.z(), v1.z(), v2.z()})
        );

        // Get max value of all triangle sides
        const vec3 max(
                std::max({v0.x(), v1.x(), v2.x()}),
                std::max({v0.y(), v1.y(), v2.y()}),
                std::max({v0.z(), v1.z(), v2.z()})
        );

        return {min, max};
    }


    double degrees_to_radians(double degrees) {
        return degrees * M_PI / 180.0;
    }

private:
    vec3 v0, v1, v2;

    void apply_translation(const vec3 &translation) {
        v0 += translation;
        v1 += translation;
        v2 += translation;
    }

    void apply_rotation(const vec3 &rotation, const double angle, const point3 &object_center) {
        // calculate center of the entire thing (e.g. if it is a cube then we rotate around the center of the cube)
        const vec3 center = object_center;
        // Translate triangle to origin
        apply_translation(-center);

        // Create a inverse rotation matrix to mirror rotation direction
        const mat3 R = mat3::rotation_matrix(rotation, degrees_to_radians(angle));
        const mat3 inverse = R.inverse();

        // Apply rotation matrix to each of the vectors
        v0 = inverse.operator*(v0);
        v1 = inverse.operator*(v1);
        v2 = inverse.operator*(v2);

        // Translate back to original position
        apply_translation(center);
    }

    //TODO
    void apply_shear(const vec3 &shear) {
    }
};

#endif //TRIANGLE_H
