#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <memory>
#include "hittable.h"
#include "mat3.h"

class Triangle : public Hittable {
public:
    Triangle(const vec3 &_v0, const vec3 &_v1, const vec3 &_v2, const double _uv0[2], const double _uv1[2],
             const double _uv2[2], const color &_triangleColor,
             const std::shared_ptr<Material> &material) : v0(_v0),
                                                          v1(_v1),
                                                          v2(_v2),
                                                          uv0{_uv0[0], _uv0[1]},
                                                          uv1{_uv1[0], _uv1[1]},
                                                          uv2{_uv2[0], _uv2[1]},
                                                          Hittable(_triangleColor, material) {
    }

    // MT (Moeller-Trumbore) intersection algorithm -> https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection.html
    // Put the initial x,y,z triangle into u,v space and then calculate the positions of pixels inside barycentric coordinates
    bool intersect(Ray &ray) const override {
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
        double t = dot(edge2, qvec) * invDet;
        if (t > 0.0001f) ray.t = std::min(ray.t, t);

        return true;
    }

    [[nodiscard]] vec3 calculateNormal(const point3 &hitPoint, const Ray &ray) const override {
        const vec3 firstEdge = v1 - v0;
        const vec3 secondEdge = v2 - v0;
        return unitVector(cross(firstEdge, secondEdge));
    }

    [[nodiscard]] point3 calculateCenter() const override {
        return (v0 + v1 + v2) / 3.0;
    }

    void applyModelTransform(const vec3 &translation, const vec3 &rotation, const vec3 &shear,
                             const double angle, const point3 &objectCenter) override {
        if (angle != 0) applyRotation(rotation, angle, objectCenter);
        applyTranslation(translation);
        applyShear(shear);
    }


    // Make the angle value inside the rotation matrix
    void
    applyViewTransform(const vec3 &translation, const vec3 &rotation, double angle, const point3 &camera) override {
        //Create the rotation matrix
        const mat3 R = mat3::rotationMatrix(rotation, degreesToRadians(angle));

        // Apply rotation matrix to each of the vectors
        v0 = R.operator*(v0);
        v1 = R.operator*(v1);
        v2 = R.operator*(v2);

        // Apply the translation to the objects to simulate camera movement
        applyTranslation(-translation);
    }

    [[nodiscard]] BoundingBox getBoundingBox() const override {
        //Get min value of all triangle sides
        const vec3 min(
                vecMin({v0.x(), v1.x(), v2.x()}),
                vecMin({v0.y(), v1.y(), v2.y()}),
                vecMin({v0.z(), v1.z(), v2.z()})
        );

        // Get max value of all triangle sides
        const vec3 max(
                vecMax({v0.x(), v1.x(), v2.x()}),
                vecMax({v0.y(), v1.y(), v2.y()}),
                vecMax({v0.z(), v1.z(), v2.z()})
        );

        return {min, max};
    }


    static double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    [[nodiscard]] vec3 getV0() const override {
        return v0;
    }

    [[nodiscard]] vec3 getV1() const override {
        return v1;
    }

    [[nodiscard]] vec3 getV2() const override {
        return v2;
    }

    [[nodiscard]] vec3
    calculateBarycentricCoordinates(const vec3 &p) const override {
        vec3 v_0 = v1 - v0;
        vec3 v_1 = v2 - v0;
        vec3 v_2 = p - v0;

        double d00 = dot(v_0, v_0);
        double d01 = dot(v_0, v_1);
        double d11 = dot(v_1, v_1);
        double d20 = dot(v_2, v_0);
        double d21 = dot(v_2, v_1);

        double denom = d00 * d11 - d01 * d01;
        double vTemp = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double uTemp = 1.0 - vTemp - w;

        return {uTemp, vTemp, w};
    }

    [[nodiscard]] double interpolateCoordinate1(const vec3 &barycentric) const override {
        return uv0[0] * barycentric.x() + uv1[0] * barycentric.y() + uv2[0] * barycentric.z();
    }

    [[nodiscard]] double interpolateCoordinate2(const vec3 &barycentric) const override {
        return uv0[1] * barycentric.x() + uv1[1] * barycentric.y() + uv2[1] * barycentric.z();
    }

    [[nodiscard]] double area() const override {
        vec3 edge1 = v1 - v0;
        vec3 edge2 = v2 - v0;
        vec3 crossProduct = cross(edge1, edge2);
        return 0.5 * crossProduct.length();
    }

    [[nodiscard]] double volume() const override {
        return std::abs(v0.x() * (v1.y() * v2.z() - v1.z() * v2.y()) -
                        v0.y() * (v1.x() * v2.z() - v1.z() * v2.x()) +
                        v0.z() * (v1.x() * v2.y() - v1.y() * v2.x())) / 6.0;
    }

private:
    vec3 v0, v1, v2;
    double uv0[2], uv1[2], uv2[2];

    void applyTranslation(const vec3 &translation) {
        v0 += translation;
        v1 += translation;
        v2 += translation;
    }

    void applyRotation(const vec3 &rotation, const double angle, const point3 &objectCenter) {
        // calculate center of the entire thing (e.g. if it is a cube then we rotate around the center of the cube)
        const vec3 center = objectCenter;
        // Translate triangle to origin
        applyTranslation(-center);

        // Create a inverse rotation matrix to mirror rotation direction
        const mat3 R = mat3::rotationMatrix(rotation, degreesToRadians(angle));
        const mat3 inverse = R.inverse();

        // Apply rotation matrix to each of the vectors
        v0 = inverse.operator*(v0);
        v1 = inverse.operator*(v1);
        v2 = inverse.operator*(v2);

        // Translate back to original position
        applyTranslation(center);
    }

    //TODO
    void applyShear(const vec3 &shear) {
    }
};

#endif //TRIANGLE_H
