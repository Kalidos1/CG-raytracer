#ifndef RAYTRACER_CAMERA_H
#define RAYTRACER_CAMERA_H

#include "core/Vec3.h"
#include "core/Ray.h"
#include "geometry/Disk.h"
#include <random>

class Camera {
public:

    Camera(const point3 &position, const Vec3 &direction,
           double fov, double aspectRatio,
           double imageWidth, double imageHeight, double lensRadius = 0.0, double focalDistance = 1.0) :
            position(position), direction(direction), lensRadius(lensRadius),
            focalDistance(focalDistance), imageWidth(imageWidth), imageHeight(imageHeight) {

        // Setup CS for camera (right-handed)
        w = -unitVector(direction);
        const Vec3 up = Vec3(0, 1, 0);
        u = unitVector(cross(up, w));
        v = cross(w, u);

        // Calculate film dimensions
        double fovInRadians = fov * M_PI / 180.0;
        filmHeight = 2.0 * focalDistance * tan(0.5 * fovInRadians);
        filmWidth = filmHeight * aspectRatio;
        filmArea = filmHeight * filmWidth;

        if (lensRadius > 0.0) {
            auto lensMaterial = std::make_shared<DiffuseMaterial>(Color(1, 1, 1));
            lens = std::make_shared<Disk>(position, -w, lensRadius, lensMaterial);
        }
    }

    // Needed to avoid having to rebuild datastructure
    Camera() : position(0, 0, 0), direction(0, 0, -1), imageWidth(800), imageHeight(800),
               lensRadius(0), focalDistance(1.0) {
        // Initialize coordinate system
        w = -unitVector(direction);
        const Vec3 up = Vec3(0, 1, 0);
        u = unitVector(cross(up, w));
        v = cross(w, u);

        // Calculate film dimensions
        double fov = 60.0 * M_PI / 180.0;  // 60 degrees in radians
        filmHeight = 2.0 * focalDistance * tan(0.5 * fov);
        filmWidth = filmHeight * (imageWidth / imageHeight);
        filmArea = filmHeight * filmWidth;
    }

    [[nodiscard]] std::shared_ptr<Disk> getLens() const {
        return lens;
    }

    // Check if a hittable object is the camera lens
    bool isLens(const std::shared_ptr<Hittable> &object) const {
        return lens && object == lens;
    }

    Ray generateRay(double pixelX, double pixelY) {
        // Convert pixel coordinates to normalized device coordinates [-1, 1]
        double sx = (2.0 * pixelX / imageWidth) - 1.0;
        double sy = 1.0 - (2.0 * pixelY / imageHeight);

        // Convert to film plane coordinates (matching your worldToScreen method)
        double filmX = sx * filmWidth * 0.5;
        double filmY = sy * filmHeight * 0.5;

        // Calculate point on film plane using camera's coordinate system
        Vec3 filmPoint = position - focalDistance * w + filmX * u + filmY * v;

        // Ray from camera position to film point
        Vec3 rayDirection = unitVector(filmPoint - position);

        return Ray(position, rayDirection);
    }

    double evaluateImportance(const Vec3 &hitPoint, const Vec3 &cameraPoint) {
        Vec3 dir = hitPoint - cameraPoint;
        const double distanceSqrd = dir.lengthSquared();
        dir = unitVector(dir);

        const double cosTheta = -dot(dir, w);

        // Grazing angle
        if (cosTheta <= 1e-6) return 0.0;

        // Do not need that section probably????
        const double t = focalDistance / cosTheta;
        const Vec3 filmPoint = cameraPoint + t * dir;

        // Calculate normalized screen coordinates
        const Vec3 relPos = filmPoint - (position - focalDistance * w);
        const double sx = (dot(relPos, u) + filmWidth / 2) / filmWidth;
        const double sy = (dot(relPos, v) + filmHeight / 2) / filmHeight;

        // Outside film bounds = no contribution
        if (sx < -1 || sx > 1 || sy < -1 || sy > 1) return 0.0;

        // Geometry Term
        const double G = cosTheta * cosTheta / distanceSqrd;

        // Pdf = Pinhole camera is essentially 1.0
        return lensRadius > 0.0 ? 1.0 / (filmArea * lens->area() * G) : 1.0 / (filmArea * G);
    }

    // Need to check this method to map them correctly at the end -> Might currently only be lucky
    bool worldToScreen(const Vec3 &worldPos, const Vec3 &cameraPoint, double &outX, double &outY) {
        const Vec3 toWorld = worldPos - cameraPoint;

        // Check if point is in front of camera
        double const cosTheta = -dot(toWorld, w);
        if (cosTheta <= 1e-6) return false;

        // Project onto film plane
        const double t = focalDistance / cosTheta;
        const Vec3 filmPoint = cameraPoint + t * toWorld; // Might need to change this to cameraPoint

        // Get position relative to film center
        const Vec3 relPos = filmPoint - (position - focalDistance * w);

        // Use the same normalization as ray generation
        // Do not need anymore
        double margin = 1;
        double sx = dot(relPos, u) / (filmWidth / margin);
        double sy = dot(relPos, v) / (filmHeight / margin);

        // Check if within expanded camera bounds
        if (sx < -1 || sx > 1 || sy < -1 || sy > 1) {
            return false;
        }

        // Convert to pixel coordinates
        outX = (sx + 0.5) * imageWidth;
        outY = (0.5 - sy) * imageHeight;

        return true;
    }

    [[nodiscard]] point3 getPosition() const { return position; }

    [[nodiscard]] point3 getDirection() const { return direction; }

    [[nodiscard]] point3 getW() const { return w; }

    [[nodiscard]] point3 getU() const { return u; }

    [[nodiscard]] point3 getV() const { return v; }

    [[nodiscard]] double getFilmArea() const { return filmArea; }

    [[nodiscard]] double getLensRadius() const { return lensRadius; }

private:
    // Camera parameters
    point3 position;         // Camera position in world space
    Vec3 direction;          // Normalized camera direction
    double imageWidth;       // Width of the image in pixels
    double imageHeight;      // Height of the image in pixels

    // Camera coordinate system (orthonormal basis)
    Vec3 u, v, w;            // Camera basis vectors

    // Film plane properties
    std::shared_ptr<Disk> lens;
    double lensRadius;
    double focalDistance;    // Distance from camera to film plane
    double filmWidth;        // Width of the film in world units
    double filmHeight;       // Height of the film in world units
    double filmArea;         // Area of the film in world units

};

#endif //RAYTRACER_CAMERA_H
