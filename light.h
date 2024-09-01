#ifndef LIGHT_H
#define LIGHT_H


#include <vector>

#include "color.h"
#include "mat3.h"
#include "vec3.h"

class Light {
public:
    color lightColor;
    point3 origin;
    vec3 direction;
    double intensity;

    Light(const color &_lightColor, const point3 &_origin, const vec3 &_direction,
          double _intensity) : lightColor(_lightColor), origin(_origin), direction(_direction),
                               intensity(_intensity) {
    }

    // Create lights list and create soft shadows through that
    std::vector<std::shared_ptr<Light>> createPlaneLight() {
        std::vector<std::shared_ptr<Light>> lights;
        lights.emplace_back(std::make_shared<Light>(lightColor, origin, direction, intensity));
        lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(0.1, 0, 0), direction, intensity));
        lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(-0.1, 0, 0), direction, intensity));
        //lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(0, 0.1, 0), direction, intensity));
        //lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(0.1, 0.1, 0), direction, intensity));
        // lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(-0.1, 0.1, 0), direction, intensity));
        //lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(0, -0.1, 0), direction, intensity));
        // lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(0.1, -0.1, 0), direction, intensity));
        // lights.emplace_back(std::make_shared<Light>(lightColor, origin + point3(-0.1, -0.1, 0), direction, intensity));

        return lights;
    }

    void applyViewTransform(const vec3 &translation, const vec3 &rotation, double angle) {
        //Create the rotation matrix
        const mat3 R = mat3::rotationMatrix(rotation, degreesToRadians(angle));

        origin = R * origin;

        // Apply the translation to the objects to simulate camera movement
        applyTranslation(-translation);
    }

    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

private:
    void applyTranslation(const vec3 &translation) {
        origin = origin - translation;
    }
};


#endif //LIGHT_H
