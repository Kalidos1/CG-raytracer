#ifndef LIGHT_H
#define LIGHT_H


#include <vector>

#include "color.h"
#include "mat3.h"
#include "vec3.h"

class Light {
public:
    color light_color;
    point3 origin;
    vec3 direction;
    double intensity;

    Light(const color &_light_color, const point3 &_origin, const vec3 &_direction,
          double _intensity) : light_color(_light_color), origin(_origin), direction(_direction),
                               intensity(_intensity) {
    }

    // Create lights list and create soft shadows through that
    std::vector<std::shared_ptr<Light>> createPlaneLight() {
        std::vector<std::shared_ptr<Light>> lights;
        lights.emplace_back(std::make_shared<Light>(light_color, origin, direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(0.1, 0, 0), direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(-0.1, 0, 0), direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(0, 0.1, 0), direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(0.1, 0.1, 0), direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(-0.1, 0.1, 0), direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(0, -0.1, 0), direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(0.1, -0.1, 0), direction, intensity));
//        lights.emplace_back(std::make_shared<Light>(light_color, origin + point3(-0.1, -0.1, 0), direction, intensity));

        return lights;
    }

    void apply_view_transform(const vec3 &translation, const vec3 &rotation, double angle) {
        //Create the rotation matrix
        const mat3 R = mat3::rotation_matrix(rotation, degrees_to_radians(angle));

        origin = R * origin;

        // Apply the translation to the objects to simulate camera movement
        apply_translation(-translation);
    }

    double degrees_to_radians(double degrees) {
        return degrees * M_PI / 180.0;
    }

private:
    void apply_translation(const vec3 &translation) {
        origin = origin - translation;
    }
};


#endif //LIGHT_H
