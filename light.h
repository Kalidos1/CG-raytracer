#ifndef LIGHT_H
#define LIGHT_H


#include "color.h"
#include "vec3.h"

struct Light {
    color light_color;
    point3 origin;
    vec3 direction;
    double intensity;

    Light(const color&_light_color, const point3&_origin, const vec3&_direction,
          double _intensity) : light_color(_light_color), origin(_origin), direction(_direction),
                               intensity(_intensity) {
    }

    //calculate light direction (hit point)
};


#endif //LIGHT_H
