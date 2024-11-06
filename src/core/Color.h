#ifndef RAYTRACER_COLOR_H
#define RAYTRACER_COLOR_H

#include "Vec3.h"

#include <iostream>

using Color = Vec3;

inline void writeColor(std::ostream &out, const Color &pixelColor) {
    // Write the translated [0,255] value of each Color component.
    out << static_cast<int>(255.999 * pixelColor.x()) << ' '
        << static_cast<int>(255.999 * pixelColor.y()) << ' '
        << static_cast<int>(255.999 * pixelColor.z()) << '\n';
}

#endif //RAYTRACER_COLOR_H
