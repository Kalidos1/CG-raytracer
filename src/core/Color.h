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

inline bool hasEmission(const Color &color) {
    // Check if any component is significantly above zero
    // Using a small threshold to handle floating-point precision
    const double threshold = 1e-4;
    return color.x() > threshold || color.y() > threshold || color.z() > threshold;
}

#endif //RAYTRACER_COLOR_H
