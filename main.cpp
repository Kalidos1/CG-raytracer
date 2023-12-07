#define USE_MATH_DEFINES

#include "color.h"
#include "vec3.h"
#include "ray.h"
#include "sphere.h"
#include "plane.h"
#include "triangle.h"
#include "bounded_plane.h"
#include "cube.h"

#include <iostream>
#include <fstream>
#include <vector>

#include "light.h"

bool occluded(const std::shared_ptr<Light>&light, const vec3&hit_point,
              const std::vector<std::shared_ptr<Hittable>>&hittables,
              double t) {
    //Calculate light direction and shadow ray (From hitpoint to light source)
    const vec3 light_direction = unit_vector(light->origin - hit_point);
    const Ray shadow_ray(hit_point, light_direction);

    // Check if the shadow ray intersects with any object
    for (const auto&hittable: hittables) {
        // Check if the shadow ray intersects with the object
        // If the distance from the hit point -> Intersection point is greater than hit point
        // -> Light source than we do not consider this point because it is technically behind the light
        if (hittable->intersect(shadow_ray, t) && t < (light->origin - hit_point).length()) {
            // Check if the intersection point is on the same object
            const vec3 intersection_point = shadow_ray.origin + shadow_ray.direction * t;
            if ((intersection_point - hit_point).length() > 1e-6) return true;
        }
    }
    return false;
}


vec3 trace(const Ray&ray, const std::vector<std::shared_ptr<Hittable>>&hittables,
           const std::vector<std::shared_ptr<Light>>&lights, const int depth) {
    if (depth <= 0) {
        // Maximum recursion depth reached, return background color
        return {0.7, 0.8, 1.0};
    }

    double t = std::numeric_limits<double>::infinity();
    int hit_object = -1;

    // Find the nearest object intersection
    for (int i = 0; i < hittables.size(); ++i) {
        double current_t = t;
        if (hittables[i]->intersect(ray, current_t) && current_t < t) {
            t = current_t;
            hit_object = i;
        }
    }

    if (hit_object != -1) {
        const std::shared_ptr<Hittable>&hitObject = hittables[hit_object];


        // Calculate the hit point and normal of the object
        const vec3 hit_point = ray.origin + ray.direction * t;
        const vec3 normal = hitObject->calculate_normal(hit_point, ray);

        color hit_color = color(0, 0, 0);
        for (auto&light: lights) {
            // Check if the material is mirror, and if so, compute reflection recursively
            if (const auto mirrorMaterial = std::dynamic_pointer_cast<Mirror>(hitObject->material)) {
                const vec3 reflected = reflect(unit_vector(ray.direction), normal);
                const Ray reflected_ray(hit_point, reflected);
                return trace(reflected_ray, hittables, lights, depth - 1);
            }

            const color shade_color = hitObject->material->shade(light, hit_point, ray,
                                                                 normal,
                                                                 hitObject->object_color);

            if (occluded(light, hit_point, hittables, t)) hit_color += shade_color * 0.7;
            else hit_color += clamp(shade_color, 0, 1);
        }
        return clamp(hit_color, 0, 1);
    }

    // Blue sky
    return {0.7, 0.8, 1.0};
}


int main() {
    // Open file to save image
    std::ofstream myFile;
    myFile.open("../image_fixed.ppm");

    // Image
    const int image_width = 800;
    const int image_height = 800;

    // Camera
    point3 camera = point3(1, 1, 5);
    vec3 camera_direction = vec3(-0.25, -0.25, -1);

    // Lights
    color white = color(1, 1, 1);
    std::vector<std::shared_ptr<Light>> lights;
    lights.emplace_back(std::make_shared<Light>(white, point3(2, 3, 0), vec3(0, 0, 0), 0.5));
    lights.emplace_back(std::make_shared<Light>(white, point3(1.0, 3.6, 0), vec3(0, 0, 0), 0.5));
    lights.emplace_back(std::make_shared<Light>(white, point3(1.5, 3.4, 0), vec3(0, 0, 0), 0.5));
    lights.emplace_back(std::make_shared<Light>(white, point3(2.5, 3.2, 3), vec3(0, 0, 0), 0.5));

    //Materials
    auto phong_material = std::make_shared<PhongMaterial>(15);
    auto lambertian_material = std::make_shared<LambertianMaterial>();
    auto checkered_material = std::make_shared<CheckeredMaterial>(5, color(0.7, 0.7, 0.7), color(0.3, 0.3, 0.3));
    auto mirror_material = std::make_shared<Mirror>(vec3(0.8, 0.3, 0.3));

    //Objects
    std::vector<std::shared_ptr<Hittable>> hittables;
    //.emplace_back() -> More efficient way to add elements to ::vector of hittables

    // Spheres

    // Red sphere
    //hittables.emplace_back(std::make_shared<Sphere>(point3(0, -4, -5), 1.0, color(1, 0, 0), phong_material));
    // Blue sphere
    hittables.emplace_back(std::make_shared<Sphere>(point3(-5, -2, -5), 2, color(0, 0, 1), lambertian_material));
    // Green sphere
    hittables.emplace_back(std::make_shared<Sphere>(point3(2, -2, -5), 0.8, color(0, 1, 0), phong_material));
    // Center
    //hittables.emplace_back(std::make_shared<Sphere>(point3(0, 0, 0), 0.2, color(0.5, 0.5, 0.5), phong_material));

    // Triangles
    hittables.emplace_back(std::make_shared<Triangle>(point3(-1, 0.5, -5), point3(1, 0.5, -5), point3(0, 2, -5),
                                                      color(0, 1, 0), phong_material));

    // Quad
    //hittables.emplace_back(std::make_shared<BoundedPlane>(point3(0, -4, -5), point3(0.5, 1, 0), 2,
    //                                                  color(0, 1, 1), phong_material));

    //hittables.emplace_back(std::make_shared<BoundedPlane>(point3(2, -2, -5), point3(-0.5, 1, -0.25), 2,
    //                                                  color(1, 1, 0), lambertian_material));
    // Cube
    hittables.emplace_back(std::make_shared<Cube>(point3(0, 0, 0), color(0, 1, 1), 1, lambertian_material));

    // Floor
    // TODO: 3. Implement .obj loader
    // TODO: 4. Check for model and view transform
    // TODO: 5. Go over stuff and fix everything that is not fixed
    // TODO: 6. Creating a spehrical light source instead of a vector
    // TODO: Create documentation along the way

    hittables.emplace_back(std::make_shared<Plane>(vec3(0, -5, 0), vec3(0, 1, 0), white,
                                                   checkered_material));

    hittables.emplace_back(std::make_shared<Plane>(vec3(-3, -2, -100), vec3(-0.5, -0.5, 1), white,
                                                   checkered_material));

    // TODO Check how this works again, might need to transform everything respective to the camera
    // Irgendwas mit den vorzeichen probably von der rotation und translation etc.

    // Transform objects with model transform
    for (const auto&object: hittables) {
        //object->apply_model_transform(vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0), 45);
    }

    // Transfrom camera -> Move all objects as if we would move the camera
    for (const auto&object: hittables) {
        //object->apply_view_transform(vec3(0, 0, 0), vec3(0, 0, 0), 20, camera);
    }
    // Also move light depending on translation to simulate camera movement
    // Might also need to change floor
    //light = vec3(2, 5, 0);

    // Render
    myFile << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    // Go over every pixel in image height and width
    for (int j = 0; j < image_height; ++j) {
        std::clog << "\rScanlines remaining: " << image_height - j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            color pixel_color;
            // 4x supersampling -> read a bit more about supersampling
            // Cast multiple rays through different sub-pixel locations within the pixel (Make each pixel into 4 parts)
            // Average the colors obtained from these 4 rays to get the final pixel color
            for (int sy = 0; sy < 2; ++sy) {
                for (int sx = 0; sx < 2; ++sx) {
                    // Get values between 0 and 1 to normalize coords
                    // -> Does allow mapping of pixels to point on the image regardless of resolution
                    const double u = (i - image_width / 2 + 0.5 + sx * 0.5) / image_width;
                    const double v = (image_height / 2 - j - 0.5 - sy * 0.5) / image_height;

                    Ray ray(camera, camera_direction + vec3(u, v, 0));

                    pixel_color = pixel_color + trace(ray, hittables, lights, 2);
                }
            }


            // const double u = static_cast<double>(i - image_width / 2) / image_width;
            // const double v = static_cast<double>(image_height / 2 - j) / image_height;
            // Ray ray(camera, camera_direction + vec3(u, v, 0));
            //vec3 pixel_color = trace(ray, hittables, light, light_color);

            write_color(myFile, pixel_color * 0.25);
        }
    }

    myFile.close();
}
