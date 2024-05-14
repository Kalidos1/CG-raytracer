#define USE_MATH_DEFINES

#include "color.h"
#include "vec3.h"
#include "ray.h"
#include "triangle.h"

#include <iostream>
#include <fstream>
#include <vector>

#include "bounding_box.h"
#include "light.h"
#include "obj_loader.h"
#include "bvh_builder.h"

#include <chrono>
#include <filesystem>
#include <algorithm>
#include <limits>

using namespace std::chrono;

// Tone mapping after https://bruop.github.io/tonemapping/
double tone_map(const double hit) {
    const double L_white = 4.0;
    return hit * (1 + hit / (L_white * L_white)) / (1.0 + hit);
    //return hit / (1.0 * hit);
    /* Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return (hit * (a * hit + b)) / (hit * (c * hit + d) + e);
    */
    /* Lottes 2016, "Advanced Techniques and Optimization of HDR Color Pipelines"
    const float a = 1.6;
    const float d = 0.977;
    const float hdrMax = 8.0;
    const float midIn = 0.18;
    const float midOut = 0.267;

    const float b =
            (-pow(midIn, a) + pow(hdrMax, a) * midOut) /
            ((pow(hdrMax, a * d) - pow(midIn, a * d)) * midOut);
    const float c =
            (pow(hdrMax, a * d) * pow(midIn, a) - pow(hdrMax, a) * pow(midIn, a * d) * midOut) /
            ((pow(hdrMax, a * d) - pow(midIn, a * d)) * midOut);

    return pow(hit, a) / (pow(hit, a * d) * b + c);
    */
}

double gammaCorrect(const double color_value) {
    if (color_value <= 0.0) {
        return 0.0;
    }
    // 2.2 = Standard gamma correction value
    return std::pow(color_value, 2.2);
}

bool occluded(const std::shared_ptr<Light> &light, const vec3 &hit_point,
              const std::vector<std::shared_ptr<Hittable>> &hittables,
              double t) {
    //Calculate light direction and shadow ray (From hitpoint to light source)
    const vec3 light_direction = unit_vector(light->origin - hit_point);
    Ray shadow_ray(hit_point, light_direction);

    // Check if the shadow ray intersects with any object
    for (const auto &hittable: hittables) {
        // Check if the shadow ray intersects with the object
        // If the distance from the hit point -> Intersection point is greater than hit point
        // -> Light source than we do not consider this point because it is technically behind the light
        if (hittable->intersect(shadow_ray) && t < (light->origin - hit_point).length()) {
            // Check if the intersection point is on the same object
            const vec3 intersection_point = shadow_ray.origin + shadow_ray.direction * t;
            if ((intersection_point - hit_point).length() > 1e-6) return true;
        }
    }
    return false;
}


vec3 trace(Ray &ray, std::vector<std::shared_ptr<Hittable>> &hittables,
           const std::vector<std::shared_ptr<Light>> &lights,
           const int depth, BVHBuilder bvhBuilder) {
    if (depth <= 0) {
        // Maximum recursion depth reached, return background color
        return {0.7, 0.8, 1.0};
    }

    int hit_object = -1;

    bvhBuilder.intersect_bvh(ray, hittables, hit_object);


    //Find the nearest object intersection
//    for (int i = 0; i < hittables.size(); ++i) {
//        if (hittables[i]->intersect(ray)) {
//            hit_object = i;
//        }
//    }

    if (hit_object != -1 && ray.t < std::numeric_limits<double>::max()) {
        const std::shared_ptr<Hittable> &hitObject = hittables[hit_object];

        // Calculate the hit point and normal of the object
        const vec3 hit_point = ray.origin + ray.direction * ray.t;
        const vec3 normal = hitObject->calculate_normal(hit_point, ray);

        color hit_color = color(0, 0, 0);
        color shade_color = color(0, 0, 0);
        for (auto &light: lights) {
            // Check if the material is mirror, and if so, compute reflection recursively
//            if (const auto mirrorMaterial = std::dynamic_pointer_cast<Mirror>(hitObject->material)) {
//                const vec3 reflected = reflect(unit_vector(ray.direction), normal);
//                const Ray reflected_ray(hit_point, reflected);
//                return trace(reflected_ray, hittables, lights, depth - 1);
//            }

            shade_color = hitObject->material->shade(light, hit_point, ray,
                                                     normal, hitObject->object_color);


            if (occluded(light, hit_point, hittables, ray.t)) {
                shade_color *= 0.7;
                hit_color += shade_color;
            } else {
                hit_color += shade_color;
            }

            hit_color += shade_color;
        }

        // Apply tone mapping
        const double r_value = tone_map(hit_color.x());
        const double g_value = tone_map(hit_color.y());
        const double b_value = tone_map(hit_color.z());

        // Gamma correct the final value
        color final_color = {
                gammaCorrect(r_value), gammaCorrect(g_value),
                gammaCorrect(b_value)
        };

        //Clamp final color values to not overshoot the color range
        return clamp(final_color, 0, 1);
    }

    // Blue sky
    return {0.7, 0.8, 1.0};
}

void render(const int files) {
    for (int i = 0; i <= files; i++) {
        // Open file to save image
        std::ofstream myFile;
        std::string fileName = "../image_fixed" + std::to_string(i) + ".ppm";
        myFile.open(fileName);

        // Image
        const int image_width = 800;
        const int image_height = 800;

        // Camera
        point3 camera = point3(0, 0, 10);
        vec3 camera_direction = vec3(0, 0, -1);

        // Lights
        color white = color(1, 1, 1);
        Light light = Light(white, point3(0, 13, 15), vec3(0, 0, 0), 9);

        //Materials
        auto phong_material = std::make_shared<PhongMaterial>(5);
        auto lambertian_material = std::make_shared<LambertianMaterial>();
        auto checkered_material = std::make_shared<
                CheckeredMaterial>(5, color(0.7, 0.7, 0.7), color(0.3, 0.3, 0.3));
        auto mirror_material = std::make_shared<Mirror>(vec3(0.8, 0.3, 0.3));


        //Objects
        std::vector<std::shared_ptr<Hittable>> hittables;

        //hittables.reserve(number_of_triangles);

//        for (int j = 0; j < number_of_triangles; j++) {
//            auto r0 = vec3(std::rand() % 10 + (-5), std::rand() % 10 + (-5), std::rand() % 10 + (-5));
//            auto r1 = vec3(std::rand() % 10 + (-5), std::rand() % 10 + (-5), std::rand() % 10 + (-5));
//            auto r2 = vec3(std::rand() % 10 + (-5), std::rand() % 10 + (-5), std::rand() % 10 + (-5));
//
//            auto vertex0 = (r0 * 9) - vec3(5, 5, 5);
//
//            hittables.emplace_back(
//                    std::make_shared<Triangle>(vertex0, vertex0 + r1, vertex0 + r2, color(1, 1, 0),
//                                               lambertian_material));
//        }

        // Get filename in subfolder
        std::string filename = "obj_files/teapot.obj";
        std::filesystem::path filepath = std::filesystem::current_path().parent_path() / filename;
        std::cout << "Attempting to open file: " << filepath << std::endl;

        // Create the obj. loader from https://github.com/Bly7/OBJ-Loader
        objl::Loader loader;
        bool loaded = loader.LoadFile(filepath);
        if (loaded) {
            std::vector<Triangle> triangles;

            for (auto currentMesh: loader.LoadedMeshes) {
                // Go through every 3 indices and create triangles
                for (int j = 0; j < currentMesh.Indices.size(); j += 3) {
                    // Get vertex indices of the triangle
                    int idx1 = currentMesh.Indices[j];
                    int idx2 = currentMesh.Indices[j + 1];
                    int idx3 = currentMesh.Indices[j + 2];

                    // Create vertices of the triangle
                    vec3 v1(currentMesh.Vertices[idx1].Position.X, currentMesh.Vertices[idx1].Position.Y,
                            currentMesh.Vertices[idx1].Position.Z);
                    vec3 v2(currentMesh.Vertices[idx2].Position.X, currentMesh.Vertices[idx2].Position.Y,
                            currentMesh.Vertices[idx2].Position.Z);
                    vec3 v3(currentMesh.Vertices[idx3].Position.X, currentMesh.Vertices[idx3].Position.Y,
                            currentMesh.Vertices[idx3].Position.Z);

                    // Create the triangle
                    Triangle triangle(v1, v2, v3, color(1, 1, 0), lambertian_material);

                    // first vertex coordinates -> Update min and mx by comparing them -> Min smaller -> Max larger
                    // Calculate center -> sum up all vertex coordinates -> divide the sum of x y and z by total number of vertices

                    // Add triangle to the list
                    triangles.push_back(triangle);
                }
            }

            // Add all triangles to the scene
            hittables.reserve(triangles.size());
            std::cout << "Triangles size: " << triangles.size() << std::endl;
            for (const auto &triangle: triangles) {
                hittables.emplace_back(std::make_shared<Triangle>(triangle));
            }
        }

        // Transform objects with model transform
//        for (const auto &object: hittables) {
//            object->apply_model_transform(vec3(0, 0, 0), vec3(0, 1, 0), vec3(0, 0, 0), 25,
//                                          object->calculate_center());
//        }

        const vec3 rotation_vector = vec3(0, 1, 0);

        // Transform camera -> Move all objects as if we would move the camera
        for (const auto &object: hittables) {
            //object->apply_view_transform(vec3(0, 0, 0), rotation_vector, 20, camera);
        }

        // Make transform to light to simulate camera movement
//        light.apply_view_transform(vec3(0, 0, 0), rotation_vector, 20);
        // Create the light plane which simulates 3x3 light
        const std::vector<std::shared_ptr<Light>> lights = light.createPlaneLight();

        auto startBVH = high_resolution_clock::now();

        BVHBuilder bvhBuilder = *new BVHBuilder(hittables.size());
        bvhBuilder.build_bvh(hittables);

        auto stopBVH = high_resolution_clock::now();
        auto durationBVH = duration_cast<microseconds>(stopBVH - startBVH);

        // Render
        myFile << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        // Go over every pixel in image height and width
        auto startPixel = high_resolution_clock::now();
        for (int j = 0; j < image_height; ++j) {
            std::clog << "\rScanlines remaining: " << image_height - j << ' ' << std::flush;
            for (int k = 0; k < image_width; ++k) {
                color pixel_color;
                // 4x supersampling
                // Cast multiple rays through different sub-pixel locations within the pixel (Make each pixel into 4 parts)
                // Average the colors obtained from these 4 rays to get the final pixel color
//                for (int sy = 0; sy < 2; ++sy) {
//                    for (int sx = 0; sx < 2; ++sx) {
//                        // Get values between 0 and 1 to normalize coords
//                        // -> Does allow mapping of pixels to point on the image regardless of resolution
//                        const double u = (k - image_width / 2 + 0.5 + sx * 0.5) / image_width;
//                        const double v = (image_height / 2 - j - 0.5 - sy * 0.5) / image_height;
//
//                        Ray ray(camera, camera_direction + vec3(u, v, 0));
//                        pixel_color = pixel_color + trace(ray, hittables, lights, 1, rootNodeIdx);
//                    }
//                }

                const double u = static_cast<double>(k - image_width / 2) / image_width;
                const double v = static_cast<double>(image_height / 2 - j) / image_height;
                Ray ray(camera, camera_direction + vec3(u, v, 0));
                pixel_color = trace(ray, hittables, lights, 2, bvhBuilder);


                // Apply color correction and write colors to file
                write_color(myFile, pixel_color);
            }
        }
        auto stopPixel = high_resolution_clock::now();
        auto durationPixel = duration_cast<microseconds>(stopPixel - startPixel);

        std::cout << "\nTime taken by BVH construction: "
                  << durationBVH.count() << " microseconds" << std::endl;

        std::cout << "Time taken by tracing scene: "
                  << durationPixel.count() << " microseconds" << std::endl;

        myFile.close();
    }
}


int main() {
    const int frames = 0;
    auto start = high_resolution_clock::now();
    render(frames);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);

    std::cout << "Time taken total: "
              << duration.count() << " microseconds" << std::endl;
}
