#define USE_MATH_DEFINES

#include "color.h"
#include "vec3.h"
#include "ray.h"
#include "triangle.h"
#include "bounding_box.h"
#include "light.h"
#include "obj_loader.h"
#include "bvh_builder.h"
#include "uniform_grid.h"
#include "bsp_trees.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <limits>

#define STB_IMAGE_IMPLEMENTATION

#include "stb_image.h"


using namespace std::chrono;

class DataStructureFactory {
public:
    enum class DataStructureType {
        BVH,
        Grid,
        BSP
    };

    static std::unique_ptr<DataStructure>
    create(DataStructureType type, int size, Split split, GridType gridType, TreeType treeType) {
        switch (type) {
            case DataStructureType::BVH:
                return std::make_unique<BVHBuilder>(size, split);
            case DataStructureType::Grid:
                return std::make_unique<UniformGrid>(size, gridType);
            case DataStructureType::BSP:
                return std::make_unique<BSPTree>(size, treeType);
            default:
                throw std::runtime_error("Unknown data structure type");
        }
    }
};

// Tone mapping after https://bruop.github.io/tonemapping/
double toneMap(const double hit) {
    const double lWhite = 4.0;
    return hit * (1 + hit / (lWhite * lWhite)) / (1.0 + hit);
}

double gammaCorrect(const double colorValue) {
    if (colorValue <= 0.0) {
        return 0.0;
    }
    // Standard gamma correction value
    return std::pow(colorValue, 2.2);
}

bool occluded(const std::shared_ptr<Light> &light, const vec3 &hitPoint,
              std::vector<std::shared_ptr<Hittable>> &hittables,
              double t, std::unique_ptr<DataStructure> &dataStructure) {
    //Calculate light direction and shadow ray (From hitpoint to light source)
    const vec3 lightDirection = unitVector(light->origin - hitPoint);
    Ray shadowRay(hitPoint, lightDirection);

    int hitObjectTemp = -1;

    // TODO: Can use other intersect method to only get the first hit -> Do not need to find the closest one
    dataStructure->intersect(shadowRay, hittables, hitObjectTemp);

    if (hitObjectTemp != -1 && shadowRay.t < std::numeric_limits<double>::max()) {
        const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectTemp];

        // Check if the shadow ray intersects with the object
        // If the distance from the hit point -> Intersection point is greater than hit point
        // -> Light source than we do not consider this point because it is technically behind the light
        if (hitObject->intersect(shadowRay) && t < (light->origin - hitPoint).length()) {
            // Check if the intersection point is on the same object
            const vec3 intersectionPoint = shadowRay.origin + shadowRay.direction * t;
            if ((intersectionPoint - hitPoint).length() > 1e-6) return true;
        }
    }
    return false;
}


vec3 trace(Ray &ray, std::vector<std::shared_ptr<Hittable>> &hittables,
           const std::vector<std::shared_ptr<Light>> &lights,
           const int depth, std::unique_ptr<DataStructure> &dataStructure) {
    if (depth <= 0) {
        return {0.7, 0.8, 1.0};
    }

    int hitObjectTemp = -1;

    dataStructure->intersect(ray, hittables, hitObjectTemp);

    //Find the nearest object intersection
//    for (int i = 0; i < hittables.size(); ++i) {
//        if (hittables[i]->intersect(ray)) {
//            hitObjectTemp = i;
//        }
//    }

    if (hitObjectTemp != -1 && ray.t < std::numeric_limits<double>::max()) {
        const std::shared_ptr<Hittable> &hitObject = hittables[hitObjectTemp];

        // Calculate the hit point and normal of the object
        const vec3 hitPoint = (ray.origin + ray.direction * ray.t);
        const vec3 barycentric = hitObject->calculateBarycentricCoordinates(hitPoint);
        const vec3 normal = hitObject->calculateNormal(hitPoint, ray);

        color hitColor = color(0, 0, 0);
        color shadeColor = color(0, 0, 0);
        for (auto &light: lights) {
            // Check if the material is mirror, and if so, compute reflection recursively
//            if (const auto mirrorMaterial = std::dynamic_pointer_cast<Mirror>(hitObject->material)) {
//                const vec3 reflected = reflect(unitVector(ray.direction), normal);
//                const Ray reflected_ray(hitPoint, reflected);
//                return trace(reflected_ray, hittables, lights, depth - 1);
//            }

            // Interpolate texture coordinates
            double interpolatedUV[2] = {hitObject->interpolateCoordinate1(barycentric),
                                        hitObject->interpolateCoordinate2(barycentric)};

            shadeColor = hitObject->material->shade(light, hitPoint, ray,
                                                    normal, hitObject->objectColor, interpolatedUV);


            if (occluded(light, hitPoint, hittables, ray.t, dataStructure)) {
                shadeColor *= 0.7;
                hitColor += shadeColor;
            } else {
                hitColor += shadeColor;
            }
        }

        // Apply tone mapping
        const double rValue = toneMap(hitColor.x());
        const double gValue = toneMap(hitColor.y());
        const double bValue = toneMap(hitColor.z());

        // Gamma correct the final value
        color finalColor = {
                gammaCorrect(rValue), gammaCorrect(gValue),
                gammaCorrect(bValue)
        };

        //Clamp final color values to not overshoot the color range
        return clamp(finalColor, 0, 1);
    }

    // Blue sky
    return {0.7, 0.8, 1.0};
}

// Function to load a texture
unsigned char *load_texture(const std::string &filepath, int &width, int &height, int &channels) {
    unsigned char *data = stbi_load(filepath.c_str(), &width, &height, &channels, STBI_rgb);
    if (!data) {
        std::cerr << "Failed to load texture image at path: " << filepath << std::endl;
        throw std::runtime_error("Failed to load texture image");
    }
    channels = 3; // Set channels to 3 to avoid calculation discrepancies
    return data;
}

/*
 * TODO:
 * 4. BERICHT ERSTMAL
 * 5. Check all things and make some overall improvements to code variables etc. -> Sort code very important!!!!
 * 6. Check why Linear and BVHs are all quite slower than Grid
 * 7. Make some images to show how different volumes are constructed
 * 8. Obj loader fix
 * 9. Maybe noch OCTREE STUFF
 */

struct Tile {
    int startX, startY, endX, endY;
};

void
renderTile(const Tile &tile, std::vector<color> &pixelColors, point3 camera, vec3 cameraDirection, int imageWidth,
           int imageHeight, std::vector<std::shared_ptr<Hittable>> &hittables,
           const std::vector<std::shared_ptr<Light>> &lights,
           std::unique_ptr<DataStructure> &dataStructure, bool supersampling) {
    for (int j = tile.startY; j < tile.endY; ++j) {
        std::clog << "\rScanlines remaining: " << imageHeight - j << ' ' << std::flush;
        for (int k = tile.startX; k < tile.endX; ++k) {
            if (supersampling) {
                color pixelColor;
                for (int sy = 0; sy < 2; ++sy) {
                    for (int sx = 0; sx < 2; ++sx) {
                        // Get values between 0 and 1 to normalize coords
                        // -> Does allow mapping of pixels to point on the image regardless of resolution
                        const double u = (k - imageWidth / 2 + 0.5 + sx * 0.5) / imageWidth;
                        const double v = (imageHeight / 2 - j - 0.5 - sy * 0.5) / imageHeight;

                        Ray ray(camera, cameraDirection + vec3(u, v, 0));
                        pixelColor = pixelColor + trace(ray, hittables, lights, 2, dataStructure);
                    }
                }
                pixelColors[j * imageWidth + k] = pixelColor * 0.25;
            } else {
                const double u = static_cast<double>(k - imageWidth / 2) / imageWidth;
                const double v = static_cast<double>(imageHeight / 2 - j) / imageHeight;
                Ray ray(camera, cameraDirection + vec3(u, v, 0));
                color pixelColor = trace(ray, hittables, lights, 2, dataStructure);

                // Apply color correction and write colors to vector
                pixelColors[j * imageWidth + k] = pixelColor;
            }
        }
    }
}

void render(const int files) {
    for (int i = 0; i <= files; i++) {
        // Open file to save image
        std::ofstream myFile;
        std::string fileName = "../image_fixed" + std::to_string(i) + ".ppm";
        myFile.open(fileName);

        // Image
        const int imageWidth = 800;
        const int imageHeight = 800;

        // Camera
        point3 camera = point3(0, 0, 15);
        // Avoid floating point arithmetics
        vec3 cameraDirection = vec3(1e-10, 1e-10, 1e-10 + -1);

        // Lights
        color white = color(1, 1, 1);
        Light light = Light(white, point3(25, 20, -10), vec3(0, 0, 0), 3);

        //Materials
        //auto phongMaterial = std::make_shared<PhongMaterial>(5);
        auto lambertianMaterial = std::make_shared<LambertianMaterial>();
        auto checkeredMaterial = std::make_shared<
                CheckeredMaterial>(5, color(0.7, 0.7, 0.7), color(0.3, 0.3, 0.3));
        auto mirrorMaterial = std::make_shared<Mirror>(vec3(0.8, 0.3, 0.3));


        //Objects
        std::vector<std::shared_ptr<Hittable>> hittables;
        // MIGHT BE BETTER TO JUST USE TRIANGLES

        //hittables.reserve(number_of_triangles);

        //Load texture
//        std::string filename_image = "obj_files/fabric.png";
        //      std::filesystem::path filepath_image = std::filesystem::current_path().parent_path() / filename_image;
        int textureWidth, textureHeight, textureChannels;
        //   unsigned char *textureData = load_texture(filepath_image.string(), textureWidth, textureHeight,
        //                                       textureChannels);

        unsigned char *textureData = nullptr;

        // Get filename in subfolder
        std::string filename = "obj_files/teapot.obj";
        std::filesystem::path filepath = std::filesystem::current_path().parent_path() / filename;
        std::cout << "Attempting to open file: " << filepath << std::endl;


        // Create the obj. loader from https://github.com/Bly7/OBJ-Loader
        auto startLoadingObj = high_resolution_clock::now();
        objl::Loader loader;
        bool loaded = loader.LoadFile(filepath);
        // Go over every pixel in image height and width
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

                    const double textureV0[2] = {currentMesh.Vertices[idx1].TextureCoordinate.X,
                                                 currentMesh.Vertices[idx1].TextureCoordinate.Y};
                    const double textureV1[2] = {currentMesh.Vertices[idx2].TextureCoordinate.X,
                                                 currentMesh.Vertices[idx2].TextureCoordinate.Y};
                    const double textureV2[2] = {currentMesh.Vertices[idx3].TextureCoordinate.X,
                                                 currentMesh.Vertices[idx3].TextureCoordinate.Y};

                    const color ambientColor = {currentMesh.MeshMaterial.Ka.X, currentMesh.MeshMaterial.Ka.Y,
                                                currentMesh.MeshMaterial.Ka.Z};
                    const color diffuseColor = {currentMesh.MeshMaterial.Kd.X, currentMesh.MeshMaterial.Kd.Y,
                                                currentMesh.MeshMaterial.Kd.Z};
                    const color specularColor = {currentMesh.MeshMaterial.Ks.X, currentMesh.MeshMaterial.Ks.Y,
                                                 currentMesh.MeshMaterial.Ks.Z};

                    auto phongMaterialTest = std::make_shared<PhongMaterial>(32, textureData, textureWidth,
                                                                             textureHeight, textureChannels,
                                                                             ambientColor, diffuseColor,
                                                                             specularColor);

                    // Create the triangle
                    Triangle triangle(v1, v2, v3, textureV0, textureV1, textureV2, color(0.7, 0.2, 0.2),
                                      phongMaterialTest);

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
        auto stopLoadingObj = high_resolution_clock::now();
        auto durationLoadingObj = duration_cast<microseconds>(stopLoadingObj - startLoadingObj);

        // Transform objects with model transform
//        for (const auto &object: hittables) {
//            object->applyModelTransform(vec3(0, 0, 0), vec3(0, 1, 0), vec3(0, 0, 0), 25,
//                                          object->calculateCenter());
//        }

        const vec3 rotation_vector = vec3(0, 1, 0);

        // Transform camera -> Move all objects as if we would move the camera
        for (const auto &object: hittables) {
            //object->applyViewTransform(vec3(0, 1, 0), rotation_vector, 180, camera);
        }

        // Make transform to light to simulate camera movement
        //light.applyViewTransform(vec3(0, 0, 0), rotation_vector, 45);
        // Create the light plane which simulates 3x3 light
        const std::vector<std::shared_ptr<Light>> lights = light.createPlaneLight();

        auto startBVH = high_resolution_clock::now();

        // Change variables according to wanted data structure
        auto dataStructureType = DataStructureFactory::DataStructureType::BSP; // Grid | BVH | BSP
        GridType gridType = GridType::Compact; // Compact | Hashed
        Split bvhSplitType = Split::Middle; // Middle | SAH | Linear | LinearSAH
        TreeType treeType = TreeType::KD; // KD | Octree

        // Create the acceleration structure
        auto dataStructure = DataStructureFactory::create(dataStructureType, hittables.size(), bvhSplitType, gridType,
                                                          treeType);
        dataStructure->build(hittables);

        auto stopBVH = high_resolution_clock::now();
        auto durationBVH = duration_cast<microseconds>(stopBVH - startBVH);

        // Render
        myFile << "P3\n" << imageWidth << ' ' << imageHeight << "\n255\n";

        // Create a vector to store all pixel colors
        std::vector<color> pixelColors(imageWidth * imageHeight);

        // Go over every pixel in image height and width
        auto startPixel = high_resolution_clock::now();
        // Create a queue of tiles
        std::queue<Tile> tileQueue;

        const int tileSize = 32;
        for (int y = 0; y < imageHeight; y += tileSize) {
            for (int x = 0; x < imageWidth; x += tileSize) {
                tileQueue.push({x, y,
                                std::min(x + tileSize, imageWidth),
                                std::min(y + tileSize, imageHeight)});
            }
        }

        // Create a mutex to protect the queue
        std::mutex queueMutex;

        // Function for worker threads
        auto worker = [&]() {
            while (true) {
                Tile tile;
                {
                    std::lock_guard<std::mutex> lock(queueMutex);
                    if (tileQueue.empty()) break;
                    tile = tileQueue.front();
                    tileQueue.pop();
                }
                renderTile(tile, pixelColors, camera, cameraDirection, imageWidth, imageHeight, hittables, lights,
                           dataStructure, true);
            }
        };

        // Determine number of threads
        unsigned int threadCount = std::thread::hardware_concurrency();

        // Create and start threads
        std::vector<std::thread> threads;
        for (unsigned int p = 0; p < threadCount; ++p) {
            threads.emplace_back(worker);
        }

        // Wait for all threads to finish
        for (auto &thread: threads) {
            thread.join();
        }

        auto stopPixel = high_resolution_clock::now();
        auto durationPixel = duration_cast<microseconds>(stopPixel - startPixel);

        auto startWritingToFile = high_resolution_clock::now();
        // Write all pixel colors to file at once
        for (const auto &pixel_color: pixelColors) {
            writeColor(myFile, pixel_color);
        }
        auto stopWritingToFile = high_resolution_clock::now();
        auto durationWritingToFile = duration_cast<microseconds>(stopWritingToFile - startWritingToFile);

        std::cout << "Time taken loading obj: "
                  << durationLoadingObj.count() / 1000 << " milliseconds" << std::endl;

        std::cout << "\nTime taken by BVH construction: "
                  << durationBVH.count() / 1000 << " milliseconds" << std::endl;

        std::cout << "Time taken by tracing scene: "
                  << durationPixel.count() / 1000 << " milliseconds" << std::endl;

        std::cout << "Time taken by writing buffer to file: "
                  << durationWritingToFile.count() / 1000 << " milliseconds" << std::endl;


        stbi_image_free(textureData);
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
              << duration.count() / 1000 << " milliseconds" << std::endl;
}
