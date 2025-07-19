#ifndef RAYTRACER_OBJLOADER_H
#define RAYTRACER_OBJLOADER_H

#define STB_IMAGE_IMPLEMENTATION

#include "utils/stb_image.h"
#include "geometry/Hittable.h"
#include "obj_loader.h"
#include "geometry/Triangle.h"
#include "geometry/Sphere.h"
#include "Camera.h"

class ObjLoader {
public:
    struct LoadedMeshData {
        std::vector<std::shared_ptr<Hittable>> hittables;
        std::vector<std::shared_ptr<Hittable>> lights;
        unsigned char *textureData;
        int textureWidth;
        int textureHeight;
        int textureChannels;
    };

    static LoadedMeshData
    loadFromFile(const std::string &objPath, const std::string &texturePath = "", const Camera &camera = Camera()) {
        // Load the OBJ file
        objl::Loader loader;
        std::filesystem::path filepath = std::filesystem::current_path().parent_path() / objPath;
        if (!loader.LoadFile(filepath.string())) {
            throw std::runtime_error("Failed to load OBJ file: " + filepath.string());
        }

        // Load the texture
        int textureWidth, textureHeight, textureChannels;
        unsigned char *textureData = nullptr;
        if (!texturePath.empty()) {
            filepath = std::filesystem::current_path().parent_path() / texturePath;
            textureData = loadTexture(filepath.string(), textureWidth, textureHeight, textureChannels);
        }

        std::vector<std::shared_ptr<Hittable>> hittables, lights;

        // Extract the mesh data
        for (const auto &currentMesh: loader.LoadedMeshes) {
            for (int j = 0; j < currentMesh.Indices.size(); j += 3) {
                int idx1 = currentMesh.Indices[j];
                int idx2 = currentMesh.Indices[j + 1];
                int idx3 = currentMesh.Indices[j + 2];

                Vec3 v1(currentMesh.Vertices[idx1].Position.X, currentMesh.Vertices[idx1].Position.Y,
                        currentMesh.Vertices[idx1].Position.Z);
                Vec3 v2(currentMesh.Vertices[idx2].Position.X, currentMesh.Vertices[idx2].Position.Y,
                        currentMesh.Vertices[idx2].Position.Z);
                Vec3 v3(currentMesh.Vertices[idx3].Position.X, currentMesh.Vertices[idx3].Position.Y,
                        currentMesh.Vertices[idx3].Position.Z);

                double textureV0[2] = {currentMesh.Vertices[idx1].TextureCoordinate.X,
                                       currentMesh.Vertices[idx1].TextureCoordinate.Y};
                double textureV1[2] = {currentMesh.Vertices[idx2].TextureCoordinate.X,
                                       currentMesh.Vertices[idx2].TextureCoordinate.Y};
                double textureV2[2] = {currentMesh.Vertices[idx3].TextureCoordinate.X,
                                       currentMesh.Vertices[idx3].TextureCoordinate.Y};

                Color ambientColor = {currentMesh.MeshMaterial.Ka.X, currentMesh.MeshMaterial.Ka.Y,
                                      currentMesh.MeshMaterial.Ka.Z};
                Color diffuseColor = {currentMesh.MeshMaterial.Kd.X, currentMesh.MeshMaterial.Kd.Y,
                                      currentMesh.MeshMaterial.Kd.Z};
                Color specularColor = {currentMesh.MeshMaterial.Ks.X, currentMesh.MeshMaterial.Ks.Y,
                                       currentMesh.MeshMaterial.Ks.Z};
                float specularComponent = currentMesh.MeshMaterial.Ns;
                int illumination = currentMesh.MeshMaterial.illum;

                auto isLight = currentMesh.MeshMaterial.name;
                if (isLight == "light" || isLight == "window_light_emissive" || isLight == "cone_light_emissive" ||
                    isLight == "Light" || isLight == "MASTER_Light_Bulb" ||
                    isLight == "Emmision" || isLight == "Streetlight_Glass" ||
                    isLight == "Paris_StringLights_01_White_Color" ||
                    isLight == "Spotlight_Main" ||
                    isLight == "Spotlight_Emissive" ||
                    isLight == "Spotlight_Glass" ||
                    isLight == "Paris_StringLights_01_Green_Color" ||
                    isLight == "Paris_StringLights_01_Red_Color" ||
                    isLight == "Paris_StringLights_01_Blue_Color" ||
                    isLight == "Paris_StringLights_01_Pink_Color" ||
                    isLight == "Paris_StringLights_01_Orange_Color"
                    || isLight == "Lantern"
                        ) {
                    Color lightColor = {currentMesh.MeshMaterial.Ka.X, currentMesh.MeshMaterial.Ka.Y,
                                        currentMesh.MeshMaterial.Ka.Z};
                    Color testColor = {17, 17, 17};
                    auto emissiveMaterial = std::make_shared<EmissiveMaterial>(lightColor, illumination);
                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      lightColor, emissiveMaterial));
                    lights.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                   lightColor, emissiveMaterial));
                } else if (isLight == "mirror" || isLight == "breakfast_room:Artwork" ||
                           isLight == "Netflix" || isLight == "Mirror" ||
                           isLight == "Cermin" || isLight == "Picture" || isLight == "Picture2" ||
                           isLight == "Picture3" || isLight == "Picture4") {
                    auto mirrorMaterial = std::make_shared<MirrorMaterial>();
                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      ambientColor, mirrorMaterial));
                } else if (isLight == "glass" || isLight == "breakfast_room:Ceramic" ||
                           isLight == "Kaca") {
                    const auto glass = std::make_shared<TransparentMaterial>(ambientColor, 1.5);
                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      ambientColor, glass));
                } else if (isLight == "water" | isLight == "grey_and_white_room:Transluscent") {
                    const auto water = std::make_shared<TransparentMaterial>(ambientColor, 1.3);
                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      ambientColor, water));
                } else if (isLight == "diamond") {
                    const auto diamond = std::make_shared<TransparentMaterial>(ambientColor, 2.5);
                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      ambientColor, diamond));
                } else if (specularComponent > 0) {
                    double roughness = sqrt(2.0 / (specularComponent + 2.0));
                    auto specularMaterial = std::make_shared<SpecularMaterial>(diffuseColor, roughness);
                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      diffuseColor, specularMaterial));
                } else if (isLight == "specular" || isLight == "breakfast_room:Black_Rubber" ||
                           isLight == "breakfast_room:Ceramic" || isLight == "breakfast_room:Ceramic_001" ||
                           isLight == "breakfast_room:Chrome"
                           || isLight == "breakfast_room:Floor_Tiles"
                           || isLight == "breakfast_room:Gold_Paint"
                           || isLight == "breakfast_room:Material_002"
                           || isLight == "breakfast_room:Material_005"
                           || isLight == "breakfast_room:Paint___Black_Satin"
                           || isLight == "breakfast_room:Paint___White_Gloss"
                           || isLight == "breakfast_room:Paint___White_Matt"
                           || isLight == "breakfast_room:White_Marble"
                           || isLight == "breakfast_room:White_Plastic" ||
                           isLight == "grey_and_white_room:BrushedStainlessSteel"
                           || isLight == "grey_and_white_room:FireplaceGlass"
                           || isLight == "Tree_Branch"
                           || isLight == "Tree"
                           || isLight == "TV"
                           || isLight == "Sofa_Kecil"
                           || isLight == "Sofa"
                           || isLight == "Gelas"
                           || isLight == "Cusion"
                           || isLight == "Book2"
                           || isLight == "Book"
                           || isLight == "Aluminium") {
                    double roughness = sqrt(2.0 / (specularComponent + 2.0));
                    auto specularMaterial = std::make_shared<SpecularMaterial>(diffuseColor, roughness);
                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      diffuseColor, specularMaterial));
                } else {
                    auto diffuseMaterial = std::make_shared<DiffuseMaterial>(ambientColor);

                    hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                      diffuseColor, diffuseMaterial));
                }

            }
        }

        /*
         * Transparent Materials
         * 1.33 = Water
         * 1.5 = Glass
         * 2.4 = Diamond
         * ...
         * */
        const auto water = std::make_shared<TransparentMaterial>(Color(1, 1, 1), 1.3);
        const auto glass = std::make_shared<TransparentMaterial>(Color(1, 1, 1), 1.5);
        const auto diamond = std::make_shared<TransparentMaterial>(Color(1, 1, 1), 2.5);
        const auto mirror = std::make_shared<MirrorMaterial>();


        // Sphere 1
//        const Color sphereColorOne = {0.2, 0.3, 0.8}; // Blue
//        const auto sphereMaterialOne = std::make_shared<DiffuseMaterial>(sphereColorOne);
//        const auto sphereMaterialOneSpecular = std::make_shared<SpecularMaterial>(sphereColorOne, 0.25);
//        const point3 sphereCenterOne = point3{-0.65, 1, 0};
//
//        // Sphere 2
//        const Color sphereColorTwo = {0.4, 0.1, 0.6}; // Purple
//        const auto sphereMaterialTwo = std::make_shared<SpecularMaterial>(sphereColorTwo);
//        const point3 sphereCenterTwo = point3{0, 1, 0};
//
//        // Sphere 3
//        const Color sphereColorThree = {0.54, 0.45, 0.091}; // Yellow
//        const auto sphereMaterialThree = std::make_shared<MixedMaterial>(sphereColorThree);
//        const auto sphereMaterialThreeSpecular = std::make_shared<SpecularMaterial>(sphereColorThree, 0.75);
//        const point3 sphereCenterThree = point3{0.65, 1, 0};
//
//
//        // 3 Spheres
//        hittables.emplace_back(std::make_shared<Sphere>(sphereCenterOne,
//                                                        0.25,
//                                                        sphereColorOne,
//                                                        sphereMaterialOne));
//        hittables.emplace_back(std::make_shared<Sphere>(sphereCenterTwo,
//                                                        0.25,
//                                                        sphereColorTwo,
//                                                        sphereMaterialTwo));
//        hittables.emplace_back(std::make_shared<Sphere>(sphereCenterThree,
//                                                        0.25,
//                                                        sphereColorThree,
//                                                        sphereMaterialThree));

        // Fix camera in BDPT (For some reason it is as camera and it just returns a Black image)
        // I think because it has the COlor 0 so it retuns a 0 aswell, not sure though
        if (camera.getLensRadius() > 0.0) {
            hittables.push_back(camera.getLens());
        }

        return {hittables, lights, textureData, textureWidth, textureHeight, textureChannels};
    }

private:
    static unsigned char *loadTexture(const std::string &filepath, int &width, int &height, int &channels) {
        unsigned char *data = stbi_load(filepath.c_str(), &width, &height, &channels, STBI_rgb);
        if (!data) {
            std::cerr << "Failed to load texture image at path: " << filepath << std::endl;
            throw std::runtime_error("Failed to load texture image");
        }
        channels = 3; // Set channels to 3 to avoid calculation discrepancies
        return data;
    }
};

#endif //RAYTRACER_OBJLOADER_H
