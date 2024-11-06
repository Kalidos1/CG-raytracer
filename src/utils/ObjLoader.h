#ifndef RAYTRACER_OBJLOADER_H
#define RAYTRACER_OBJLOADER_H

#define STB_IMAGE_IMPLEMENTATION

#include "utils/stb_image.h"
#include "geometry/Hittable.h"
#include "obj_loader.h"
#include "geometry/Triangle.h"

class ObjLoader {
public:
    struct LoadedMeshData {
        std::vector<std::shared_ptr<Hittable>> hittables;
        unsigned char *textureData;
        int textureWidth;
        int textureHeight;
        int textureChannels;
    };

    static LoadedMeshData loadFromFile(const std::string &objPath, const std::string &texturePath = "") {
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

        // Extract the mesh data
        std::vector<std::shared_ptr<Hittable>> hittables;
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

                auto phongMaterial = std::make_shared<PhongMaterial>(specularComponent, textureData,
                                                                     textureWidth, textureHeight, textureChannels,
                                                                     ambientColor, diffuseColor, specularColor);

                hittables.emplace_back(std::make_shared<Triangle>(v1, v2, v3, textureV0, textureV1, textureV2,
                                                                  diffuseColor, phongMaterial));
            }
        }

        return {hittables, textureData, textureWidth, textureHeight, textureChannels};
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
