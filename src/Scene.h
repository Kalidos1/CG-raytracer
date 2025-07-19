#ifndef RAYTRACER_SCENE_H
#define RAYTRACER_SCENE_H

#include "./utils/ObjLoader.h"
#include "acceleration/DataStructure.h"
#include "acceleration/BspTrees.h"
#include "acceleration/UniformGrid.h"
#include "acceleration/BvhBuilder.h"
#include "lighting/Light.h"
#include "utils/Camera.h"

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

class Scene {
public:
    struct SceneConfig {
        int imageWidth;
        int imageHeight;
        Camera camera;
        std::string objFilePath;
        std::string texturePath;
    };

    static SceneConfig createDefaultConfig() {
        // 2.8
        const point3 cameraPosition = Vec3(-1400, 350, -100);
        const Vec3 cameraDirection = Vec3(1e-10 + 100, 1e-10 + -0.1, 1e-10 + -1);
        const int imageWidth = 800;
        const int imageHeight = 800;

        Camera camera(cameraPosition,
                      cameraDirection,
                      57.5, // Best FOV (But synced now with camera and light rays)
                      static_cast<double>(imageWidth) / imageHeight,
                      imageWidth,
                      imageHeight, 0.0, 1.0);

        return {
                imageWidth,    // imageWidth
                imageHeight,    // imageHeight
                camera,
                //"assets/obj_files/CornellBox-Original.obj",  // objFilePath
                "assets/obj_files/exterior.obj",  // objFilePath
                ""      // texturePath
        };
    }

    static std::pair<std::vector<std::shared_ptr<Hittable>>, Light> setupObjects(const SceneConfig &config) {
        auto meshData = ObjLoader::loadFromFile(config.objFilePath, config.texturePath, config.camera);
        return {meshData.hittables, Light(meshData.lights)};
    }


    [[nodiscard]] static std::unique_ptr<DataStructure> setupAccelerationStructure(
            const std::vector<std::shared_ptr<Hittable>> &hittables,
            DataStructureFactory::DataStructureType type = DataStructureFactory::DataStructureType::Grid, // Grid | BVH | BSP
            Split splitType = Split::Middle, // Middle | SAH | Linear | LinearSAH
            GridType gridType = GridType::Compact, // Compact | Hashed
            TreeType treeType = TreeType::KD // KD | Octree
    ) {
        auto structure = DataStructureFactory::create(type, hittables.size(), splitType, gridType, treeType);
        structure->build(hittables);
        return structure;
    }

    static void transformObjects(std::vector<std::shared_ptr<Hittable>> &objects,
                                 const Vec3 &translation = Vec3(0, 0, 0),
                                 const Vec3 &rotation = Vec3(0, 1, 0),
                                 float scale = 45.0f) {
        for (const auto &object: objects) {
            object->applyModelTransform(translation, rotation, Vec3(0, 0, 0), scale,
                                        object->calculateCenter());
        }
    }

    static void transformView(std::vector<std::shared_ptr<Hittable>> &objects,
                              const Vec3 &translation = Vec3(0, 0, 0),
                              const Vec3 &rotation = Vec3(0, 0, 1),
                              float scale = 20.0f) {
        for (const auto &object: objects) {
            object->applyViewTransform(translation, rotation, scale,
                                       point3(0, 1, 2.8));
        }
    }
};

#endif //RAYTRACER_SCENE_H
