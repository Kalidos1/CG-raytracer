#ifndef RAYTRACER_SCENE_H
#define RAYTRACER_SCENE_H

#include "./utils/ObjLoader.h"
#include "acceleration/DataStructure.h"
#include "acceleration/BspTrees.h"
#include "acceleration/UniformGrid.h"
#include "acceleration/BvhBuilder.h"

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
        point3 camera;
        Vec3 cameraDirection;
        std::string objFilePath;
        std::string texturePath;
    };

    static SceneConfig createDefaultConfig() {
        return {
                800,    // imageWidth
                800,    // imageHeight
                point3(0, 1, 2.8),  // camera
                Vec3(1e-10, 1e-10, 1e-10 + -1),  // cameraDirection
                "assets/obj_files/CornellBox-Original.obj",  // objFilePath
                ""      // texturePath
        };
    }

    [[nodiscard]] static std::vector<std::shared_ptr<Light>> setupLights() {
        Color white(1, 1, 1);
        Light light(white, point3(0, 1.95, 0), Vec3(0, 0, 0), 1.0);
        return light.createPlaneLight();
    }

    [[nodiscard]] static std::vector<std::shared_ptr<Hittable>> setupObjects(const SceneConfig &config) {
        auto meshData = ObjLoader::loadFromFile(config.objFilePath, config.texturePath);
        return meshData.hittables;
    }

    [[nodiscard]] static std::unique_ptr<DataStructure> setupAccelerationStructure(
            const std::vector<std::shared_ptr<Hittable>> &hittables,
            DataStructureFactory::DataStructureType type = DataStructureFactory::DataStructureType::BVH, // Grid | BVH | BSP
            Split splitType = Split::SAH, // Middle | SAH | Linear | LinearSAH
            GridType gridType = GridType::Compact, // Compact | Hashed
            TreeType treeType = TreeType::KD // KD | Octree
    ) {
        auto structure = DataStructureFactory::create(type, hittables.size(), splitType, gridType, treeType);
        structure->build(hittables);
        return structure;
    }

//    static void transformObjects(std::vector<std::shared_ptr<Hittable>> &objects,
//                                 const Vec3 &translation = Vec3(0, 0, 0),
//                                 const Vec3 &rotation = Vec3(0, 1, 0),
//                                 float scale = 1.0f)        { for (const auto& object : objects) {
//            object->applyModelTransform(translation, rotation, Vec3(0, 0, 0), scale,
//                                        object->calculateCenter());
//        }
};

#endif //RAYTRACER_SCENE_H
