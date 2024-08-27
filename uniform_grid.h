#ifndef RAYTRACER_UNIFORM_GRID_H
#define RAYTRACER_UNIFORM_GRID_H

#include <vector>
#include "data_structure.h"

enum class GridType {
    Compact, Hashed
};

class UniformGrid : public DataStructure {
public:
    explicit UniformGrid(int numTriangles, GridType type = GridType::Compact) : gridDimensions(vec3(0, 0, 0)),
                                                                                cellSize(vec3(0, 0, 0)), gridCells(),
                                                                                objectList(numTriangles),
                                                                                sceneBounds(), gridType(type),
                                                                                totalCells(0) {}

    void build(const std::vector<std::shared_ptr<Hittable>> &hittables) override {
        createGrid(hittables);
        if (gridType == GridType::Compact) {
            buildCompactGrid(hittables);
        } else {
            buildHashedGrid(hittables);
        }
    }

    void intersect(Ray &ray, const std::vector<std::shared_ptr<Hittable>> &hittables, int &hitObject) override {
        if (sceneBounds.intersect(ray) >= std::numeric_limits<double>::infinity()) return;

        // Initialize ray parameters
        vec3 rayOrigin = (ray.origin - sceneBounds.min) / cellSize;
        vec3 rayDir = ray.direction / cellSize;
        vec3 invDir = vec3(1.0, 1.0, 1.0) / rayDir;

        // Calculate initial cell
        int x = std::clamp(static_cast<int>(rayOrigin.x()), 0, static_cast<int>(gridDimensions.x()) - 1);
        int y = std::clamp(static_cast<int>(rayOrigin.y()), 0, static_cast<int>(gridDimensions.y()) - 1);
        int z = std::clamp(static_cast<int>(rayOrigin.z()), 0, static_cast<int>(gridDimensions.z()) - 1);

        // Calculate cell stepping direction
        int stepX = (rayDir.x() >= 0) ? 1 : -1;
        int stepY = (rayDir.y() >= 0) ? 1 : -1;
        int stepZ = (rayDir.z() >= 0) ? 1 : -1;

        // Calculate t-max values
        double tMaxX = (x + std::max(0, stepX) - rayOrigin.x()) * invDir.x();
        double tMaxY = (y + std::max(0, stepY) - rayOrigin.y()) * invDir.y();
        double tMaxZ = (z + std::max(0, stepZ) - rayOrigin.z()) * invDir.z();

        // Calculate t-delta values
        double tDeltaX = std::abs(invDir.x());
        double tDeltaY = std::abs(invDir.y());
        double tDeltaZ = std::abs(invDir.z());

        while (x >= 0 && x < gridDimensions.x() &&
               y >= 0 && y < gridDimensions.y() &&
               z >= 0 && z < gridDimensions.z()) {


            if (gridType == GridType::Compact) {
                // Use compact grid
                int cellIndex = static_cast<int>(x + y * gridDimensions.x() +
                                                 z * gridDimensions.x() * gridDimensions.y());

                for (int j = gridCells[cellIndex]; j < gridCells[cellIndex + 1]; ++j) {
                    int objIndex = objectList[j];
                    double nearestHit = ray.t;
                    if (hittables[objIndex]->intersect(ray)) {
                        if (ray.t < nearestHit) nearestHit = ray.t;
                        else return; // Early termination if we already have a nearer intersection
                        hitObject = objIndex;
                    }
                }
            } else {
                // Use hashed grid
                vec3 cellCenter = sceneBounds.min + vec3(x + 0.5, y + 0.5, z + 0.5) * cellSize;
                int hashIndex = hashFunction(cellCenter);
                const std::vector<int> &objectsInCell = hashTable[hashIndex];

                for (int objIndex: objectsInCell) {
                    double nearestHit = ray.t;
                    if (hittables[objIndex]->intersect(ray)) {
                        if (ray.t < nearestHit) nearestHit = ray.t;
                        else return; // Early termination if we already have a nearer intersection
                        hitObject = objIndex;
                    }
                }
            }

            // Move to next cell
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                x += stepX;
                tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                y += stepY;
                tMaxY += tDeltaY;
            } else {
                z += stepZ;
                tMaxZ += tDeltaZ;
            }
        }
    }

private:
    vec3 gridDimensions, cellSize;
    int totalCells;
    GridType gridType;
    BoundingBox sceneBounds;

    // Compact Grid
    std::vector<int> objectList;
    std::vector<int> gridCells;

    // Hashed Grid
    std::vector<std::vector<int>> hashTable;


    void createGrid(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        // Calculate global bounding box
        sceneBounds = calculateSceneBounds(hittables);

        // Estimate the volume of the bounding box
        const double boundingBoxVolume = sceneBounds.volume();
        const int gridDensity = 4; // Can be adjusted, might make a nice comparison
        const vec3 boundingBoxSize = sceneBounds.max - sceneBounds.min;
        const double cellsPerUnit = std::cbrt((gridDensity * hittables.size()) / boundingBoxVolume);

        gridDimensions[0] = std::max(1, static_cast<int>(std::ceil(boundingBoxSize.x() * cellsPerUnit)));
        gridDimensions[1] = std::max(1, static_cast<int>(std::ceil(boundingBoxSize.y() * cellsPerUnit)));
        gridDimensions[2] = std::max(1, static_cast<int>(std::ceil(boundingBoxSize.z() * cellsPerUnit)));

        // Create empty grid with everything 0, 0
        cellSize = boundingBoxSize / gridDimensions;
        totalCells = gridDimensions.x() * gridDimensions.y() * gridDimensions.z();
        if (gridType == GridType::Compact) {
            gridCells.resize(totalCells + 1, 0);
        } else {
            hashTable.resize(nextPrime(totalCells));
        }
    }

    void buildCompactGrid(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        // Count objects per cell
        for (const auto &hittable: hittables) {
            std::vector<int> overlappingCells = getOverlappingCells(hittable->getBoundingBox());
            for (int cellIndex: overlappingCells) {
                gridCells[cellIndex]++;
            }
        }

        // Compute offsets
        for (int i = 1; i <= totalCells; ++i) {
            gridCells[i] += gridCells[i - 1];
        }

        // Allocate object list
        objectList.resize(gridCells[totalCells]);

        // Fill object list with corresponding indexes
        for (int i = hittables.size() - 1; i >= 0; --i) {
            std::vector<int> overlappingCells = getOverlappingCells(hittables[i]->getBoundingBox());
            for (int cellIndex: overlappingCells) {
                objectList[--gridCells[cellIndex]] = i;
            }
        }
    }

    std::vector<int> getOverlappingCells(const BoundingBox &box) {
        // Minimum cell and maximum cell that we intersect with our box
        vec3 minCell = (box.min - sceneBounds.min) / cellSize;
        vec3 maxCell = (box.max - sceneBounds.min) / cellSize;

        // Create variables -> Ensure that we are inside the bounds of the cell
        // Go in every direction and count every cell that we go through
        std::vector<int> cells;
        // x direction
        int minRangeX = std::max(0, static_cast<int>(minCell.x()));
        int maxRangeX = std::min(static_cast<int>(gridDimensions.x()) - 1, static_cast<int>(maxCell.x()));
        // y direction
        int minRangeY = std::max(0, static_cast<int>(minCell.y()));
        int maxRangeY = std::min(static_cast<int>(gridDimensions.y()) - 1, static_cast<int>(maxCell.y()));
        // z direction
        int minRangeZ = std::max(0, static_cast<int>(minCell.z()));
        int maxRangeZ = std::min(static_cast<int>(gridDimensions.z()) - 1, static_cast<int>(maxCell.z()));

        for (int x = minRangeX; x <= maxRangeX; x++) {
            for (int y = minRangeY; y <= maxRangeY; y++) {
                for (int z = minRangeZ; z <= maxRangeZ; z++) {
                    int index = x + y * gridDimensions.x() + z * gridDimensions.x() * gridDimensions.y();
                    cells.push_back(index);
                }
            }
        }
        return cells;
    }

    void buildHashedGrid(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        // Go through every object in our scene and add them with the hash value to the hashTable
        for (int i = 0; i < hittables.size(); ++i) {
            std::vector<int> overlappingCells = getOverlappingCells(hittables[i]->getBoundingBox());

            for (int cellIndex: overlappingCells) {
                vec3 cellCenter = getCellCenter(cellIndex);
                int hashIndex = hashFunction(cellCenter);
                hashTable[hashIndex].push_back(i);
            }
        }
    }

    [[nodiscard]] vec3 getCellCenter(int cellIndex) const {
        int gridX = static_cast<int>(gridDimensions.x());
        int gridY = static_cast<int>(gridDimensions.y());

        int z = cellIndex / (gridX * gridY);
        int y = (cellIndex % (gridX * gridY)) / gridX;
        int x = cellIndex % gridX;

        return sceneBounds.min + vec3(x + 0.5, y + 0.5, z + 0.5) * cellSize;
    }

    static BoundingBox calculateSceneBounds(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BoundingBox bounds;
        for (const auto &hittable: hittables) {
            bounds = bounds.bbUnion(hittable->getBoundingBox());
        }
        return bounds;
    }

    static int nextPrime(int n) {
        while (true) {
            if (isPrime(n)) return n;
            n++;
        }
    }

    static bool isPrime(int n) {
        if (n <= 1) return false;
        for (int i = 2; i * i <= n; i++) {
            if (n % i == 0) return false;
        }
        return true;
    }

    [[nodiscard]] int hashFunction(const vec3 &position) const {
        int mortonCode = encodeMorton3(position);
        return mortonCode % totalCells;
    }

    // TODO: MORTON CODE IN EXTRA CLASS FOR BOTH STRUCTURES
    static inline int expandBits(int x) {
        if (x == (1 << 10)) --x;
        x = (x | (x << 16)) & 0b00000011000000000000000011111111;
        x = (x | (x << 8)) & 0b00000011000000001111000000001111;
        x = (x | (x << 4)) & 0b00000011000011000011000011000011;
        x = (x | (x << 2)) & 0b00001001001001001001001001001001;
        return x;
    }

    [[nodiscard]] int encodeMorton3(const vec3 &v) const {
        // Normalize to [0, 1] inside the scene bounds
        const vec3 normalize = sceneBounds.offset(v);

        int averageResolution = (gridDimensions.x() + gridDimensions.y() + gridDimensions.z()) / 3;

        double scale = averageResolution - 1;

        // Scale to [0, scale]
        int x = static_cast<int>(normalize.x() * scale);
        int y = static_cast<int>(normalize.y() * scale);
        int z = static_cast<int>(normalize.z() * scale);

        return morton3D(x, y, z);
    }

    static inline int morton3D(int x, int y, int z) {
        return (expandBits(x) << 2) | (expandBits(y) << 1) |
               expandBits(z);
    }
};


#endif //RAYTRACER_UNIFORM_GRID_H
