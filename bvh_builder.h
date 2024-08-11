#ifndef RAYTRACER_BVH_BUILDER_H
#define RAYTRACER_BVH_BUILDER_H


#include <vector>
#include <memory>
#include <algorithm>
#include <queue>
#include "vec3.h"
#include "hittable.h"

enum Split {
    Middle, SAH, Linear, LinearSAH
};

struct Bin {
    BoundingBox bounds;
    int triCount = 0;
};

struct BVHNode {
    BoundingBox boundingBox;
    int leftFirst{}, triCount{}, totalTriCount{};

    [[nodiscard]] bool isLeaf() const { return triCount > 0; }
};

struct MortonPrimitive {
    uint32_t mortonCode;
    int primitiveIndex;
};

class BVHBuilder {
public:
    explicit BVHBuilder(int numTriangles, Split split) : triIdx(new int[numTriangles]),
                                                         bvhNode(new BVHNode[2 * numTriangles]), rootNodeIdx(0),
                                                         nodesUsed(1), numTriangles(numTriangles), split(split) {}

    void build_bvh(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        if (split == Split::Linear) {
            build_bvh_bottom_up(hittables, 0);
        } else if (split == Split::LinearSAH) {
            build_bvh_bottom_up(hittables, 32);
        } else {
            for (int i = 0; i < numTriangles; i++) triIdx[i] = i;

            BVHNode &root = bvhNode[rootNodeIdx];
            root.leftFirst = 0, root.triCount = numTriangles;
            update_node_bounds(rootNodeIdx, hittables);
            subdivide(rootNodeIdx, hittables);
        }
    }

    void intersect_bvh(Ray &ray, std::vector<std::shared_ptr<Hittable>> &hittables, int &hit_object) {
        intersect_bvh(ray, rootNodeIdx, hittables, hit_object);
    }

    void
    intersect_bvh(Ray &ray, const int nodeIdx, std::vector<std::shared_ptr<Hittable>> &hittables, int &hit_object) {
        BVHNode *node = &bvhNode[nodeIdx], *stack[64];
        int stackPtr = 0;

        // Precompute inverse ray direction and direction signs
        vec3 invDir(1.0 / ray.direction.x(), 1.0 / ray.direction.y(), 1.0 / ray.direction.z());
        int dirIsNeg[3] = {invDir.x() < 0, invDir.y() < 0, invDir.z() < 0};

        while (true) {
            if (node->isLeaf()) {
                for (int i = 0; i < node->triCount; i++) {
                    int objIndex = triIdx[node->leftFirst + i];
                    if (hittables[objIndex]->intersect(ray)) {
                        hit_object = objIndex;
                        return; // Early termination (Only get first intersection)
                    }
                }
                if (stackPtr == 0) break;
                else node = stack[--stackPtr];
                continue;
            }
            BVHNode *child1 = &bvhNode[node->leftFirst];
            BVHNode *child2 = &bvhNode[node->leftFirst + 1];

            double dist1 = child1->boundingBox.intersect(ray);
            double dist2 = child2->boundingBox.intersect(ray);

            if (dist1 > dist2) {
                std::swap(dist1, dist2);
                std::swap(child1, child2);
            }

            if (dist1 == std::numeric_limits<double>::infinity()) {
                if (stackPtr == 0) break;
                else node = stack[--stackPtr];
            } else {
                node = child1;
                if (dist2 != std::numeric_limits<double>::infinity()) stack[stackPtr++] = child2;
            }
        }
//        BVHNode &node = bvhNode[nodeIdx];
//        if (!node.boundingBox.intersect(ray)) return;
//
//        // TODO: Can be speed up if we check where the ray is going and only check there for intersections instead of on the right side aswell
//        if (node.isLeaf()) {
//            for (int i = 0; i < node.triCount; i++) {
//                int objIndex = triIdx[node.leftFirst + i];
//                if (hittables[objIndex]->intersect(ray)) {
//                    hit_object = objIndex;
//                }
//            }
//        } else {
//            intersect_bvh(ray, node.leftFirst, hittables, hit_object);
//            intersect_bvh(ray, node.leftFirst + 1, hittables, hit_object);
//        }
    }

private:
    int *triIdx;
    BVHNode *bvhNode;
    int rootNodeIdx, nodesUsed, numTriangles;
    Split split;

    void update_node_bounds(int nodeIdx, const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BVHNode &node = bvhNode[nodeIdx];
        int imin = std::numeric_limits<int>::min();
        int imax = std::numeric_limits<int>::max();
        node.boundingBox.min = vec3(imax, imax, imax);
        node.boundingBox.max = vec3(imin, imin, imin);
        for (int first = node.leftFirst, i = 0; i < node.triCount; i++) {
            const int leafTriIdx = triIdx[first + i];
            const BoundingBox &bounding_box = hittables[leafTriIdx]->get_bounding_box();
            node.boundingBox.min = min(node.boundingBox.min, bounding_box.min);
            node.boundingBox.max = max(node.boundingBox.max, bounding_box.max);
        }
    }

    double find_best_split(BVHNode &node, int &axis, double &splitPos,
                           const std::vector<std::shared_ptr<Hittable>> &hittables) {
        double bestCost = 1e30f;
        // TODO: Only check longest axis
        for (int a = 0; a < 3; a++) {
            double boundsMin = node.boundingBox.min[a];
            double boundsMax = node.boundingBox.max[a];
            if (boundsMin == boundsMax) continue;

            // populate bins
            const int number_of_intervals = 8;
            Bin bins[number_of_intervals];
            double scale = number_of_intervals / (boundsMax - boundsMin);
            for (int k = 0; k < node.triCount; k++) {
                const std::shared_ptr<Hittable> &triangle = hittables[triIdx[node.leftFirst + k]];
                int binIdx = std::min(number_of_intervals - 1,
                                      (int) ((triangle->calculate_center()[a] - boundsMin) * scale));
                bins[binIdx].triCount++;
                bins[binIdx].bounds.grow(triangle->get_v0());
                bins[binIdx].bounds.grow(triangle->get_v1());
                bins[binIdx].bounds.grow(triangle->get_v2());
            }

            double leftArea[number_of_intervals - 1], rightArea[number_of_intervals - 1];
            int leftCount[number_of_intervals - 1], rightCount[number_of_intervals - 1];

            BoundingBox leftBox, rightBox;
            int leftSum = 0, rightSum = 0;
            for (int i = 0; i < number_of_intervals - 1; i++) {
                leftSum += bins[i].triCount;
                leftCount[i] = leftSum;
                leftBox.grow(bins[i].bounds);
                leftArea[i] = leftBox.area();

                rightSum += bins[number_of_intervals - 1 - i].triCount;
                rightCount[number_of_intervals - 2 - i] = rightSum;
                rightBox.grow(bins[number_of_intervals - 1 - i].bounds);
                rightArea[number_of_intervals - 2 - i] = rightBox.area();
            }

            scale = (boundsMax - boundsMin) / number_of_intervals;
            for (int i = 0; i < number_of_intervals - 1; i++) {
                double planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
                if (planeCost < bestCost) {
                    axis = a;
                    splitPos = boundsMin + scale * (i + 1);
                    bestCost = planeCost;
                }
            }
        }
        return bestCost;
    }

    void subdivide(int nodeIdx, const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BVHNode &node = bvhNode[nodeIdx];
        if (node.triCount <= 2) return;

        int axis = 0;
        double splitPos = 0.0;

        if (split == Split::SAH) {
            // SAH Split
            find_best_split(node, axis, splitPos, hittables);

            // terminate recursion
//            double noSplitCost = calculate_node_cost(node);
//            if (splitCost >= noSplitCost) return;

        } else {
            // Middle Split
            vec3 extent = node.boundingBox.max - node.boundingBox.min;
            if (extent.y() > extent.x()) axis = 1;
            if (extent.z() > extent[axis]) axis = 2;
            splitPos = node.boundingBox.min[axis] + extent[axis] * 0.5f;
        }

        // in-place partition
        int i = node.leftFirst;
        int j = i + node.triCount - 1;
        while (i <= j) {
            if (hittables[triIdx[i]]->calculate_center()[axis] < splitPos) i++;
            else std::swap(triIdx[i], triIdx[j--]);
        }

        int leftCount = i - node.leftFirst;
        if (leftCount == 0 || leftCount == node.triCount) return;

        //create child nodes
        int leftChildIdx = nodesUsed++;
        int rightChildIdx = nodesUsed++;
        bvhNode[leftChildIdx].leftFirst = node.leftFirst;
        bvhNode[leftChildIdx].triCount = leftCount;
        bvhNode[rightChildIdx].leftFirst = i;
        bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
        node.leftFirst = leftChildIdx;
        node.triCount = 0;
        update_node_bounds(leftChildIdx, hittables);
        update_node_bounds(rightChildIdx, hittables);

        //recursive call
        subdivide(leftChildIdx, hittables);
        subdivide(rightChildIdx, hittables);
    }

    static inline uint32_t expandBits(uint32_t x) {
        x = (x | (x << 16)) & 0x30000FF;
        x = (x | (x << 8)) & 0x300F00F;
        x = (x | (x << 4)) & 0x30C30C3;
        x = (x | (x << 2)) & 0x9249249;
        return x;
    }

    static inline uint32_t encodeMorton3(const vec3 &v, const BoundingBox &globalBounds, int gridResolution) {
        // Normalize to [0, 1]
        const vec3 normalize = (v - globalBounds.min) / (globalBounds.max - globalBounds.min);

        int limit = gridResolution - 1;

        // Scale to [0, limit] depending on the average triangle volume
        int x = static_cast<int>(normalize.x() * limit);
        int y = static_cast<int>(normalize.y() * limit);
        int z = static_cast<int>(normalize.z() * limit);

        return (morton3D(x, y, z));
    }

    static inline uint32_t morton3D(int x, int y, int z) {
        return (expandBits(x) << 2) | (expandBits(y) << 1) |
               expandBits(z);
    }

    static int determine_grid_resolution(double averageTriangleVolume, const BoundingBox &globalBounds) {
        vec3 boundsSize = globalBounds.max - globalBounds.min;

        // Estimate the volume of the bounding box
        double boundingBoxVolume = globalBounds.volume();

        // Estimate the number of cells
        double numCells = boundingBoxVolume / averageTriangleVolume;

        double gridSizeX = std::cbrt(numCells * (boundsSize.x() / boundingBoxVolume));
        double gridSizeY = std::cbrt(numCells * (boundsSize.y() / boundingBoxVolume));
        double gridSizeZ = std::cbrt(numCells * (boundsSize.z() / boundingBoxVolume));
        const int gridResolution = (gridSizeX + gridSizeY + gridSizeZ) / 3;

        return std::max(1, std::min(gridResolution, 1024));
    }

    static double calculate_sah_cost(const BVHNode &node1, const BVHNode &node2) {
        return node1.boundingBox.area() * node1.totalTriCount + node2.boundingBox.area() * node2.totalTriCount;
    }

    void build_sah_nodes(int offset, int numPrimitives) {
        // Create vector out of remaining nodes while maintaining indexes
        std::vector<int> remainingNodes(numPrimitives);
        for (int i = 0; i < numPrimitives; i++) {
            remainingNodes[i] = offset + i;
        }

        // Go through every node and calculate the SAH Cost of every different combination
        while (remainingNodes.size() > 1) {
            double bestCost = std::numeric_limits<double>::max();
            int bestFirst = -1, bestSecond = -1;
            int listIdxFirst = -1, listIdxSecond = -1;

            // Find the best pair to merge
            for (int i = 0; i < remainingNodes.size(); i++) {
                for (int j = i + 1; j < remainingNodes.size(); j++) {
                    int firstNodeIdx = remainingNodes[i];
                    int secondNodeIdx = remainingNodes[j];

                    double current_sah_cost = calculate_sah_cost(bvhNode[firstNodeIdx], bvhNode[secondNodeIdx]);
                    if (current_sah_cost < bestCost) {
                        bestCost = current_sah_cost;
                        bestFirst = firstNodeIdx;
                        bestSecond = secondNodeIdx;
                        listIdxFirst = i;
                        listIdxSecond = j;
                    }
                }
            }

            // Do not change the structure is important -> The indexes should be the same
            // Swap the corresponding bounding boxes and underlying leftfirst values to keep list structure intact
            if (listIdxFirst != 0) {
                std::swap(bvhNode[remainingNodes[0]], bvhNode[bestFirst]);
            }

            if (listIdxSecond != 1) {
                std::swap(bvhNode[remainingNodes[1]], bvhNode[bestSecond]);
            }

            // Create new parent node out of the best combination
            int parentIdx = nodesUsed++;

            bvhNode[parentIdx].boundingBox = bvhNode[remainingNodes[0]].boundingBox.bb_union(
                    bvhNode[remainingNodes[1]].boundingBox);
            bvhNode[parentIdx].leftFirst = remainingNodes[0];
            bvhNode[parentIdx].triCount = 0;
            bvhNode[parentIdx].totalTriCount =
                    bvhNode[remainingNodes[0]].totalTriCount + bvhNode[remainingNodes[1]].totalTriCount;

            // Remove the first 2 combined nodes and add the parent to the list
            remainingNodes.erase(remainingNodes.begin(), remainingNodes.begin() + 2);
            remainingNodes.push_back(parentIdx);
        }
    }


    void build_bvh_bottom_up(const std::vector<std::shared_ptr<Hittable>> &hittables, int sahSwitchThreshold) {
        double totalTriangleVolume = 0.0;
        double averageTriangleVolume = 0.0;

        for (const auto &hittable: hittables) {
            double triangleVolume = hittable->volume();
            totalTriangleVolume += triangleVolume;
        }

        if (numTriangles > 0) averageTriangleVolume = totalTriangleVolume / numTriangles;

        std::vector<MortonPrimitive> mortonPrims(numTriangles);
        BoundingBox globalBounds;
        std::vector<BoundingBox> primitiveBounds(numTriangles);

        // Calculate global bounding box
        int imin = std::numeric_limits<int>::min();
        int imax = std::numeric_limits<int>::max();
        globalBounds.min = vec3(imax, imax, imax);
        globalBounds.max = vec3(imin, imin, imin);
        for (int i = 0; i < numTriangles; i++) {
            BoundingBox boundingBox = hittables[i]->get_bounding_box();
            primitiveBounds[i] = boundingBox;
            globalBounds.grow(boundingBox);
        }

        const int gridResolution = determine_grid_resolution(averageTriangleVolume, globalBounds);

        // Compute Morton Codes
        for (int i = 0; i < numTriangles; i++) {
            const vec3 center = primitiveBounds[i].center();
            mortonPrims[i].mortonCode = encodeMorton3(center, globalBounds, gridResolution);
            mortonPrims[i].primitiveIndex = i;
        }

        // Sort primitives based on morton code
        std::sort(mortonPrims.begin(), mortonPrims.end(), [](const MortonPrimitive &a, const MortonPrimitive &b) {
            return a.mortonCode < b.mortonCode;
        });


        // Initialize leaf nodes
        int totalNodes = 2 * numTriangles + 1; // Total nodes needed (leaves + internal)
        bvhNode = new BVHNode[totalNodes];
        nodesUsed = numTriangles;

        for (int i = 0; i < numTriangles; i++) {
            int primIdx = mortonPrims[i].primitiveIndex;
            bvhNode[i].boundingBox = primitiveBounds[primIdx];
            bvhNode[i].leftFirst = i;
            bvhNode[i].triCount = 1;
            bvhNode[i].totalTriCount = 1;
            triIdx[i] = primIdx;
        }

        // Build hierarchy
        int offset = 0;
        while (numTriangles > 1) {
            if (numTriangles <= sahSwitchThreshold && split == Split::LinearSAH) {
                // Switch to SAH-based building
                // Instead of going through everything might be possible to just save the cost everytime to one of the nodes and then compare again
                // 1. Go through all remaining boxes
                // 2. Calculate every possible box combinations and their cost -> How it would be
                // 3. Merge the best one
                // 4. Continue with the rest of them until only one is left
                build_sah_nodes(offset, numTriangles);
                break;
            } else {
                int newNumTriangles = (numTriangles + 1) / 2;

                for (int i = 0; i < newNumTriangles; i++) {
                    int leftIdx = 2 * i + offset;
                    int rightIdx = std::min(leftIdx + 1, numTriangles + offset - 1);

                    int parentIdx = nodesUsed++;

                    if (leftIdx == rightIdx) {
                        bvhNode[parentIdx].boundingBox = bvhNode[leftIdx].boundingBox;
                        bvhNode[parentIdx].leftFirst = leftIdx;
                        bvhNode[parentIdx].triCount = 0;
                        bvhNode[parentIdx].totalTriCount = bvhNode[leftIdx].totalTriCount;
                    } else {
                        bvhNode[parentIdx].boundingBox = bvhNode[leftIdx].boundingBox.bb_union(
                                bvhNode[rightIdx].boundingBox);
                        bvhNode[parentIdx].leftFirst = leftIdx;
                        bvhNode[parentIdx].triCount = 0;
                        bvhNode[parentIdx].totalTriCount =
                                bvhNode[leftIdx].totalTriCount + bvhNode[rightIdx].totalTriCount;
                    }
                }
                offset += numTriangles;
                numTriangles = newNumTriangles;
            }
        }
        rootNodeIdx = nodesUsed - 1;
    }
};

#endif //RAYTRACER_BVH_BUILDER_H
