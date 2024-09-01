#ifndef RAYTRACER_BSP_TREES_H
#define RAYTRACER_BSP_TREES_H

#include <vector>
#include <stack>
#include "data_structure.h"

enum class TreeType {
    KD, Octree
};

struct KDNode {
    std::shared_ptr<KDNode> left;
    std::shared_ptr<KDNode> right;
    int splitAxis;
    double splitPos;
    std::vector<int> hittableIndices;
    bool isLeaf;

    KDNode() : isLeaf(false), splitAxis(0), splitPos(0.0) {}
};

class BSPTree : public DataStructure {
public:
    // Not sure about the maxdepth maybe there are some calculations or other things todo here
    // Maybe also read about it alittle bit -> Tradeoff between leaf tirnalges and build time
    explicit BSPTree(int numTriangles, TreeType type = TreeType::KD) : root(nullptr), maxDepth(50), minTriangles(5) {}

    void build(const std::vector<std::shared_ptr<Hittable>> &hittables) override {
        sceneBounds = calculateSceneBounds(hittables);
        std::vector<int> indices(hittables.size());
        for (int i = 0; i < indices.size(); ++i) {
            indices[i] = i;
        }
        root = buildRecursive(hittables, indices, sceneBounds, 0);
    }

    void intersect(Ray &ray, const std::vector<std::shared_ptr<Hittable>> &hittables, int &hitObject) override {
        double tmin = sceneBounds.intersect(ray);
        double tmax = sceneBounds.tmaxBox;
        bool liesOutsideBox =
                tmin >= std::numeric_limits<double>::infinity() || tmax >= std::numeric_limits<double>::infinity();
        if (root && !liesOutsideBox) {
            searchNode(root, ray, tmin, tmax, hitObject, hittables);
        }
    }

private:
    std::shared_ptr<KDNode> root;
    int maxDepth;
    int minTriangles;
    BoundingBox sceneBounds;

    std::shared_ptr<KDNode>
    buildRecursive(const std::vector<std::shared_ptr<Hittable>> &hittables, const std::vector<int> &indices,
                   const BoundingBox &box, int depth) {
        // Terminate condition
        if (terminate(hittables, depth)) {
            auto leafNode = std::make_shared<KDNode>(); // Might be better to use a unique_ptr
            leafNode->isLeaf = true;
            leafNode->hittableIndices = indices;
            return leafNode;
        }

        // Split based on spatial median and rr axis
        int splitAxis = depth % 3;
        double splitPos = (box.min[splitAxis] + box.max[splitAxis]) / 2.0f;

        //Split bounding box and hittables
        BoundingBox leftBox, rightBox;
        std::vector<std::shared_ptr<Hittable>> leftHittables, rightHittables;
        std::vector<int> leftIndices, rightIndices;
        leftBox = box;
        rightBox = box;
        leftBox.max[splitAxis] = splitPos;
        rightBox.min[splitAxis] = splitPos;

        for (int i = 0; i < hittables.size(); ++i) {
            const auto &hittable = hittables[i];
            if (hittable->getBoundingBox().min[splitAxis] <= splitPos) {
                leftHittables.push_back(hittable);
                leftIndices.push_back(indices[i]);
            }
            if (hittable->getBoundingBox().max[splitAxis] >= splitPos) {
                rightHittables.push_back(hittable);
                rightIndices.push_back(indices[i]);
            }
        }

        //Build child nodes recursively
        auto kdNode = std::make_shared<KDNode>();
        kdNode->splitAxis = splitAxis;
        kdNode->splitPos = splitPos;
        kdNode->left = buildRecursive(leftHittables, leftIndices, leftBox, depth + 1);
        kdNode->right = buildRecursive(rightHittables, rightIndices, rightBox, depth + 1);

        return kdNode;
    }

    bool terminate(const std::vector<std::shared_ptr<Hittable>> &hittables, int depth) {
        return hittables.size() <= minTriangles || depth >= maxDepth;
    }

    void searchNode(const std::shared_ptr<KDNode> &node, Ray &ray, double tmin, double tmax, int &hitObject,
                    const std::vector<std::shared_ptr<Hittable>> &hittables) {
        std::stack<std::tuple<std::shared_ptr<KDNode>, double, double>> stack;

        stack.emplace(node, tmin, tmax);

        while (!stack.empty()) {
            auto [currentNode, currentTmin, currentTmax] = stack.top();
            stack.pop();

            if (currentNode->isLeaf) {
                for (const auto &index: currentNode->hittableIndices) {
                    double nearestHit = ray.t;
                    if (hittables[index]->intersect(ray)) {
                        if (ray.t < nearestHit) nearestHit = ray.t;
                        else return; // Early termination if we already have a nearer intersection
                        hitObject = index;
                    }
                }
            } else {
                double splitPos = currentNode->splitPos;
                int axis = currentNode->splitAxis;
                double thit = (splitPos - ray.origin[axis]) / ray.direction[axis];

                bool isFirstChildLeft = (ray.direction[axis] >= 0);
                auto firstChild = isFirstChildLeft ? currentNode->left : currentNode->right;
                auto secondChild = isFirstChildLeft ? currentNode->right : currentNode->left;

                if (thit <= currentTmin) {
                    // Ray intersects only with the second child
                    stack.emplace(secondChild, currentTmin, currentTmax);
                } else if (thit >= currentTmax || thit < 0) {
                    // Ray intersects only with the first child
                    stack.emplace(firstChild, currentTmin, currentTmax);
                } else {
                    // Ray intersects both children
                    stack.emplace(secondChild, thit, currentTmax);
                    stack.emplace(firstChild, currentTmin, thit);
                }
            }
        }
    }

    static BoundingBox calculateSceneBounds(const std::vector<std::shared_ptr<Hittable>> &hittables) {
        BoundingBox bounds;
        for (const auto &hittable: hittables) {
            bounds = bounds.bbUnion(hittable->getBoundingBox());
        }
        return bounds;
    }
};

#endif //RAYTRACER_BSP_TREES_H
