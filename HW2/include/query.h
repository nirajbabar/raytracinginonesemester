#pragma once

#include "bvh.h"
#include "camera.h"
#include "ray.h"
#include "MeshOBJ.h"
#include "brdf.h"

struct Light;

void render(    
    const size_t numTriangles,
    int W, int H,
    const Camera cam,
    const Vec3 missColor,
    const int max_depth,
    const BVHNode* __restrict__ nodes,
    const AABB* __restrict__ aabbs,
    const Triangle* __restrict__ triangles,
    const int32_t* __restrict__ triObjectIds,
    const Material* __restrict__ objectMaterials,
    const int numObjectMaterials,
    const int numRays,
    const Light* __restrict__ lights,
    const int numLights,
    Vec3* __restrict__ output);

HYBRID_FUNC inline HitRecord intersectTriangle(const Ray& r,
                                          const Triangle& tri,
                                          float tmin,
                                          float tmax)
{
    HitRecord rec{};
    rec.triangleIdx = -1;

    const Vec3 e1 = tri.v1 - tri.v0;
    const Vec3 e2 = tri.v2 - tri.v0;
    const Vec3 pvec = cross(r.direction(), e2);
    const float det = dot(e1, pvec);

    if (fabsf(det) < 1e-8f) {
        rec.hit = false;
        return rec;
    }
    const float invDet = 1.0f / det;

    const Vec3 tvec = r.origin() - tri.v0;
    const float u = dot(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f) {
        rec.hit = false;
        return rec;
    }

    const Vec3 qvec = cross(tvec, e1);
    const float v = dot(r.direction(), qvec) * invDet;
    if (v < 0.0f || (u + v) > 1.0f) {
        rec.hit = false;
        return rec;
    }
    const float t = dot(e2, qvec) * invDet;
    if (t < tmin || t > tmax) {
        rec.hit = false;
        return rec;
    }

    rec.hit = true;
    rec.t = t;
    rec.p = r.origin() + r.direction() * t;

    // Use geometric normal for sidedness (robust), shading normal for BRDF.
    Vec3 geomN = normalize(cross(e1, e2));
    rec.front_face = dot(r.direction(), geomN) < 0.0f;
    if (!rec.front_face) geomN = -geomN;

    Vec3 shadingN = (1.0f - u - v) * tri.n0 + u * tri.n1 + v * tri.n2;
    if (length_squared(shadingN) < 1e-12f) {
        shadingN = geomN;
    } else {
        shadingN = normalize(shadingN);
        // Keep shading normal in same hemisphere as geometric normal.
        if (dot(shadingN, geomN) < 0.0f) shadingN = -shadingN;
    }

    rec.normal = shadingN;
    rec.mat = Material();

    return rec;
}

HYBRID_FUNC inline void assignMaterialToHit(
    HitRecord& hitRecord,
    const int numTriangles,
    const int32_t* __restrict__ triObjectIds,
    const Material* __restrict__ objectMaterials,
    const int numObjectMaterials)
{
    if (!hitRecord.hit ||
        triObjectIds == nullptr ||
        objectMaterials == nullptr ||
        hitRecord.triangleIdx < 0 ||
        hitRecord.triangleIdx >= numTriangles) {
        return;
    }

    const int objId = triObjectIds[hitRecord.triangleIdx];
    if (objId >= 0 && objId < numObjectMaterials) {
        hitRecord.mat = objectMaterials[objId];
    }
}

HYBRID_FUNC inline Vec3 TraceRayIterative(
    const Ray& primaryRay,
    const int maxDepth,
    const Vec3 missColor,
    const int numTriangles,
    const BVHNode* __restrict__ nodes,
    const AABB* __restrict__ aabbs,
    const Triangle* __restrict__ triangles,
    const int32_t* __restrict__ triObjectIds,
    const Material* __restrict__ objectMaterials,
    const int numObjectMaterials,
    const Light* __restrict__ lights,
    const int numLights)
{
    if (maxDepth <= 0) return make_vec3(0.0f, 0.0f, 0.0f);

    Ray ray = primaryRay;
    Vec3 radiance = make_vec3(0.0f, 0.0f, 0.0f);
    Vec3 throughput = make_vec3(1.0f, 1.0f, 1.0f);

    for (int depth = 0; depth < maxDepth; ++depth) {
        HitRecord hitRecord;
        SearchBVH(numTriangles, ray, nodes, aabbs, triangles, hitRecord);
        if (!hitRecord.hit) {
            radiance = radiance + throughput * missColor;
            break;
        }

        assignMaterialToHit(hitRecord, numTriangles, triObjectIds, objectMaterials, numObjectMaterials);

        Vec3 direct = ShadeDirect(ray, hitRecord, lights, numLights,
                                  numTriangles, nodes, aabbs, triangles);
        radiance = radiance + throughput * direct;

        if (hitRecord.mat.kr <= 0.0f) break;

        const Vec3 N = normalize(hitRecord.normal);
        const Vec3 reflDir = reflect_dir(unit_vector(ray.direction()), N);
        ray = Ray(hitRecord.p + N * RT_EPS, reflDir);

        const Vec3 reflTint = hitRecord.mat.specularColor;
        throughput = throughput * (hitRecord.mat.kr * reflTint);

        if (throughput.x < 1e-4f && throughput.y < 1e-4f && throughput.z < 1e-4f) {
            break;
        }
    }

    return clamp(radiance);
}

HYBRID_FUNC inline void SearchBVH(
    const int numTriangles,
    const Ray& ray,
    const BVHNode* __restrict__ nodes,
    const AABB* __restrict__ aabbs,
    const Triangle* __restrict__ triangles,
    HitRecord& hitRecord)
{

    constexpr float kRayTMin = 1e-4f;
    const float tmin = kRayTMin;
    float bestT = FLT_MAX;
    HitRecord bestHit;
    bestHit.triangleIdx = -1;
    bestHit.hit = false;
    bestHit.t = -1.0;
    bestHit.p = make_vec3(0.0f, 0.0f, 0.0f);
    bestHit.normal = make_vec3(0.0f, 0.0f, 0.0f);
    bestHit.front_face = false;
    bestHit.mat = Material();

    constexpr int STACK_CAPACITY = 512;
    std::uint32_t stack[STACK_CAPACITY];
    std::uint32_t* stack_ptr = stack;
    bool stackOverflow = false;
    *stack_ptr++ = 0; // root node is always 0
    

    while (stack_ptr > stack) {
        const std::uint32_t nodeIdx = *--stack_ptr;

        if (!intersectAABB(ray, aabbs[nodeIdx], tmin, bestT)) {
            continue;
        }

        const BVHNode node = nodes[nodeIdx];
        const std::uint32_t obj_idx = node.object_idx;

        if (obj_idx != 0xFFFFFFFF) {
            if (obj_idx < static_cast<std::uint32_t>(numTriangles)) {
                HitRecord rec = intersectTriangle(ray, triangles[obj_idx], tmin, bestT);
                if (rec.hit) {
                    rec.triangleIdx = static_cast<int>(obj_idx);
                    bestT = rec.t;
                    bestHit = rec;
                }
            }
            continue;
        }

        const std::uint32_t left_idx = node.left_idx;
        const std::uint32_t right_idx = node.right_idx;

        if (left_idx != 0xFFFFFFFF) {
            if (intersectAABB(ray, aabbs[left_idx], tmin, bestT)) {
                if (stack_ptr - stack < STACK_CAPACITY) {
                    *stack_ptr++ = left_idx;
                } else {
                    stackOverflow = true;
                }
            }
        }

        if (right_idx != 0xFFFFFFFF) {
            if (intersectAABB(ray, aabbs[right_idx], tmin, bestT)) {
                if (stack_ptr - stack < STACK_CAPACITY) {
                    *stack_ptr++ = right_idx;
                } else {
                    stackOverflow = true;
                }
            }
        }
    }

    // Safety fallback: if traversal overflowed, complete with brute-force test to avoid artifacts.
    if (stackOverflow) {
        for (int i = 0; i < numTriangles; ++i) {
            HitRecord rec = intersectTriangle(ray, triangles[i], tmin, bestT);
            if (rec.hit) {
                rec.triangleIdx = i;
                bestT = rec.t;
                bestHit = rec;
            }
        }
    }

    hitRecord = bestHit;
}
