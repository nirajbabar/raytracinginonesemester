#include "MeshOBJ.h"
#include "buffers.h"
#include "bvh.h"
#include "visualizer.h"
#include <numeric>
#include <chrono>

RayTracer::BVHState RayTracer::BVHState::fromChunk(char*& chunk, size_t P)
{
    BVHState state;
    obtain(chunk, state.Nodes, 2 * P - 1, 128);
    obtain(chunk, state.AABBs, 2 * P - 1, 128);
    return state;
}

int main(int argc, char** argv)
{

#ifdef __CUDACC__
    cudaFree(0);
#endif

    using vec3 = Vec3;
    using point3 = Vec3;

    std::vector<std::string> paths;
    if (argc >= 2) {
        for(int i=1; i<argc; ++i) paths.push_back(argv[i]);
    } else {
        paths.push_back("../assets/meshes/frog.obj");
    }

    Mesh globalMesh;
    int nextObjectId = 0;

    for (const auto& path : paths)
    {
        std::cout << "Loading OBJ: " << path << "\n";
        Mesh tempMesh;
        if (!LoadOBJ_ToMesh(path, tempMesh, nextObjectId))
        {
            std::cerr << "Failed to load OBJ: " << path << "\n";
            continue;
        }
        
        std::cout << "  -> Loaded " << tempMesh.indices.size() / 3 << " triangles.\n";
        AppendMesh(globalMesh, tempMesh);
    }

    if (globalMesh.positions.empty())
    {
        std::cerr << "No valid geometry loaded.\n";
        return 1;
    }

    std::function<char*(size_t)> bvhFunc = [](size_t P) {
#ifdef __CUDACC__
        char* buffer;
        if (cudaMalloc(&buffer, P) != cudaSuccess) {
            std::cerr << "Failed to allocate device memory for BVH\n";
            exit(1);
        }
        return buffer;
#else
        char* buffer = new char[P];
        return buffer;
#endif
    };

    AABB sceneAABB;

    size_t P = globalMesh.indices.size() / 3;
    size_t bvh_chunk_size = required<RayTracer::BVHState>(P);
    char* bvh_chunk = bvhFunc(bvh_chunk_size);
    RayTracer::BVHState bvhState = RayTracer::BVHState::fromChunk(bvh_chunk, P);

    AccStruct::BVH bvh;

    bvh.calculateAABBs(globalMesh, bvhState.AABBs);

    AABB SceneBoundingBox;
    #ifdef __CUDACC__
        AABB default_aabb;
        // Reduce directly on device memory
        SceneBoundingBox = thrust::reduce(
            thrust::device_pointer_cast(bvhState.AABBs + (P - 1)),
            thrust::device_pointer_cast(bvhState.AABBs + (2*P - 1)),
            default_aabb,
            [] __device__ __host__ (const AABB& lhs, const AABB& rhs) {
                return AABB::merge(lhs, rhs);
            });

        thrust::device_vector<unsigned int> TriangleIndices(P);
        thrust::copy(thrust::make_counting_iterator<std::uint32_t>(0),
            thrust::make_counting_iterator<std::uint32_t>(P),
            TriangleIndices.begin());

        // // --- GPU Warmup & Benchmark ---
        // // Backup leaves to restore state after warmup
        // thrust::device_vector<AABB> d_leaves_backup(
        //     thrust::device_pointer_cast(bvhState.AABBs + (P - 1)), 
        //     thrust::device_pointer_cast(bvhState.AABBs + (2 * P - 1)));
        
        // printf("Warming up GPU...\n");
        // bvh.buildBVH(
        //     bvhState.Nodes,
        //     bvhState.AABBs,
        //     SceneBoundingBox,
        //     &TriangleIndices,
        //     static_cast<int>(P)
        // );
        
        // // Restore state for fair timing
        // thrust::copy(d_leaves_backup.begin(), d_leaves_backup.end(), 
        //              thrust::device_pointer_cast(bvhState.AABBs + (P - 1)));
        // thrust::sequence(TriangleIndices.begin(), TriangleIndices.end());
        // cudaDeviceSynchronize();

        auto start_gpu = std::chrono::high_resolution_clock::now();

        bvh.buildBVH(
            bvhState.Nodes,
            bvhState.AABBs,
            SceneBoundingBox,
            &TriangleIndices,
            static_cast<int>(P)
        );
        cudaDeviceSynchronize();
        auto end_gpu = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms_gpu = end_gpu - start_gpu;
        printf("GPU LBVH Build Time (warm restart): %.3f ms\n", ms_gpu.count());

    #else
        SceneBoundingBox = std::accumulate(
            bvhState.AABBs + (P - 1),
            bvhState.AABBs + (2 * P - 1),
            AABB(),
            [](const AABB& lhs, const AABB& rhs) {
                return AABB::merge(lhs, rhs);
            });

        std::vector<unsigned int> TriangleIndices(P);
        std::iota(TriangleIndices.begin(), TriangleIndices.end(), 0);
        auto start_cpu =  std::chrono::high_resolution_clock::now();
        bvh.buildBVH(
            bvhState.Nodes,
            bvhState.AABBs,
            SceneBoundingBox,
            TriangleIndices,
            static_cast<int>(P)
        );

        auto end_cpu = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms_cpu = end_cpu - start_cpu;
        printf("CPU LBVH Build Time: %.3f ms\n", ms_cpu.count());

    #endif

    

//     std::cout << "Exporting all AABBs to bvh_frog.obj...\n";
//     size_t totalNodes = 2 * P - 1;
//     std::vector<AABB> allAABBs(totalNodes);
// #ifdef __CUDACC__
//     cudaMemcpy(allAABBs.data(), bvhState.AABBs, sizeof(AABB) * totalNodes, cudaMemcpyDeviceToHost);
// #else
//     for(size_t i=0; i<totalNodes; ++i) allAABBs[i] = bvhState.AABBs[i];
// #endif
//     ExportAABBsToOBJ("bvh_.obj", allAABBs.data(), totalNodes);

    return 0;
}