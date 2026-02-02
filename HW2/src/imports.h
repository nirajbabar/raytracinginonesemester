#ifndef IMPORTS_CUH
#define IMPORTS_CUH

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <algorithm>
#include <cstddef>


#ifdef __CUDACC__
    #include <cuda_runtime.h>
    #include <device_launch_parameters.h>

    #include <thrust/random.h>
    #include <thrust/iterator/counting_iterator.h>
    #include <thrust/functional.h>
    #include <thrust/remove.h>
    #include <thrust/gather.h>
    #include <thrust/unique.h>
    #include <thrust/host_vector.h>
    #include <thrust/device_vector.h>
    #include <thrust/reduce.h>
    #include <thrust/sort.h>
    #include <thrust/complex.h>
    #include <thrust/fill.h>
    #include <thrust/fill.h>
    #include <cub/cub.cuh>
    #define HYBRID_FUNC __host__ __device__

    // force glm to work with cuda
    #define GLM_FORCE_CUDA

    #define cudaCheckError() {                                          \
        cudaError_t e = cudaGetLastError();                             \
        if(e != cudaSuccess) {                                          \
            printf("Cuda failure %s:%d: '%s'\n", __FILE__, __LINE__, cudaGetErrorString(e)); \
            exit(EXIT_FAILURE);                                         \
        }                                                               \
    }

#else
    #define HYBRID_FUNC
#endif


#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>



#endif // IMPORTS_CUH