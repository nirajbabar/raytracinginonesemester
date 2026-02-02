<!-- g++ -x c++ main.cu bvh.cu -std=c++11 -I ../third_party/glm/ -o bvh_check

nvcc --extended-lambda --expt-relaxed-constexpr -I ../third_party/glm/ main.cu bvh.cu -o bvh_check -->

## For CPU Build
mkdir build && cd build
cmake ..
make

## For GPU Build 
mkdir build && cd build
cmake -DENABLE_GPU=ON ..
make

