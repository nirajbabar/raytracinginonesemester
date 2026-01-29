cmake_minimum_required(VERSION 3.16)
project(renderer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(renderer
    src/render.cpp
    src/MeshOBJ.cpp
)

find_package(glm QUIET)
if(glm_FOUND)
  target_link_libraries(renderer PRIVATE glm::glm)
endif()

target_include_directories(renderer PRIVATE ${CMAKE_SOURCE_DIR}/include)
