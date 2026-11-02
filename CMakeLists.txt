cmake_minimum_required(VERSION 3.16)
project(SolidMesh LANGUAGES CXX)

add_library(SolidMeshCore STATIC src/mesh.cpp)
add_library(SolidMesh::core ALIAS SolidMeshCore)

target_compile_features(SolidMeshCore PUBLIC cxx_std_20)
set_target_properties(SolidMeshCore PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(SolidMeshCore PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_include_directories(SolidMeshCore SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
)

# 优先复用父项目的 Eigen 目标，否则直接使用附带子模块的头文件。
if(TARGET Eigen3::Eigen)
    target_link_libraries(SolidMeshCore PUBLIC Eigen3::Eigen)
else()
    target_include_directories(SolidMeshCore SYSTEM PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen"
    )
endif()
