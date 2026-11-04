cmake_minimum_required(VERSION 3.20)
project(SolidMesh VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SOLIDMESH_BUILD_TESTS "Build SolidMesh topology regression tests." OFF)

file(GLOB_RECURSE SOLIDMESH_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
)

file(GLOB_RECURSE SOLIDMESH_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx"
)

add_library(SolidMeshCore STATIC
    ${SOLIDMESH_HEADERS}
    ${SOLIDMESH_SOURCES}
)
add_library(SolidMesh::core ALIAS SolidMeshCore)

target_compile_features(SolidMeshCore PUBLIC cxx_std_20)

target_include_directories(SolidMeshCore
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(SOLIDMESH_BUILD_TESTS)
    enable_testing()
    add_executable(solidmesh_tests tests/mesh_test.cpp)
    target_link_libraries(solidmesh_tests PRIVATE SolidMesh::core)
    add_test(NAME solidmesh_topology COMMAND solidmesh_tests)
    add_test(NAME solidmesh_assets COMMAND solidmesh_tests "${CMAKE_CURRENT_SOURCE_DIR}/assets")
endif()
