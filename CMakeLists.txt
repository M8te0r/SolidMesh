cmake_minimum_required(VERSION 3.20)
project(SolidMesh VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(SOLIDMESH_TOP_LEVEL ON)
else()
    set(SOLIDMESH_TOP_LEVEL OFF)
endif()

option(SOLIDMESH_BUILD_EXAMPLE "Build the SolidMesh sample app." ${SOLIDMESH_TOP_LEVEL})

set(SOLIDMESH_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

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

if(SOLIDMESH_BUILD_EXAMPLE)
    add_subdirectory(example)
endif()
