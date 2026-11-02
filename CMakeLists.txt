cmake_minimum_required(VERSION 3.16)
project(SolidMesh VERSION 0.1.0 LANGUAGES CXX)

option(SOLIDMESH_BUILD_EXAMPLE "Build the SolidMesh sample app." OFF)
option(SOLIDMESH_BUILD_TESTS "Build SolidMesh native contract tests." OFF)

set(SOLIDMESH_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(include)
add_subdirectory(src)

if(SOLIDMESH_BUILD_EXAMPLE)
    add_subdirectory(example)
endif()

if(SOLIDMESH_BUILD_TESTS)
    enable_testing()
    add_executable(solidmesh_tests tests/mesh_test.cpp)
    target_link_libraries(solidmesh_tests PRIVATE SolidMesh::core)
    add_test(NAME solidmesh_native_contract COMMAND solidmesh_tests)
    add_executable(solidmesh_ccw_tests tests/edge_cells_ccw_test.cpp)
    target_link_libraries(solidmesh_ccw_tests PRIVATE SolidMesh::core)
    add_test(NAME solidmesh_ccw_valid_stars COMMAND solidmesh_ccw_tests)
    foreach(case broken_opposite duplicate_incidence stale_halfface missing_owner wrong_incidence self_opposite)
        add_test(NAME solidmesh_ccw_reject_${case} COMMAND solidmesh_ccw_tests ${case})
    endforeach()
endif()
