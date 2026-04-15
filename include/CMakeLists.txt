file(GLOB_RECURSE POLYHEDRA_HEADERS CONFIGURE_DEPENDS "*.h" "*.hh" "*.hpp")

add_library(polyhedra-core STATIC ${POLYHEDRA_HEADERS})
add_library(SolidMesh::core ALIAS polyhedra-core)

target_compile_features(polyhedra-core PUBLIC cxx_std_17)

target_include_directories(polyhedra-core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)
