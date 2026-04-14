file(GLOB_RECURSE POLYHEDRA_HEADERS CONFIGURE_DEPENDS "*.h" "*.hh" "*.hpp")
file(GLOB_RECURSE POLYHEDRA_SOURCES CONFIGURE_DEPENDS "*.c" "*.cpp" "*.cc" "*.cxx")

add_library(polyhedra-core STATIC
    ${POLYHEDRA_HEADERS}
    ${POLYHEDRA_SOURCES}
)

add_library(SolidMesh::core ALIAS polyhedra-core)

target_compile_features(polyhedra-core PUBLIC cxx_std_17)

target_include_directories(polyhedra-core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)
