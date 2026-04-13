file(GLOB_RECURSE POLYHEDRA_HEADERS CONFIGURE_DEPENDS "*.h" ".hh" "*.hpp")
file(GLOB_RECURSE POLYHEDRA_SOURCES CONFIGURE_DEPENDS "*.c" "*.cpp" "*.cc" "*.cxx")


add_library(polyhedra-core STATIC
    ${POLYHEDRA_HEADERS}
    ${POLYHEDRA_SOURCES}
)

target_include_directories(polyhedra-core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}    # 允许别的项目 include <polyhedra-core/...>
        # ${CMAKE_SOURCE_DIR}/dependencies
        # ${CMAKE_SOURCE_DIR}/dependencies/TetGen
)