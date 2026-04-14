file(GLOB SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc" "*.cxx")

add_executable(polyhedra_test ${SOURCES})

target_link_libraries(polyhedra_test PRIVATE SolidMesh::core)

set_target_properties(polyhedra_test PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${SOLIDMESH_ROOT_DIR}"
)

# Copy assets next to the executable so it works regardless of CWD.
add_custom_command(TARGET polyhedra_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${SOLIDMESH_ROOT_DIR}/assets"
        "$<TARGET_FILE_DIR:polyhedra_test>/assets"
    COMMENT "Copying assets to output directory"
)
