file(GLOB SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc" "*.cxx")

add_executable(SolidMesh_Test ${SOURCES})

target_link_libraries(SolidMesh_Test PRIVATE SolidMesh::core)

set_target_properties(SolidMesh_Test PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "${SOLIDMESH_ROOT_DIR}"
)

# Copy assets next to the executable so it works regardless of CWD.
add_custom_command(TARGET SolidMesh_Test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${SOLIDMESH_ROOT_DIR}/assets"
        "$<TARGET_FILE_DIR:SolidMesh_Test>/assets"
    COMMENT "Copying assets to output directory"
)
