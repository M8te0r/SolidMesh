file(GLOB SOURCES "*.cpp" "*.cc" "*.cxx")

add_executable(polyhedra_test ${SOURCES})

target_link_libraries(polyhedra_test PRIVATE polyhedra-core)
target_include_directories(polyhedra_test PRIVATE ${CMAKE_SOURCE_DIR})

# 根据构建类型设置输出目录
set_target_properties(polyhedra_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/$<CONFIG>/app"
    VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

# Copy assets next to the executable so it works regardless of CWD
add_custom_command(TARGET polyhedra_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/assets"
        "$<TARGET_FILE_DIR:polyhedra_test>/assets"
    COMMENT "Copying assets to output directory"
)