cmake_minimum_required(VERSION 3.10.0)
project(SolidMesh VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Release)


# 应用程序输出文件夹（根据构建类型）
set(APP_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin/$<CONFIG>")

add_subdirectory(src)
add_subdirectory(app)


