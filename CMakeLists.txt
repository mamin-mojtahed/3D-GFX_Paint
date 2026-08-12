cmake_minimum_required(VERSION 3.16)
project(3D-GFX_Paint)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(external/glew)
add_subdirectory(external/glfw)

file(GLOB SRC_FILES
        src/*.h
        src/*.cpp)

add_executable(3D-GFX_Paint
        ${SRC_FILES})

target_include_directories(3D-GFX_Paint PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external/glfw/include)
target_include_directories(3D-GFX_Paint PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/external/glew/include)
target_link_libraries(3D-GFX_Paint glfw libglew_static)
