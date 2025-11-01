cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)

# Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE "GLVND")

set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

include_directories(
    ${SRC_DIR}
    ${SRC_DIR}/imgui
    ${SRC_DIR}/imgui/backends
    ${SRC_DIR}/engine
    /usr/include
)

set(SOURCES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/imgui/imgui.cpp
    ${SRC_DIR}/imgui/imgui_draw.cpp
    ${SRC_DIR}/imgui/imgui_tables.cpp
    ${SRC_DIR}/imgui/imgui_widgets.cpp
    ${SRC_DIR}/imgui/backends/imgui_impl_glfw.cpp
    ${SRC_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)

add_executable(physics ${SOURCES})

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

target_link_libraries(physics PRIVATE glfw OpenGL::GL)

if(NOT MSVC)
    # Use -O3 by default in Release, tune as needed
    target_compile_options(physics PRIVATE
        $<$<CONFIG:RELEASE>:-O3 -march=native -funroll-loops>
        $<$<CONFIG:DEBUG>:-g -O0 -fsanitize=address,undefined>
    )
endif()
