cmake_minimum_required(VERSION 3.10)
project(PhysicsEngine)

set(CMAKE_CXX_STANDARD 17)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE "GLVND")

# Define the source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# Include paths relative to the src directory
include_directories(
    ${SRC_DIR}
    ${SRC_DIR}/imgui
    ${SRC_DIR}/imgui/backends
    ${SRC_DIR}/engine
    /usr/include
)

# Source files relative to src directory
set(SOURCES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/engine/World.cpp
    ${SRC_DIR}/imgui/imgui.cpp
    ${SRC_DIR}/imgui/imgui_draw.cpp
    ${SRC_DIR}/imgui/imgui_tables.cpp
    ${SRC_DIR}/imgui/imgui_widgets.cpp
    ${SRC_DIR}/imgui/backends/imgui_impl_glfw.cpp
    ${SRC_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)

# Link libraries
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

add_executable(physics ${SOURCES})
target_link_libraries(physics PRIVATE glfw OpenGL::GL)