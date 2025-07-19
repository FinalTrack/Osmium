# Osmium

A fast 2D physics engine in C++ featuring rigid body dynamics, SAT-based collision detection, impulse-based resolution, a dynamic quadtree for broad-phase optimization, and multithreaded execution. Includes an interactive ImGui debugger.

## Demo
[Link](https://www.youtube.com/watch?v=go6uJhxqp_U)


## Getting Started

### Prerequisites

* A C++17 compatible compiler (e.g., GCC, Clang, MSVC).
* CMake (for building the project).
* GLFW (for windowing and OpenGL context).
* OpenGL (typically available with your graphics drivers).

### Building the Project

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/FinalTrack/Osmium.git
    cd Osmium
    ```

2.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```

3.  **Run CMake to configure the project:**
    ```bash
    cmake ..
    ```
    *If GLFW is not found automatically, you might need to install it via your system's package manager

4.  **Build the project:**
    ```bash
    cmake --build .
    ```

### Running the Application

After a successful build, you can run the executable from the `build` directory:

```bash
./physics
