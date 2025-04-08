#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <chrono>
#include "engine/World.hpp"
#include <sstream>
#include <random>

const int WIDTH = 800;
const int HEIGHT = 800;
const float DT = 0.016f; 
const float PI = 3.14159265f;
World world(80.0f, AABB(Vec2(0, 0), Vec2(WIDTH, HEIGHT)), 0.9f, 0.2f);

struct Settings
{
    bool showMeshes = true;
    bool showBoundingBoxes = false;
    bool showGrid = false;
    bool showCollisions = false;
    bool showNormals = false;
    int currentMesh = 0;
    float scale = 1.0f;
} settings;

void setupProjection() {
    glViewport(0, 0, WIDTH, HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, WIDTH, HEIGHT, 0.0, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void renderGridLines() {
    glLineWidth(1.0f);  
    glColor3f(0.3f, 0.3f, 0.3f);

    glBegin(GL_LINES);

    for (float x = 0; x <= WIDTH; x += world.cellSize) {
        glVertex2f(x, 0);
        glVertex2f(x, HEIGHT);
    }

    for (float y = 0; y <= HEIGHT; y += world.cellSize) {
        glVertex2f(0, y);
        glVertex2f(WIDTH, y);
    }

    glEnd();
}

void renderBoundingBoxes() {

    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);  

    for (const auto& aabb : world.aabbs) {
        glVertex2f(aabb.min.x, aabb.min.y);
        glVertex2f(aabb.max.x, aabb.min.y);

        glVertex2f(aabb.max.x, aabb.min.y);
        glVertex2f(aabb.max.x, aabb.max.y);

        glVertex2f(aabb.max.x, aabb.max.y);
        glVertex2f(aabb.min.x, aabb.max.y);

        glVertex2f(aabb.min.x, aabb.max.y);
        glVertex2f(aabb.min.x, aabb.min.y);
    }

    glEnd();
}

void renderMesh(int meshID, const Vec2& position, float pscale) {
    if (meshID == 1000) { 
        const float radius = 10.0f * pscale; 
        const int segments = 32;   

        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f, 0.0f, 1.0f); 

        for (int i = 0; i < segments; ++i) {
            float angle = 2.0f * PI * (float)i / (float)segments;
            float x = position.x + radius * std::cos(angle);
            float y = position.y + radius * std::sin(angle);

            glVertex2f(x, y);
        }
        
        glEnd();
        return; 
    }
    
    if (meshID < 0 || meshID >= meshes.size()) return;

    const Mesh& mesh = meshes[meshID];
    size_t n = mesh.points.size();

    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 1.0f); 

    for (size_t i = 0; i < n; ++i) {
        Vec2 p1 = position + mesh.points[i] * pscale;
        Vec2 p2 = position + mesh.points[(i + 1) % n] * pscale;

        glVertex2f(p1.x, p1.y);
        glVertex2f(p2.x, p2.y);
    }
    glEnd();

    if(!settings.showNormals)
        return;

    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); 

    for (size_t i = 0; i < n; ++i) {
        Vec2 p1 = position + mesh.points[i] * pscale;
        Vec2 p2 = position + mesh.points[(i + 1) % n] * pscale;
        Vec2 midpoint = (p1 + p2) * 0.5f;

        Vec2 normalEnd = midpoint + mesh.normals[i] * 10.0f; 

        glVertex2f(midpoint.x, midpoint.y);
        glVertex2f(normalEnd.x, normalEnd.y);
    }
    glEnd();
}

void renderCollisions() {

    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 1.0f); 

    for (const auto& pair : world.collisionPairs) {
        int id1 = pair.first;
        int id2 = pair.second;

        const AABB& aabb1 = world.aabbs[id1];
        const AABB& aabb2 = world.aabbs[id2];

        Vec2 center1 = (aabb1.min + aabb1.max) * 0.5f;
        Vec2 center2 = (aabb2.min + aabb2.max) * 0.5f;

        glVertex2f(center1.x, center1.y);
        glVertex2f(center2.x, center2.y);
    }

    glEnd();
}

std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<float> velocityDist(-1.0f, 1.0f);

void addObjectAtPosition(float x, float y) {
    Vec2 position(x, y);
    Vec2 velocity(velocityDist(rng), velocityDist(rng));
    world.addObject(position, velocity * 100.0, settings.currentMesh, 10.0*settings.scale*settings.scale, settings.scale);
}

const float HOLD_THRESHOLD = 0.5f; 
bool isMouseHeld = false;
bool isContinuousMode = false;
double mouseHoldTime = 0.0;

void handleMouseInput(GLFWwindow* window) {
    static auto lastTime = std::chrono::high_resolution_clock::now();

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        double mouseX, mouseY;
        glfwGetCursorPos(window, &mouseX, &mouseY);

        if (!ImGui::GetIO().WantCaptureMouse) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = currentTime - lastTime;

            if (!isMouseHeld) {
                addObjectAtPosition(static_cast<float>(mouseX), static_cast<float>(mouseY));
                isMouseHeld = true;
                isContinuousMode = false;
                mouseHoldTime = 0.0;
            } else {
                mouseHoldTime += elapsed.count();

                if (mouseHoldTime >= HOLD_THRESHOLD) {
                    isContinuousMode = true;
                }

                if (isContinuousMode && mouseHoldTime >= DT) { 
                    addObjectAtPosition(static_cast<float>(mouseX), static_cast<float>(mouseY));
                    mouseHoldTime = 0.0; 
                }
            }
            lastTime = currentTime;
        }
    } else {
        isMouseHeld = false;
        isContinuousMode = false;
    }
}

int main() {

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    addMesh({
        Vec2(-10, -10),
        Vec2(10, -10),
        Vec2(10, 10),
        Vec2(-10, 10),
    });

    addMesh({
        Vec2(0, -10),
        Vec2(10, 10),
        Vec2(-10, 10),
    });

    // OpenGL version and profile settings
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_FALSE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Physics Engine Debugger", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); 

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");


    setupProjection();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        handleMouseInput(window);

        // Physics simulation update
        world.resetForces(Vec2(0.0, 20.0));
        world.updateVelocities(DT);
        world.updatePositions(DT);
        world.genCollisionPairs();
        world.resolveBorder();
        world.resolveCollisions();
        world.applyCorrections();

        // Clear the screen
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        if(settings.showGrid)
            renderGridLines();
        if(settings.showMeshes)
        {
            for (size_t i = 0; i < world.positions.size(); ++i) {
                if(world.active[i] == 0)
                    continue;
                int meshID = world.meshIDs[i];
                renderMesh(meshID, world.positions[i], world.scales[i]);
            }
        }
        if(settings.showBoundingBoxes)
            renderBoundingBoxes();
        if(settings.showCollisions)
            renderCollisions();

        // ImGui Rendering
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Debug Window");
        ImGui::Text("Physics Engine Debugger");
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        ImGui::Text("Number of Objects: %zu", world.aabbs.size());
        ImGui::Text("Number of Collisions: %zu", world.collisionPairs.size());
        ImGui::Text("Number of Grid Cell: %zu", world.grid.size());
        ImGui::End();

        // ImGui::Begin("Spatial Hashing Grid");
        // for (const auto& [cellIndex, objects] : world.grid) {
        //     std::stringstream ss;
        //     ss << "Cell " << cellIndex << ": ";
        //     for (int id : objects) {
        //         ss << id << " ";
        //     }
        //     ImGui::Text("%s", ss.str().c_str());
        // }
        // ImGui::End();

        ImGui::Begin("Render Options");
        ImGui::Checkbox("Show Meshes", &settings.showMeshes);
        ImGui::Checkbox("Show Bounding Boxes", &settings.showBoundingBoxes);
        ImGui::Checkbox("Show Grid", &settings.showGrid);
        ImGui::Checkbox("Show Collisions", &settings.showCollisions);
        ImGui::Checkbox("Show Normals", &settings.showNormals);
        ImGui::End();

        ImGui::Begin("Shape Selection");
        ImGui::Text("Choose Shape:");

        ImGui::RadioButton("Square", &settings.currentMesh, 0);
        ImGui::RadioButton("Triangle", &settings.currentMesh, 1);
        ImGui::RadioButton("Circle", &settings.currentMesh, 1000);
        ImGui::SliderFloat("Scale", &settings.scale, 0.25f, 4.0f);

        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
