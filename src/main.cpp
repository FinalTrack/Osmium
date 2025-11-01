#include "imgui/imgui.h"
#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include "engine/Engine.hpp"
#include <sstream>
#include <random>
#include <functional>
#include <set>

const int WIDTH = 1200;
const int HEIGHT = 800;
const float DT = 0.016f; 
const float PI = 3.14159265f;
World world(WIDTH, HEIGHT);

struct Settings
{
    bool showMeshes = true;
    bool showBoundingBoxes = false;
    bool showGrid = false;
    bool showCollisions = false;
    int currentMesh = 0;
    float scale = 1.0f;
    float restitution = 0.7f;
} settings;

void setupProjection() {
    glViewport(0, 0, WIDTH, HEIGHT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, WIDTH, HEIGHT, 0.0, -1.0, 1.0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void renderGridLines() 
{
    std::set<std::array<int, 4>> rects;
    for(int id = 0; id < world.allocated; id++)
    {
        if(world.bodies[id].active != 1)
            continue;
        int lvl = world.bodies[id].level;
        int sz = world.quad.length >> lvl;
        int cnt = 1 << lvl;
        
        int offset = world.bodies[id].ind - world.quad.levels[lvl];
        int x = offset % cnt;
        int y = offset / cnt;

        rects.insert({lvl, sz, x, y});
    }

    for(auto [lvl, sz, x, y]: rects)
    {
        float col = 0.05 * lvl;
        glColor3f(col, col, col); 
        glRectf(x * sz, y * sz, (x+1)*sz, (y+1)*sz);
    }
}

void renderBoundingBoxes() {

    glBegin(GL_LINES);
    glColor3f(0.0f, 1.0f, 0.0f);  

    for (const Body& body: world.bodies) {
        if(body.active == 0)
            continue;

        const AABB& aabb = body.aabb;

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

void renderMesh(const Body& body, float mx, float my) {
    const int meshID = body.meshID;

    if(body.active == 2)
        glColor3f(0.5f, 0.0f, 1.0f);
    else if(body.contains(Vec2(mx, my)))
        glColor3f(0.0f, 0.5f, 1.0f);
    else
        glColor3f(0.0f, 0.0f, 1.0f);

    if (meshID == 1000) { 
        const float radius = meshdata::RADIUS * body.scale; 
        const int segments = 32;

        // Draw circle
        glBegin(GL_LINE_LOOP);

        for (int i = 0; i < segments; ++i) {
            float angle = 2.0f * PI * static_cast<float>(i) / segments;
            float x = body.position.x + radius * std::cos(angle);
            float y = body.position.y + radius * std::sin(angle);
            glVertex2f(x, y);
        }
        glEnd();

        Vec2 dir = Vec2(body.cosTheta, body.sinTheta) * radius;
        Vec2 end = body.position + dir;

        glBegin(GL_LINES);
        glVertex2f(body.position.x, body.position.y);
        glVertex2f(end.x, end.y);
        glEnd();

        return;
    }

    const std::vector<Vec2>& transformed = body.transformed;
    int n = transformed.size();

    // Draw polygon edges
    glBegin(GL_LINES);

    for (int i = 0; i < n; ++i) {
        const Vec2& p1 = transformed[i];
        const Vec2& p2 = transformed[(i + 1) % n];
        glVertex2f(p1.x, p1.y);
        glVertex2f(p2.x, p2.y);
    }
    glEnd();
}

void renderCollisions() {
    glColor3f(1.0f, 1.0f, 0.0f); 

    glBegin(GL_LINES);

    for (const auto& result : world.collisionData) {
        if (!result.collide) continue;

        for (int i = 0; i < result.collide; ++i) {
            const Vec2 pt = result.contact[i];
            Vec2 st = pt - result.normal * 5.0f;
            Vec2 end = pt + result.normal * 5.0f;

            glVertex2f(st.x, st.y);    
            glVertex2f(end.x, end.y);
        }
    }

    glEnd();
}

std::mt19937 rng(std::random_device{}());
std::uniform_real_distribution<float> velocityDist(-1.0f, 1.0f);
std::uniform_real_distribution<float> rang(0.0f, 2*PI);

float getUnit(int mid)
{
    if(mid == 2)
        return 2;
    if(mid == 3)
        return 2;
    return 1;
}

void addObjectAtPosition(float x, float y) 
{
    Vec2 position(x, y);
    float r = 10.0f * settings.scale * getUnit(settings.currentMesh);
    float d = 1.0;
    float mass = d * r * r;
    float moi = mass * r * r;
    float vx = velocityDist(rng);
    float vy = velocityDist(rng);
    world.addBody(position, Vec2(vx, vy) * 0.0, settings.currentMesh, mass, moi, settings.scale, 0.0f, settings.restitution);
}

float uTime = 0.0f, cTime = 0.0f, rTime = 0.0f;
float uAvg = 0.0f, cAvg = 0.0f, rAvg = 0.0f;
int frame = 0;
const int sample = 60;



int main() {

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    meshdata::addMesh({
        Vec2(-10, -10),
        Vec2(10, -10),
        Vec2(10, 10),
        Vec2(-10, 10),
    });

    meshdata::addMesh({
        Vec2(0, -10),
        Vec2(10, 10),
        Vec2(-10, 10),
    });

    meshdata::addMesh({
        Vec2(-20, -20),
        Vec2(-6, -20),
        Vec2(16, -10),
        Vec2(20, 15),
        Vec2(12, 20),
        Vec2(-16, 20),
        Vec2(-20, 16),
    });

    meshdata::addMesh({
        Vec2(-40, 0),
        Vec2(40, 20),
        Vec2(-40, 20),
    });

    meshdata::addMesh({
        Vec2(-10, -10),
        Vec2(10, -10),
        Vec2(20, 10),
        Vec2(-20, 10),
    });

    meshdata::addMesh({
        Vec2(-(WIDTH - 100)/2, -20),
        Vec2((WIDTH - 100)/2, -20),
        Vec2((WIDTH - 100)/2, 20),
        Vec2(-(WIDTH - 100)/2, 20),
    });
    meshdata::addMesh({
        Vec2(-20, -(HEIGHT - 100)/2),
        Vec2(20, -(HEIGHT - 100)/2),
        Vec2(20, (HEIGHT - 100)/2),
        Vec2(-20, (HEIGHT - 100)/2),
    });

    world.addBody(Vec2(WIDTH/2, HEIGHT - 105), 5, 1.0f, 0.0f, 0.2f);
    world.addBody(Vec2(105, HEIGHT/2), 6, 1.0f, 0.0f, 0.2f);
    world.addBody(Vec2(WIDTH-105, HEIGHT/2), 6, 1.0f, 0.0f, 0.2f);
    world.addBody(Vec2(WIDTH/2, HEIGHT/2), 1000, 10.0f, 0.0f, 0.2f);
    world.addBody(Vec2(260, 640), 3, 5.0f, 0.0f, 0.2f);

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

    int mousehold = 0;
    Engine engine(8, &world);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImVec2 mouse = io.MousePos;
        float mouseX = mouse.x;
        float mouseY = mouse.y;

        if(!io.WantCaptureKeyboard && !io.WantCaptureMouse)
        {
            if(ImGui::IsMouseDown(ImGuiMouseButton_Left)) 
                mousehold++;
            else    
                mousehold = 0;
            if(ImGui::IsMouseClicked(ImGuiMouseButton_Left) || (mousehold > sample && mousehold % 4 == 0)) {
                addObjectAtPosition(mouseX, mouseY);
            }

            if(ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                for(int i = 0; i < world.allocated; i++)
                {
                    if(world.bodies[i].active != 1)
                        continue;
                    if(world.bodies[i].contains(Vec2(mouseX, mouseY)))
                        world.deleteBody(i);
                }
            }
        }
        world.resetForces(Vec2(0.0, 20.0));
        float tu, tc, tr;
        engine.updateStep(DT, tu, tc, tr);
        uTime += tu;
        cTime += tc;
        rTime += tr;

        ++frame;
        if (frame > sample) {
            frame = 0;
            uAvg = uTime / sample;
            cAvg = cTime / sample;
            rAvg = rTime / sample;

            uTime = cTime = rTime = 0.0f;
        }

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLineWidth(1.0f);  



        if(settings.showGrid)
            renderGridLines();
        if(settings.showMeshes)
        {
            for(const Body& b: world.bodies)
            {
                if(b.active == 0)
                    continue;
                renderMesh(b, mouseX, mouseY);
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
        ImGui::Text("Active Objects: %zu", world.activeCount);
        ImGui::Text("Allocated Objects: %zu", world.allocated);
        ImGui::Text("Intersection Pairs: %zu", world.collisionPairs.size());
        ImGui::Text("Collision Pairs: %zu", world.colCnt);
        ImGui::End();

        ImGui::Begin("Render Options");
        ImGui::Checkbox("Show Meshes", &settings.showMeshes);
        ImGui::Checkbox("Show Bounding Boxes", &settings.showBoundingBoxes);
        ImGui::Checkbox("Show Grid", &settings.showGrid);
        ImGui::Checkbox("Show Collisions", &settings.showCollisions);
        ImGui::End();

        ImGui::Begin("Shape Selection");
        ImGui::RadioButton("Square", &settings.currentMesh, 0);
        ImGui::RadioButton("Triangle", &settings.currentMesh, 1);
        ImGui::RadioButton("Rock", &settings.currentMesh, 2);
        ImGui::RadioButton("Ramp", &settings.currentMesh, 3);
        ImGui::RadioButton("Trapezoid", &settings.currentMesh, 4);
        ImGui::RadioButton("Circle", &settings.currentMesh, 1000);
        ImGui::SliderFloat("Scale", &settings.scale, 0.25f, 4.0f);
        ImGui::SliderFloat("Restitution", &settings.restitution, 0.0f, 1.0f);

        ImGui::End();

        ImGui::Begin("Performance");

        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        ImGui::Text("Update: %.2f µs", uAvg);
        ImGui::Text("Collision: %.2f µs", cAvg);
        ImGui::Text("Resolve: %.2f µs", rAvg);
        ImGui::Text("Total: %.2f µs", uAvg + cAvg + rAvg);

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
