#include <iostream>
#include <cstdio>
#include <cstddef>
#include <cmath>

#include <vector>
#include <new>
#include <array>

#include <execution>
#include <algorithm>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Tracy.hpp"

#define RAND_FLOAT ((float)rand() / (float)RAND_MAX)

const int max_num_of_shapes = 32768;
const int num_of_shape_types = 4;
const float max_search_range = 0.125f;
const float world_max_x = 1.0f;
const float world_min_x = -1.0f;
const float world_max_y = 1.0f;
const float world_min_y = -1.0f;
const float max_shape_size = 0.005f;
const float speed = 0.1f;
const float target_blend = 0.2f;

constexpr float ratio_triangle = 0.5f;
constexpr float ratio_square = 0.70710678118f;
constexpr float ratio_hexagone = 0.86602580756f;
constexpr float ratio_octogone = 0.92388141058f;

const float PI = 3.14159265358979323846f;

int attractor_type[4] = { 0, 1, 2, 3 };

int totalCollisionCheck = 0;

struct tri_list {
    void set_color(int i, unsigned char r, unsigned char g, unsigned char b);
    void set_position(int i, float x, float y);

    unsigned char m_red[3];
    unsigned char m_green[3];
    unsigned char m_blue[3];
    float m_px[3];
    float m_py[3];
};

void tri_list::set_color(int i, unsigned char r, unsigned char g, unsigned char b) {
    m_red[i] = r;
    m_green[i] = g;
    m_blue[i] = b;
}

void tri_list::set_position(int i, float x, float y) {
    m_px[i] = x;
    m_py[i] = y;
}

struct app {
    app() : m_num_of_shapes(0) {}
    virtual ~app() {}

    virtual int update(float dt, tri_list* tri, int drawMin, int drawMax) = 0;

    virtual void destroy_shapes(int num = max_num_of_shapes) = 0;
    virtual void spawn_triangle(float x, float y, float size) = 0;
    virtual void spawn_rectangle(float x, float y, float size) = 0;
    virtual void spawn_hexagon(float x, float y, float radius) = 0;
    virtual void spawn_octagon(float x, float y, float radius) = 0;

    int m_num_of_shapes;
};

inline int clampi(int v, int min, int max) {
    return v < min ? min : (v > max ? max : v);
}



// void* operator new(size_t size);
// void operator delete(void* p);

// shape
struct point_2d {
    point_2d() : m_x(0), m_y(0) {}
    point_2d(float x, float y) : m_x(x), m_y(y) {}

    float get_x() const { return m_x; }
    float get_y() const { return m_y; }

private:
    float m_x;
    float m_y;
};



struct shape {
    shape(float x, float y, int sideCount);
    shape();
    ~shape();

    void update(float dt);

    void collision_all();

    int draw(tri_list* tri);
    bool test(const shape& shape) const;

    bool is_within(const float x, float y) const;
    int get_type() const;

    bool broadPhaseTest(const shape& shape);
    bool shape::SmallCircleTest(const shape& shape);

    void find_target(const shape& shape);
    void space_partitioning();
    void check_collision(const shape& shape);

    float get_x() const { return m_pos_x; }
    float get_y() const { return m_pos_y; }

#define SPACE_GRID_SIZE 32
    static int cell_size_array[SPACE_GRID_SIZE * SPACE_GRID_SIZE];
    static int cell_start_array[SPACE_GRID_SIZE * SPACE_GRID_SIZE];

    static std::array<shape, max_num_of_shapes> shape_array;
    static std::array<shape, max_num_of_shapes> sorted_shape_array;
    static std::array<int, max_num_of_shapes> shape_index_array;
    static size_t shape_count;

    int grid_index_pos_x;
    int grid_index_pos_y;

    int shape_index;

    float color_r, color_g, color_b;

private:
    float m_pos_x, m_pos_y;
    float m_dir_x, m_dir_y;
    float m_target_x, m_target_y;
    float m_min_distance;
    std::vector<point_2d> m_points;
    int side_count;
    float m_inside_radius;
};


std::array<shape, max_num_of_shapes> shape::shape_array;
std::array<shape, max_num_of_shapes> shape::sorted_shape_array;
std::array<int, max_num_of_shapes> shape::shape_index_array;
int shape::cell_size_array[SPACE_GRID_SIZE * SPACE_GRID_SIZE];
int shape::cell_start_array[SPACE_GRID_SIZE * SPACE_GRID_SIZE];
size_t shape::shape_count = 0;

shape::shape(float x, float y, int sideCount)
    : m_pos_x(x), m_pos_y(y), m_dir_x(RAND_FLOAT), m_dir_y(RAND_FLOAT)
    , m_target_x(0), m_target_y(0), m_min_distance(max_search_range), side_count(sideCount), shape_index(shape_count)
    , color_r((rand() % 255) / 2.f), color_g(rand() % 255), color_b(rand() % 255) {

    shape_index_array[shape_count] = shape_count;

    m_points.reserve(sideCount);
    float radians = 0.0f;
    for (int a = 0; a < sideCount; a++, radians += (PI / sideCount * 2))
    {
        m_points.push_back(point_2d(cosf(radians) * max_shape_size, -sinf(radians) * max_shape_size));
    }

    switch (sideCount)
    {
    case 3:
        m_inside_radius = max_shape_size * ratio_triangle;
        break;
    case 4:
        m_inside_radius = max_shape_size * ratio_square;
        break;

    case 6:
        m_inside_radius = max_shape_size * ratio_hexagone;
        break;

    case 8:
        m_inside_radius = max_shape_size * ratio_octogone;
        break;
    default:

        m_inside_radius = max_shape_size * ratio_triangle;
        break;
    }
}

shape::shape()
{

}

shape::~shape() {
}

int shape::get_type() const
{
    switch (side_count)
    {
    case 3:
        return 0;
    case 4:
        return 1;
    case 6:
        return 2;
    case 8:
        return 3;
    default:
        return 0;
    }
}

bool edge_test(float p0x, float p0y, float p1x, float p1y, float x, float y)
{
    float nx = -(p1y - p0y);
    float ny = p1x - p0x;

    float dot = nx * (x - p0x) + ny * (y - p0y);

    return dot < 0;
}

bool shape::is_within(float x, float y) const
{
    int sum = 0;

    for (int a = 0; a < side_count; a++)
    {
        sum += edge_test(m_pos_x + m_points[a].get_x(), m_pos_y + m_points[a].get_y(),
            m_pos_x + m_points[(a + 1) % side_count].get_x(), m_pos_y + m_points[(a + 1) % side_count].get_y(), x,
            y);
    }

    return sum == side_count;
}


bool shape::test(const shape& shape) const
{
    for (int a = 0; a < side_count; a++)
    {
        if (shape.is_within(m_pos_x + m_points[a].get_x(), m_pos_y + m_points[a].get_y()))
            return true;
    }

    return false;
}

int shape::draw(tri_list* tri)
{

    for (int i = 1; i < side_count - 1; i++)
    {
        tri->set_color(0, color_r, color_g, color_b);
        tri->set_color(1, color_r, color_g, color_b);
        tri->set_color(2, color_r, color_g, color_b);

        tri->set_position(0, m_pos_x + m_points[i].get_x(), m_pos_y + m_points[i].get_y());
        tri->set_position(1, m_pos_x + m_points[i + 1].get_x(), m_pos_y + m_points[i + 1].get_y());
        tri->set_position(2, m_pos_x + m_points[0].get_x(), m_pos_y + m_points[0].get_y());

        tri++;
    }

    return side_count - 2;
}

void shape::find_target(const shape& shape) {
    float delta_x = shape.m_pos_x - m_pos_x;
    float delta_y = shape.m_pos_y - m_pos_y;
    float distance = sqrtf(delta_x * delta_x + delta_y * delta_y);

    if (distance < m_min_distance && shape.get_type() == attractor_type[get_type()]) {
        m_min_distance = distance;
        m_target_x = delta_x / distance;
        m_target_y = delta_y / distance;
    }
    
}

bool shape::broadPhaseTest(const shape& shape) {
    float sqrDist = (shape.m_pos_x - m_pos_x) * (shape.m_pos_x - m_pos_x) + (shape.m_pos_y - m_pos_y) * (shape.m_pos_y - m_pos_y);
    return sqrDist < 4 * max_shape_size * max_shape_size;
}

bool shape::SmallCircleTest(const shape& shape) {
    float sqrDist = (shape.m_pos_x - m_pos_x) * (shape.m_pos_x - m_pos_x) + (shape.m_pos_y - m_pos_y) * (shape.m_pos_y - m_pos_y);
    return sqrDist < 4 * m_inside_radius * shape.m_inside_radius;
}

void shape::check_collision(const shape& shape) {
    if (broadPhaseTest(shape) == false)
        return;

    if (test(shape) || shape.test(*this)) {
        float delta_x = shape.get_x() - m_pos_x;
        float delta_y = shape.get_y() - m_pos_y;

        float length = sqrtf(delta_x * delta_x + delta_y * delta_y);
        m_dir_x = -delta_x / length;
        m_dir_y = -delta_y / length;
    }
}

void shape::space_partitioning()
{
    ZoneScoped;
    int zoneIndexX = (int)((m_pos_x - world_min_x) / (world_max_x - world_min_x) * SPACE_GRID_SIZE);
    grid_index_pos_x = clampi(zoneIndexX, 0, SPACE_GRID_SIZE - 1);

    int zoneIndexY = (int)((m_pos_y - world_min_y) / (world_max_y - world_min_y) * SPACE_GRID_SIZE);
    grid_index_pos_y = clampi(zoneIndexY, 0, SPACE_GRID_SIZE - 1);

    cell_size_array[grid_index_pos_x * SPACE_GRID_SIZE + grid_index_pos_y]++;
}

void ComputeCellIndexArray()
{
    int currentTotal = 0;
    int oldTotal = 0;
    for (size_t i = 0; i < SPACE_GRID_SIZE; i++)
    {
        for (size_t j = 0; j < SPACE_GRID_SIZE; j++)
        {
            currentTotal += shape::cell_size_array[i * SPACE_GRID_SIZE + j];
            shape::cell_start_array[i * SPACE_GRID_SIZE + j] = oldTotal;
            oldTotal = currentTotal;
        }
    }
}

void SortShapeArray()
{
    for (int i = 0; i < shape::shape_count; i++)
    {
        shape::sorted_shape_array[i] = shape::shape_array[shape::shape_index_array[i]];
    }
}


void shape::collision_all()
{
    ZoneScoped;

    for (int i = std::max(0, grid_index_pos_x - 1); i <= std::min(grid_index_pos_x + 1, SPACE_GRID_SIZE - 1); i++)
    {
        for (int j = std::max(0, grid_index_pos_y - 1); j <= std::min(grid_index_pos_y + 1, SPACE_GRID_SIZE - 1); j++)
        {
            //cellIndices.push_back(i * SPACE_GRID_SIZE + j);
            int cellIndex = i * SPACE_GRID_SIZE + j;
            int minIndex = cell_start_array[cellIndex];
            int size = cell_size_array[cellIndex];
            for (int i = minIndex; i < minIndex + size; i++)
            {
                if (shape::sorted_shape_array[i].shape_index != shape_index)
                {
                    find_target(shape::sorted_shape_array[i]);
                    check_collision(shape::sorted_shape_array[i]);
                    //totalCollisionCheck++;
                }

            }
        }
    }
    /*for (int i = 0; i < shape_count; i++)
    {
        if (shape_array[shape_index_array[i]].shape_index != shape_index)
        {
            find_target(shape_array[shape_index_array[i]]);
            check_collision(shape_array[shape_index_array[i]]);
            totalCollisionCheck++;
        }

    }*/
   
}

void shape::update(float dt) {
    ZoneScoped;


    // Blend in target shape position
    m_dir_x = m_dir_x * (1.0f - target_blend) + m_target_x * target_blend;
    m_dir_y = m_dir_y * (1.0f - target_blend) + m_target_y * target_blend;

    // Reset target
    m_min_distance = max_search_range;
    m_target_x = m_dir_x;
    m_target_y = m_dir_y;

    // Normalize direction
    float length = sqrtf(m_dir_x * m_dir_x + m_dir_y * m_dir_y);
    m_dir_x /= length;
    m_dir_y /= length;

    // Move
    m_pos_x += dt * m_dir_x * speed;
    m_pos_y += dt * m_dir_y * speed;

    // Wrap around window frame
    if (m_pos_x > world_max_x)
        m_pos_x -= (world_max_x - world_min_x);
    if (m_pos_x < world_min_x)
        m_pos_x += (world_max_x - world_min_x);
    if (m_pos_y > world_max_y)
        m_pos_y -= (world_max_y - world_min_y);
    if (m_pos_y < world_min_y)
        m_pos_y += (world_max_y - world_min_y);

    /*unsigned char c = ((grid_index_pos_x+ grid_index_pos_y) % 2 == 0) ? 255 : 0;
    color_r = c;
    color_g = c;
    color_b = c;*/

}


struct default_app : public app {
    default_app();
    virtual ~default_app();

    int update(float dt, tri_list* tri, int drawMin, int drawMax) override;

    void destroy_shapes(int num = max_num_of_shapes) override;
    void spawn_triangle(float x, float y, float size) override;
    void spawn_rectangle(float x, float y, float size) override;
    void spawn_hexagon(float x, float y, float radius) override;
    void spawn_octagon(float x, float y, float radius) override;
};

default_app::default_app() {
}

default_app::~default_app() {
    destroy_shapes();
}

void default_app::destroy_shapes(int num) {
    /*for (int i = 0; i < shape::shape_count; i++)
    {
        delete shape::shape_array[i];
    }*/

    //shape::shape_array.fill(nullptr);
    shape::shape_count = 0;
}



void default_app::spawn_triangle(float x, float y, float size) {
    shape::shape_array[shape::shape_count] = shape(x, y, 3);
    shape::shape_count++;
}

void default_app::spawn_rectangle(float x, float y, float size) {
    shape::shape_array[shape::shape_count] = shape(x, y, 4);
    shape::shape_count++;
}

void default_app::spawn_hexagon(float x, float y, float radius) {
    shape::shape_array[shape::shape_count] = shape(x, y, 6);
    shape::shape_count++;
}

void default_app::spawn_octagon(float x, float y, float radius) {
    shape::shape_array[shape::shape_count] = shape(x, y, 8);
    shape::shape_count++;
}

int default_app::update(float dt, tri_list* tri, int drawMin, int drawMax) {
    ZoneScoped;
    int tri_count = 0;

    totalCollisionCheck = 0;

    {
        ZoneScopedN("Clear Grid");
        // Clean cell size array, cell start array doesn't need to.
        for (size_t i = 0; i < SPACE_GRID_SIZE; i++)
        {
            for (size_t j = 0; j < SPACE_GRID_SIZE; j++)
            {
                shape::cell_size_array[i * SPACE_GRID_SIZE + j] = 0;
            }
        }
    }
    
    {
        ZoneScopedN("Space Partitioning");
        
        // Place each shape in the corresponding cell.
        for (int i = 0; i < shape::shape_count; i++) {
            shape::shape_array[i].space_partitioning();
        }
    }
    
    {
        ZoneScopedN("Compute Cell index array");
        // Compute cell start array
        ComputeCellIndexArray();
    }
    

    {
        ZoneScopedN("Sort Array");
        // Sort the shape array by cell indices
        if (shape::shape_count != 0)
        {
            /*std::sort(shape::shape_array.begin(), shape::shape_array.begin() + shape::shape_count,
                [](const shape a, const shape b) {
                    if (a.grid_index_pos_x != b.grid_index_pos_x)
                        return a.grid_index_pos_x < b.grid_index_pos_x;
                    return a.grid_index_pos_y < b.grid_index_pos_y;
                }
            );*/

            std::sort(shape::shape_index_array.begin(), shape::shape_index_array.begin() + shape::shape_count,
                [](const int a, const int b) {
                    if (shape::shape_array[a].grid_index_pos_x != shape::shape_array[b].grid_index_pos_x)
                        return shape::shape_array[a].grid_index_pos_x < shape::shape_array[b].grid_index_pos_x;
                    return shape::shape_array[a].grid_index_pos_y < shape::shape_array[b].grid_index_pos_y;
                }
            );
        }
    }
    
    SortShapeArray();

    {
        ZoneScopedN("Check collision");

        std::for_each(std::execution::par_unseq, shape::shape_array.begin(), shape::shape_array.begin() + shape::shape_count, [](shape& s)
            {
                s.collision_all();
            });

        // Collisions
        /*for (int i = 0; i < shape::shape_count; i++) {
            shape::shape_array[i].collision_all();
        }*/
    }
    

    {
        ZoneScopedN("Update and draw");
        // Update and draw shapes
        for (int i = 0; i < shape::shape_count; i++)
        {
            shape::shape_array[i].update(dt);

            /*if (shape::shape_array[i]->grid_index_pos_x != drawMin || shape::shape_array[i]->grid_index_pos_y != drawMax)
                continue;*/
            tri_count += shape::shape_array[i].draw(&tri[tri_count]);
        }
    }
    

    return tri_count;
}

struct factory_builder
{
    app* (*func)();
    const char* name;
}

factories[] =
{
    { []() -> app* { return new default_app(); }, "default" }
};

app* factory(int index) {
    if (index < 0 || index >= (sizeof(factories) / sizeof(factories[0])))
        return nullptr;

    return factories[index].func();
}

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

const char* glsl_version = "#version 330";

int main(int ac, char* av[]) {


    GLFWwindow* window;

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        fprintf(stderr, "Could not initialize GLFW.\n");
        return EXIT_FAILURE;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    window = glfwCreateWindow(640, 480, "ISART Workshop", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable VSync
    //
    gladLoadGL();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    bool show_demo_window = false;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    io.IniFilename = nullptr;

    app* current_app = factory(0);
    std::vector<tri_list> triangle_list;
    int tri_count = 0;
    int quad_count = 0;
    int hexa_count = 0;
    int octa_count = 0;

    const char* vtx_shader_src =
        "#version 330 core\n"
        "layout(location = 0) in vec2 vtxPos;\n"
        "layout(location = 1) in vec3 vtxColor;\n"
        "out vec3 fragColor;\n"
        "void main() {\n"
        "   gl_Position = vec4(vtxPos, 0.0, 1.0);\n"
        "   fragColor = vtxColor;\n"
        "}\n";

    const char* pix_shader_src =
        "#version 330 core\n"
        "in vec3 fragColor;\n"
        "out vec4 color;\n"
        "void main() {\n"
        "   color = vec4(fragColor, 1.0);\n"
        "}\n";

    GLuint vtx_shader_id = glCreateShader(GL_VERTEX_SHADER);
    GLuint pix_shader_id = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vtx_shader_id, 1, &vtx_shader_src, nullptr);
    glShaderSource(pix_shader_id, 1, &pix_shader_src, nullptr);

    glCompileShader(vtx_shader_id);
    glCompileShader(pix_shader_id);

    GLuint prog_id = glCreateProgram();
    glAttachShader(prog_id, vtx_shader_id);
    glAttachShader(prog_id, pix_shader_id);
    glLinkProgram(prog_id);

    glDetachShader(prog_id, pix_shader_id);
    glDetachShader(prog_id, vtx_shader_id);

    glDeleteShader(vtx_shader_id);
    glDeleteShader(pix_shader_id);

    GLuint vao_id;
    glGenVertexArrays(1, &vao_id);
    glBindVertexArray(vao_id);

    GLuint vbo_id;
    glGenBuffers(1, &vbo_id);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id);


    int drawMin = 0;
    int drawMax = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (show_demo_window) {
            ImGui::ShowDemoWindow(&show_demo_window);
        }

        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!");
            ImGui::Checkbox("Demo Window", &show_demo_window);
            ImGui::Checkbox("Another Window", &show_another_window);

            ImGui::ColorEdit3("clear color", (float*)&clear_color);

            if (ImGui::Button("Button")) {
                counter++;
            }
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::End();
        }

        //triangle_list.clear();

        // fill update, draw and stats here
        if (current_app) {

            ImGui::Begin("Shapes");
            ImGui::DragInt("Draw Min", &drawMin, 1, 0, shape::shape_count);
            ImGui::DragInt("Draw Max", &drawMax, 1, 0, shape::shape_count);

            ImGui::Text("Average collision check : %f", (float)totalCollisionCheck / (float)shape::shape_count);

            ImGui::Text("Triangles: %d", tri_count);
            ImGui::SameLine();
            if (ImGui::Button("triangleAdd##Add")) {
                tri_count += 1000;
                for (size_t i = 0; i < 1000; i++)
                {
                    current_app->spawn_triangle(((float)rand() / RAND_MAX) * 4, ((float)rand() / RAND_MAX) * -2, max_shape_size);
                }

                triangle_list.resize(triangle_list.size() + 1000);
            }
            ImGui::Text("Rectangles: %d", quad_count);
            ImGui::SameLine();
            if (ImGui::Button("quadAdd##Add")) {
                quad_count++;
                current_app->spawn_rectangle(0.0f, 0.0f, max_shape_size);
                triangle_list.resize(triangle_list.size() + 2);
            }
            ImGui::Text("Hexagons: %d", hexa_count);
            ImGui::SameLine();
            if (ImGui::Button("hexaAdd##Add")) {
                hexa_count++;
                current_app->spawn_hexagon(0.0f, 0.0f, max_shape_size);
                triangle_list.resize(triangle_list.size() + 4);
            }
            ;
            ImGui::Text("Octagons: %d", octa_count);
            ImGui::SameLine();
            if (ImGui::Button("octaAdd##Add"))
            {
                octa_count++;
                current_app->spawn_octagon(0.0f, 0.0f, max_shape_size);
                triangle_list.resize(triangle_list.size() + 6);
            }

            if (ImGui::Button("Reset all")) {
                tri_count = 0;
                quad_count = 0;
                hexa_count = 0;
                octa_count = 0;
                current_app->destroy_shapes();
                triangle_list.resize(0);
            }
            ImGui::End();

            current_app->update(io.DeltaTime, triangle_list.data(), drawMin, drawMax);
        }

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        if (!triangle_list.empty()) {
            struct vtx
            {
                float x, y, r, g, b;
            };
            std::vector<vtx> verts;
            verts.reserve(triangle_list.size() * 3);
            for (const auto& t : triangle_list) {
                for (int v = 0; v < 3; v++) {
                    verts.push_back({
                        t.m_px[v], t.m_py[v],
                        t.m_red[v] * (1.f / 255.0f),
                        t.m_green[v] / 255.0f,
                        t.m_blue[v] / 255.0f
                        });
                }
            }

            glUseProgram(prog_id);
            glBindVertexArray(vao_id);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_id);

            glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(vtx),
                verts.data(), GL_DYNAMIC_DRAW);

            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
                sizeof(vtx), (void*)offsetof(vtx, x));

            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                sizeof(vtx), (void*)offsetof(vtx, r));

            glDrawArrays(GL_TRIANGLES, 0, (GLsizei)verts.size());

            glDisableVertexAttribArray(0);
            glDisableVertexAttribArray(1);
        }
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        FrameMark;
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
