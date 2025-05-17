
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <iostream>
#include <chrono>
#include <thread>

// ImGui includes
#include <imgui.h>
#include <imgui_internal.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// GLK and GUIK libraries
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/lines.hpp>
#include <glk/colormap.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>

#include <guik/gl_canvas.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <GLFW/glfw3.h>

class Ros2GLViewer : public guik::Application
{
public:
    Ros2GLViewer() : guik::Application()
    {
        std::cout << "Basic Viewer Application initialized" << std::endl;
    }

    ~Ros2GLViewer()
    {
        // Cleanup
    }

    bool init(const char *window_name, const Eigen::Vector2i &size, const char *glsl_version = "#version 330") override
    {
        if (!guik::Application::init(window_name, size, glsl_version))
        {
            return false;
        }

        // Get shader directory
        std::string data_directory = "./data";

        // Initialize the main OpenGL canvas
        main_canvas_.reset(new guik::GLCanvas(data_directory, framebuffer_size()));
        if (!main_canvas_->ready())
        {
            std::cerr << "Failed to initialize GL canvas" << std::endl;
            close();
            return false;
        }

        // Create grid
        grid_ = &glk::Primitives::instance()->primitive(glk::Primitives::GRID);

        // --- install GLFW callbacks ---
        GLFWwindow *win = this->window;
        glfwSetWindowUserPointer(win, this);

        // Mouse‐button → camera_control_.mouse()
        glfwSetMouseButtonCallback(win, [](GLFWwindow *w, int button, int action, int /*mods*/)
                                   {
            auto self = static_cast<Ros2GLViewer*>(glfwGetWindowUserPointer(w));
            double x, y;
            glfwGetCursorPos(w, &x, &y);
            bool down = (action != GLFW_RELEASE);
            self->camera_control_.mouse({ int(x), int(y) }, button, down); });

        glfwSetKeyCallback(win, [](GLFWwindow *w, int key, int /*sc*/, int action, int /*mods*/)
                           {
            if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT) {
                auto self = static_cast<Ros2GLViewer*>(glfwGetWindowUserPointer(w));
                double x,y; glfwGetCursorPos(w, &x, &y);
                bool down = (action != GLFW_RELEASE);
                self->camera_control_.mouse({int(x),int(y)}, 2, down);
            } });

        // Cursor movement → camera_control_.drag()
        glfwSetCursorPosCallback(win, [](GLFWwindow *w, double x, double y)
                                 {
            auto self = static_cast<Ros2GLViewer*>(glfwGetWindowUserPointer(w));
            bool leftDown = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
            bool midDown = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
            bool shiftDown = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                              glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

            // detect which condition is held
            int btn = -1;
            if (shiftDown && leftDown)
            {
                btn = 2;
            }
            else if (leftDown)
            {
                btn = 0;
            }
            else if (midDown)
            {
                btn = 2;
            }
            
            if (btn >= 0) {
                self->camera_control_.drag({ int(x), int(y) }, btn);
            } });

        // Scroll wheel → camera_control_.scroll()
        glfwSetScrollCallback(win, [](GLFWwindow *w, double dx, double dy)
                              {
            auto self = static_cast<Ros2GLViewer*>(glfwGetWindowUserPointer(w));
            self->camera_control_.scroll({ float(dx), float(dy) }); });

        return true;
    }

    void draw_ui() override
    {
        // Show main canvas settings
        main_canvas_->draw_ui();

        // Main options window
        ImGui::Begin("Basic Viewer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        // FPS counter
        ImGui::Separator();
        ImGui::Text("Swipe/drag pad: orbit");
        ImGui::Text("Scroll pad: zoom");
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        ImGui::End();
    }

    void draw_gl() override
    {
        main_canvas_->bind();

        // Clear the canvas
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // upload camera view
        Eigen::Matrix4f view = camera_control_.view_matrix();
        main_canvas_->shader->set_uniform("view_matrix", view);

        // Draw coordinate system
        main_canvas_->shader->set_uniform("color_mode", 2);
        main_canvas_->shader->set_uniform("model_matrix", (Eigen::UniformScaling<float>(2.0f) * Eigen::Isometry3f::Identity()).matrix());
        const auto &coord = glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM);
        coord.draw(*main_canvas_->shader);

        // Draw grid on the floor
        main_canvas_->shader->set_uniform("color_mode", 1);
        main_canvas_->shader->set_uniform("model_matrix", (Eigen::Translation3f(Eigen::Vector3f::UnitZ() * -0.02) * Eigen::Isometry3f::Identity()).matrix());
        main_canvas_->shader->set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
        grid_->draw(*main_canvas_->shader);

        // Add your custom rendering here

        main_canvas_->unbind();
        main_canvas_->render_to_screen();
    }

    void framebuffer_size_callback(const Eigen::Vector2i &size) override
    {
        main_canvas_->set_size(size);
    }

private:
    // Canvas
    std::unique_ptr<guik::GLCanvas> main_canvas_;

    // GLK primitives
    const glk::Drawable *grid_;

    // Camara control
    guik::ArcCameraControl camera_control_;
};

int main(int, char **)
{
    // Create application
    std::unique_ptr<Ros2GLViewer> app(new Ros2GLViewer());

    // Initialize application
    if (!app->init("Basic Viewer", Eigen::Vector2i(1280, 720)))
    {
        return 1;
    }

    // Run application
    app->run();

    return 0;
}