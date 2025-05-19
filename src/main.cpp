
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

#include <rclcpp/serialized_message.hpp>
#include <rclcpp/generic_subscription.hpp>

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
#include <imgui_impl_glfw.h>

class Ros2GLViewer : public guik::Application
{
public:
    Ros2GLViewer(std::shared_ptr<rclcpp::Node> node) : guik::Application(), node_(node)
    {
        std::cout << "Basic Viewer Application initialized" << std::endl;
        // Start ROS2 spinning thread
        last_topic_refresh_ = std::chrono::steady_clock::now() - topic_refresh_interval_ * 2;
        ros_thread_ = std::thread([this]()
                                  {
            while(rclcpp::ok() && !should_exit_) {
                rclcpp::spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } });
    }

    ~Ros2GLViewer()
    {
        // Cleanup
        should_exit_ = true;
        if (ros_thread_.joinable())
        {
            ros_thread_.join();
        }
        rclcpp::shutdown();
    }

    // Mouse‐button → camera_control_.mouse()
    static void MouseButtonCallback(GLFWwindow *w, int button, int action, int mods)
    {
        ImGui_ImplGlfw_MouseButtonCallback(w, button, action, mods);
        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;
        auto self = static_cast<Ros2GLViewer *>(glfwGetWindowUserPointer(w));
        double x, y;
        glfwGetCursorPos(w, &x, &y);
        bool down = (action != GLFW_RELEASE);
        self->camera_control_.mouse({int(x), int(y)}, button, down);
    }

    // Key → camera_control_.key()
    static void KeyCallback(GLFWwindow *w, int key, int sc, int action, int mods)
    {
        ImGui_ImplGlfw_KeyCallback(w, key, sc, action, mods);
        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureKeyboard)
            return;
        if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT)
        {
            auto self = static_cast<Ros2GLViewer *>(glfwGetWindowUserPointer(w));
            double x, y;
            glfwGetCursorPos(w, &x, &y);
            bool down = (action != GLFW_RELEASE);
            // treat shift+drag as middle-button pan
            self->camera_control_.mouse({int(x), int(y)}, 2, down);
        }
    }

    // Cursor movement → camera_control_.drag()
    static void CursorPosCallback(GLFWwindow *w, double x, double y)
    {
        ImGui_ImplGlfw_CursorPosCallback(w, x, y);
        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;
        auto self = static_cast<Ros2GLViewer *>(glfwGetWindowUserPointer(w));
        bool leftDown = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        bool midDown = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        bool shiftDown = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                          glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
        int btn = -1;
        if (shiftDown && leftDown)
            btn = 2; // pan
        else if (leftDown)
            btn = 0; // orbit
        else if (midDown)
            btn = 2; // pan
        if (btn >= 0)
            self->camera_control_.drag({int(x), int(y)}, btn);
    }

    // Scroll wheel → camera_control_.scroll()
    static void ScrollCallback(GLFWwindow *w, double dx, double dy)
    {
        ImGui_ImplGlfw_ScrollCallback(w, dx, dy);
        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;
        auto self = static_cast<Ros2GLViewer *>(glfwGetWindowUserPointer(w));
        self->camera_control_.scroll({float(dx), float(dy)});
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

        // Install free-function callbacks
        glfwSetMouseButtonCallback(win, MouseButtonCallback);
        glfwSetKeyCallback(win, KeyCallback);
        glfwSetCursorPosCallback(win, CursorPosCallback);
        glfwSetScrollCallback(win, ScrollCallback);

        return true;
    }

    void draw_ui() override
    {
        // Show main canvas settings
        main_canvas_->draw_ui();

        // Main options window
        ImGui::Begin("ros2 Viewer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        // FPS counter
        ImGui::Separator();
        ImGui::Text("Swipe/drag pad: orbit");
        ImGui::Text("Scroll pad: zoom");
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

        // Button to toggle Topics window
        if (ImGui::Button("Show ROS2 Topics"))
        {
            // Toggle window visibility
            show_topics_window = !show_topics_window;
            // Refresh topics when opening
            if (show_topics_window)
            {
                refresh_topic_list();
            }
        }

        // Show topics window if enabled
        if (show_topics_window)
        {

            // Check if we need to refresh
            auto now = std::chrono::steady_clock::now();
            if (now - last_topic_refresh_ > topic_refresh_interval_)
            {
                refresh_topic_list();
            }

            // Display topics - just the names
            ImGui::Text("Available ROS2 Topics:");
            ImGui::Separator();

            ImGui::BeginChild("TopicsList", ImVec2(400, 300), true);
            {
                std::lock_guard<std::mutex> lock(topics_mutex_);

                for (const auto &name : topic_names_)
                {
                    // determine alive/red status
                    bool alive = false;
                    auto it = last_msg_time_.find(name);
                    if (it != last_msg_time_.end())
                        alive = (now - it->second) < alive_threshold_;

                    // draw tiny colored circle
                    const float r = 5.0f;
                    ImU32 col = alive
                                    ? IM_COL32(0, 255, 0, 255)  // green
                                    : IM_COL32(255, 0, 0, 255); // red

                    ImGui::Dummy(ImVec2(r * 2, r * 2)); // reserve
                    ImVec2 p = ImGui::GetItemRectMin(); // upper-left
                    ImGui::GetWindowDrawList()
                        ->AddCircleFilled({p.x + r, p.y + r}, r, col); // paint

                    ImGui::SameLine();
                    ImGui::TextUnformatted(name.c_str());
                }
            }
            ImGui::EndChild();

            // Refresh button
            if (ImGui::Button("Refresh Topics"))
            {
                refresh_topic_list();
            }
        }

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

    void refresh_topic_list()
    {
        std::lock_guard<std::mutex> lock(topics_mutex_);

        // Discover all topic→types pairs
        auto topics_and_types = node_->get_topic_names_and_types();

        // Clear & rebuild the list, but only keep topics with >0 publishers
        topic_names_.clear();
        for (auto &kv : topics_and_types)
        {
            const auto &name = kv.first;
            if (node_->count_publishers(name) == 0)
            {
                // no one is publishing → skip
                continue;
            }
            topic_names_.push_back(name);
        }

        // Prune dead subscriptions (topics that disappeared from DDS)
        {
            std::unordered_set<std::string> current(topic_names_.begin(),
                                                    topic_names_.end());
            for (auto it = subs_.begin(); it != subs_.end();)
            {
                if (!current.count(it->first))
                {
                    last_msg_time_.erase(it->first);
                    it = subs_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        // Create heartbeat subscriptions only for the kept topics
        for (auto &name : topic_names_)
        {
            if (!subs_.count(name))
            {
                auto &types = topics_and_types[name];
                if (types.empty())
                    continue; // should never happen
                auto type_name = types[0];

                auto s = node_->create_generic_subscription(
                    name, type_name, rclcpp::QoS(1),
                    [this, name](std::shared_ptr<rclcpp::SerializedMessage>)
                    {
                        std::lock_guard<std::mutex> lk(topics_mutex_);
                        last_msg_time_[name] = std::chrono::steady_clock::now();
                    });
                subs_[name] = s;
                last_msg_time_[name] = std::chrono::steady_clock::time_point{};
            }
        }

        // Stamp the refresh time
        last_topic_refresh_ = std::chrono::steady_clock::now();
    }

private:
    // Canvas
    std::unique_ptr<guik::GLCanvas> main_canvas_;

    // GLK primitives
    const glk::Drawable *grid_;

    // Camara control
    guik::ArcCameraControl camera_control_;

    // ros2 node
    std::shared_ptr<rclcpp::Node> node_;
    std::thread ros_thread_;
    bool should_exit_ = false;

    // For topic discovery
    std::mutex topics_mutex_;
    std::vector<std::string> topic_names_; // Simplified to just store names
    std::chrono::time_point<std::chrono::steady_clock> last_topic_refresh_;
    const std::chrono::milliseconds topic_refresh_interval_{1000};

    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_msg_time_;
    std::chrono::milliseconds alive_threshold_{1000}; // e.g. 1.0s

    bool show_topics_window = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("ros2_gui_viewer");
    // Create application
    std::unique_ptr<Ros2GLViewer> app(new Ros2GLViewer(ros_node));

    // Initialize application
    if (!app->init("Basic Viewer", Eigen::Vector2i(1280, 720)))
    {
        return 1;
    }

    // Run application
    app->run();

    return 0;
}