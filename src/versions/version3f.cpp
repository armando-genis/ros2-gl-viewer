
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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.h>

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
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::durationFromSec(10.0));

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
        // get the current time
        auto now = std::chrono::steady_clock::now();
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
        }

        if (ImGui::Button("Select Reference Frame"))
        {
            show_frames_window = !show_frames_window;
            if (show_frames_window)
            {
                refresh_frame_list();
            }
        }

        ImGui::SameLine();
        ImGui::Text("Current frame: %s", fixed_frame_.c_str());

        // Show frames window if enabled
        if (show_frames_window)
        {

            // Check if we need to refresh
            if (now - last_frame_refresh_ > frame_refresh_interval_)
            {
                refresh_frame_list();
            }

            // Display frames and allow selection
            ImGui::Text("Available Frames:");
            ImGui::Separator();

            ImGui::BeginChild("FramesList", ImVec2(400, 100), true);
            for (const auto &frame : available_frames_)
            {
                if (ImGui::Selectable(frame.c_str(), fixed_frame_ == frame))
                {
                    fixed_frame_ = frame;
                }
            }
            ImGui::EndChild();
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
    }

    void refresh_frame_list()
    {
        std::lock_guard<std::mutex> lock(frames_mutex_);

        try
        {
            // Get the complete YAML representation of the TF tree
            std::string yaml_string = tf_buffer_->allFramesAsYAML();

            // Clear previous frame list
            available_frames_.clear();

            // Parse the YAML to extract frame names and relationships
            if (!yaml_string.empty())
            {

                std::istringstream yaml_stream(yaml_string);
                std::string line;
                std::string current_frame;

                while (std::getline(yaml_stream, line))
                {
                    // Line with frame name
                    if (!line.empty() && line[0] != ' ' && line[0] != '\t' && line.find(':') != std::string::npos)
                    {
                        current_frame = line.substr(0, line.find(':'));
                        available_frames_.push_back(current_frame);
                        std::cout << "Found frame: " << current_frame << std::endl;
                    }

                    // Line with parent info
                    if (line.find("parent:") != std::string::npos)
                    {
                        size_t start = line.find('\'');
                        size_t end = line.find('\'', start + 1);
                        if (start != std::string::npos && end != std::string::npos)
                        {
                            std::string parent = line.substr(start + 1, end - start - 1);
                            std::cout << "Found parent reference: " << parent << std::endl;

                            // add parent to available frames if not already present
                            if (std::find(available_frames_.begin(), available_frames_.end(), parent) == available_frames_.end())
                            {
                                available_frames_.push_back(parent);
                                std::cout << green << "Adding missing parent frame: " << parent << reset << std::endl;
                            }
                        }
                    }
                }
            }

            // Print discovered frames
            std::cout << "All frames (" << available_frames_.size() << "): ";
            for (const auto &f : available_frames_)
            {
                std::cout << f << "  ";
            }
            std::cout << std::endl;

            // Check for important frames
            bool has_base_link = (std::find(available_frames_.begin(), available_frames_.end(), "base_link") != available_frames_.end());

            if (!has_base_link)
            {
                std::cout << red << "WARNING: 'base_link' frame is missing but likely referenced as parent!" << reset << std::endl;
                std::cout << yellow << "Consider running: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link" << reset << std::endl;
            }

            // If frames were found, check TF tree integrity
            if (!available_frames_.empty())
            {
                check_frame_connectivity();
            }
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get frames: %s", ex.what());
            return;
        }

        last_frame_refresh_ = std::chrono::steady_clock::now();
    }

    void check_frame_connectivity()
    {
        std::cout << "Checking frame connectivity for critical transforms:" << std::endl;

        // Check the most important transformations
        std::vector<std::pair<std::string, std::string>> critical_transforms;

        // If base_link exists and we have other frames, check connectivity
        auto it_base = std::find(available_frames_.begin(), available_frames_.end(), "base_link");
        if (it_base != available_frames_.end())
        {
            for (const auto &frame : available_frames_)
            {
                if (frame != "base_link")
                {
                    critical_transforms.push_back({"base_link", frame});
                }
            }
        }

        // Check connectivity for each critical transform
        for (const auto &pair : critical_transforms)
        {
            bool can_transform = false;
            std::string reason;

            try
            {
                // First try direct transform
                can_transform = tf_buffer_->canTransform(pair.second, pair.first, tf2::TimePointZero);

                if (can_transform)
                {

                    auto transform = tf_buffer_->lookupTransform(pair.second, pair.first, tf2::TimePointZero);
                    auto &t = transform.transform.translation;
                }
                else
                {
                    reason = "canTransform returned false";
                }
            }
            catch (const tf2::TransformException &ex)
            {
                reason = ex.what();
            }

            if (!can_transform)
            {
                std::cout << red << "Transform " << pair.first << " -> " << pair.second
                          << ": FAILED (" << reason << ")" << reset << std::endl;
            }
        }
    }

private:
    // color for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

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

    // TF2 for frame transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Currently selected frame
    std::mutex frames_mutex_;
    std::string fixed_frame_ = "map";
    std::vector<std::string> available_frames_;

    // For frame selection UI
    bool show_frames_window = false;
    std::chrono::milliseconds frame_refresh_interval_{1000};
    std::chrono::time_point<std::chrono::steady_clock> last_frame_refresh_;

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