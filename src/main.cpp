// OpenGL Version: 4.6 (Compatibility Profile) Mesa 23.2.1-1ubuntu3.1~22.04.3
// GLSL Version: 4.60

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>

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
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// GLK and GUIK libraries
#include <glk/primitives/primitives.hpp>
#include <glk/lines.hpp>
#include <glk/colormap.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/TextRendering.hpp>
#include <glk/PathRendering.hpp>
#include <glk/CloudRendering.hpp>
#include "glk/PclLoader.hpp"
#include "glk/LaneletLoader.hpp"
#include "glk/modelUpload.hpp"
#include "glk/ThickLines.hpp"

#include <guik/gl_canvas.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <GL/glut.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <regex>

#include <stb_image.h>

#include "glk/MapBoxRendering.hpp"

static Eigen::Isometry3f toEigen(const geometry_msgs::msg::Transform &t)
{
    Eigen::Quaternionf q(t.rotation.w,
                         t.rotation.x,
                         t.rotation.y,
                         t.rotation.z);
    Eigen::Vector3f tr(t.translation.x,
                       t.translation.y,
                       t.translation.z);
    Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
    iso.rotate(q);
    iso.pretranslate(tr);
    return iso;
}

struct CloudTopic
{
    std::string topic_name;
    std::string frame_id;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    std::vector<Eigen::Vector3f> debug_pts;

    GLuint vao = 0;
    GLuint vbo = 0;
    size_t num_points = 0;
};

struct MarkerArrayTopic
{
    std::string topic_name;
    std::string frame_id;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub;
    visualization_msgs::msg::MarkerArray::SharedPtr marker_array;
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    std::chrono::steady_clock::time_point last_update;
};

struct PathTopic
{
    std::string topic_name;
    std::string frame_id;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub;
    nav_msgs::msg::Path::SharedPtr path;
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    std::chrono::steady_clock::time_point last_update;

    // Path type: 0 = normal line, 1 = car path (ribbon with width)
    int path_type = 0;
    float path_width = 1.5f; // Width in meters for car path

    // Color mode for car path: 0 = flat color, 1 = gradient (center to edges), 2 = gradient (edges to center)
    int color_mode = 0;
    
    // Path color selection: 0 = orange, 1 = blue
    int path_color_selection = 0;

    // OpenGL resources for rendering the path
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint color_vbo = 0; // Separate VBO for vertex colors (gradient mode)
    size_t num_points = 0;
};

class Ros2GLViewer : public guik::Application
{
public:
    Ros2GLViewer(std::shared_ptr<rclcpp::Node> node) : guik::Application(), node_(node)
    {
        std::cout << "Basic Viewer Application initialized" << std::endl;
        // Initialize active camera to right camera by default
        active_camera_ = &right_camera_control_;

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

        text_renderer_.cleanupTextRendering();
        path_renderer_.cleanupPathRendering();
        cloud_renderer_.cleanupCloudRendering();
        // Cleanup GLB shader
        if (glb_shader_program_ != 0)
        {
            glDeleteProgram(glb_shader_program_);
        }

        for (auto &icon : dock_icons_)
        {
            if (icon.texture_id != 0)
            {
                glDeleteTextures(1, &icon.texture_id);
            }
        }

        // Cleanup ROS2
        should_exit_ = true;
        if (ros_thread_.joinable())
        {
            ros_thread_.join();
        }
        rclcpp::shutdown();

        // Cleanup TF2
        if (tf_buffer_ != nullptr)
        {
            tf_buffer_->clear();
        }
        if (tf_listener_ != nullptr)
        {
            tf_listener_->~TransformListener();
        }
    }

    void setupCustomImGuiColors()
    {

        // color pallete generetor: https://colorkit.co/color-palette-generator/090b07-11150d-192014-212b1a-314026-526a40-739559-94bf73-a5d580-b5ea8c/
        // color set to hex to 1.0 rgb: https://rgbcolorpicker.com/0-1#google_vignette

        ImGuiStyle &style = ImGui::GetStyle();

        // ===== SPECIAL EFFECTS =====
        style.WindowMenuButtonPosition = ImGuiDir_Left; // Window menu button position
        style.ColorButtonPosition = ImGuiDir_Right;     // Color button position
        style.Alpha = 1.0f;                             // Global alpha multiplier
        style.DisabledAlpha = 0.5f;                     // Alpha for disabled items
    }

    // Mouse‐button → active_camera_.mouse()
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

        // Don't handle camera movement if we're dragging the splitter
        if (self->is_dragging_splitter_)
        {
            return;
        }

        // Determine which camera to use based on mouse position
        if (self->split_screen_mode_)
        {
            int split_x = static_cast<int>(self->framebuffer_size().x() * self->split_ratio_);
            if (x < split_x)
            {
                self->active_camera_ = &self->left_camera_control_;
            }
            else
            {
                self->active_camera_ = &self->right_camera_control_;
            }
        }
        else
        {
            self->active_camera_ = &self->right_camera_control_;
        }

        self->active_camera_->mouse({int(x), int(y)}, button, down);
    }

    // Key → active_camera_.key()
    static void KeyCallback(GLFWwindow *w, int key, int sc, int action, int mods)
    {
        ImGui_ImplGlfw_KeyCallback(w, key, sc, action, mods);
        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureKeyboard)
            return;
        if (key == GLFW_KEY_LEFT_SHIFT || key == GLFW_KEY_RIGHT_SHIFT)
        {
            auto self = static_cast<Ros2GLViewer *>(glfwGetWindowUserPointer(w));

            // Don't handle camera movement if we're dragging the splitter
            if (self->is_dragging_splitter_)
            {
                return;
            }

            double x, y;
            glfwGetCursorPos(w, &x, &y);
            bool down = (action != GLFW_RELEASE);

            // Determine which camera to use based on mouse position
            if (self->split_screen_mode_)
            {
                int split_x = static_cast<int>(self->framebuffer_size().x() * self->split_ratio_);
                if (x < split_x)
                {
                    self->active_camera_ = &self->left_camera_control_;
                }
                else
                {
                    self->active_camera_ = &self->right_camera_control_;
                }
            }
            else
            {
                self->active_camera_ = &self->right_camera_control_;
            }

            // treat shift+drag as middle-button pan
            self->active_camera_->mouse({int(x), int(y)}, 2, down);
        }
    }

    // Cursor movement → active_camera_.drag()
    static void CursorPosCallback(GLFWwindow *w, double x, double y)
    {
        ImGui_ImplGlfw_CursorPosCallback(w, x, y);
        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;
        auto self = static_cast<Ros2GLViewer *>(glfwGetWindowUserPointer(w));

        // Don't handle camera movement if we're dragging the splitter
        if (self->is_dragging_splitter_)
        {
            return;
        }

        bool leftDown = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        bool midDown = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        bool shiftDown = (glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                          glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        // Determine which camera to use based on mouse position
        if (self->split_screen_mode_)
        {
            int split_x = static_cast<int>(self->framebuffer_size().x() * self->split_ratio_);
            if (x < split_x)
            {
                self->active_camera_ = &self->left_camera_control_;
            }
            else
            {
                self->active_camera_ = &self->right_camera_control_;
            }
        }
        else
        {
            self->active_camera_ = &self->right_camera_control_;
        }

        int btn = -1;
        if (shiftDown && leftDown)
            btn = 2; // pan
        else if (leftDown)
            btn = 0; // orbit
        else if (midDown)
            btn = 2; // pan
        if (btn >= 0)
            self->active_camera_->drag({int(x), int(y)}, btn);
    }

    // Scroll wheel → active_camera_.scroll()
    static void ScrollCallback(GLFWwindow *w, double dx, double dy)
    {
        ImGui_ImplGlfw_ScrollCallback(w, dx, dy);
        ImGuiIO &io = ImGui::GetIO();
        if (io.WantCaptureMouse)
            return;
        auto self = static_cast<Ros2GLViewer *>(glfwGetWindowUserPointer(w));

        // Don't handle camera movement if we're dragging the splitter
        if (self->is_dragging_splitter_)
        {
            return;
        }

        // Determine which camera to use based on mouse position
        double x, y;
        glfwGetCursorPos(w, &x, &y);
        if (self->split_screen_mode_)
        {
            int split_x = static_cast<int>(self->framebuffer_size().x() * self->split_ratio_);
            if (x < split_x)
            {
                self->active_camera_ = &self->left_camera_control_;
            }
            else
            {
                self->active_camera_ = &self->right_camera_control_;
            }
        }
        else
        {
            self->active_camera_ = &self->right_camera_control_;
        }

        self->active_camera_->scroll({float(dx), float(dy)});
    }

    bool init(const char *window_name, const Eigen::Vector2i &size, const char *glsl_version = "#version 330") override
    {
        if (!guik::Application::init(window_name, size, glsl_version))
        {
            return false;
        }

        setupCustomImGuiColors();

        // Add these lines to check OpenGL version
        const char *version = (const char *)glGetString(GL_VERSION);
        const char *vendor = (const char *)glGetString(GL_VENDOR);
        const char *renderer = (const char *)glGetString(GL_RENDERER);
        const char *glsl_version_str = (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION);

        std::cout << "OpenGL Version: " << version << std::endl;
        std::cout << "OpenGL Vendor: " << vendor << std::endl;
        std::cout << "OpenGL Renderer: " << renderer << std::endl;
        std::cout << "GLSL Version: " << glsl_version_str << std::endl;

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

        // ======= TEXT SHADER INITIALIZATION =======
        if (!text_renderer_.initTextRendering())
        {
            std::cerr << "Failed to initialize text rendering" << std::endl;
            return false;
        }

        // ======= PATH SHADER INITIALIZATION =======
        if (!path_renderer_.initPathRendering())
        {
            std::cerr << "Failed to initialize path rendering" << std::endl;
            return false;
        }
        std::cout << green << "Successfully initialized Path shader" << reset << std::endl;

        // ======= CLOUD SHADER INITIALIZATION =======
        if (!cloud_renderer_.initCloudRendering())
        {
            std::cerr << "Failed to initialize cloud rendering" << std::endl;
            return false;
        }
        std::cout << green << "Successfully initialized Cloud shader" << reset << std::endl;

        float cloud_height    = 300.0f;
        float cloud_thickness = 30.0f;
        float cloud_coverage  = 0.65f;
        float cloud_absorp    = 1.03f;
        float wind_x = 5.0f, wind_z = 3.0f, max_d = 10000.0f;

        cloud_renderer_.updateCloudParameters(
        cloud_coverage, cloud_height, cloud_thickness,
        cloud_absorp,   wind_x,       wind_z,          max_d);

        // vertical axis is Z now → {halfX, halfY, halfZ}
        cloud_renderer_.createVolumeBox({2000.f, 2000.f, 50.f});  // 400×400×240 m box
        cloud_renderer_.setVolumeHalfExtents({2000.f, 2000.f, 50.f});

        Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
        M.block<3,1>(0,3) = Eigen::Vector3f(
            0.f,
            0.f,
            cloud_height + 0.5f * cloud_thickness   // place the slab in Z
        );
        cloud_renderer_.setVolumeModelMatrix(M);


        // ======= GLB SHADER INITIALIZATION =======
        if (!model_upload_.createGLBShader(glb_shader_program_))
        {
            std::cerr << "Failed to create GLB shader" << std::endl;
            return false;
        }
        std::cout << green << "Successfully initialized GLB shader" << reset << std::endl;

        if (!initMapboxMap())
        {
            std::cerr << "Failed to initialize Mapbox map" << std::endl;
            // return false;
        }

        // Load dock icon textures
        for (auto &icon : dock_icons_)
        {
            if (!icon.image_path.empty())
            {
                icon.texture_id = loadImageTexture(icon.image_path);
                if (icon.texture_id != 0)
                {
                    std::cout << "Loaded texture for " << icon.tooltip << std::endl;
                }
            }
        }

        return true;
    }

    void draw_ui() override
    {

        if (!rclcpp::ok())
        {
            close(); // tells guik::Application to stop its run loop
            return;
        }
        // get the current time
        auto now = std::chrono::steady_clock::now();
        // get the tf based on the frame
        update_tf_transforms();

        // Topic hz
        {
            std::lock_guard<std::mutex> lock(topics_mutex_);
            // Just clean up old entries, Hz is already calculated in callback
            auto cutoff = now - std::chrono::seconds(5); // Clean up old topic data
            for (auto it = topic_message_times_.begin(); it != topic_message_times_.end();)
            {
                if (it->second.empty() || it->second.back() < cutoff)
                {
                    it = topic_message_times_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }
        // Check if we need to refresh the topic list
        if (now - last_topic_refresh_ > topic_refresh_interval_)
        {
            refresh_topic_list();
        }
        // Check if we need to refresh the frame list
        if (now - last_frame_refresh_ > frame_refresh_interval_)
        {
            refresh_frame_list();
        }

        // Show main canvas settings
        main_canvas_->draw_ui();

        // Main options window
        ImGui::Begin("ros2 Viewer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        // FPS counter
        ImGui::Separator();
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

        // ✅ ADD: Memory monitoring
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 60 == 0)
        { // Check every ~1 second at 60fps
            checkMemoryUsage();
        }
        ImGui::Text("Memory: %zu MB", last_memory_check_);

        // ============================================
        // Point Size Control (RViz-style)
        // ============================================
        ImGui::Separator();
        ImGui::Text("Point Cloud Settings:");
        ImGui::SliderFloat("ROS2 Topic Point Size", &ros2_topic_point_size_, 0.0f, 50.0f, "%.1f");
        ImGui::SliderFloat("PCD File Point Size", &pcd_file_point_size_, 0.0f, 50.0f, "%.1f");
        
        // ============================================
        // Rendering Distance Control
        // ============================================
        ImGui::Separator();
        ImGui::Text("Rendering Distance:");
        ImGui::SliderFloat("Far Clipping Plane", &far_clipping_distance_, 100.0f, 50000.0f, "%.0f");
        ImGui::Text("Objects beyond %.0f units will not be rendered", far_clipping_distance_);

        // ============================================
        // Split-screen mode control
        // ============================================
        ImGui::Separator();
        ImGui::Text("Display Mode:");
        if (ImGui::Checkbox("Split Screen Mode", &split_screen_mode_))
        {
            std::cout << "Split screen mode: " << (split_screen_mode_ ? "ON" : "OFF") << std::endl;
        }
        if (split_screen_mode_)
        {
            ImGui::Text("  Left: Map | Right: PCD/Meshes/Lanelet");

            // Split ratio control
            ImGui::Separator();
            ImGui::Text("Split Control:");
            ImGui::SliderFloat("Split Ratio", &split_ratio_, 0.05f, 0.95f, "%.2f");
            ImGui::SameLine();
            if (ImGui::Button("Reset to 50/50"))
            {
                split_ratio_ = 0.5f;
                std::cout << "🔄 Split ratio reset to 50/50" << std::endl;
            }

            // Show current split percentages
            int left_percent = static_cast<int>(split_ratio_ * 100);
            int right_percent = 100 - left_percent;
            ImGui::Text("Current split: %d%% / %d%%", left_percent, right_percent);

            // Camera controls for split-screen mode
            ImGui::Separator();
            ImGui::Text("Camera Controls:");

            if (ImGui::Button("Reset Left Camera (Map)"))
            {
                left_camera_control_ = guik::ArcCameraControl();
                std::cout << "🔄 Left camera (map) reset" << std::endl;
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset Right Camera (3D)"))
            {
                right_camera_control_ = guik::ArcCameraControl();
                std::cout << "🔄 Right camera (3D) reset" << std::endl;
            }

            // Show which camera is currently active
            if (active_camera_ == &left_camera_control_)
            {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Active: Left Camera (Map)");
            }
            else
            {
                ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Active: Right Camera (3D)");
            }

            ImGui::Text("Tip: Click in left/right half to control that camera");
            ImGui::Text("Tip: Drag the circular handle to resize split");
        }
        else
        {
            ImGui::Text("  Single view: All content together");
            ImGui::Text("  Using Right Camera for all content");
        }

        // ============================================
        // Mapbox map status
        // ============================================
        ImGui::Separator();
        ImGui::Text("Mapbox Map Status:");
        if (map_snapshotter_)
        {
            ImGui::Text("  Loading: %s", map_snapshotter_->isLoading() ? "Yes" : "No");
            ImGui::Text("  Ready: %s", map_snapshotter_->isReady() ? "Yes" : "No");
            ImGui::Text("  Texture: %s", map_snapshotter_->hasValidTexture() ? "Valid" : "Invalid");
            ImGui::Text("  Pending: %s", map_snapshotter_->hasPendingTexture() ? "Yes" : "No");
            ImGui::Text("  Tile Geometry: %zu loaded", map_snapshotter_->getLoadedGeometryCount());
        }
        else
        {
            ImGui::Text("  Not initialized");
        }

        // ============================================
        // TF Frames
        // ============================================
        ImGui::Separator();
        ImGui::Text("TF Frames: %zu", frame_transforms_.size());
        if (ImGui::Button("Reset TF Buffer"))
        {
            resetTFBuffer();
        }
        ImGui::Separator();

        if (ImGui::Button("Select Reference Frame"))
        {
            show_frames_window = !show_frames_window;
        }

        ImGui::SameLine();
        ImGui::Text("Current frame: %s", fixed_frame_.c_str());

        // Show frames window if enabled
        if (show_frames_window)
        {
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

        ImGui::Separator();

        // Button to toggle Topics window
        if (ImGui::Button("Show ROS2 Topics"))
        {
            // Toggle window visibility
            show_topics_window = !show_topics_window;
        }

        ImGui::SameLine();
        ImGui::Text("Num of topics: %zu", topic_names_.size());

        // Show topics window if enabled
        if (show_topics_window)
        {
            // Display topics - just the names
            ImGui::Text("Available ROS2 Topics:");
            ImGui::Separator();

            ImGui::BeginChild("TopicsList", ImVec2(420, 300), true);
            {
                std::lock_guard<std::mutex> lock(topics_mutex_);
                for (int i = 0; i < (int)topic_names_.size(); ++i)
                {
                    const auto &name = topic_names_[i];

                    // Get Hz value first
                    float hz = 0.0f;
                    auto hz_it = topic_hz_.find(name);
                    if (hz_it != topic_hz_.end())
                    {
                        hz = hz_it->second;
                    }

                    // Format Hz text with exactly one decimal place
                    char hz_text[32];
                    snprintf(hz_text, sizeof(hz_text), "%.1f Hz", hz);

                    // Calculate elements dimensions
                    const float circle_radius = 5.0f;
                    const float padding = 3.0f;
                    const float rounding = 3.0f; // Rounded corners

                    // FIXED WIDTH for Hz square to prevent movement
                    const float hz_square_fixed_width = 60.0f; // Fixed width for all Hz squares
                    ImVec2 hz_text_size = ImGui::CalcTextSize(hz_text);
                    float line_height = ImGui::GetTextLineHeight();

                    // Use consistent height for all elements (increased to accommodate topic type)
                    float element_height = std::max(line_height * 2.2f, circle_radius * 2); // Increased height for type display

                    // Store initial cursor position for the row
                    ImVec2 row_start = ImGui::GetCursorScreenPos();

                    // === DRAW ALIVE INDICATOR CIRCLE ===
                    bool alive = false;
                    auto it = last_msg_time_.find(name);
                    if (it != last_msg_time_.end())
                        alive = (now - it->second) < alive_threshold_;

                    ImU32 circle_col = alive ? IM_COL32(114, 175, 67, 255) : IM_COL32(255, 39, 41, 255);
                    float circle_y_center = row_start.y + element_height * 0.5f;
                    ImGui::GetWindowDrawList()->AddCircleFilled(
                        {row_start.x + circle_radius, circle_y_center},
                        circle_radius,
                        circle_col);

                    // === DRAW HZ SQUARE ===
                    float hz_square_x = row_start.x + circle_radius * 2 + 8.0f; // Space after circle
                    float hz_square_height = hz_text_size.y + padding * 2;
                    float hz_square_y = row_start.y + (element_height - hz_square_height) * 0.5f; // Center vertically

                    ImVec2 hz_square_min = {hz_square_x, hz_square_y};
                    ImVec2 hz_square_max = {hz_square_x + hz_square_fixed_width, hz_square_y + hz_square_height};

                    // Draw rounded white square background
                    ImGui::GetWindowDrawList()->AddRectFilled(
                        hz_square_min,
                        hz_square_max,
                        IM_COL32(25, 29, 32, 200), // White color
                        rounding                   // Rounded corners
                    );

                    // Optional: Add a subtle border around the square
                    ImGui::GetWindowDrawList()->AddRect(
                        hz_square_min,
                        hz_square_max,
                        IM_COL32(93, 103, 113, 200), // Light gray border
                        rounding,                    // Rounded corners
                        0,                           // No flags
                        1.0f                         // Border thickness
                    );

                    // Draw Hz text CENTERED in the fixed-width square
                    ImVec2 hz_text_pos = {
                        hz_square_x + (hz_square_fixed_width - hz_text_size.x) * 0.5f, // Center horizontally
                        hz_square_y + padding};

                    ImGui::GetWindowDrawList()->AddText(
                        hz_text_pos,
                        IM_COL32(253, 234, 234, 255), // Black text
                        hz_text);

                    // === DRAW TOPIC NAME ===
                    float topic_name_x = hz_square_max.x + 8.0f;                              // Space after Hz square
                    float topic_name_y = row_start.y + (element_height - line_height) * 0.5f; // Center vertically

                    // Set cursor position for the selectable topic name
                    ImGui::SetCursorScreenPos({topic_name_x, row_start.y});

                    // Create invisible button area for the full remaining width to make selection easier
                    float available_width = ImGui::GetContentRegionAvail().x;
                    bool is_selected = (i == selected_topic_idx_);

                    // Draw selectable with proper vertical centering
                    ImGui::PushID(i);
                    if (ImGui::Selectable("##topic_select", is_selected, 0, ImVec2(available_width, element_height)))
                        selected_topic_idx_ = i;
                    ImGui::PopID();

                    // Draw topic name text over the selectable area
                    ImGui::GetWindowDrawList()->AddText(
                        {topic_name_x + 4.0f, topic_name_y}, // Small left padding
                        is_selected ? IM_COL32(255, 255, 255, 255) : ImGui::GetColorU32(ImGuiCol_Text),
                        name.c_str());

                    // Draw topic type below the name (smaller text)
                    auto type_it = topic_types_.find(name);
                    if (type_it != topic_types_.end())
                    {
                        std::string type_short = type_it->second;
                        // Shorten common ROS2 message types for display
                        if (type_short.find("sensor_msgs/msg/PointCloud2") != std::string::npos)
                        {
                            type_short = "PointCloud2";
                        }
                        else if (type_short.find("sensor_msgs/msg/NavSatFix") != std::string::npos)
                        {
                            type_short = "NavSatFix";
                        }
                        else if (type_short.find("visualization_msgs/msg/MarkerArray") != std::string::npos)
                        {
                            type_short = "MarkerArray";
                        }
                        else if (type_short.find("nav_msgs/msg/Path") != std::string::npos)
                        {
                            type_short = "Path";
                        }
                        else if (type_short.find("geometry_msgs/msg/PoseStamped") != std::string::npos)
                        {
                            type_short = "PoseStamped";
                        }
                        else if (type_short.find("nav_msgs/msg/OccupancyGrid") != std::string::npos)
                        {
                            type_short = "OccupancyGrid";
                        }
                        else
                        {
                            // Extract just the message name from the full type
                            size_t last_slash = type_short.find_last_of('/');
                            if (last_slash != std::string::npos)
                            {
                                type_short = type_short.substr(last_slash + 1);
                            }
                        }

                        ImGui::GetWindowDrawList()->AddText(
                            {topic_name_x + 4.0f, topic_name_y + line_height + 2.0f}, // Below the topic name
                            IM_COL32(150, 150, 150, 200),                             // Gray color for type
                            type_short.c_str());
                    }
                }
            }
            ImGui::EndChild();

            if (selected_topic_idx_ >= 0 && selected_topic_idx_ < (int)topic_names_.size())
            {
                const std::string &selected_topic = topic_names_[selected_topic_idx_];
                auto type_it = topic_types_.find(selected_topic);
                std::string topic_type = (type_it != topic_types_.end()) ? type_it->second : "unknown";

                // Determine button text and action based on topic type
                std::string button_text = "+ Subscribe";
                std::string action_type = "unknown";

                if (topic_type.find("sensor_msgs/msg/PointCloud2") != std::string::npos)
                {
                    button_text = "+ Add PointCloud";
                    action_type = "pointcloud";
                }
                else if (topic_type.find("sensor_msgs/msg/NavSatFix") != std::string::npos)
                {
                    button_text = "+ Add GPS";
                    action_type = "gps";
                }
                else if (topic_type.find("visualization_msgs/msg/MarkerArray") != std::string::npos)
                {
                    button_text = "+ Add MarkerArray";
                    action_type = "markerarray";
                }
                else if (topic_type.find("nav_msgs/msg/Path") != std::string::npos)
                {
                    button_text = "+ Add Path";
                    action_type = "path";
                }
                else if (topic_type.find("geometry_msgs/msg/PoseStamped") != std::string::npos)
                {
                    button_text = "+ Add Pose";
                    action_type = "pose";
                }
                else if (topic_type.find("nav_msgs/msg/OccupancyGrid") != std::string::npos)
                {
                    button_text = "+ Add OccupancyGrid";
                    action_type = "occupancygrid";
                }
                else
                {
                    button_text = "+ Add Generic";
                    action_type = "generic";
                }

                ImGui::Text("Subscribe to: %s", selected_topic.c_str());
                ImGui::Text("Type: %s", topic_type.c_str());

                // Special UI for Path topics - allow selecting path type and color mode
                static int path_type_selection = 0; // 0 = normal line, 1 = car path
                static int path_color_mode = 0;     // 0 = flat, 1 = gradient
                static int path_color_selection = 0; // 0 = orange, 1 = blue
                if (action_type == "path")
                {
                    ImGui::Separator();
                    ImGui::Text("Path Visualization Type:");
                    ImGui::RadioButton("Normal Path (Line)", &path_type_selection, 0);
                    ImGui::SameLine();
                    ImGui::RadioButton("Car Path (1.5m width)", &path_type_selection, 1);

                    // Show color selection for both path types
                    ImGui::Separator();
                    ImGui::Text("Path Color:");
                    ImGui::RadioButton("Orange", &path_color_selection, 0);
                    ImGui::SameLine();
                    ImGui::RadioButton("Blue", &path_color_selection, 1);

                    // Show color mode options only for car path
                    if (path_type_selection == 1)
                    {
                        ImGui::Separator();
                        ImGui::Text("Car Path Color Mode:");
                        ImGui::RadioButton("Flat Color", &path_color_mode, 0);
                        ImGui::SameLine();
                        ImGui::RadioButton("Gradient (Center → Edges)", &path_color_mode, 1);
                        ImGui::SameLine();
                        ImGui::RadioButton("Gradient (Edges → Center)", &path_color_mode, 2);
                        ImGui::Text("  Mode 1: solid at center, transparent at edges");
                        ImGui::Text("  Mode 2: solid at edges, transparent at center");
                    }
                }

                if (ImGui::Button(button_text.c_str()))
                {
                    std::cout << green << "Adding subscription for topic: " << selected_topic
                              << " (type: " << action_type << ")" << reset << std::endl;

                    // Call appropriate subscription function based on type
                    if (action_type == "pointcloud")
                    {
                        addPointCloudTopic(selected_topic);
                    }
                    else if (action_type == "gps")
                    {
                        addGPSTopic(selected_topic);
                    }
                    else if (action_type == "markerarray")
                    {
                        addMarkerArrayTopic(selected_topic);
                    }
                    else if (action_type == "path")
                    {
                        addPathTopic(selected_topic, path_type_selection, path_color_mode, path_color_selection);
                    }
                    else if (action_type == "pose")
                    {
                        // addPoseTopic(selected_topic);
                    }
                    else if (action_type == "occupancygrid")
                    {
                        // addOccupancyGridTopic(selected_topic);
                    }
                    else
                    {
                        // addGenericTopic(selected_topic);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("- Remove"))
                {
                    std::cout << red << "Removing topic: " << selected_topic << reset << std::endl;

                    // Try to remove from all subscription types
                    removePointCloudTopic(selected_topic);
                    removeMarkerArrayTopic(selected_topic);
                    removePathTopic(selected_topic);
                    // removeGPSTopic(selected_topic); // GPS is handled separately

                    selected_topic_idx_ = -1;
                }
            }
        }

        // Show currently subscribed marker array topics
        {
            std::lock_guard lk(marker_array_topics_mutex_);
            if (!marker_array_topics_.empty())
            {
                ImGui::Separator();
                ImGui::Text("Subscribed Marker Array Topics:");
                for (const auto &mt : marker_array_topics_)
                {
                    ImGui::Text("  • %s (%zu markers)", mt->topic_name.c_str(),
                                mt->marker_array ? mt->marker_array->markers.size() : 0);
                }
            }
        }

        // Show currently subscribed path topics
        {
            std::lock_guard lk(path_topics_mutex_);
            if (!path_topics_.empty())
            {
                ImGui::Separator();
                ImGui::Text("Subscribed Path Topics:");
                for (const auto &pt : path_topics_)
                {
                    const char *type_str = (pt->path_type == 0) ? "Line" : "Car (1.5m)";
                    const char *color_mode_str;
                    if (pt->color_mode == 0)
                    {
                        color_mode_str = "Flat";
                    }
                    else if (pt->color_mode == 1)
                    {
                        color_mode_str = "Grad(C→E)";
                    }
                    else
                    {
                        color_mode_str = "Grad(E→C)";
                    }
                    
                    const char *path_color_str = (pt->path_color_selection == 0) ? "Orange" : "Blue";

                    if (pt->path_type == 0)
                    {
                        // Normal path - show color selection
                        ImGui::Text("  • %s (%zu poses) [%s, %s]", pt->topic_name.c_str(),
                                    pt->path ? pt->path->poses.size() : 0, type_str, path_color_str);
                    }
                    else
                    {
                        // Car path - show color mode and color selection
                        ImGui::Text("  • %s (%zu poses) [%s, %s, %s]", pt->topic_name.c_str(),
                                    pt->path ? pt->path->poses.size() : 0, type_str, color_mode_str, path_color_str);
                    }
                }
            }
        }

        ImGui::Separator();

        // ============================================
        // Robot Mesh
        // ============================================
        // /workspace/models/sdv.glb
        // /workspace/models/910.glb
        static bool show_load_input_robot = false;
        static char filepath_glb[256] = "";
        static int selected_robot_frame = 0;

        if (ImGui::Button("Load robot mesh"))
            show_load_input_robot = !show_load_input_robot;

        if (show_load_input_robot)
        {
            ImGui::InputText("model File", filepath_glb, sizeof(filepath_glb));
            ImGui::Separator();
            ImGui::Text("Assign to TF frame:");
            ImGui::SameLine();

            bool has = !available_frames_.empty();
            // preview what would be loaded (or show “map”)
            const char *preview = has
                                      ? available_frames_[selected_robot_frame].c_str()
                                      : "(map)";

            if (has)
            {
                if (ImGui::BeginCombo("##robot_frames", preview))
                {
                    for (int i = 0; i < (int)available_frames_.size(); ++i)
                    {
                        bool sel = (selected_robot_frame == i);
                        if (ImGui::Selectable(available_frames_[i].c_str(), sel))
                            selected_robot_frame = i;
                        if (sel)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }
            else
            {
                ImGui::TextDisabled("No TF frames (using default)");
            }

            ImGui::SameLine();
            if (ImGui::Button("Open"))
            {
                try
                {

                    if (mesh_glb_loaded)
                    {
                        glBindVertexArray(0); // ✅ Unbind any currently bound VAO
                        glUseProgram(0);      // ✅ Unbind any shader program
                        model_upload_.cleanupMesh(loaded_mesh_glb);
                        mesh_glb_loaded = false;
                    }

                    // 1) load the mesh
                    loaded_mesh_glb = model_upload_.loadModel(filepath_glb);
                    mesh_glb_loaded = true;

                    // 2) pick the frame based on your index (or "map" if none)
                    const std::string frame_id = has
                                                     ? available_frames_[selected_robot_frame]
                                                     : "map";
                    loaded_mesh_glb.frame_id = frame_id;

                    // 3) log
                    std::cout << green
                              << "Loaded robot mesh from: " << filepath_glb
                              << " into frame: " << frame_id << " with type: " << loaded_mesh_glb.type
                              << reset << std::endl;
                }
                catch (const std::exception &e)
                {
                    std::cerr << "GLB load error: " << e.what() << std::endl;
                }
                show_load_input_robot = false;
            }
        }

        ImGui::Separator();

        static bool show_load_input_pcd = false;
        static char pcd_path[256] = "";
        static int selected_pcd_frame = 0;

        if (ImGui::Button("Load PCD"))
        {
            show_load_input_pcd = !show_load_input_pcd;
        }

        if (show_load_input_pcd)
        {
            ImGui::InputText("PCD File", pcd_path, IM_ARRAYSIZE(pcd_path));
            ImGui::SameLine();
            if (ImGui::Button("OK"))
            {
                // construct the buffer from the file
                pcd_loader_.PointCloudBuffer(pcd_path, pcd_frame, false);
                show_load_input_pcd = false;
            }
            // TF
            ImGui::Separator();
            ImGui::Text("Assign to TF frame:");
            ImGui::SameLine();
            if (!available_frames_.empty())
            {
                const char *preview = available_frames_[selected_pcd_frame].c_str();
                if (ImGui::BeginCombo("##ply_frames", preview))
                {
                    for (int n = 0; n < (int)available_frames_.size(); ++n)
                    {

                        bool is_selected = (selected_pcd_frame == n);
                        if (ImGui::Selectable(available_frames_[n].c_str(), is_selected))
                        {
                            selected_pcd_frame = n;
                            pcd_frame = available_frames_[n];
                        }
                        if (is_selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }
            else
            {
                ImGui::TextDisabled("No TF frames");
            }
        }

        ImGui::Separator();

        static bool show_load_input_lanelet_map = false;
        static char laneletmap_path[256] = "";
        static int selected_lanelet_frame = 0;
        if (ImGui::Button("Load Lanelet Map"))
        {
            show_load_input_lanelet_map = !show_load_input_lanelet_map;
        }

        if (show_load_input_lanelet_map)

        {
            ImGui::InputText("Map File", laneletmap_path, IM_ARRAYSIZE(laneletmap_path));
            ImGui::SameLine();
            if (ImGui::Button("OK"))
            {
                // construct the buffer from the file
                lanelet_loader_.loadLanelet2Map(laneletmap_path);
                show_load_input_lanelet_map = false;
            }
            // TF
            ImGui::Separator();
            ImGui::Text("Assign to TF frame:");
            ImGui::SameLine();
            if (!available_frames_.empty())
            {
                const char *preview = available_frames_[selected_lanelet_frame].c_str();
                if (ImGui::BeginCombo("##lanelet_frames", preview))
                {
                    for (int n = 0; n < (int)available_frames_.size(); ++n)
                    {
                        bool is_selected = (selected_lanelet_frame == n);
                        if (ImGui::Selectable(available_frames_[n].c_str(), is_selected))
                        {
                            selected_lanelet_frame = n;
                            lanelet_loader_.setFrameId(available_frames_[n]);
                        }
                        if (is_selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }
            else
            {
                ImGui::TextDisabled("No TF frames");
            }
        }

        ImGui::Separator();

        static bool show_load_input_map_element = false;
        static char meshmap_path[256] = "";
        if (ImGui::Button("Load Map Elements"))
        {
            show_load_input_map_element = !show_load_input_map_element;
        }

        if (show_load_input_map_element)
        {
            ImGui::InputText("Mesh Map File", meshmap_path, IM_ARRAYSIZE(meshmap_path));
            ImGui::SameLine();
            if (ImGui::Button("OK"))
            {
                // construct the buffer from the file
                loaded_mesh_map = model_upload_.loadModel(meshmap_path);
                mesh_map_loaded = true;
                show_load_input_map_element = false;
            }
        }

        ImGui::End();

        // Render splitter bar in split-screen mode
        if (split_screen_mode_)
        {
            renderSplitterBar();
        }
    }

    // Helper method to render all 3D content (PCD, meshes, lanelet, grid, frames)
    void render3DContent(glk::GLSLShader &shader, const Eigen::Matrix4f &view, const Eigen::Matrix4f &projection)
    {

        // ============================================
        // render grid
        // ============================================
        {
            Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
            T.pretranslate(Eigen::Vector3f(0, 0, -0.02f));
            shader.set_uniform("color_mode", 1);
            shader.set_uniform("model_matrix", T.matrix());
            shader.set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
            grid_->draw(shader);
        }

        // ============================================
        // render coordinate frames
        // ============================================
        if (show_tf_frames_)
        {
            std::lock_guard lk(tf_mutex_);
            shader.set_uniform("color_mode", 1); // Use flat color mode

            // Use cube primitive to create thick axes
            const auto &cube = glk::Primitives::instance()->primitive(glk::Primitives::CUBE);

            float axis_length = tf_frame_size_;
            float axis_thickness = 0.03f;

            // First, draw the fixed frame at origin (identity transform)

            Eigen::Isometry3f identity = Eigen::Isometry3f::Identity();
            drawThickCoordinateFrame(shader, cube, identity, axis_length, axis_thickness);

            // Add fixed frame name label at origin
            Eigen::Vector3f text_position = Eigen::Vector3f(0.0f, 0.0f, -0.1f);
            text_renderer_.addWorldText(fixed_frame_, text_position, Eigen::Vector3f(1.0f, 1.0f, 1.0f), 0.005f, 0.0f);

            for (auto &p : frame_transforms_)
            {
                drawThickCoordinateFrame(shader, cube, p.second, axis_length, axis_thickness);

                // Add frame name label
                Eigen::Vector3f frame_position = p.second.translation();
                Eigen::Vector3f text_position = frame_position + Eigen::Vector3f(0.0f, 0.0f, -0.1f);
                text_renderer_.addWorldText(p.first, text_position, Eigen::Vector3f(1.0f, 1.0f, 1.0f), 0.005f, 0.0f);
            }
        }

        // ============================================
        // lights
        // ============================================

        {
            // 1) compute elapsed time
            auto now = std::chrono::steady_clock::now();
            float t = std::chrono::duration<float>(now - blink_start_).count();

            // 2) new blink logic
            constexpr float offTime = 0.5f;            // first 0.5 s = dim only
            constexpr float onTime = 2.5f;             // next 2.5 s = bright
            constexpr float period = offTime + onTime; // total = 3 s
            float phase = std::fmod(t, period);

            bool lightOn = (phase >= offTime); // off for [0,0.5), on for [0.5,3)

            shader.set_uniform("color_mode", 1);

            const std::array<const char *, 2> frames = {"light_1", "light_2"};
            for (auto frame_name : frames)
            {
                Eigen::Affine3f M = Eigen::Affine3f::Identity();
                {
                    std::lock_guard<std::mutex> lk(tf_mutex_);
                    auto it = frame_transforms_.find(frame_name);
                    if (it != frame_transforms_.end())
                        M = it->second;
                }

                // Create rounded rectangle shape (car tail light)
                // Main rectangular body with slight rounding via sphere
                const auto &sphere = glk::Primitives::instance()->primitive(glk::Primitives::SPHERE);

                // Choose color based on blink state
                Eigen::Vector4f lightColor;
                if (lightOn)
                {
                    lightColor = Eigen::Vector4f(1.0f, 0.1f, 0.1f, 1.0f); // Bright red when on
                }
                else
                {
                    lightColor = Eigen::Vector4f(0.35f, 0.05f, 0.05f, 1.0f); // Dim red when off
                }
                shader.set_uniform("material_color", lightColor);

                // Draw rounded rectangle as scaled sphere (wider than tall)
                Eigen::Affine3f lightTransform = M;
                lightTransform.rotate(Eigen::AngleAxisf(M_PI / 2.0f, Eigen::Vector3f::UnitZ())); // Rotate 90 degrees around Z
                lightTransform.scale(Eigen::Vector3f(0.17f, 0.12f, 0.08f));                      // Wide, medium height, shallow depth
                shader.set_uniform("model_matrix", lightTransform.matrix());
                sphere.draw(shader);
            }
        }

        // ============================================
        // render PCD point cloud
        // ============================================
        if (pcd_loader_.isLoaded() || pcd_loader_.getPointCount() != 0)
        {
            // Convert RViz-style point size (0-50) to shader uniforms
            // Multiply by 20 to compensate for z_dist factor in shader
            float shader_point_size = pcd_file_point_size_ * 20.0f;
            shader.set_uniform("point_scale", shader_point_size);
            shader.set_uniform("point_size", 1.0f);

            pcd_loader_.renderpcl(shader, tf_mutex_, frame_transforms_);
        }

        // ============================================
        // render meshes
        // ============================================
        if (mesh_map_loaded)
        {
            model_upload_.renderMesh(loaded_mesh_map, shader, tf_mutex_, frame_transforms_);
        }

        if (mesh_glb_loaded && loaded_mesh_glb.type == 0) // 0 = GLB type
        {
            model_upload_.renderGLBMesh(loaded_mesh_glb, glb_shader_program_, tf_mutex_, frame_transforms_);

            // ✅ CRITICAL: Re-bind the main shader after GLB rendering
            // GLB uses a custom shader program, so we must restore the main shader
            // before rendering other content (point clouds, lanelet, etc.)
            shader.use();
        }

        if (mesh_glb_loaded && loaded_mesh_glb.type == 1) // 1 = GLTF type
        {
            model_upload_.renderMesh(loaded_mesh_glb, shader, tf_mutex_, frame_transforms_);
        }

        // ============================================
        // render lanelet
        // ============================================
        if (lanelet_loader_.isLoaded())
        {
            lanelet_loader_.mapLines(shader, tf_mutex_, frame_transforms_);
            lanelet_loader_.crosswalks(shader, tf_mutex_, frame_transforms_);
            lanelet_loader_.stripes(shader, tf_mutex_, frame_transforms_);
        }
        // ============================================
        // render path topics using shader-based approach
        // ============================================
        {
            std::lock_guard lk(path_topics_mutex_);

            // Clear previous path data
            path_renderer_.clearPaths();

            // Add all path segments to the shader-based renderer
            for (auto &pt : path_topics_)
            {
                if (!pt->path || pt->path->poses.empty())
                    continue;

                // Convert path poses to Eigen vectors
                std::vector<Eigen::Vector3f> path_points;
                path_points.reserve(pt->path->poses.size());

                for (const auto &pose : pt->path->poses)
                {
                    Eigen::Vector3f world_pos = pt->transform * Eigen::Vector3f(
                                                                    pose.pose.position.x,
                                                                    pose.pose.position.y,
                                                                    pose.pose.position.z);
                    path_points.push_back(world_pos);
                }

                // Determine color based on path type and color selection
                Eigen::Vector3f path_color;
                if (pt->path_type == 0)
                {
                    // Normal path - use selected color
                    if (pt->path_color_selection == 0)
                    {
                        path_color = Eigen::Vector3f(1.0f, 0.5f, 0.0f); // Orange
                    }
                    else
                    {
                        path_color = Eigen::Vector3f(0.0f, 0.5f, 1.0f); // Blue
                    }
                }
                else
                {
                    // Car path - use selected color
                    if (pt->path_color_selection == 0)
                    {
                        path_color = Eigen::Vector3f(1.0f, 0.5f, 0.0f); // Orange
                    }
                    else
                    {
                        path_color = Eigen::Vector3f(0.0f, 0.5f, 1.0f); // Blue
                    }
                }

                // Add path segment to shader renderer
                path_renderer_.addPathSegment(path_points, path_color,
                                              pt->path_width, pt->path_type, pt->color_mode, 0.8f);
            }

            // Render all paths using the shader-based system
            path_renderer_.renderPaths(view, projection);
            
            shader.use();

        }

        // ============================================
        // render point clouds ros2 topics
        // ============================================
        {
            std::lock_guard lk(cloud_topics_mutex_);
            // set uniform color-mode to "flat color" and material color to white
            shader.set_uniform("color_mode", 1);                                           // Use flat color mode
            shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f)); // White color

            // Convert RViz-style point size (0-50) to shader uniforms
            // Multiply by 20 to compensate for z_dist factor in shader
            float shader_point_size = ros2_topic_point_size_ * 20.0f;
            shader.set_uniform("point_scale", shader_point_size);
            shader.set_uniform("point_size", 1.0f);

            // 1) One-time VAO+VBO creation
            if (dbgVao == 0)
            {
                glGenVertexArrays(1, &dbgVao);
                glGenBuffers(1, &dbgVbo);

                glBindVertexArray(dbgVao);
                glBindBuffer(GL_ARRAY_BUFFER, dbgVbo);

                // allocate a reasonable initial capacity (e.g. room for 200k points)
                const size_t initial_capacity = 200000;
                glBufferData(
                    GL_ARRAY_BUFFER,
                    initial_capacity * sizeof(pcl::PointXYZI),
                    nullptr,
                    GL_DYNAMIC_DRAW);

                // attribute 0 = vec3 position, offset 0, stride = sizeof(PointXYZI)
                glEnableVertexAttribArray(0);
                glVertexAttribPointer(
                    0,                      // index
                    3,                      // x, y, z
                    GL_FLOAT,               // type
                    GL_FALSE,               // normalized?
                    sizeof(pcl::PointXYZI), // stride
                    (void *)0               // offset
                );

                glBindVertexArray(0);
                std::cout << green
                          << "Created debug VAO/VBO for PointXYZI clouds"
                          << reset
                          << std::endl;
            }

            // 2) Render each cloud
            for (auto &ct : cloud_topics_)
            {
                if (!ct->cloud || ct->cloud->empty())
                    continue;

                size_t n = ct->cloud->size();
                // std::cout << "[PC_CB IN RENDER] "
                //           << ct->topic_name
                //           << " → "
                //           << n
                //           << " pts\n";

                // Convert RViz-style point size (0-50) to shader uniforms
                float shader_point_size = ros2_topic_point_size_ * 20.0f;
                shader.set_uniform("point_scale", shader_point_size);
                shader.set_uniform("point_size", 1.0f);

                // draw the per-cloud VAO
                shader.set_uniform("model_matrix", ct->transform.matrix());
                glBindVertexArray(ct->vao);
                glDrawArrays(GL_POINTS, 0, GLsizei(n));
                glBindVertexArray(0);

                // update the debug VBO with the same PointXYZI data
                glBindBuffer(GL_ARRAY_BUFFER, dbgVbo);
                glBufferSubData(
                    GL_ARRAY_BUFFER,
                    0, // offset
                    n * sizeof(pcl::PointXYZI),
                    ct->cloud->points.data());
                glBindBuffer(GL_ARRAY_BUFFER, 0);

                // draw the debug VAO with the same count
                glBindVertexArray(dbgVao);
                glDrawArrays(GL_POINTS, 0, GLsizei(n));
                glBindVertexArray(0);
            }
        }
    }

    // https://stackoverflow.com/questions/34866964/opengl-gllinewidth-doesnt-change-size-of-lines
    // https://registry.khronos.org/OpenGL-Refpages/gl4/html/glLineWidth.xhtml
    // https://docs.gl/gl4/glEnable
    void draw_gl() override
    {
        // 1) bind offscreen framebuffer
        main_canvas_->bind();

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_LINE_SMOOTH);

        // 2) clear & GL state
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // clear text rendering
        text_renderer_.clearWorldText();

        // Get screen dimensions
        auto fb_size = framebuffer_size();
        float w = float(fb_size.x());
        float h = float(fb_size.y());

        // Handle splitter interaction
        handleSplitterInteraction();

        // Calculate dynamic split based on split_ratio_
        int left_w = static_cast<int>(w * split_ratio_);
        int right_w = fb_size.x() - left_w;
        float left_w_f = float(left_w);
        float right_w_f = float(right_w);

        // 1) Compute View matrices for each viewport
        Eigen::Matrix4f left_view = left_camera_control_.view_matrix();
        Eigen::Matrix4f right_view = right_camera_control_.view_matrix();

        // 2) Compute Projection matrix
        float fovY = 45.f * M_PI / 180.f;
        float zNear = 0.01f, zFar = far_clipping_distance_; // Use dynamic far clipping distance
        float f = 1.f / std::tan(fovY / 2.f);

        if (split_screen_mode_)
        {
            // Split-screen mode: two separate viewports with dynamic split
            float aspect_left = left_w_f / h;
            float aspect_right = right_w_f / h;

            // Left projection matrix (for map)
            Eigen::Matrix4f proj_left = Eigen::Matrix4f::Zero();
            proj_left(0, 0) = f / aspect_left;
            proj_left(1, 1) = f;
            proj_left(2, 2) = (zFar + zNear) / (zNear - zFar);
            proj_left(2, 3) = (2.f * zFar * zNear) / (zNear - zFar);
            proj_left(3, 2) = -1.f;

            // Right projection matrix (for PCD/meshes/lanelet)
            Eigen::Matrix4f proj_right = Eigen::Matrix4f::Zero();
            proj_right(0, 0) = f / aspect_right;
            proj_right(1, 1) = f;
            proj_right(2, 2) = (zFar + zNear) / (zNear - zFar);
            proj_right(2, 3) = (2.f * zFar * zNear) / (zNear - zFar);
            proj_right(3, 2) = -1.f;

            // ============================================
            // LEFT VIEWPORT: Map only
            // ============================================
            glViewport(0, 0, left_w, (GLsizei)fb_size.y());

            // Clear left viewport completely (both color and depth)
            glScissor(0, 0, left_w, (GLsizei)fb_size.y());
            glEnable(GL_SCISSOR_TEST);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDisable(GL_SCISSOR_TEST);

            // Ensure depth testing is properly set up for this viewport
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);

            // Render Mapbox map on left side
            if (map_snapshotter_)
            {
                // Always try to update texture first (this will upload pending data)
                map_snapshotter_->updateTexture();

                // Now check if we have a valid texture to render
                if (map_snapshotter_->hasValidTexture())
                {
                    // Convert Eigen matrices to GLM matrices for the map renderer
                    glm::mat4 glm_view = glm::mat4(1.0f);
                    glm::mat4 glm_proj = glm::mat4(1.0f);

                    // Convert Eigen to GLM (column-major order)
                    for (int i = 0; i < 4; ++i)
                    {
                        for (int j = 0; j < 4; ++j)
                        {
                            glm_view[j][i] = left_view(i, j);
                            glm_proj[j][i] = proj_left(i, j);
                        }
                    }
                    map_snapshotter_->renderMap(glm_proj, glm_view);
                    map_snapshotter_->renderCircleOverlay(glm_proj, glm_view);
                }
            }

            // Clear depth buffer between viewports to prevent cross-contamination
            glClear(GL_DEPTH_BUFFER_BIT);

            // Render cloud volume for left viewport (map view)
            {
                Eigen::Matrix4f view_inv = left_view.inverse();
                Eigen::Vector3f camera_position = view_inv.block<3,1>(0,3);

                auto now  = std::chrono::steady_clock::now();
                float t   = std::chrono::duration<float>(now - blink_start_).count();
                cloud_renderer_.updateTime(t);

                cloud_renderer_.renderCloudVolume(left_view, proj_left, camera_position);

            }

            // ============================================
            // RIGHT VIEWPORT: PCD, meshes, lanelet, grid, frames
            // ============================================
            glViewport(left_w, 0, right_w, (GLsizei)fb_size.y());

            // Clear right viewport completely (both color and depth)
            glScissor(left_w, 0, right_w, (GLsizei)fb_size.y());
            glEnable(GL_SCISSOR_TEST);
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glDisable(GL_SCISSOR_TEST);

            // Ensure depth testing is properly set up for this viewport
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);

            // 4) bind the shader program for right viewport
            auto &shader = *main_canvas_->shader;
            shader.use();
            shader.set_uniform("view_matrix", right_view);
            shader.set_uniform("projection_matrix", proj_right);
            model_upload_.setMatrices(right_view, proj_right);

            // Render all 3D content on right side
            render3DContent(shader, right_view, proj_right);

            // Ensure depth buffer is properly written for this viewport
            glFlush();

            // render text
            text_renderer_.renderWorldText(right_view, proj_right);
        }
        else
        {
            // Single view mode: all content together
            float aspect = w / h;
            Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
            proj(0, 0) = f / aspect;
            proj(1, 1) = f;
            proj(2, 2) = (zFar + zNear) / (zNear - zFar);
            proj(2, 3) = (2.f * zFar * zNear) / (zNear - zFar);
            proj(3, 2) = -1.f;

            // Set full viewport
            glViewport(0, 0, fb_size.x(), fb_size.y());

            // 4) bind the shader program
            auto &shader = *main_canvas_->shader;
            shader.use();
            shader.set_uniform("view_matrix", right_view);
            shader.set_uniform("projection_matrix", proj);
            model_upload_.setMatrices(right_view, proj);

            // Render all 3D content
            render3DContent(shader, right_view, proj);

            // render text
            text_renderer_.renderWorldText(right_view, proj);
        }

        // 9) unbind & blit
        main_canvas_->unbind();
        main_canvas_->render_to_screen();
    }

    void framebuffer_size_callback(const Eigen::Vector2i &size) override
    {
        // Call parent implementation
        guik::Application::framebuffer_size_callback(size);

        // Update main canvas with new size
        if (main_canvas_)
        {
            main_canvas_.reset(new guik::GLCanvas("./data", size));
        }

        // Update Mapbox snapshotter size if it exists
        if (map_snapshotter_)
        {
            std::cout << "🔄 Updating Mapbox snapshotter size to: " << size.x() << "x" << size.y() << std::endl;
        }

        std::cout << "📐 Window resized to: " << size.x() << "x" << size.y() << std::endl;
    }

    void handleSplitterInteraction()
    {
        if (!split_screen_mode_)
            return;

        auto fb_size = framebuffer_size();
        float w = float(fb_size.x());
        float h = float(fb_size.y());

        // Calculate splitter position
        float splitter_x = w * split_ratio_;
        float splitter_left = splitter_x - splitter_width_ * 0.5f;
        float splitter_right = splitter_x + splitter_width_ * 0.5f;

        // Check if mouse is over splitter
        ImVec2 mouse_pos = ImGui::GetMousePos();
        bool mouse_over_splitter = (mouse_pos.x >= splitter_left && mouse_pos.x <= splitter_right &&
                                    mouse_pos.y >= 0 && mouse_pos.y <= h);

        is_hovering_splitter_ = mouse_over_splitter;

        // Handle mouse interactions
        if (ImGui::IsMouseClicked(0) && mouse_over_splitter)
        {
            is_dragging_splitter_ = true;
        }

        if (is_dragging_splitter_)
        {
            if (ImGui::IsMouseDown(0))
            {
                // Update split ratio based on mouse position
                float new_ratio = mouse_pos.x / w;
                split_ratio_ = std::max(0.05f, std::min(0.95f, new_ratio)); // Clamp between 5% and 95%
            }
            else
            {
                is_dragging_splitter_ = false;
            }
        }

        // Set cursor based on hover state
        if (mouse_over_splitter || is_dragging_splitter_)
        {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        }
    }

    void renderSplitterBar()
    {
        if (!split_screen_mode_)
            return;

        auto fb_size = framebuffer_size();
        float w = float(fb_size.x());
        float h = float(fb_size.y());

        // Calculate splitter position
        float splitter_x = w * split_ratio_;
        float splitter_left = splitter_x - splitter_width_ * 0.5f;
        float splitter_right = splitter_x + splitter_width_ * 0.5f;

        // Use ImGui's background draw list for overlay rendering
        ImDrawList *draw_list = ImGui::GetBackgroundDrawList();

        // Choose color based on state
        ImU32 bar_color;
        ImU32 handle_color;

        if (is_dragging_splitter_)
        {
            bar_color = IM_COL32(184, 184, 184, 153);    // Light gray when dragging (20% lighter)
            handle_color = IM_COL32(184, 184, 184, 153); // Light gray handle (20% lighter)
        }
        else if (is_hovering_splitter_)
        {
            bar_color = IM_COL32(153, 153, 153, 153);    // Light gray when hovering
            handle_color = IM_COL32(153, 153, 153, 153); // Light gray handle
        }
        else
        {
            bar_color = IM_COL32(102, 102, 102, 102); // Dark gray normally
            handle_color = IM_COL32(51, 51, 51, 230); // Dark handle
        }

        // Draw the vertical splitter bar
        draw_list->AddRectFilled(
            ImVec2(splitter_left, 0),
            ImVec2(splitter_right, h),
            bar_color);

        // Draw rounded rectangle handle in the center
        float handle_y = h * 0.5f;
        ImVec2 handle_center(splitter_x, handle_y);

        // Define rounded rectangle dimensions
        float handle_width = handle_radius_ * 1.2f;   // 1.0f * 1.2 = 1.2f
        float handle_height = handle_radius_ * 2.4f;  // 2.0f * 1.2 = 2.4f
        float corner_radius = handle_radius_ * 0.36f; // 0.3f * 1.2 = 0.36f

        ImVec2 rect_min(
            handle_center.x - handle_width * 0.5f,
            handle_center.y - handle_height * 0.5f);
        ImVec2 rect_max(
            handle_center.x + handle_width * 0.5f,
            handle_center.y + handle_height * 0.5f);

        // Draw the rounded rectangle
        draw_list->AddRectFilled(
            rect_min,
            rect_max,
            handle_color,
            corner_radius);

        // Add a subtle border to the handle for better visibility
        draw_list->AddRect(
            rect_min,
            rect_max,
            IM_COL32(255, 255, 255, 100),
            corner_radius,
            0, 2.0f);
    }

    void checkMemoryUsage()
    {
        // Linux memory check
        std::ifstream status("/proc/self/status");
        std::string line;
        while (std::getline(status, line))
        {
            if (line.substr(0, 6) == "VmRSS:")
            {
                // Extract memory value
                std::istringstream iss(line);
                std::string label, value, unit;
                iss >> label >> value >> unit;
                last_memory_check_ = std::stoul(value) / 1024; // Convert to MB
                break;
            }
        }
    }

    void drawThickCoordinateFrame(glk::GLSLShader &shader, const glk::Drawable &cube,
                                  const Eigen::Isometry3f &transform, float length, float thickness)
    {
        // X-axis (Red) - Use Affine3f instead of Isometry3f for scaling
        {
            Eigen::Affine3f x_transform(transform.matrix());
            x_transform.translate(Eigen::Vector3f(length / 2, 0, 0)); // Move to center of axis
            x_transform.scale(Eigen::Vector3f(length, thickness, thickness));

            shader.set_uniform("model_matrix", x_transform.matrix());
            shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f)); // Red
            cube.draw(shader);
        }

        // Y-axis (Green)
        {
            Eigen::Affine3f y_transform(transform.matrix());
            y_transform.translate(Eigen::Vector3f(0, length / 2, 0)); // Move to center of axis
            y_transform.scale(Eigen::Vector3f(thickness, length, thickness));

            shader.set_uniform("model_matrix", y_transform.matrix());
            shader.set_uniform("material_color", Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f)); // Green
            cube.draw(shader);
        }

        // Z-axis (Blue)
        {
            Eigen::Affine3f z_transform(transform.matrix());
            z_transform.translate(Eigen::Vector3f(0, 0, length / 2)); // Move to center of axis
            z_transform.scale(Eigen::Vector3f(thickness, thickness, length));

            shader.set_uniform("model_matrix", z_transform.matrix());
            shader.set_uniform("material_color", Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f)); // Blue
            cube.draw(shader);
        }
    }

    // Function to load image as OpenGL texture
    GLuint loadImageTexture(const std::string &path)
    {
        int width, height, channels;
        unsigned char *data = stbi_load(path.c_str(), &width, &height, &channels, 0);

        if (!data)
        {
            // std::cerr << "Failed to load image: " << path << std::endl;
            return 0;
        }

        // Debug: Print image info
        std::cout << "Loaded image: " << path << std::endl;
        std::cout << "  Resolution: " << width << "x" << height << std::endl;
        std::cout << "  Channels: " << channels << std::endl;

        GLuint texture_id;
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);

        // Improved texture parameters for better quality
        if (width >= 100 || height >= 100)
        {
            // For larger images, use linear filtering with mipmaps
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        }
        else
        {
            // For smaller images, use nearest neighbor to maintain sharpness
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        }

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // Upload texture data
        GLenum format = (channels == 4) ? GL_RGBA : GL_RGB;
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);

        // Generate mipmaps for better scaling quality
        if (width >= 100 || height >= 100)
        {
            glGenerateMipmap(GL_TEXTURE_2D);
            std::cout << "  Generated mipmaps for better scaling" << std::endl;
        }

        stbi_image_free(data);

        // Store original dimensions for better scaling decisions
        std::cout << "  Texture ID: " << texture_id << std::endl;

        return texture_id;
    }

    bool initMapboxMap()
    {
        try
        {
            std::cout << "Starting Mapbox initialization..." << std::endl;

            // Check if we have valid framebuffer size
            auto size = framebuffer_size();
            if (size.x() <= 0 || size.y() <= 0)
            {
                std::cerr << "Invalid framebuffer size: " << size.x() << "x" << size.y() << std::endl;
                return false;
            }
            std::cout << "Framebuffer size: " << size.x() << "x" << size.y() << std::endl;

            // Create the MapSnapshotter
            map_snapshotter_ = std::make_unique<mbgl::SimpleMapSnapshotter>(
                mbgl::Size{static_cast<uint32_t>(size.x()), static_cast<uint32_t>(size.y())},
                1.0f,
                "pk.eyJ1IjoiYXJtYWdlbmlzcyIsImEiOiJjbWRucWJ2enUwNHdwMm5wczE2YWU0ejZ4In0.lVcJwTwb-2n0yGo4bKeWEQ",
                "AIzaSyDLvsW4iy1cwlRv7JGd6xp49cs60YNuyJs");

            map_snapshotter_->setCircleLatLng(25.651454988788377, -100.29369354881304);

            if (!map_snapshotter_)
            {
                std::cerr << "Failed to create MapSnapshotter" << std::endl;
                return false;
            }

            // 🆕 NEW: Set a Mapbox style URL instead of manual layers
            std::cout << "Setting Mapbox style URL..." << std::endl;
            map_snapshotter_->setStyleURL("mapbox://styles/mapbox/streets-v12"); // Modern streets style

            // �� NEW: Set up camera with Mapbox types - CLOSER VIEW for 3D buildings
            std::cout << "Setting up camera with Mapbox types..." << std::endl;
            mbgl::CameraOptions camera;
            // 25.64843558669156, -100.29043509301644
            float lat = 25.64843558669156;
            float log = -100.29043509301644;
            camera.withCenter(mbgl::LatLng{lat, log});
            camera.withZoom(15.0); // Much closer zoom level for building details
            camera.withPitch(0.0); // Steeper pitch to see buildings better
            camera.withBearing(0.0);
            map_snapshotter_->setCameraOptions(camera);

            std::cout << "Setting up geographic coordinate system..." << std::endl;

            // Set the geographic bounds for your map view
            mbgl::LatLngBounds mapBounds = mbgl::LatLngBounds::hull(
                mbgl::LatLng{lat, log}, // Southwest (southwest)
                mbgl::LatLng{lat, log}  // Northeast (northeast)
            );

            // Update the map snapshotter with bounds
            map_snapshotter_->setMapBounds(mapBounds);

            std::cout << "✅ Geographic bounds set: "
                      << "N:" << mapBounds.north() << " S:" << mapBounds.south()
                      << " E:" << mapBounds.east() << " W:" << mapBounds.west() << std::endl;

            // 🆕 NEW: Start map loading to fetch the image
            std::cout << "Starting map loading..." << std::endl;
            map_snapshotter_->snapshot([](std::exception_ptr error, std::vector<uint8_t> imageData, int width, int height)
                                       {
                if (error) {
                    std::cerr << "❌ Map loading failed" << std::endl;
                } else {
                    std::cout << "✅ Map loaded successfully: " << width << "x" << height << " (" << imageData.size() << " bytes)" << std::endl;
                } });

            std::cout << "✅ Mapbox initialization completed successfully with style URL!" << std::endl;
            std::cout << "   Map will load layers and sources from Mapbox servers" << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Mapbox initialization error: " << e.what() << std::endl;
            return false;
        }
        catch (...)
        {
            std::cerr << "Unknown error during Mapbox initialization" << std::endl;
            return false;
        }
    }

    // =================================================================================
    //  ros2 and tf2 related functions
    // =================================================================================

    void update_tf_transforms()
    {

        std::lock_guard<std::mutex> lock(tf_mutex_);
        // Skip if no fixed frame selected
        if (fixed_frame_.empty())
            return;

        frame_transforms_.clear();
        // Refresh transforms for each available frame
        for (const auto &frame : available_frames_)
        {
            if (frame == fixed_frame_)
                continue;
            try
            {
                auto tf_stamped = tf_buffer_->lookupTransform(
                    fixed_frame_,      // target frame
                    frame,             // source frame
                    tf2::TimePointZero // latest
                );
                frame_transforms_[frame] = toEigen(tf_stamped.transform);
            }
            catch (const tf2::TransformException &)
            {
                // Remove on failure
                frame_transforms_.erase(frame);
            }
        }
        // print size of frame_transforms_
        // std::cout << green << "TF Transforms: " << frame_transforms_.size() << reset << std::endl;
    }

    void resetTFBuffer()
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);

        std::cout << yellow << "Resetting TF Buffer..." << reset << std::endl;

        // Clear current transforms
        frame_transforms_.clear();
        // Reset TF listener and buffer
        tf_listener_.reset();
        tf_buffer_.reset();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            node_->get_clock(),
            tf2::durationFromSec(10.0));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::cout << green << "TF Buffer reset complete" << reset << std::endl;
    }

    void refresh_frame_list()
    {

        std::lock_guard<std::mutex>
            lock(frames_mutex_);

        // Get all frames from TF2
        std::string all_frames;
        try
        {
            all_frames = tf_buffer_->allFramesAsString();
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get frames: %s", ex.what());
            return;
        }

        // std::cout << "All frames: " << all_frames << std::endl;

        // Parse the string to get individual frame names
        std::unordered_map<std::string, std::string> frame_to_parent;
        std::istringstream ss(all_frames);
        std::string line;
        while (std::getline(ss, line))
        {
            // Skip empty lines
            if (line.empty())
                continue;

            // Check if the line matches the expected format
            if (line.find("Frame ") == 0 && line.find(" exists with parent ") != std::string::npos)
            {
                // Extract frame name between "Frame " and " exists with parent"
                size_t start_pos = 6; // Length of "Frame "
                size_t end_pos = line.find(" exists with parent");
                size_t parent_start = end_pos + std::string(" exists with parent ").size();

                if (end_pos != std::string::npos && end_pos > start_pos)
                {
                    std::string frame_name = line.substr(start_pos, end_pos - start_pos);
                    std::string parent_name = line.substr(parent_start);

                    if (!parent_name.empty() && parent_name.back() == '.')
                        parent_name.pop_back();

                    // std::cout << "Found frame: '" << frame_name << "' with parent: '" << parent_name << "'" << std::endl;
                    frame_to_parent[frame_name] = parent_name;
                }
            }
        }

        // check if the frame-parent connection is valid

        rclcpp::Time now = node_->get_clock()->now();

        available_frames_.clear();
        for (const auto &[frame, parent] : frame_to_parent)
        {
            available_frames_.push_back(frame);
            if (std::find(available_frames_.begin(), available_frames_.end(), parent) == available_frames_.end())
            {
                available_frames_.push_back(parent);
            }
        }

        // Remove duplicates
        std::sort(available_frames_.begin(), available_frames_.end());
        available_frames_.erase(
            std::unique(available_frames_.begin(), available_frames_.end()),
            available_frames_.end());

        // std::cout << green << "Available frames: " << available_frames_.size() << reset << std::endl;
        // for (const auto &frame : available_frames_)
        // {
        //     std::cout << green << " - " << frame << reset << std::endl;
        // }

        last_frame_refresh_ = std::chrono::steady_clock::now();
    }

    void refresh_topic_list()
    {
        std::lock_guard<std::mutex> lock(topics_mutex_);

        // Discover all topic→types pairs
        auto topics_and_types = node_->get_topic_names_and_types();

        // Clear & rebuild the list, but only keep topics with >0 publishers
        topic_names_.clear();
        topic_types_.clear();
        for (auto &kv : topics_and_types)
        {
            const auto &name = kv.first;
            if (node_->count_publishers(name) == 0)
            {
                // no one is publishing → skip
                continue;
            }
            topic_names_.push_back(name);
            // Store the first type (most topics have only one type)
            if (!kv.second.empty())
            {
                topic_types_[name] = kv.second[0];
            }
        }

        // Prune dead subscriptions (topics that disappeared from DDS)
        {
            std::unordered_set<std::string> current(topic_names_.begin(),
                                                    topic_names_.end());
            for (auto it = subs_.begin(); it != subs_.end();)
            {
                if (!current.count(it->first))
                {
                    // Topic live no longer exists
                    last_msg_time_.erase(it->first);

                    // hz topic clean
                    topic_message_times_.erase(it->first); // Clean up frequency data
                    topic_hz_.erase(it->first);

                    // Clean up topic type data
                    topic_types_.erase(it->first);

                    it = subs_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        // Create lightweight heartbeat subscriptions with minimal memory footprint
        for (auto &name : topic_names_)
        {
            if (!subs_.count(name))
            {
                auto &types = topics_and_types[name];
                if (types.empty())
                    continue; // should never happen
                auto type_name = types[0];
                try
                {
                    // OPTIMIZATION 1: Use minimal QoS settings to reduce memory
                    auto s = node_->create_generic_subscription(
                        name, type_name,
                        rclcpp::QoS(1)
                            .keep_last(1)           // Only keep 1 message
                            .best_effort()          // Use best effort delivery
                            .durability_volatile(), // Don't persist messages
                        [this, name](std::shared_ptr<rclcpp::SerializedMessage> /* msg */)
                        {
                            std::lock_guard<std::mutex> lk(topics_mutex_);
                            auto now = std::chrono::steady_clock::now();
                            last_msg_time_[name] = now;

                            // FIXED: Use a proper sliding window approach for stable Hz calculation
                            auto &times = topic_message_times_[name];
                            times.push_back(now);

                            // Keep only recent messages (last 2 seconds worth)
                            auto cutoff = now - std::chrono::seconds(2);
                            while (times.size() > ROS2_LIKE_WINDOW_SIZE)
                            {
                                times.pop_front();
                            }

                            // Calculate Hz only if we have enough samples
                            if (times.size() >= 2)
                            {
                                auto duration = times.back() - times.front();
                                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                                if (ms > 0)
                                {
                                    float intervals = static_cast<float>(times.size() - 1);
                                    topic_hz_[name] = (intervals * 1000.0f) / ms;
                                }
                            }
                            else
                            {
                                topic_hz_[name] = 0.0f;
                            }
                        });
                    subs_[name] = s;
                    last_msg_time_[name] = std::chrono::steady_clock::time_point{};
                }
                catch (const std::exception &e)
                {
                    // std::cerr << "Failed to create subscription for topic " << name << ": " << e.what() << std::endl;
                }
            }
        }
    }

    // ================================
    // ROS2 point cloud topic
    // ================================
    void removePointCloudTopic(const std::string &topic)
    {
        std::lock_guard lk(cloud_topics_mutex_);
        for (auto it = cloud_topics_.begin(); it != cloud_topics_.end(); ++it)
        {
            if ((*it)->topic_name == topic)
            {
                std::cout << red << "Removing topic: " << topic << reset << std::endl;
                (*it)->sub.reset();
                cloud_topics_.erase(it);
                return;
            }
        }
        std::cout << red << "Topic not found: " << topic << reset << std::endl;
    }

    void addPointCloudTopic(const std::string &topic)
    {
        // avoid duplicates
        {
            std::lock_guard lk(cloud_topics_mutex_);
            for (auto &ct_ptr : cloud_topics_)
                if (ct_ptr->topic_name == topic)
                    return;
        }

        // create a new CloudTopic and initialize its cloud pointer
        auto ct = std::make_shared<CloudTopic>();
        ct->topic_name = topic;
        ct->cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // subscription callback
        ct->sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this, ct, topic](sensor_msgs::msg::PointCloud2::UniquePtr msg)
            {
                // build a fresh cloud
                auto new_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                pcl::fromROSMsg(*msg, *new_cloud);

                if (new_cloud->empty())
                {
                    std::cout << "[PC_CB] " << topic << " → empty cloud\n";
                    return;
                }

                {
                    std::lock_guard lk(cloud_topics_mutex_);
                    ct->cloud = new_cloud;
                    ct->frame_id = msg->header.frame_id;
                    ct->debug_pts.clear();
                    for (size_t i = 0; i < 10 && i < ct->cloud->size(); ++i)
                        ct->debug_pts.emplace_back(
                            ct->cloud->points[i].x,
                            ct->cloud->points[i].y,
                            ct->cloud->points[i].z);
                    ct->num_points = ct->debug_pts.size();

                    try
                    {
                        auto tf_stamped = tf_buffer_->lookupTransform(
                            fixed_frame_, ct->frame_id, tf2::TimePointZero);
                        ct->transform = toEigen(tf_stamped.transform);
                    }
                    catch (...)
                    { /* leave identity */
                    }
                }

                // std::cout << "[PC_CB] " << topic
                //           << " → " << ct->cloud->size() << " pts"
                //           << "  (debug_pts=" << ct->debug_pts.size() << ")\n";
            });

        // store the new topic
        {
            std::lock_guard lk(cloud_topics_mutex_);
            cloud_topics_.push_back(ct);
        }
    }

    // ================================
    // ROS2 GPS topic
    // ================================
    void addGPSTopic(const std::string &topic)
    {
        // Avoid duplicates
        if (gnss_subscription_)
        {
            std::cout << yellow << "GPS subscription already exists for: " << topic << reset << std::endl;
            return;
        }

        gnss_subscription_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            topic,
            rclcpp::SensorDataQoS(),
            [this, topic](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
            {
                // Check if GPS data is valid
                if (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX)
                {
                    current_latitude_ = msg->latitude;
                    current_longitude_ = msg->longitude;
                    gps_data_valid_ = true;

                    // Update map circle position (green dot)
                    if (map_snapshotter_)
                    {
                        map_snapshotter_->setCircleLatLng(current_latitude_, current_longitude_);

                        // 🎯 Move the LEFT camera to follow GPS position WITHOUT changing map origin
                        // Convert GPS lat/lng to world space coordinates (meters)
                        glm::vec2 gps_world_pos = map_snapshotter_->latLngToWorldMeters(
                            mbgl::LatLng(current_latitude_, current_longitude_));

                        // Set the left camera's center to the GPS world position
                        // This makes the camera look at the GPS position while keeping the map origin fixed
                        left_camera_control_.setCenter(Eigen::Vector3f(gps_world_pos.x, gps_world_pos.y, 0.0f));

                        // std::cout << "📍 Left camera following GPS at world position: ("
                        //           << gps_world_pos.x << ", " << gps_world_pos.y << ")" << std::endl;
                    }
                }
                else
                {
                    gps_data_valid_ = false;
                    std::cout << "⚠️ GPS fix not available, status: " << (int)msg->status.status << std::endl;
                }
            });

        std::cout << green << "✅ Subscribed to GPS topic: " << topic << reset << std::endl;
    }

    void removeGPSTopic(const std::string &topic)
    {
        if (gnss_subscription_)
        {
            std::cout << red << "Removing GPS subscription: " << topic << reset << std::endl;
            gnss_subscription_.reset();
            gps_data_valid_ = false;
        }
        else
        {
            std::cout << red << "No GPS subscription found for: " << topic << reset << std::endl;
        }
    }

    // ================================
    // ROS2 MarkerArray topic
    // ================================

    void addMarkerArrayTopic(const std::string &topic)
    {
        // avoid duplicates
        {
            std::lock_guard lk(marker_array_topics_mutex_);
            for (auto &mt_ptr : marker_array_topics_)
                if (mt_ptr->topic_name == topic)
                    return;
        }

        // create a new MarkerArrayTopic
        auto mt = std::make_shared<MarkerArrayTopic>();
        mt->topic_name = topic;
        mt->marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

        // subscription callback
        mt->sub = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            topic, rclcpp::SensorDataQoS(),
            [this, mt, topic](visualization_msgs::msg::MarkerArray::SharedPtr msg)
            {
                {
                    std::lock_guard lk(marker_array_topics_mutex_);
                    mt->marker_array = msg;
                    mt->last_update = std::chrono::steady_clock::now();

                    if (!msg->markers.empty())
                    {
                        mt->frame_id = msg->markers[0].header.frame_id;

                        try
                        {
                            auto tf_stamped = tf_buffer_->lookupTransform(
                                fixed_frame_, mt->frame_id, tf2::TimePointZero);
                            mt->transform = toEigen(tf_stamped.transform);
                        }
                        catch (...)
                        { /* leave identity */
                        }
                    }
                }

                std::cout << "[MA_CB] " << topic << " → " << msg->markers.size() << " markers\n";
            });

        // store the new topic
        {
            std::lock_guard lk(marker_array_topics_mutex_);
            marker_array_topics_.push_back(mt);
        }
    }

    void removeMarkerArrayTopic(const std::string &topic)
    {
        std::lock_guard lk(marker_array_topics_mutex_);
        for (auto it = marker_array_topics_.begin(); it != marker_array_topics_.end(); ++it)
        {
            if ((*it)->topic_name == topic)
            {
                std::cout << red << "Removing marker array topic: " << topic << reset << std::endl;
                (*it)->sub.reset();
                marker_array_topics_.erase(it);
                return;
            }
        }
        std::cout << red << "Marker array topic not found: " << topic << reset << std::endl;
    }

    // ================================
    // ROS2 Path topic
    // ================================

    void addPathTopic(const std::string &topic, int path_type = 0, int color_mode = 0, int path_color_selection = 0)
    {
        // avoid duplicates
        {
            std::lock_guard lk(path_topics_mutex_);
            for (auto &pt_ptr : path_topics_)
                if (pt_ptr->topic_name == topic)
                    return;
        }

        // create a new PathTopic
        auto pt = std::make_shared<PathTopic>();
        pt->topic_name = topic;
        pt->path = std::make_shared<nav_msgs::msg::Path>();
        pt->path_type = path_type;   // Set the path type (0 = normal, 1 = car)
        pt->color_mode = color_mode; // Set the color mode (0 = flat, 1 = gradient)
        pt->path_color_selection = path_color_selection; // Set the color selection (0 = orange, 1 = blue)

        // subscription callback
        pt->sub = node_->create_subscription<nav_msgs::msg::Path>(
            topic, rclcpp::QoS(10),
            [this, pt, topic](nav_msgs::msg::Path::SharedPtr msg)
            {
                {
                    std::lock_guard lk(path_topics_mutex_);
                    pt->path = msg;
                    pt->last_update = std::chrono::steady_clock::now();

                    if (!msg->poses.empty())
                    {
                        pt->frame_id = msg->header.frame_id;
                        pt->num_points = msg->poses.size();

                        try
                        {
                            auto tf_stamped = tf_buffer_->lookupTransform(
                                fixed_frame_, pt->frame_id, tf2::TimePointZero);
                            pt->transform = toEigen(tf_stamped.transform);
                        }
                        catch (...)
                        { /* leave identity */
                        }
                    }
                }

                // std::cout << "[PATH_CB] " << topic << " → " << msg->poses.size() << " poses\n";
            });

        // store the new topic
        {
            std::lock_guard lk(path_topics_mutex_);
            path_topics_.push_back(pt);
        }

        const char *type_str = (path_type == 0) ? "Normal Line" : "Car Path (1.5m width)";
        const char *color_mode_str;
        if (color_mode == 0)
        {
            color_mode_str = "Flat";
        }
        else if (color_mode == 1)
        {
            color_mode_str = "Gradient (Center→Edges)";
        }
        else
        {
            color_mode_str = "Gradient (Edges→Center)";
        }
        
        const char *path_color_str = (path_color_selection == 0) ? "Orange" : "Blue";

        if (path_type == 0)
        {
            std::cout << green << "✅ Subscribed to Path topic: " << topic
                      << " as " << type_str << " [" << path_color_str << "]" << reset << std::endl;
        }
        else
        {
            std::cout << green << "✅ Subscribed to Path topic: " << topic
                      << " as " << type_str << " [" << color_mode_str << ", " << path_color_str << "]" << reset << std::endl;
        }
    }

    void removePathTopic(const std::string &topic)
    {
        std::lock_guard lk(path_topics_mutex_);
        for (auto it = path_topics_.begin(); it != path_topics_.end(); ++it)
        {
            if ((*it)->topic_name == topic)
            {
                std::cout << red << "Removing path topic: " << topic << reset << std::endl;

                // Cleanup OpenGL resources
                if ((*it)->vao != 0)
                {
                    glDeleteVertexArrays(1, &(*it)->vao);
                }
                if ((*it)->vbo != 0)
                {
                    glDeleteBuffers(1, &(*it)->vbo);
                }
                if ((*it)->color_vbo != 0)
                {
                    glDeleteBuffers(1, &(*it)->color_vbo);
                }

                (*it)->sub.reset();
                path_topics_.erase(it);
                return;
            }
        }
        std::cout << red << "Path topic not found: " << topic << reset << std::endl;
    }

private:
    // color for the terminals
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

    // Camera controls - separate for each viewport
    guik::ArcCameraControl left_camera_control_;  // For map viewport
    guik::ArcCameraControl right_camera_control_; // For 3D content viewport
    guik::ArcCameraControl *active_camera_;       // Currently active camera

    // control variables for memory, GPU utilization and tf buffer
    size_t last_memory_check_ = 0;
    size_t last_gpu_memory_check_ = 0;
    float last_gpu_utilization_ = 0.0f;

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
    std::vector<std::string> topic_names_;                     // Simplified to just store names
    std::unordered_map<std::string, std::string> topic_types_; // Map topic name to type
    std::chrono::time_point<std::chrono::steady_clock> last_topic_refresh_;
    const std::chrono::milliseconds topic_refresh_interval_{1000};

    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_msg_time_;
    std::chrono::milliseconds alive_threshold_{1000}; // e.g. 1.0s

    // topic hz
    std::unordered_map<std::string, std::deque<std::chrono::steady_clock::time_point>> topic_message_times_;
    std::unordered_map<std::string, float> topic_hz_;
    static constexpr size_t ROS2_LIKE_WINDOW_SIZE = 100;

    bool show_topics_window = false;

    // TF visualization
    bool show_tf_frames_ = true;
    float tf_frame_size_ = 0.4f; // Size of the coordinate axes for each frame

    // Store transforms from fixed frame to all other frames
    std::mutex tf_mutex_;
    std::unordered_map<std::string, Eigen::Isometry3f> frame_transforms_ = {
        {"map", Eigen::Isometry3f::Identity()} // Default frame
    };

    // Store point cloud topics
    std::vector<std::shared_ptr<CloudTopic>> cloud_topics_;
    std::mutex cloud_topics_mutex_;
    int selected_topic_idx_{-1};

    // point cloud VAO/VBO
    GLuint dbgVao = 0;
    GLuint dbgVbo = 0;

    // Store marker array topics
    std::vector<std::shared_ptr<MarkerArrayTopic>> marker_array_topics_;
    std::mutex marker_array_topics_mutex_;
    int selected_marker_topic_idx_{-1};

    // Store path topics
    std::vector<std::shared_ptr<PathTopic>> path_topics_;
    std::mutex path_topics_mutex_;

    // lights
    std::chrono::steady_clock::time_point blink_start_ = std::chrono::steady_clock::now();

    // FreeType and text rendering
    TextRenderer text_renderer_;

    // Shader-based path rendering
    glk::PathRenderer path_renderer_;
    
    // Volumetric cloud rendering
    glk::CloudRenderer cloud_renderer_;

    // PCD loader
    PclLoader pcd_loader_;
    std::string pcd_frame = "map";

    // for ply and glb model upload
    modelUpload model_upload_;
    GLuint glb_shader_program_ = 0;

    // mesh map variables
    PlyMesh loaded_mesh_map;
    bool mesh_map_loaded = false;

    // lanelet2 loader
    LaneletLoader lanelet_loader_;

    // mesh glb
    PlyMesh loaded_mesh_glb;
    bool mesh_glb_loaded = false;

    // Mapbox map variables
    std::unique_ptr<mbgl::SimpleMapSnapshotter> map_snapshotter_;
    bool map_initialized_ = false;
    GLuint map_texture_id_ = 0;

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> gnss_subscription_;
    bool gps_data_valid_ = false;
    double current_latitude_ = 25.651454988788377;
    double current_longitude_ = -100.29369354881304;

    // Split-screen mode control
    bool split_screen_mode_ = true;

    // Resizable splitter variables
    float split_ratio_ = 0.5f; // 0.0 = all left, 1.0 = all right, 0.5 = 50/50
    bool is_dragging_splitter_ = false;
    bool is_hovering_splitter_ = false;
    const float splitter_width_ = 8.0f; // Width of the splitter bar
    const float handle_radius_ = 12.0f; // Radius of the circular handle

    // Add these to your Ros2GLViewer class header (private section)
    enum class DockItem
    {
        TOPICS = 0,
        FRAMES,
        FILES,
        MODELS,
        SETTINGS,
        INFO
    };

    DockItem current_panel_ = DockItem::TOPICS;

    struct DockIcon
    {
        const char *icon;
        const char *tooltip;
        DockItem item;
        GLuint texture_id = 0;  // Add texture ID
        std::string image_path; // Add image path
    };

    std::vector<DockIcon> dock_icons_ = {
        {"📁", "Reset tf", DockItem::FILES, 0, "8.png"},
        {"🔗", "TF Frames", DockItem::FRAMES, 0, "7.png"},
        {"📡", "ROS2 Topics", DockItem::TOPICS, 0, "4.png"},
        {"🤖", "3D Models", DockItem::MODELS, 0, "5.png"},
        {"⚙️", "Lanelet", DockItem::SETTINGS, 0, "6.png"},
        {"ℹ️", "Info", DockItem::INFO, 0, "1.png"}};

    // Point size controls (RViz-style: 0-50 range)
    float ros2_topic_point_size_ = 100.0f; // Default 10 (medium size)
    float pcd_file_point_size_ = 5.0f;     // Default 5 (small size)
    
    // Rendering distance control
    float far_clipping_distance_ = 2000.0f; // Default far clipping plane distance
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("ros2_gui_viewer");
    // Create application
    std::unique_ptr<Ros2GLViewer> app(new Ros2GLViewer(ros_node));

    // Initialize application
    if (!app->init("Red Wine", Eigen::Vector2i(1280, 720)))
    {
        return 1;
    }

    // Set up signal handling for graceful shutdown
    signal(SIGINT, [](int)
           {
        std::cout << "\n🛑 Received SIGINT, shutting down gracefully..." << std::endl;
        rclcpp::shutdown(); });

    // Run application
    app->run();

    return 0;
}