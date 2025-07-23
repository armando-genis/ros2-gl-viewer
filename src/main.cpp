// OpenGL Version: 4.6 (Compatibility Profile) Mesa 23.2.1-1ubuntu3.1~22.04.3
// GLSL Version: 4.60

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
#include <glk/primitives/primitives.hpp>
#include <glk/lines.hpp>
#include <glk/colormap.hpp>
#include <glk/glsl_shader.hpp>
#include <glk/frame_buffer.hpp>
#include <glk/TextRendering.hpp>
#include "glk/PclLoader.hpp"
#include "glk/LaneletLoader.hpp"
#include "glk/modelUpload.hpp"

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

class Ros2GLViewer : public guik::Application
{
public:
    Ros2GLViewer(std::shared_ptr<rclcpp::Node> node) : guik::Application(), node_(node)
    {
        std::cout << "Basic Viewer Application initialized" << std::endl;
        // Initialize TF2
        // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock(), tf2::durationFromSec(10.0));

        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        tf_initialized_ = false;

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
        text_renderer_.cleanupTextRendering();
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
    }

    void setupCustomImGuiColors()
    {

        // color pallete generetor: https://colorkit.co/color-palette-generator/090b07-11150d-192014-212b1a-314026-526a40-739559-94bf73-a5d580-b5ea8c/
        // color set to hex to 1.0 rgb: https://rgbcolorpicker.com/0-1#google_vignette

        ImGuiStyle &style = ImGui::GetStyle();
        ImVec4 *colors = style.Colors;

        // Background colors
        colors[ImGuiCol_WindowBg] = ImVec4(0.322f, 0.416f, 0.251f, 0.9f); // Dark gray background
        colors[ImGuiCol_ChildBg] = ImVec4(0.098f, 0.125f, 0.078f, 0.40f); // Child window background
        colors[ImGuiCol_PopupBg] = ImVec4(0.098f, 0.125f, 0.078f, 0.40f); // Popup background

        // Button colors
        colors[ImGuiCol_Button] = ImVec4(0.451f, 0.584f, 0.349f, 0.8f); // Green button
        // colors[ImGuiCol_Button] = ImVec4(0.306f, 0.341f, 0.376f, 0.8f); // gray button

        colors[ImGuiCol_ButtonHovered] = ImVec4(0.471f, 0.576f, 0.541f, 1.00f); // Hovered button
        colors[ImGuiCol_ButtonActive] = ImVec4(0.4f, 0.42f, 0.369f, 1.00f);     // Pressed button

        // Title bar
        colors[ImGuiCol_TitleBg] = ImVec4(0.035f, 0.043f, 0.027f, 1.00f);
        colors[ImGuiCol_TitleBgActive] = ImVec4(0.035f, 0.043f, 0.027f, 0.792f);
        colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.035f, 0.043f, 0.027f, 0.792f);

        // Slider
        colors[ImGuiCol_SliderGrab] = ImVec4(0.639f, 0.718f, 0.792f, 1.00f);
        colors[ImGuiCol_SliderGrabActive] = ImVec4(0.639f, 0.718f, 0.792f, 1.00f);

        colors[ImGuiCol_FrameBg] = ImVec4(0.098f, 0.125f, 0.078f, 0.40f);
        colors[ImGuiCol_FrameBgHovered] = ImVec4(0.098f, 0.125f, 0.078f, 0.90f);
        colors[ImGuiCol_FrameBgActive] = ImVec4(0.098f, 0.125f, 0.078f, 0.90f);

        colors[ImGuiCol_Separator] = ImVec4(0.451f, 0.584f, 0.349f, 0.8f);
        colors[ImGuiCol_SeparatorHovered] = ImVec4(0.471f, 0.576f, 0.541f, 1.00f);
        colors[ImGuiCol_SeparatorActive] = ImVec4(0.4f, 0.42f, 0.369f, 1.00f);

        // Style adjustments
        style.WindowRounding = 5.0f;    // Main window corners
        style.ChildRounding = 5.0f;     // Child window corners
        style.FrameRounding = 3.0f;     // Buttons, input fields, sliders
        style.PopupRounding = 3.0f;     // Popup windows
        style.ScrollbarRounding = 3.0f; // Scrollbar corners
        style.GrabRounding = 3.0f;      // Slider handles, scrollbar grabs
        style.TabRounding = 3.0f;       // Tab corners

        // ===== SIZES & SPACING =====
        style.WindowPadding = ImVec2(13.0f, 13.0f);  // Padding inside windows
        style.FramePadding = ImVec2(8.0f, 4.0f);     // Padding inside frames (buttons, inputs)
        style.ItemSpacing = ImVec2(8.0f, 6.0f);      // Spacing between items
        style.ItemInnerSpacing = ImVec2(6.0f, 4.0f); // Spacing inside items
        style.IndentSpacing = 20.0f;                 // Indentation for tree nodes
        style.ScrollbarSize = 16.0f;                 // Scrollbar width
        style.GrabMinSize = 12.0f;                   // Minimum slider/scrollbar grab size

        // ===== SPECIAL EFFECTS =====
        style.WindowMenuButtonPosition = ImGuiDir_Left; // Window menu button position
        style.ColorButtonPosition = ImGuiDir_Right;     // Color button position
        style.Alpha = 1.0f;                             // Global alpha multiplier
        style.DisabledAlpha = 0.5f;                     // Alpha for disabled items
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

        // ======= GLB SHADER INITIALIZATION =======
        if (!model_upload_.createGLBShader(glb_shader_program_))
        {
            std::cerr << "Failed to create GLB shader" << std::endl;
            return false;
        }
        std::cout << green << "Successfully initialized GLB shader" << reset << std::endl;

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
        // get the current time
        auto now = std::chrono::steady_clock::now();
        // get the tf based on the frame
        update_tf_transforms();

        // Topic hz
        updateTopicFrequencies(now);
        // Check if we need to refresh
        if (now - last_topic_refresh_ > topic_refresh_interval_)
        {
            refresh_topic_list();
        }

        if (now - last_frame_refresh_ > frame_refresh_interval_)
        {
            refresh_frame_list();
        }

        drawMacOSDock();
        drawTopicsPopup();
        drawFramesPopup();

        // Show main canvas settings
        main_canvas_->draw_ui();

        // Main options window
        ImGui::Begin("ros2 Viewer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        // Position window at bottom of screen
        // ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
        // float panel_height = 300.0f; // Adjust height as needed

        // ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
        // ImGui::SetNextWindowSize(ImVec2(450.0f, viewport_size.y), ImGuiCond_Always);

        // // Main options window - Fixed bottom panel
        // ImGui::Begin("ros2 Viewer", nullptr,
        //              ImGuiWindowFlags_NoMove |
        //                  ImGuiWindowFlags_NoResize |
        //                  ImGuiWindowFlags_NoCollapse |
        //                  ImGuiWindowFlags_NoTitleBar // Optional: removes title bar
        // );

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

        ImGui::Separator();
        if (ImGui::Button("Reset TF Buffer"))
        {
            resetTFBuffer();
        }

        ImGui::SameLine();
        ImGui::Text("TF Frames: %zu", frame_transforms_.size());

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

                    // Use consistent height for all elements
                    float element_height = std::max(line_height, circle_radius * 2);

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
                }
            }
            ImGui::EndChild();

            if (selected_topic_idx_ >= 0 && selected_topic_idx_ < (int)topic_names_.size())
            {
                ImGui::Text("Subscribe to:");
                ImGui::SameLine();
                if (ImGui::Button("+ Add"))
                {
                    addPointCloudTopic(topic_names_[selected_topic_idx_]);
                }
                if (ImGui::Button("- Remove"))
                {
                    std::cout << red << "Removing topic: " << topic_names_[selected_topic_idx_] << reset << std::endl;
                    removePointCloudTopic(topic_names_[selected_topic_idx_]);
                    selected_topic_idx_ = -1;
                }
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

        ImGui::Separator();

        // /workspace/models/sdv.ply

        // /workspace/models/drift.glb
        // /workspace/models/buggy.glb
        // /workspace/models/formula_uno_car_rot.glb
        // /workspace/models/formula_uno_car.glb

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

        ImGui::End();
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

        // 1) Compute View matrix
        Eigen::Matrix4f view = camera_control_.view_matrix();

        // 2) Compute Projection matrix manually
        float w = float(framebuffer_size().x());
        float h = float(framebuffer_size().y());
        float aspect = w / h;
        float fovY = 45.f * M_PI / 180.f;
        float zNear = 0.01f, zFar = 1000.f;
        float f = 1.f / std::tan(fovY / 2.f);

        Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
        proj(0, 0) = f / aspect;
        proj(1, 1) = f;
        proj(2, 2) = (zFar + zNear) / (zNear - zFar);
        proj(2, 3) = (2.f * zFar * zNear) / (zNear - zFar);
        proj(3, 2) = -1.f;

        // 4) bind the shader program
        auto &shader = *main_canvas_->shader;
        shader.use();
        shader.set_uniform("view_matrix", view);
        shader.set_uniform("projection_matrix", proj);
        model_upload_.setMatrices(view, proj);

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
            if (!fixed_frame_.empty())
            {
                Eigen::Isometry3f identity = Eigen::Isometry3f::Identity();
                drawThickCoordinateFrame(shader, cube, identity, axis_length, axis_thickness);

                // Add fixed frame name label at origin
                Eigen::Vector3f text_position = Eigen::Vector3f(0.0f, 0.0f, -0.1f);
                text_renderer_.addWorldText(fixed_frame_, text_position, Eigen::Vector3f(1.0f, 1.0f, 1.0f), 0.005f, 0.0f);
            }

            for (auto &p : frame_transforms_)
            {
                drawThickCoordinateFrame(shader, cube, p.second, axis_length, axis_thickness);

                // Add frame name label
                Eigen::Vector3f frame_position = p.second.translation();
                Eigen::Vector3f text_position = frame_position + Eigen::Vector3f(0.0f, 0.0f, -0.1f);
                text_renderer_.addWorldText(p.first, text_position, Eigen::Vector3f(1.0f, 1.0f, 1.0f), 0.005f, 0.0f);
            }
        }

        // ✅ IMPORTANT: Rebind main shader before point clouds
        shader.use();
        shader.set_uniform("view_matrix", view);
        shader.set_uniform("projection_matrix", proj);

        // ============================================
        // render point clouds ros2 topics
        // ============================================
        {
            std::lock_guard lk(cloud_topics_mutex_);
            // set uniform color-mode to “flat color”
            shader.set_uniform("color_mode", 0);

            shader.set_uniform("point_scale", 0.0f); // NO fall-off

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
        // ============================================
        // render PCD point cloud
        // ============================================
        if (pcd_loader_.isLoaded() || pcd_loader_.getPointCount() != 0)
        {
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
        }

        if (mesh_glb_loaded && loaded_mesh_glb.type == 1) // 1 = GLTF type
        {
            model_upload_.renderMesh(loaded_mesh_glb, shader, tf_mutex_, frame_transforms_);
        }
        // ============================================
        // render lanelet2 map
        // ============================================
        if (lanelet_loader_.isLoaded())
        {
            lanelet_loader_.mapLines(shader);
            lanelet_loader_.crosswalks(shader);
            lanelet_loader_.stripes(shader);
        }

        // render text
        text_renderer_.renderWorldText(view, proj);

        // 9) unbind & blit
        main_canvas_->unbind();
        main_canvas_->render_to_screen();
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

    void ensureTFInitialized()
    {
        if (!tf_initialized_)
        {
            std::cout << "🔧 Lazy initializing TF2..." << std::endl;
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
                node_->get_clock(),
                tf2::durationFromSec(10.0));
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_initialized_ = true;
            std::cout << "✅ TF2 initialized" << std::endl;
        }
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
                    // Topic live no longer exists
                    last_msg_time_.erase(it->first);

                    // hz topic clean
                    topic_message_times_.erase(it->first); // Clean up frequency data
                    topic_hz_.erase(it->first);

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
                try
                {
                    auto s = node_->create_generic_subscription(
                        name, type_name, rclcpp::QoS(1),
                        [this, name](std::shared_ptr<rclcpp::SerializedMessage>)
                        {
                            std::lock_guard<std::mutex> lk(topics_mutex_);
                            auto now = std::chrono::steady_clock::now();
                            last_msg_time_[name] = now;

                            // ROS2-like frequency tracking
                            auto &times = topic_message_times_[name];
                            times.push_back(now);

                            // Keep window size like ROS2 (but smaller for real-time)
                            while (times.size() > ROS2_LIKE_WINDOW_SIZE)
                            {
                                times.pop_front();
                            }

                            // Calculate Hz using ROS2's method
                            if (times.size() >= 2)
                            {
                                auto total_duration = times.back() - times.front();
                                auto total_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(total_duration).count();

                                if (total_ns > 0)
                                {
                                    // ROS2's algorithm: intervals / total_time
                                    float intervals = static_cast<float>(times.size() - 1);
                                    float total_seconds = total_ns / 1e9f;
                                    topic_hz_[name] = intervals / total_seconds;
                                }
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

    // Simple function to update Hz values (call this in draw_ui)
    void updateTopicFrequencies(std::chrono::steady_clock::time_point now)
    {
        std::lock_guard<std::mutex> lock(topics_mutex_);

        if (topic_message_times_.empty())
            return; // No topics to update

        for (auto &[topic_name, times] : topic_message_times_)
        {
            // Remove old messages (older than 2 seconds)
            auto cutoff = now - std::chrono::seconds(2);
            while (!times.empty() && times.front() < cutoff)
            {
                times.pop_front();
            }

            // Recalculate Hz
            if (times.size() >= 2)
            {
                auto duration = times.back() - times.front();
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                if (ms > 0)
                {
                    float intervals = static_cast<float>(times.size() - 1);
                    topic_hz_[topic_name] = (intervals * 1000.0f) / ms;
                }
            }
            else
            {
                topic_hz_[topic_name] = 0.0f;
            }
        }
    }

    // reference:
    // - https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
    // - https://robotics.stackexchange.com/questions/95228/how-to-get-list-of-all-tf-frames-programatically
    // - https://wiki.ros.org/tf/TfUsingPython
    void refresh_frame_list()
    {
        ensureTFInitialized();

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

    void update_tf_transforms()
    {
        ensureTFInitialized();

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

        tf_initialized_ = false;

        // Reset TF listener and buffer
        tf_listener_.reset();
        tf_buffer_.reset();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            node_->get_clock(),
            tf2::durationFromSec(10.0));
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::cout << green << "TF Buffer reset complete" << reset << std::endl;
    }

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
        ct->cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();

        // subscription callback
        ct->sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this, ct, topic](sensor_msgs::msg::PointCloud2::UniquePtr msg)
            {
                // build a fresh cloud
                auto new_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
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

    // Function to load image as OpenGL texture
    GLuint loadImageTexture(const std::string &path)
    {
        int width, height, channels;
        unsigned char *data = stbi_load(path.c_str(), &width, &height, &channels, 0);

        if (!data)
        {
            std::cerr << "Failed to load image: " << path << std::endl;
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

    void drawMacOSDock()
    {
        ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImVec2 viewport_size = viewport->Size;

        // Dock dimensions
        const float dock_height = 85.0f;
        const float icon_size = 55.0f;
        const float icon_spacing = 15.0f;
        const float dock_padding = 15.0f;

        // Performance panel dimensions
        const float perf_panel_width = 200.0f;
        const float panel_spacing = 20.0f;

        // Calculate dock width based on number of icons
        float dock_width = (icon_size * dock_icons_.size()) + (icon_spacing * (dock_icons_.size() - 1)) + (dock_padding * 2);

        // Calculate total width of both dock and performance panel
        float total_width = dock_width + panel_spacing + perf_panel_width;

        // Position dock so the ENTIRE layout
        ImVec2 dock_pos = ImVec2(
            (viewport_size.x - total_width) * 0.5f, // Center the whole thing
            viewport_size.y - dock_height - 20.0f   // 20px margin from bottom
        );

        // Set dock window properties
        ImGui::SetNextWindowPos(dock_pos);
        ImGui::SetNextWindowSize(ImVec2(dock_width, dock_height));

        // Dock window flags
        ImGuiWindowFlags dock_flags =
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoBringToFrontOnFocus;

        // Custom dock styling
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 25.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(dock_padding, dock_padding));

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(icon_spacing, 0));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.95f)); // Dark translucent
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));

        ImGui::Begin("##Dock", nullptr, dock_flags);

        // Draw dock icons horizontally
        for (size_t i = 0; i < dock_icons_.size(); ++i)
        {
            const auto &icon = dock_icons_[i];

            if (i > 0)
            {
                ImGui::SameLine();
            }

            // Highlight selected icon
            bool is_selected = (current_panel_ == icon.item);
            if (is_selected)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.4f, 0.6f, 1.0f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.5f, 0.7f, 1.0f, 0.9f));
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.2f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));
            }

            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, icon_size * 0.25f);

            // Create button with image or icon
            ImGui::PushID(static_cast<int>(i));

            bool clicked = false;

            if (icon.texture_id != 0)
            {
                // manually create the same background as text buttons
                ImVec2 button_pos = ImGui::GetCursorScreenPos();
                ImVec2 button_size = ImVec2(icon_size, icon_size);

                // Check if button is hovered
                ImVec2 mouse_pos = ImGui::GetMousePos();
                bool is_hovered = (mouse_pos.x >= button_pos.x && mouse_pos.x <= button_pos.x + button_size.x &&
                                   mouse_pos.y >= button_pos.y && mouse_pos.y <= button_pos.y + button_size.y);

                // Get the same colors used by regular buttons
                ImU32 bg_color;
                if (is_selected)
                {
                    bg_color = is_hovered ? ImGui::GetColorU32(ImVec4(0.5f, 0.7f, 1.0f, 0.9f)) : // ButtonHovered color
                                   ImGui::GetColorU32(ImVec4(0.4f, 0.6f, 1.0f, 0.8f));           // Button color
                }
                else
                {
                    bg_color = is_hovered ? ImGui::GetColorU32(ImVec4(0.3f, 0.3f, 0.3f, 0.8f)) : // ButtonHovered color
                                   ImGui::GetColorU32(ImVec4(0.2f, 0.2f, 0.2f, 0.6f));           // Button color
                }

                // Draw custom background with same rounding as text buttons
                ImGui::GetWindowDrawList()->AddRectFilled(
                    button_pos,
                    ImVec2(button_pos.x + button_size.x, button_pos.y + button_size.y),
                    bg_color,
                    icon_size * 0.25f);

                // Create invisible button for click detection
                ImGui::SetCursorScreenPos(button_pos);
                clicked = ImGui::InvisibleButton(("##invisible_" + std::to_string(i)).c_str(), button_size);

                // Draw the image centered on top
                ImVec2 image_size = ImVec2(icon_size - 8.0f, icon_size - 8.0f);
                ImVec2 image_pos = ImVec2(
                    button_pos.x + (button_size.x - image_size.x) * 0.5f,
                    button_pos.y + (button_size.y - image_size.y) * 0.5f);

                // Try adding a subtle border around the image for better definition
                float button_rounding = icon_size * 0.25f;
                ImVec2 border_padding = ImVec2(1.0f, 1.0f);

                ImGui::GetWindowDrawList()->AddRect(
                    ImVec2(button_pos.x + border_padding.x, button_pos.y + border_padding.y),
                    ImVec2(button_pos.x + button_size.x - border_padding.x, button_pos.y + button_size.y - border_padding.y),
                    IM_COL32(255, 255, 255, 30), // Very subtle white border
                    button_rounding - border_padding.x,
                    0, 1.0f // Border thickness
                );

                ImGui::GetWindowDrawList()->AddImage(
                    (ImTextureID)(intptr_t)icon.texture_id,
                    image_pos,
                    ImVec2(image_pos.x + image_size.x, image_pos.y + image_size.y),
                    ImVec2(0, 0), ImVec2(1, 1),
                    IM_COL32(255, 255, 255, 255) // White tint
                );
            }
            else
            {
                // Fallback to text button if no texture
                clicked = ImGui::Button(icon.icon, ImVec2(icon_size, icon_size));
            }

            if (clicked)
            {
                current_panel_ = icon.item;

                // Handle different dock items
                switch (icon.item)
                {
                case DockItem::TOPICS:
                    show_topics_popup_ = !show_topics_popup_;
                    std::cout << "Toggled ROS2 Topics popup" << std::endl;
                    break;
                case DockItem::FRAMES:
                    show_frames_popup_ = !show_frames_popup_;
                    std::cout << "Toggled TF Frames popup" << std::endl;
                    break;
                case DockItem::FILES:
                    show_files_popup_ = !show_files_popup_;
                    std::cout << "Toggled Files popup" << std::endl;
                    break;
                case DockItem::MODELS:
                    show_models_popup_ = !show_models_popup_;
                    std::cout << "Toggled Models popup" << std::endl;
                    break;
                case DockItem::SETTINGS:
                    show_settings_popup_ = !show_settings_popup_;
                    std::cout << "Toggled Settings popup" << std::endl;
                    break;
                case DockItem::INFO:
                    show_info_popup_ = !show_info_popup_;
                    std::cout << "Toggled Info popup" << std::endl;
                    break;
                }
            }

            ImGui::PopID();

            // Tooltip on hover
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", icon.tooltip);
            }

            ImGui::PopStyleVar();    // FrameRounding
            ImGui::PopStyleColor(2); // Button colors
        }

        ImGui::End();
        ImGui::PopStyleColor(2); // Window colors
        ImGui::PopStyleVar(3);

        // Performance info panel positioned relative to the dock
        ImVec2 info_pos = ImVec2(
            dock_pos.x + dock_width + panel_spacing, // Use the calculated spacing
            viewport_size.y - dock_height - 20.0f);

        ImGui::SetNextWindowPos(info_pos);
        ImGui::SetNextWindowSize(ImVec2(perf_panel_width, dock_height));

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 15.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(15.0f, 15.0f));
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.9f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));

        ImGui::Begin("##Performance", nullptr, dock_flags);

        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

        // Memory monitoring
        static int frame_count = 0;
        frame_count++;
        if (frame_count % 60 == 0)
        { // Check every ~1 second at 60fps
            checkMemoryUsage();
        }
        ImGui::Text("Memory: %zu MB", last_memory_check_);

        // numbers of topics
        // ImGui::Text("Topics: %zu", topic_names_.size());

        ImGui::End();
        ImGui::PopStyleColor(2);
        ImGui::PopStyleVar(2);
    }

    void drawTopicsPopup()
    {
        if (!show_topics_popup_)
            return;

        // Apply dock-style theming to the popup
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 15.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(15.0f, 15.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 6.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.0f);

        // Dock-style colors
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.95f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.3f, 0.3f, 0.3f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_TitleBg, ImVec4(0.15f, 0.15f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.2f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.2f, 0.2f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.3f, 0.3f, 0.9f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.4f, 0.4f, 0.4f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));

        // Set initial position and size
        ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(ImVec2(100, 100), ImGuiCond_FirstUseEver);

        // Create movable window
        ImGuiWindowFlags popup_flags =
            ImGuiWindowFlags_NoCollapse;

        if (ImGui::Begin("📡 ROS2 Topics Manager", &show_topics_popup_, popup_flags))
        {

            // Header with topic count
            ImGui::Text("📊 Available Topics: %zu", topic_names_.size());
            ImGui::Separator();

            // Topics list with custom styling
            ImGui::BeginChild("TopicsList", ImVec2(0, 280), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);
            {
                std::lock_guard<std::mutex> lock(topics_mutex_);
                auto now = std::chrono::steady_clock::now();

                for (int i = 0; i < (int)topic_names_.size(); ++i)
                {
                    const auto &name = topic_names_[i];

                    // Get Hz value
                    float hz = 0.0f;
                    auto hz_it = topic_hz_.find(name);
                    if (hz_it != topic_hz_.end())
                    {
                        hz = hz_it->second;
                    }

                    // Format Hz text
                    char hz_text[32];
                    snprintf(hz_text, sizeof(hz_text), "%.1f Hz", hz);

                    // Calculate elements dimensions
                    const float circle_radius = 6.0f;
                    const float padding = 4.0f;
                    const float rounding = 4.0f;
                    const float hz_square_fixed_width = 65.0f;

                    ImVec2 hz_text_size = ImGui::CalcTextSize(hz_text);
                    float line_height = ImGui::GetTextLineHeight();
                    float element_height = std::max(line_height + 8.0f, circle_radius * 2 + 4.0f);

                    // Store initial cursor position
                    ImVec2 row_start = ImGui::GetCursorScreenPos();

                    // Check if topic is alive
                    bool alive = false;
                    auto it = last_msg_time_.find(name);
                    if (it != last_msg_time_.end())
                        alive = (now - it->second) < alive_threshold_;

                    // Draw status indicator circle
                    ImU32 circle_col = alive ? IM_COL32(46, 204, 64, 255) : IM_COL32(255, 65, 54, 255);
                    float circle_y_center = row_start.y + element_height * 0.5f;
                    ImGui::GetWindowDrawList()->AddCircleFilled(
                        {row_start.x + circle_radius + 2.0f, circle_y_center},
                        circle_radius,
                        circle_col);

                    // Draw Hz badge
                    float hz_square_x = row_start.x + circle_radius * 2 + 12.0f;
                    float hz_square_height = hz_text_size.y + padding * 2;
                    float hz_square_y = row_start.y + (element_height - hz_square_height) * 0.5f;

                    ImVec2 hz_square_min = {hz_square_x, hz_square_y};
                    ImVec2 hz_square_max = {hz_square_x + hz_square_fixed_width, hz_square_y + hz_square_height};

                    // Hz badge background
                    ImU32 badge_color = hz > 0 ? IM_COL32(40, 44, 52, 220) : IM_COL32(60, 20, 20, 220);
                    ImGui::GetWindowDrawList()->AddRectFilled(
                        hz_square_min, hz_square_max,
                        badge_color, rounding);

                    // Hz badge border
                    ImU32 border_color = hz > 0 ? IM_COL32(100, 120, 140, 180) : IM_COL32(120, 60, 60, 180);
                    ImGui::GetWindowDrawList()->AddRect(
                        hz_square_min, hz_square_max,
                        border_color, rounding, 0, 1.0f);

                    // Hz text centered in badge
                    ImVec2 hz_text_pos = {
                        hz_square_x + (hz_square_fixed_width - hz_text_size.x) * 0.5f,
                        hz_square_y + padding};
                    ImU32 hz_text_color = hz > 0 ? IM_COL32(220, 220, 220, 255) : IM_COL32(180, 120, 120, 255);
                    ImGui::GetWindowDrawList()->AddText(hz_text_pos, hz_text_color, hz_text);

                    // Topic name area
                    float topic_name_x = hz_square_max.x + 12.0f;
                    float topic_name_y = row_start.y + (element_height - line_height) * 0.5f;

                    // Set cursor for selectable
                    ImGui::SetCursorScreenPos({topic_name_x, row_start.y});

                    float available_width = ImGui::GetContentRegionAvail().x;
                    bool is_selected = (i == selected_topic_idx_);

                    // Custom selectable styling
                    if (is_selected)
                    {
                        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.3f, 0.5f, 0.8f, 0.6f));
                        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.4f, 0.6f, 0.9f, 0.7f));
                    }
                    else
                    {
                        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.2f, 0.2f, 0.2f, 0.3f));
                        ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.25f, 0.25f, 0.25f, 0.5f));
                    }

                    ImGui::PushID(i);
                    if (ImGui::Selectable("##topic_select", is_selected, 0, ImVec2(available_width, element_height)))
                    {
                        selected_topic_idx_ = i;
                    }
                    ImGui::PopID();
                    ImGui::PopStyleColor(2);

                    // Draw topic name
                    ImU32 text_color = is_selected ? IM_COL32(255, 255, 255, 255) : IM_COL32(200, 200, 200, 255);
                    ImGui::GetWindowDrawList()->AddText(
                        {topic_name_x + 8.0f, topic_name_y},
                        text_color,
                        name.c_str());
                }
            }
            ImGui::EndChild();

            ImGui::Separator();

            // Action buttons
            if (selected_topic_idx_ >= 0 && selected_topic_idx_ < (int)topic_names_.size())
            {
                ImGui::Text("📝 Selected: %s", topic_names_[selected_topic_idx_].c_str());

                ImGui::Spacing();

                // Subscribe button
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.2f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.3f, 0.7f, 0.3f, 0.9f));
                if (ImGui::Button("➕ Subscribe", ImVec2(120, 30)))
                {
                    addPointCloudTopic(topic_names_[selected_topic_idx_]);
                }
                ImGui::PopStyleColor(2);

                ImGui::SameLine();

                // Unsubscribe button
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.2f, 0.2f, 0.8f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.3f, 0.3f, 0.9f));
                if (ImGui::Button("➖ Unsubscribe", ImVec2(120, 30)))
                {
                    std::cout << "Removing topic: " << topic_names_[selected_topic_idx_] << std::endl;
                    removePointCloudTopic(topic_names_[selected_topic_idx_]);
                    selected_topic_idx_ = -1;
                }
                ImGui::PopStyleColor(2);
            }
            else
            {
                ImGui::TextDisabled("💡 Select a topic to subscribe/unsubscribe");
            }
        }
        ImGui::End();

        // Pop all style modifications
        ImGui::PopStyleColor(8);
        ImGui::PopStyleVar(4);
    }

    void drawFramesPopup()
    {
        if (!show_frames_popup_)
            return;

        ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImVec2 viewport_size = viewport->Size;

        // Calculate dock position (same as in drawMacOSDock)
        const float dock_height = 80.0f;
        const float icon_size = 50.0f;
        const float icon_spacing = 15.0f;
        const float dock_padding = 15.0f;
        const float perf_panel_width = 200.0f;
        const float panel_spacing = 20.0f;

        float dock_width = (icon_size * dock_icons_.size()) + (icon_spacing * (dock_icons_.size() - 1)) + (dock_padding * 2);
        float total_width = dock_width + panel_spacing + perf_panel_width;

        ImVec2 dock_pos = ImVec2(
            (viewport_size.x - total_width) * 0.5f,
            viewport_size.y - dock_height - 20.0f);

        // Popup dimensions
        const float popup_width = 450.0f;
        const float popup_height = 280.0f;
        const float popup_margin = 15.0f;

        // Position popup above the dock, centered
        ImVec2 popup_pos = ImVec2(
            (viewport_size.x - popup_width) * 0.5f,  // Center horizontally
            dock_pos.y - popup_height - popup_margin // Above dock with margin
        );

        // Apply dock-style theming
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 20.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0f, 18.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 8.0f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);

        // Elegant dark theme with subtle transparency
        ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.08f, 0.96f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.35f, 0.35f, 0.35f, 0.8f));
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.12f, 0.12f, 0.9f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.95f, 0.95f, 1.0f));

        // Set fixed position and size
        ImGui::SetNextWindowPos(popup_pos);
        ImGui::SetNextWindowSize(ImVec2(popup_width, popup_height));

        ImGuiWindowFlags popup_flags =
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoSavedSettings;

        if (ImGui::Begin("##FramesPopup", &show_frames_popup_, popup_flags))
        {

            // Stylish header with icon and current frame info
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.8f, 1.0f, 1.0f)); // Blue accent
            ImGui::Text("🔗 Transform Frames");
            ImGui::PopStyleColor();

            ImGui::SameLine();
            ImGui::Spacing();
            ImGui::SameLine();

            // Current frame indicator
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.7f, 0.9f, 0.7f, 1.0f)); // Green accent
            ImGui::Text("Current: %s", fixed_frame_.empty() ? "None" : fixed_frame_.c_str());
            ImGui::PopStyleColor();

            // Separator with custom color
            ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(0.4f, 0.4f, 0.4f, 0.8f));
            ImGui::Separator();
            ImGui::PopStyleColor();

            ImGui::Spacing();

            // Frame count info
            ImGui::Text("📊 Available Frames: %zu", available_frames_.size());
            ImGui::Spacing();

            // Custom styled frame list
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.05f, 0.05f, 0.05f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.25f, 0.25f, 0.25f, 0.6f));
            ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 12.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);

            ImGui::BeginChild("FramesList", ImVec2(0, 140), true, ImGuiWindowFlags_AlwaysVerticalScrollbar);

            // Custom selectable styling
            ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, ImVec2(0.0f, 0.5f));

            for (const auto &frame : available_frames_)
            {
                bool is_current = (fixed_frame_ == frame);

                // Custom colors for current vs other frames
                if (is_current)
                {
                    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.2f, 0.6f, 0.8f, 0.7f));
                    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.3f, 0.7f, 0.9f, 0.8f));
                    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.4f, 0.8f, 1.0f, 0.9f));
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
                }
                else
                {
                    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.15f, 0.15f, 0.15f, 0.4f));
                    ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.25f, 0.25f, 0.25f, 0.6f));
                    ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(0.35f, 0.35f, 0.35f, 0.8f));
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.85f, 0.85f, 0.85f, 1.0f));
                }

                // Create selectable item with icon
                std::string display_text = (is_current ? "🎯 " : "📍 ") + frame;

                if (ImGui::Selectable(display_text.c_str(), is_current, 0, ImVec2(0, 28)))
                {
                    if (!is_current)
                    {
                        fixed_frame_ = frame;
                        std::cout << "🔗 Selected TF frame: " << frame << std::endl;
                    }
                }

                // Tooltip for long frame names
                if (ImGui::IsItemHovered() && frame.length() > 25)
                {
                    ImGui::SetTooltip("%s", frame.c_str());
                }

                ImGui::PopStyleColor(4);
            }

            // Show message if no frames available
            if (available_frames_.empty())
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 40.0f);
                ImGui::SetCursorPosX(ImGui::GetContentRegionAvail().x * 0.5f - 100.0f);
                ImGui::Text("🔍 No frames detected");
                ImGui::SetCursorPosX(ImGui::GetContentRegionAvail().x * 0.5f - 80.0f);
                ImGui::Text("Check your TF tree");
                ImGui::PopStyleColor();
            }

            ImGui::PopStyleVar(); // SelectableTextAlign
            ImGui::EndChild();
            ImGui::PopStyleVar(2);   // Child rounding and border
            ImGui::PopStyleColor(2); // Child colors

            ImGui::Spacing();

            // Action buttons at bottom
            ImGui::Separator();
            ImGui::Spacing();

            // Reset TF button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.4f, 0.2f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.7f, 0.5f, 0.3f, 0.9f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.8f, 0.6f, 0.4f, 1.0f));
            if (ImGui::Button("🔄 Reset TF Buffer", ImVec2(150, 28)))
            {
                resetTFBuffer();
                std::cout << "🔄 TF Buffer reset" << std::endl;
            }
            ImGui::PopStyleColor(3);

            ImGui::SameLine();

            // Close button
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.5f, 0.5f, 0.6f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.6f, 0.6f, 0.6f, 0.8f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
            if (ImGui::Button("✕ Close", ImVec2(80, 28)))
            {
                show_frames_popup_ = false;
            }
            ImGui::PopStyleColor(3);

            // Frame count info at bottom right
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetContentRegionAvail().x - 120.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f));
            ImGui::Text("TF Frames: %zu", frame_transforms_.size());
            ImGui::PopStyleColor();
        }
        ImGui::End();

        // Pop all style modifications
        ImGui::PopStyleColor(4);
        ImGui::PopStyleVar(5);
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

    // Camara control
    guik::ArcCameraControl camera_control_;

    // control variables for memory, GPU utilization and tf buffer
    size_t last_memory_check_ = 0;
    bool tf_initialized_ = false;
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
    std::vector<std::string> topic_names_; // Simplified to just store names
    std::chrono::time_point<std::chrono::steady_clock> last_topic_refresh_;
    const std::chrono::milliseconds topic_refresh_interval_{1000};

    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_msg_time_;
    std::chrono::milliseconds alive_threshold_{1000}; // e.g. 1.0s

    // topic hz
    std::unordered_map<std::string, std::deque<std::chrono::steady_clock::time_point> > topic_message_times_;
    std::unordered_map<std::string, float> topic_hz_;
    static constexpr size_t ROS2_LIKE_WINDOW_SIZE = 100;

    bool show_topics_window = false;

    // TF visualization
    bool show_tf_frames_ = true;
    float tf_frame_size_ = 0.4f; // Size of the coordinate axes for each frame

    // Store transforms from fixed frame to all other frames
    std::mutex tf_mutex_;
    std::unordered_map<std::string, Eigen::Isometry3f> frame_transforms_;

    // Store point cloud topics
    std::vector<std::shared_ptr<CloudTopic> > cloud_topics_;
    std::mutex cloud_topics_mutex_;
    int selected_topic_idx_{-1};

    // point cloud VAO/VBO
    GLuint dbgVao = 0;
    GLuint dbgVbo = 0;

    // lights
    std::chrono::steady_clock::time_point blink_start_ = std::chrono::steady_clock::now();

    // FreeType and text rendering
    TextRenderer text_renderer_;

    // PCD loader
    PclLoader pcd_loader_;
    std::string pcd_frame = "map";

    // lanelet2 loader
    LaneletLoader lanelet_loader_;

    // for ply and glb model upload
    modelUpload model_upload_;
    GLuint glb_shader_program_ = 0;

    // mesh map variables
    PlyMesh loaded_mesh_map;
    bool mesh_map_loaded = false;

    // mesh glb
    PlyMesh loaded_mesh_glb;
    bool mesh_glb_loaded = false;

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
        {"📡", "ROS2 Topics", DockItem::TOPICS, 0, "4.png"},
        {"🔗", "TF Frames", DockItem::FRAMES, 0, "7.png"},
        {"📁", "Reset tf", DockItem::FILES, 0, "8.png"},
        {"🤖", "3D Models", DockItem::MODELS, 0, "5.png"},
        {"⚙️", "Lanelet", DockItem::SETTINGS, 0, "6.png"},
        {"ℹ️", "Info", DockItem::INFO, 0, ""}};

    bool show_topics_popup_ = false;
    bool show_frames_popup_ = false;
    bool show_files_popup_ = false;
    bool show_models_popup_ = false;
    bool show_settings_popup_ = false;
    bool show_info_popup_ = false;
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

    // Run application
    app->run();

    return 0;
}