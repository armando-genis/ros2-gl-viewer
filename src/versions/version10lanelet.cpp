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

#include <guik/gl_canvas.hpp>
#include <guik/camera_control.hpp>
#include <guik/imgui_application.hpp>

#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <GL/glut.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <ft2build.h>
#include FT_FREETYPE_H
#include <map>

// lanelet 2
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/geometry/Point.h>
using namespace lanelet;

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

struct PlyMesh
{
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0; // Element buffer object
    size_t vertex_count = 0;
    size_t index_count = 0; // Total number of indices
    std::string frame_id;
};

struct VertexColor
{
    float x, y, z;
    uint8_t r, g, b;
};

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

        return true;
    }

    void draw_ui() override
    {
        // get the current time
        auto now = std::chrono::steady_clock::now();
        // get the tf based on the frame
        update_tf_transforms();

        // Show main canvas settings
        main_canvas_->draw_ui();

        // Main options window
        ImGui::Begin("ros2 Viewer", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        // FPS counter
        ImGui::Separator();
        ImGui::Text("Swipe/drag pad: orbit");
        ImGui::Text("Scroll pad: zoom");
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

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
                for (int i = 0; i < (int)topic_names_.size(); ++i)
                {
                    const auto &name = topic_names_[i];
                    // circle-alive indicator (unchanged)
                    bool alive = false;
                    auto it = last_msg_time_.find(name);
                    if (it != last_msg_time_.end())
                        alive = (now - it->second) < alive_threshold_;
                    const float r = 5.0f;
                    ImU32 col = alive ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255);
                    ImGui::Dummy(ImVec2(r * 2, r * 2));
                    ImVec2 p = ImGui::GetItemRectMin();
                    ImGui::GetWindowDrawList()->AddCircleFilled({p.x + r, p.y + r}, r, col);

                    ImGui::SameLine();
                    // make it selectable
                    bool is_selected = (i == selected_topic_idx_);
                    if (ImGui::Selectable(name.c_str(), is_selected))
                        selected_topic_idx_ = i;
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

        static char pcd_path[256] = "";
        if (ImGui::Button("Load PCD"))
        {
            ImGui::OpenPopup("Load PCD File");
        }

        if (ImGui::BeginPopup("Load PCD File"))
        {
            ImGui::InputText("PCD File", pcd_path, IM_ARRAYSIZE(pcd_path));
            ImGui::SameLine();
            if (ImGui::Button("OK"))
            {
                // construct the buffer from the file
                PointCloudBuffer(pcd_path);
                has_pcd_ = true;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        static char laneletmap_path[256] = "";
        if (ImGui::Button("Load Lanelet Map"))
        {
            ImGui::OpenPopup("Load Map File");
        }

        if (ImGui::BeginPopup("Load Map File"))
        {
            ImGui::InputText("Map File", laneletmap_path, IM_ARRAYSIZE(laneletmap_path));
            ImGui::SameLine();
            if (ImGui::Button("OK"))
            {
                // construct the buffer from the file
                loadLanelet2Map(laneletmap_path);
                has_lanelet2_map_ = true;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        static bool show_load_input = false;
        static char filepath[256] = "";
        static int selected_ply_frame = 0;

        if (ImGui::Button("Load PLY"))
        {
            show_load_input = !show_load_input;
        }

        // /workspace/models/Buggy.ply
        // /workspace/models/miniBuggy.ply
        // /workspace/models/miniBuggy_2.ply
        // /workspace/models/sdv.ply

        if (show_load_input)
        {
            ImGui::InputText("PLY File", filepath, sizeof(filepath));
            ImGui::Separator();
            ImGui::Text("Assign to TF frame:");
            ImGui::SameLine();
            if (!available_frames_.empty())
            {
                const char *preview = available_frames_[selected_ply_frame].c_str();
                if (ImGui::BeginCombo("##ply_frames", preview))
                {
                    for (int n = 0; n < (int)available_frames_.size(); ++n)
                    {
                        bool is_selected = (selected_ply_frame == n);
                        if (ImGui::Selectable(available_frames_[n].c_str(), is_selected))
                        {
                            selected_ply_frame = n;
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
            ImGui::SameLine();
            if (ImGui::Button("Open"))
            {
                try
                {
                    loaded_mesh = loadPlyBinaryLE(filepath);
                    mesh_loaded = true;
                    const std::string &frame_id = available_frames_[selected_ply_frame];

                    std::cout << green
                              << "Loaded PLY mesh from: "
                              << filepath
                              << " into frame: "
                              << frame_id
                              << reset
                              << std::endl;

                    loaded_mesh.frame_id = frame_id;
                }
                catch (const std::exception &e)
                {
                    std::cerr << "PLY load error: " << e.what() << std::endl;
                }
                show_load_input = false; // hide input after loading
            }
        }

        ImGui::SliderFloat("ROS Topic Point Size", &topic_point_size_, 1.0f, 20.0f);
        ImGui::SliderFloat("PCD Point Size", &pcd_point_size_, 1.0f, 20.0f);

        ImGui::End();
    }

    void draw_gl() override
    {
        // 1) bind offscreen framebuffer
        main_canvas_->bind();

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE);

        // 2) clear & GL state
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

        // 5) draw axes
        {
            Eigen::Matrix4f m = Eigen::Matrix4f::Identity() * 2.0f;
            shader.set_uniform("color_mode", 2);
            shader.set_uniform("model_matrix", m);
            const auto &coord = glk::Primitives::instance()
                                    ->primitive(glk::Primitives::COORDINATE_SYSTEM);
            coord.draw(shader);
        }

        // 6) draw grid
        {
            Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
            T.pretranslate(Eigen::Vector3f(0, 0, -0.02f));
            shader.set_uniform("color_mode", 1);
            shader.set_uniform("model_matrix", T.matrix());
            shader.set_uniform("material_color", Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f));
            grid_->draw(shader);
        }

        // 7) draw TF frames if desired
        if (show_tf_frames_)
        {
            std::lock_guard lk(tf_mutex_);
            shader.set_uniform("color_mode", 2);
            const auto &coord = glk::Primitives::instance()
                                    ->primitive(glk::Primitives::COORDINATE_SYSTEM);

            for (auto &p : frame_transforms_)
            {
                shader.set_uniform("model_matrix",
                                   (p.second * Eigen::Scaling(tf_frame_size_)).matrix());
                coord.draw(shader);
            }
        }

        // 8) now always draw your point clouds in white
        {
            std::lock_guard lk(cloud_topics_mutex_);
            // set uniform color-mode to “flat color”
            shader.set_uniform("color_mode", 0);

            shader.set_uniform("point_scale", 0.0f); // NO fall-off
            shader.set_uniform("point_size", topic_point_size_);
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

        if (has_pcd_setted)
        {
            // draw the PCD points
            shader.set_uniform("point_scale", 0.0f);
            shader.set_uniform("point_size", pcd_point_size_);
            glBindVertexArray(_vao);
            glDrawArrays(GL_POINTS, 0, GLsizei(pcd_num_points));
            glBindVertexArray(0);
        }

        if (mesh_loaded)
        {
            Eigen::Isometry3f model = Eigen::Isometry3f::Identity();
            {
                std::lock_guard<std::mutex> lk(tf_mutex_);
                auto it = frame_transforms_.find(loaded_mesh.frame_id);
                if (it != frame_transforms_.end())
                    model = it->second;
            }

            shader.set_uniform("color_mode", 4);
            shader.set_uniform("model_matrix", model.matrix());
            // shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f)); // White

            glBindVertexArray(loaded_mesh.vao);
            glDrawElements(
                GL_TRIANGLES,
                GLsizei(loaded_mesh.index_count),
                GL_UNSIGNED_INT,
                nullptr);
            glBindVertexArray(0);
        }

        if (map_lines_count_ > 0 && map_lines_vao_ != 0)
        {
            shader.set_uniform("color_mode", 1);                                           // flat color
            shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 0.0f, 1.0f)); // yellow
            Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
            shader.set_uniform("model_matrix", T.matrix());
            glBindVertexArray(map_lines_vao_);
            glDrawArrays(GL_LINES, 0, GLsizei(map_lines_count_));
            glBindVertexArray(0);
        }

        if (crosswalk_lines_count_ > 0 && crosswalk_vao_ != 0)
        {
            // flat color mode
            shader.set_uniform("color_mode", 1);
            // RGBA: here white, but you could pick e.g. (1,1,0,1) for yellow zebra stripes
            shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
            Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
            shader.set_uniform("model_matrix", T.matrix());

            glBindVertexArray(crosswalk_vao_);
            // draw as lines
            glDrawArrays(GL_LINES, 0, GLsizei(crosswalk_lines_count_));
            glBindVertexArray(0);
        }

        if (stripe_lines_count_ > 0 && stripe_vao_ != 0)
        {
            shader.set_uniform("color_mode", 1);
            shader.set_uniform("material_color", Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f)); // white
            Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
            shader.set_uniform("model_matrix", T.matrix());
            glBindVertexArray(stripe_vao_);
            glDrawArrays(GL_LINES, 0, GLsizei(stripe_lines_count_));
            glBindVertexArray(0);
        }

        // 9) unbind & blit
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
                try
                {
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
                catch (const std::exception &e)
                {
                    // std::cerr << "Failed to create subscription for topic " << name << ": " << e.what() << std::endl;
                }
            }
        }
    }

    // reference:
    // - https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
    // - https://robotics.stackexchange.com/questions/95228/how-to-get-list-of-all-tf-frames-programatically
    // - https://wiki.ros.org/tf/TfUsingPython
    void refresh_frame_list()
    {
        std::lock_guard<std::mutex> lock(frames_mutex_);

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

    void PointCloudBuffer(const std::string &cloud_filename)
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        if (pcl::io::loadPCDFile(cloud_filename, *cloud) < 0)
        {
            std::cerr << "error: failed to load " << cloud_filename << "\n";
            return;
        }

        std::cout << "loaded " << cloud_filename << " with " << cloud->size() << " points\n"
                  << std::endl;

        pcd_num_points = cloud->size();

        if (_vao == 0)
        {
            glGenVertexArrays(1, &_vao);
            glGenBuffers(1, &_vbo);
            glBindVertexArray(_vao);
            glBindBuffer(GL_ARRAY_BUFFER, _vbo);
            glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->points.data(), GL_STATIC_DRAW);

            glEnableVertexAttribArray(0); // Position
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZI), (void *)0);
            glBindVertexArray(0);
            std::cout << green
                      << "Created debug VAO/VBO for PCD PointXYZI"
                      << reset
                      << std::endl;

            has_pcd_setted = true;
        }
    }

    PlyMesh loadPlyBinaryLE(const std::string &path)
    {
        std::ifstream in{path, std::ios::binary};
        if (!in)
            throw std::runtime_error("Failed to open PLY: " + path);

        bool binary_le = false;
        size_t vertex_count = 0, face_count = 0;
        struct Property
        {
            std::string name, type;
        };
        std::vector<Property> vertexProps;
        bool inVertexElement = false;
        std::string line;

        size_t idxX = -1, idxY = -1, idxZ = -1, idxR = -1, idxG = -1, idxB = -1;

        while (std::getline(in, line))
        {
            if (line.rfind("format", 0) == 0 && line.find("binary_little_endian") != std::string::npos)
                binary_le = true;
            if (line.rfind("element vertex", 0) == 0)
            {
                std::istringstream ss(line);
                std::string tmp;
                ss >> tmp >> tmp >> vertex_count;
                inVertexElement = true;
            }
            else if (line.rfind("element face", 0) == 0)
            {
                std::istringstream ss(line);
                std::string tmp;
                ss >> tmp >> tmp >> face_count;
                inVertexElement = false;
            }
            else if (inVertexElement && line.rfind("property", 0) == 0)
            {
                std::istringstream ss(line);
                std::string prop, type, name;
                ss >> prop >> type >> name;
                vertexProps.push_back({name, type});
                if (name == "x")
                    idxX = vertexProps.size() - 1;
                if (name == "y")
                    idxY = vertexProps.size() - 1;
                if (name == "z")
                    idxZ = vertexProps.size() - 1;
                if (name == "red")
                    idxR = vertexProps.size() - 1;
                if (name == "green")
                    idxG = vertexProps.size() - 1;
                if (name == "blue")
                    idxB = vertexProps.size() - 1;
            }
            if (line == "end_header")
                break;
        }

        if (!binary_le)
            throw std::runtime_error("Only binary_little_endian PLY supported");
        if (vertexProps.size() < 3)
            throw std::runtime_error("PLY header: need at least x,y,z properties");

        struct Vertex
        {
            float x, y, z;
            uint8_t r, g, b;
        };
        std::vector<Vertex> verts(vertex_count);

        // --- Read vertex data with correct types ---
        for (size_t i = 0; i < vertex_count; ++i)
        {
            float x = 0, y = 0, z = 0;
            uint8_t r = 255, g = 255, b = 255;
            for (size_t j = 0; j < vertexProps.size(); ++j)
            {
                if (vertexProps[j].type == "float")
                {
                    float v;
                    in.read(reinterpret_cast<char *>(&v), sizeof(float));
                    if (j == idxX)
                        x = v;
                    if (j == idxY)
                        y = v;
                    if (j == idxZ)
                        z = v;
                }
                else if (vertexProps[j].type == "uchar")
                {
                    uint8_t v;
                    in.read(reinterpret_cast<char *>(&v), sizeof(uint8_t));
                    if (j == idxR)
                        r = v;
                    if (j == idxG)
                        g = v;
                    if (j == idxB)
                        b = v;
                }
                else
                {
                    // skip other types (e.g. alpha, s, t)
                    if (vertexProps[j].type == "uint")
                    {
                        uint32_t dummy;
                        in.read(reinterpret_cast<char *>(&dummy), sizeof(uint32_t));
                    }
                    else
                    {
                        // Add more types if needed
                        throw std::runtime_error("Unsupported property type: " + vertexProps[j].type);
                    }
                }
            }
            verts[i] = {x, y, z, r, g, b};
        }

        // --- Read faces (same as before) ---
        std::vector<uint32_t> indices;
        indices.reserve(face_count * 3);
        for (size_t f = 0; f < face_count; ++f)
        {
            uint8_t nVerts = 0;
            in.read(reinterpret_cast<char *>(&nVerts), sizeof(nVerts));
            if (!in)
                throw std::runtime_error("Unexpected EOF in face data");

            std::vector<uint32_t> faceIdx(nVerts);
            in.read(reinterpret_cast<char *>(faceIdx.data()), nVerts * sizeof(uint32_t));
            if (!in)
                throw std::runtime_error("Unexpected EOF in face data");

            if (nVerts == 3)
            {
                indices.insert(indices.end(), faceIdx.begin(), faceIdx.end());
            }
            else
            {
                for (uint8_t k = 1; k + 1 < nVerts; ++k)
                {
                    indices.push_back(faceIdx[0]);
                    indices.push_back(faceIdx[k]);
                    indices.push_back(faceIdx[k + 1]);
                }
            }
        }

        // --- Upload to OpenGL (same as before) ---
        PlyMesh mesh;
        mesh.vertex_count = vertex_count;
        mesh.index_count = indices.size();

        glGenVertexArrays(1, &mesh.vao);
        glGenBuffers(1, &mesh.vbo);
        glGenBuffers(1, &mesh.ebo);

        glBindVertexArray(mesh.vao);

        glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vertex), (void *)(3 * sizeof(float)));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

        glBindVertexArray(0);
        return mesh;
    }

    // lanelet2 map loading
    void loadLanelet2Map(const std::string &path)
    {
        std::cout << "Loading Lanelet2 map from: " << path << std::endl;
        lanelet::Origin origin({49, 8.4});
        lanelet::projection::LocalCartesianProjector projector(origin);
        try
        {
            lanelet::LaneletMapPtr map = lanelet::load(path, projector);
            std::cout << green << "Loaded Lanelet2 map with " << map->laneletLayer.size() << " lanelets." << reset << std::endl;
            has_lanelet2_map_setted_ = true;
            map_path_ = path;

            for (auto &point : map->pointLayer)
            {
                point.x() = point.attribute("local_x").asDouble().value();
                point.y() = point.attribute("local_y").asDouble().value();
            }
            mapProcessing(map);
        }
        catch (const std::exception &e)
        {
            std::cerr << red << "Failed to load Lanelet2 map: " << e.what() << reset << std::endl;
            has_lanelet2_map_setted_ = false;
        }
    }

    void mapProcessing(lanelet::LaneletMapPtr &t_map)
    {

        int crosswalk_count = 0;
        int road_element_count = 0;
        map_lines_.clear();
        // Iterate over the lanelets in the map
        for (const auto &ll : t_map->laneletLayer)
        {
            std::vector<lanelet::ConstLineString3d> bounds;
            bounds.push_back(ll.leftBound());
            bounds.push_back(ll.rightBound());
            if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
                ll.attribute(lanelet::AttributeName::Subtype).value() == lanelet::AttributeValueString::Crosswalk)
            {
                std::cout << "Crosswalk id: " << ll.id() << std::endl;
                crosswalk_count++;

                double max_z = std::numeric_limits<double>::lowest();
                for (const auto &point : ll.leftBound())
                {
                    if (point.z() > max_z)
                    {
                        max_z = point.z();
                    }
                }

                // --- Store crosswalk polygon as a closed loop --
                std::vector<Eigen::Vector3f> polygon;
                // Left bound (forward)
                for (const auto &point : ll.leftBound())
                    polygon.emplace_back(point.x(), point.y(), point.z());
                // Right bound (reverse)
                for (int i = ll.rightBound().size() - 1; i >= 0; --i)
                    polygon.emplace_back(ll.rightBound()[i].x(), ll.rightBound()[i].y(), ll.rightBound()[i].z());
                // Close the loop
                polygon.push_back(polygon.front());
                crosswalk_polygons_.push_back(polygon);

                crosswalk_lines_.clear();
                for (auto &poly : crosswalk_polygons_)
                {
                    // each polygon is already a closed loop (first==last)
                    for (size_t i = 1; i < poly.size(); ++i)
                    {
                        crosswalk_lines_.emplace_back(poly[i - 1]);
                        crosswalk_lines_.emplace_back(poly[i]);
                    }
                }
                crosswalk_lines_count_ = crosswalk_lines_.size();

                // zebra stripes
                stripe_lines_.clear();
                int num_stripes = 5;
                double left_bound_length = 0.0;
                for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
                {
                    left_bound_length += std::sqrt(
                        std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
                        std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2) +
                        std::pow(ll.leftBound()[i + 1].z() - ll.leftBound()[i].z(), 2));
                }
                std::cout << "----->left_bound_length: " << left_bound_length << std::endl;

                if (left_bound_length > 5.0)
                {
                    num_stripes += static_cast<int>((left_bound_length - 5.0) / 1.0) * 1;
                }

                std::cout << "----->num_stripes: " << num_stripes << std::endl;

                double stripe_length = left_bound_length / (2 * num_stripes);
                for (int stripe_idx = 0; stripe_idx < num_stripes; ++stripe_idx)
                {
                    std::vector<Eigen::Vector3f> stripe_polygon;
                    double start_dist = stripe_idx * 2 * stripe_length;
                    double end_dist = start_dist + stripe_length;

                    // Add points for the stripe from the left bound
                    double accumulated_length = 0.0;
                    // Eigen::Vector2d start_left, end_left;
                    Eigen::Vector3f start_left, end_left;
                    bool start_left_set = false, end_left_set = false;
                    for (size_t i = 0; i < ll.leftBound().size() - 1; ++i)
                    {
                        double segment_length = std::sqrt(
                            std::pow(ll.leftBound()[i + 1].x() - ll.leftBound()[i].x(), 2) +
                            std::pow(ll.leftBound()[i + 1].y() - ll.leftBound()[i].y(), 2));

                        if (accumulated_length + segment_length > start_dist && !start_left_set)
                        {
                            double ratio = (start_dist - accumulated_length) / segment_length;
                            start_left.x() = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
                            start_left.y() = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
                            start_left.z() = max_z; // Use the maximum z from the left bound
                            start_left_set = true;
                        }
                        if (accumulated_length + segment_length > end_dist && !end_left_set)
                        {
                            double ratio = (end_dist - accumulated_length) / segment_length;
                            end_left.x() = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
                            end_left.y() = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
                            end_left.z() = max_z; // Use the maximum z from the left bound
                            end_left_set = true;
                            break;
                        }
                        accumulated_length += segment_length;
                    }
                    if (start_left_set && end_left_set)
                    {
                        stripe_polygon.push_back(start_left);
                        stripe_polygon.push_back(end_left);
                    }
                    // Add points for the stripe from the right bound
                    accumulated_length = 0.0;
                    Eigen::Vector3f start_right, end_right;
                    bool start_right_set = false, end_right_set = false;
                    for (size_t i = 0; i < ll.rightBound().size() - 1; ++i)
                    {
                        double segment_length = std::sqrt(
                            std::pow(ll.rightBound()[i + 1].x() - ll.rightBound()[i].x(), 2) +
                            std::pow(ll.rightBound()[i + 1].y() - ll.rightBound()[i].y(), 2));

                        if (accumulated_length + segment_length > start_dist && !start_right_set)
                        {
                            double ratio = (start_dist - accumulated_length) / segment_length;
                            start_right.x() = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i + 1].x() - ll.rightBound()[i].x());
                            start_right.y() = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i + 1].y() - ll.rightBound()[i].y());
                            start_right.z() = max_z; // Use the maximum z from the right bound
                            start_right_set = true;
                        }
                        if (accumulated_length + segment_length > end_dist && !end_right_set)
                        {
                            double ratio = (end_dist - accumulated_length) / segment_length;
                            end_right.x() = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i + 1].x() - ll.rightBound()[i].x());
                            end_right.y() = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i + 1].y() - ll.rightBound()[i].y());
                            end_right.z() = max_z; // Use the maximum z from the right bound
                            end_right_set = true;
                            break;
                        }
                        accumulated_length += segment_length;
                    }
                    if (start_right_set && end_right_set)
                    {
                        stripe_polygon.push_back(end_right);
                        stripe_polygon.push_back(start_right);
                    }

                    // Close the polygon by adding the first point again
                    if (stripe_polygon.size() >= 4)
                    {
                        stripe_polygon.push_back(stripe_polygon.front());
                        for (size_t i = 1; i < stripe_polygon.size(); ++i)
                        {
                            stripe_lines_.emplace_back(stripe_polygon[i - 1]);
                            stripe_lines_.emplace_back(stripe_polygon[i]);
                        }
                    }
                }
            }
            else
            {
                road_element_count++;
                for (const auto &bound : bounds)
                {
                    // std::cout << "Road element id: " << ll.id() << " bound: " << bound.id() << std::endl;
                    // Add the points of the bounds
                    for (size_t i = 1; i < bound.size(); ++i)
                    {
                        // std::cout << "Road element id: " << ll.id() << " bound: " << bound.id() << " point: " << point.id() << std::endl;
                        auto p0 = bound[i - 1];
                        auto p1 = bound[i];
                        map_lines_.emplace_back(p0.x(), p0.y(), p0.z());
                        map_lines_.emplace_back(p1.x(), p1.y(), p1.z());
                    }
                }
            }
        }
        map_lines_count_ = map_lines_.size();
        // Upload to OpenGL
        if (map_lines_vao_ == 0)
        {
            glGenVertexArrays(1, &map_lines_vao_);
            glGenBuffers(1, &map_lines_vbo_);
        }
        glBindVertexArray(map_lines_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, map_lines_vbo_);
        glBufferData(GL_ARRAY_BUFFER, map_lines_.size() * sizeof(Eigen::Vector3f), map_lines_.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void *)0);
        glBindVertexArray(0);

        std::cout << blue << "----> Number of crosswalk lanelets: " << crosswalk_count << reset << std::endl;
        std::cout << blue << "----> Number of road elements: " << road_element_count << reset << std::endl;

        if (crosswalk_vao_ == 0)
        {
            glGenVertexArrays(1, &crosswalk_vao_);
            glGenBuffers(1, &crosswalk_vbo_);
        }
        glBindVertexArray(crosswalk_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, crosswalk_vbo_);
        glBufferData(GL_ARRAY_BUFFER, crosswalk_lines_count_ * sizeof(Eigen::Vector3f), crosswalk_lines_.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void *)0);
        glBindVertexArray(0);

        std::cout << blue << "----> Number of crosswalk lines: " << crosswalk_lines_count_ << reset << std::endl;

        stripe_lines_count_ = stripe_lines_.size();
        if (stripe_vao_ == 0)
        {
            glGenVertexArrays(1, &stripe_vao_);
            glGenBuffers(1, &stripe_vbo_);
        }
        glBindVertexArray(stripe_vao_);
        glBindBuffer(GL_ARRAY_BUFFER, stripe_vbo_);
        glBufferData(GL_ARRAY_BUFFER, stripe_lines_.size() * sizeof(Eigen::Vector3f), stripe_lines_.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void *)0);
        glBindVertexArray(0);

        std::cout << blue << "----> Number of stripe lines: " << stripe_lines_count_ << reset << std::endl;
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

    // TF visualization
    bool show_tf_frames_ = true;
    float tf_frame_size_ = 0.4f; // Size of the coordinate axes for each frame

    // Store transforms from fixed frame to all other frames
    std::mutex tf_mutex_;
    std::unordered_map<std::string, Eigen::Isometry3f> frame_transforms_;

    // Store point cloud topics
    std::vector<std::shared_ptr<CloudTopic>> cloud_topics_;
    std::mutex cloud_topics_mutex_;
    int selected_topic_idx_{-1};

    // point cloud VAO/VBO
    GLuint dbgVao = 0;
    GLuint dbgVbo = 0;

    // pcd map
    GLuint _vao = 0, _vbo = 0;
    bool has_pcd_ = false;
    bool has_pcd_setted = false;
    int pcd_num_points = 0;

    // PointCloudTopic and pcd size
    float topic_point_size_ = 4.0f;
    float pcd_point_size_ = 10.0f;

    // mesh ply variables
    PlyMesh loaded_mesh;
    bool mesh_loaded = false;

    // lanelet2 map varianles
    std::string map_path_;
    bool has_lanelet2_map_ = false;
    bool has_lanelet2_map_setted_ = false;
    GLuint map_lines_vao_ = 0, map_lines_vbo_ = 0;
    size_t map_lines_count_ = 0;
    std::vector<Eigen::Vector3f> map_lines_;
    std::vector<std::vector<Eigen::Vector3f>> crosswalk_polygons_;

    GLuint crosswalk_vao_ = 0;
    GLuint crosswalk_vbo_ = 0;
    size_t crosswalk_lines_count_ = 0;
    std::vector<Eigen::Vector3f> crosswalk_lines_;

    size_t stripe_lines_count_ = 0;
    std::vector<Eigen::Vector3f> stripe_lines_;
    GLuint stripe_vao_ = 0;
    GLuint stripe_vbo_ = 0;
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