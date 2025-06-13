
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

#include <ft2build.h>
#include FT_FREETYPE_H
#include <map>

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
                if (ImGui::Button("➕ Add"))
                {
                    addPointCloudTopic(topic_names_[selected_topic_idx_]);
                }
            }
        }

        ImGui::End();
    }

    void draw_gl() override
    {
        // 1) bind offscreen framebuffer
        main_canvas_->bind();

        // 2) clear & GL state
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPointSize(15.0f);

        // 1) Compute View matrix
        Eigen::Matrix4f view = camera_control_.view_matrix();

        // 2) Compute Projection matrix manually
        float w = float(framebuffer_size().x());
        float h = float(framebuffer_size().y());
        float aspect = w / h;
        float fovY = 45.f * M_PI / 180.f;
        float zNear = 0.01f, zFar = 100.f;
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

        // 8) now always draw your point clouds in white
        {
            std::lock_guard lk(cloud_topics_mutex_);

            for (auto &ct : cloud_topics_)
            {
                shader.set_uniform("model_matrix", ct->transform.matrix());
                glBindVertexArray(ct->vao);
                glDrawArrays(GL_POINTS, 0, GLsizei(ct->num_points));
                glBindVertexArray(0);

                // Debug: print the first 10 points
                std::cout << blue << "PointCloud '" << ct->topic_name << "' ("
                          << ct->num_points << " pts) in frame '" << ct->frame_id
                          << "' at transform: " << ct->transform.translation().transpose()
                          << reset << std::endl;

                for (size_t i = 0; i < 10 && i < ct->debug_pts.size(); ++i)
                {
                    const auto &p = ct->debug_pts[i];
                    std::cout << "  pt[" << i << "]: " << p.transpose() << std::endl;
                }
            }
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
        available_frames_.clear();
        rclcpp::Time now = node_->get_clock()->now();

        for (const auto &[frame, parent] : frame_to_parent)
        {

            geometry_msgs::msg::Transform pose_tf;
            try
            {

                // Try to get the transform between this frame and its parent
                pose_tf = tf_buffer_->lookupTransform(parent, frame, tf2::TimePointZero).transform;
                available_frames_.push_back(frame);

                if (std::find(available_frames_.begin(), available_frames_.end(), parent) == available_frames_.end())
                {
                    available_frames_.push_back(parent);
                }
            }
            catch (const tf2::TransformException &ex)
            {
                // If we can't get the transform, don't add to available frames (I mean this should be impossible)
                std::cout << red << "Transform error: " << ex.what() << reset << std::endl;
            }
        }

        last_frame_refresh_ = std::chrono::steady_clock::now();
    }

    void update_tf_transforms()
    {
        std::lock_guard<std::mutex> lock(tf_mutex_);

        // std::cout << green << "Updating TF transforms..." << reset << std::endl;

        // Skip if no fixed frame is selected
        if (fixed_frame_.empty())
            return;

        // Update transforms for each available frame
        for (const auto &frame : available_frames_)
        {
            // Skip the fixed frame itself
            if (frame == fixed_frame_)
                continue;

            try
            {
                // Get the transform from fixed frame to this frame
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform(fixed_frame_, frame, tf2::TimePointZero);

                // Convert to Eigen transform using the helper function
                frame_transforms_[frame] = toEigen(transform_stamped.transform);
            }
            catch (const tf2::TransformException &ex)
            {
                // If we can't get the transform, remove it from our map
                frame_transforms_.erase(frame);
            }
        }
    }

    void addPointCloudTopic(const std::string &topic)
    {
        // Avoid duplicates
        {
            std::lock_guard lk(cloud_topics_mutex_);
            for (auto &ct_ptr : cloud_topics_)
                if (ct_ptr->topic_name == topic)
                    return;
        }

        auto ct = std::make_shared<CloudTopic>();
        ct->topic_name = topic;

        // one-time: create VAO+VBO
        if (ct->vao == 0)
        {
            glGenVertexArrays(1, &ct->vao);
            glGenBuffers(1, &ct->vbo);
            glBindVertexArray(ct->vao);
            glBindBuffer(GL_ARRAY_BUFFER, ct->vbo);
            // reserve room for up to 100 debug points:
            glBufferData(GL_ARRAY_BUFFER, 100 * 3 * sizeof(float),
                         nullptr, GL_DYNAMIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void *)0);
            glBindVertexArray(0);
        }

        // 1) Start with an empty cloud
        auto empty = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // 2) Subscribe & update on message
        ct->sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS(),
            [this, ct, topic](sensor_msgs::msg::PointCloud2::UniquePtr msg)
            {
                ct->cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
                pcl::fromROSMsg(*msg, *ct->cloud);

                if (ct->cloud->empty())
                {
                    std::cout << "[PC_CB] " << topic << " → empty cloud\n";
                    return; // skip empty clouds
                }

                ct->frame_id = msg->header.frame_id;

                std::cout << "[PC_CB] " << topic << " → " << ct->cloud->size() << " pts\n";
                std::cout << "  Frame ID: " << ct->frame_id << std::endl;

                ct->debug_pts.clear();
                // for (size_t i = 0; i < 10 && i < ct->cloud->size(); ++i)
                // {
                //     auto &p = ct->cloud->points[i];
                //     ct->debug_pts.emplace_back(p.x, p.y, p.z);
                //     std::cout << "px" << p.x << " py" << p.y << " pz" << p.z << std::endl;
                // }

                for (int i = 0; i < 10; ++i)
                {
                    float x = (rand() / float(RAND_MAX)) * 2.0f - 1.0f;
                    float y = (rand() / float(RAND_MAX)) * 2.0f - 1.0f;
                    float z = (rand() / float(RAND_MAX)) * 2.0f - 1.0f;
                    ct->debug_pts.emplace_back(x, y, z);
                    std::cout << "px" << x << " py" << y << " pz" << z << std::endl;
                }

                std::cout << "size: " << ct->debug_pts.size() << std::endl;

                ct->num_points = ct->debug_pts.size();

                // 3) Upload to GPU
                glBindBuffer(GL_ARRAY_BUFFER, ct->vbo);
                glBufferSubData(
                    GL_ARRAY_BUFFER,
                    0,
                    ct->num_points * sizeof(Eigen::Vector3f),
                    ct->debug_pts.data());
                glBindBuffer(GL_ARRAY_BUFFER, 0);

                try
                {
                    auto tf_stamped = tf_buffer_->lookupTransform(fixed_frame_, ct->frame_id,
                                                                  tf2::TimePointZero);

                    ct->transform = toEigen(tf_stamped.transform);
                    std::cout << "[TF OK] " << topic
                              << " → t = "
                              << ct->transform.translation().transpose()
                              << std::endl;
                }
                catch (const tf2::TransformException &e)
                {
                    // leave it at Identity if missing
                }
            });

        // 3) Store
        {
            std::lock_guard lk(cloud_topics_mutex_);
            cloud_topics_.push_back(ct);
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

    // TF visualization
    bool show_tf_frames_ = true;
    float tf_frame_size_ = 0.5f; // Size of the coordinate axes for each frame

    // Store transforms from fixed frame to all other frames
    std::mutex tf_mutex_;
    std::unordered_map<std::string, Eigen::Isometry3f> frame_transforms_;

    // std::vector<CloudTopic> cloud_topics_;
    std::vector<std::shared_ptr<CloudTopic>> cloud_topics_;
    std::mutex cloud_topics_mutex_;
    int selected_topic_idx_{-1};
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