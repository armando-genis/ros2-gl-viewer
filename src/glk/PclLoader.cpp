#include <glk/PclLoader.hpp>
#include <iostream>

PclLoader::PclLoader(/* args */)
{
}

PclLoader::~PclLoader()
{
    // Clean up OpenGL resources
    if (_vao != 0)
    {
        glDeleteVertexArrays(1, &_vao);
        _vao = 0;
    }
    if (_vbo != 0)
    {
        glDeleteBuffers(1, &_vbo);
        _vbo = 0;
    }
}

void PclLoader::renderpcl(glk::GLSLShader &shader, std::mutex &tf_mutex, const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms)
{
    shader.set_uniform("color_mode", 0);

    Eigen::Isometry3f model = Eigen::Isometry3f::Identity();
    {
        std::lock_guard<std::mutex> lk(tf_mutex);
        auto it = frame_transforms.find(pcd_frame_id_);
        if (it != frame_transforms.end())
            model = it->second;
    }

    shader.set_uniform("model_matrix", model.matrix());
    glBindVertexArray(_vao);
    glDrawArrays(GL_POINTS, 0, GLsizei(pcd_num_points));
    glBindVertexArray(0);
}

void PclLoader::PointCloudBuffer(const std::string &cloud_filename, std::string pcd_frame_id, bool enable_downsampling)
{
    pcd_frame_id_ = pcd_frame_id;

    // Clear any existing OpenGL errors first
    while (glGetError() != GL_NO_ERROR)
    {
        // Clear error queue
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();

    if (pcl::io::loadPCDFile(cloud_filename, *cloud) < 0)
    {
        std::cerr << "error: failed to load " << cloud_filename << "\n";
        return;
    }

    if (cloud->empty())
    {
        std::cerr << "error: point cloud is empty\n";
        return;
    }

    std::cout << blue << "Loading PCD file: " << cloud_filename << " with frame: " << pcd_frame_id_ << reset << std::endl;
    std::cout << "loaded " << cloud_filename << " with " << cloud->size() << " points" << std::endl;

    const size_t original_size = cloud->size();

    // Downsampling logic (only if enabled)
    if (enable_downsampling)
    {
        const size_t MAX_POINTS = 2000000; // 2 million points limit

        if (original_size > MAX_POINTS)
        {
            std::cout << "Warning: Point cloud has " << original_size
                      << " points, downsampling to " << MAX_POINTS << " points" << std::endl;

            // Simple downsampling - take every nth point
            size_t step = (original_size + MAX_POINTS - 1) / MAX_POINTS; // Ceiling division
            pcl::PointCloud<pcl::PointXYZI> downsampled;
            downsampled.points.reserve(MAX_POINTS); // Reserve memory

            for (size_t i = 0; i < original_size; i += step)
            {
                downsampled.points.push_back(cloud->points[i]);
                if (downsampled.points.size() >= MAX_POINTS)
                    break; // Safety check
            }
            downsampled.width = downsampled.points.size();
            downsampled.height = 1;
            downsampled.is_dense = false;

            *cloud = std::move(downsampled); // Use move instead of copy
            std::cout << "Downsampled to " << cloud->size() << " points" << std::endl;
        }
        else
        {
            std::cout << "Point cloud size (" << original_size << ") is within limits, no downsampling needed" << std::endl;
        }
    }
    else
    {
        std::cout << "Downsampling disabled, using all " << original_size << " points" << std::endl;
    }

    pcd_num_points = cloud->size();

    // Calculate buffer size
    size_t buffer_size = cloud->size() * sizeof(pcl::PointXYZI);
    std::cout << "Buffer size: " << buffer_size / (1024 * 1024) << " MB" << std::endl;

    // Clean up existing resources if they exist
    if (_vao != 0)
    {
        glDeleteVertexArrays(1, &_vao);
        _vao = 0;
    }
    if (_vbo != 0)
    {
        glDeleteBuffers(1, &_vbo);
        _vbo = 0;
    }

    // Clear any deletion errors
    glGetError();

    // Create new VAO/VBO with better error checking
    glGenVertexArrays(1, &_vao);
    GLenum error = glGetError();
    if (error != GL_NO_ERROR || _vao == 0)
    {
        std::cerr << "Failed to generate VAO, OpenGL Error: " << error << std::endl;
        return;
    }

    glGenBuffers(1, &_vbo);
    error = glGetError();
    if (error != GL_NO_ERROR || _vbo == 0)
    {
        std::cerr << "Failed to generate VBO, OpenGL Error: " << error << std::endl;
        glDeleteVertexArrays(1, &_vao);
        _vao = 0;
        return;
    }

    std::cout << "Generated VAO: " << _vao << ", VBO: " << _vbo << std::endl;

    // Bind VAO first
    glBindVertexArray(_vao);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error binding VAO: " << error << std::endl;
        glDeleteVertexArrays(1, &_vao);
        glDeleteBuffers(1, &_vbo);
        _vao = 0;
        _vbo = 0;
        return;
    }

    // Then bind VBO
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error binding VBO: " << error << std::endl;
        glBindVertexArray(0);
        glDeleteVertexArrays(1, &_vao);
        glDeleteBuffers(1, &_vbo);
        _vao = 0;
        _vbo = 0;
        return;
    }

    // Allocate buffer data
    std::cout << "Allocating " << buffer_size << " bytes for " << pcd_num_points << " points..." << std::endl;
    glBufferData(GL_ARRAY_BUFFER, buffer_size, cloud->points.data(), GL_STATIC_DRAW);

    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error during buffer allocation: " << error << std::endl;
        std::cerr << "Buffer size was: " << buffer_size << " bytes ("
                  << buffer_size / (1024 * 1024) << " MB)" << std::endl;

        // Get more GPU info for debugging
        GLint max_vertex_attribs, max_elements_vertices, max_elements_indices;
        glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &max_vertex_attribs);
        glGetIntegerv(GL_MAX_ELEMENTS_VERTICES, &max_elements_vertices);
        glGetIntegerv(GL_MAX_ELEMENTS_INDICES, &max_elements_indices);

        std::cerr << "GPU Limits - Max vertex attributes: " << max_vertex_attribs
                  << ", Max elements vertices: " << max_elements_vertices
                  << ", Max elements indices: " << max_elements_indices << std::endl;

        // Clean up and return
        glBindVertexArray(0);
        glDeleteVertexArrays(1, &_vao);
        glDeleteBuffers(1, &_vbo);
        _vao = 0;
        _vbo = 0;
        return;
    }

    // Setup vertex attributes
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZI), (void *)0);

    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error setting vertex attributes: " << error << std::endl;
        glBindVertexArray(0);
        glDeleteVertexArrays(1, &_vao);
        glDeleteBuffers(1, &_vbo);
        _vao = 0;
        _vbo = 0;
        return;
    }

    // Unbind
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Final error check
    error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error after cleanup: " << error << std::endl;
        return;
    }

    std::cout << green << "Successfully created VAO/VBO for PCD PointXYZI with "
              << pcd_num_points << " points" << reset << std::endl;

    has_pcd_setted = true;
}

bool PclLoader::isLoaded() const
{
    return has_pcd_setted;
}

size_t PclLoader::getPointCount() const
{
    return pcd_num_points;
}