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

void PclLoader::PointCloudBuffer(const std::string &cloud_filename, std::string pcd_frame_id)
{
    pcd_frame_id_ = pcd_frame_id;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
    if (pcl::io::loadPCDFile(cloud_filename, *cloud) < 0)
    {
        std::cerr << "error: failed to load " << cloud_filename << "\n";
        return;
    }

    std::cout << blue << "Loading PCD file: " << cloud_filename << " with frame: " << pcd_frame_id_ << reset << std::endl;

    std::cout << "loaded " << cloud_filename << " with " << cloud->size() << " points"
              << std::endl;

    pcd_num_points = cloud->size();

    if (_vao == 0)
    {
        glGenVertexArrays(1, &_vao);
        glGenBuffers(1, &_vbo);
        glBindVertexArray(_vao);
        glBindBuffer(GL_ARRAY_BUFFER, _vbo);
        glBufferData(GL_ARRAY_BUFFER, cloud->size() * sizeof(pcl::PointXYZI), cloud->points.data(), GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(pcl::PointXYZI), (void *)0);
        glBindVertexArray(0);

        // Check for OpenGL errors
        GLenum error = glGetError();
        if (error != GL_NO_ERROR)
        {
            std::cerr << "OpenGL Error: " << error << std::endl;
            return;
        }

        std::cout << green
                  << "Created debug VAO/VBO for PCD PointXYZI\n"
                  << reset
                  << std::endl;

        has_pcd_setted = true;
    }
}

bool PclLoader::isLoaded() const
{
    return has_pcd_setted;
}

size_t PclLoader::getPointCount() const
{
    return pcd_num_points;
}