#ifndef GLK_ASSIMP_LOADER_ROS_HPP
#define GLK_ASSIMP_LOADER_ROS_HPP

#include <string>
#include <vector>
#include <memory>
#include <GL/gl3w.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

namespace glk
{

    /**
     * @brief Assimp-based mesh loader with TF-frame and OpenGL buffers
     * Supports multiple mesh formats (PLY, OBJ, FBX, etc.)
     */
    class AssimpLoaderROS
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// Constructor: loads mesh, computes initial TF, sets up OpenGL VAO/VBO/EBO
        AssimpLoaderROS(const std::string &path,
                        const std::string &tf_frame_in,
                        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                        const std::string &fixed_frame = "map");

        /// OpenGL handles
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint ebo = 0;
        GLsizei index_count = 0;

        /// TF transform, updated each frame
        Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
        std::string tf_frame;

    private:
        void setupBuffers(const std::vector<float> &vertices,
                          const std::vector<unsigned int> &indices);
    };

} // namespace glk
#endif // GLK_ASSIMP_LOADER_ROS_HPP