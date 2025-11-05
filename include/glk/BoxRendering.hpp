#ifndef GLK_BOX_RENDERING_HPP
#define GLK_BOX_RENDERING_HPP

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <GL/gl3w.h>

namespace glk {

// Vertex structure for box rendering
struct BoxVertex {
    Eigen::Vector3f position;    // 3D position
    Eigen::Vector2f texCoord;    // UV coordinates for gradient (u: horizontal 0=left edge, 0.5=center, 1=right edge; v: vertical 0=bottom, 1=top)
    float colorMode;             // 0=solid, 1=edges->center gradient, 2=center->edges gradient
    
    BoxVertex() : position(0, 0, 0), texCoord(0, 0), colorMode(0.0f) {}
};

// Structure for bounding box with 8 corners
struct BoundingBox {
    std::vector<Eigen::Vector3f> corners;   // 8 corners: 4 bottom (0-3), 4 top (4-7)
    Eigen::Vector3f color;                  // RGB color
    float alpha;                            // Base alpha value
    int colorMode;                          // 0=solid, 1=edges->center gradient, 2=center->edges gradient
    bool visible;                           // Visibility flag
    float icon_z_offset;                    // Vertical offset for icon positioning
    std::string type;                       // Type of obstacle (e.g., "CAR", "PEDESTRIAN", etc.)
    
    std::vector<BoxVertex> vertices;        // Generated vertices for rendering
    
    BoundingBox() : color(1.0f, 1.0f, 1.0f), alpha(1.0f), 
                     colorMode(1), visible(true), icon_z_offset(0.0f), type("") {}
};

class BoxRenderer {
public:
    BoxRenderer();
    ~BoxRenderer();
    
    // Initialize OpenGL resources
    bool initBoxRendering();
    
    // Add a bounding box with 8 corners
    // corners: 8 corners - 4 bottom (0-3), 4 top (4-7)
    // Order: bottom-back-left, bottom-back-right, bottom-front-right, bottom-front-left,
    //        top-back-left, top-back-right, top-front-right, top-front-left
    // color: RGB color
    // alpha: transparency (0=fully transparent, 1=fully opaque)
    // colorMode: 0=solid, 1=edges->center gradient, 2=center->edges gradient
    // z_offset: vertical offset to apply to all corners (default: 0.0)
    // icon_z_offset: additional vertical offset for icon positioning (default: 0.0)
    // type: type of obstacle (e.g., "CAR", "PEDESTRIAN", etc.) - used for icon rendering
    void addBoundingBox(const std::vector<Eigen::Vector3f>& corners,
                        const Eigen::Vector3f& color,
                        float alpha = 1.0f,
                        int colorMode = 1,
                        float z_offset = 0.0f,
                        float icon_z_offset = 0.0f,
                        const std::string& type = "");
    
    // Render all boxes
    void renderBoxes(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection);
    
    // Load icon texture for rendering on top of boxes
    // Returns true if loaded successfully
    bool loadIconTexture(const std::string& image_path);
    
    // Clear all boxes
    void clearBoxes();
    
    // Cleanup OpenGL resources
    void cleanupBoxRendering();
    
private:
    // OpenGL objects
    GLuint box_vao_;
    GLuint box_vbo_;
    GLuint box_shader_program_;
    
    // Icon rendering
    GLuint icon_texture_;
    GLuint icon_vao_;
    GLuint icon_vbo_;
    GLuint icon_shader_program_;
    bool icon_texture_loaded_;
    float icon_size_;  // Size of icon in world units
    
    // Box data
    std::vector<BoundingBox> bounding_boxes_;
    
    // Helper methods
    bool createBoxShaders();
    bool createIconShaders();
    GLuint compileShaderProgram(const char* vertex_source, const char* fragment_source);
    void generateBoundingBoxGeometry(BoundingBox& bbox);
    void renderIcons(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection);
};

} // namespace glk

#endif // GLK_BOX_RENDERING_HPP

