#pragma once

#include <string>
#include <memory>
#include <GL/gl3w.h>
#include <functional>
#include <vector>
#include <cstdint>
#include <unordered_map>
#include <queue>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Core Mapbox types for geographic data
#include <mbgl/map/camera.hpp>           // CameraOptions, zoom, pitch, bearing
#include <mbgl/util/geo.hpp>             // LatLng, LatLngBounds
#include <mbgl/util/size.hpp>            // Size (width, height)
#include <mbgl/util/mat4.hpp>            // Matrix operations
#include <mbgl/map/transform_state.hpp>  // Camera and map state management

// Mapbox style system for layers and sources
#include <mbgl/style/style.hpp>          // Main style container
#include <mbgl/style/layer.hpp>          // Base layer interface
#include <mbgl/style/source.hpp>         // Base source interface

// Specific layer types for proper data structures
#include <mbgl/style/layers/background_layer.hpp>  // Background layers
#include <mbgl/style/layers/fill_layer.hpp>        // Fill/polygon layers
#include <mbgl/style/layers/line_layer.hpp>        // Line/road layers
#include <mbgl/style/layers/symbol_layer.hpp>      // Icon/text layers
#include <mbgl/style/sources/vector_source.hpp>    // Vector tile sources
#include <mbgl/style/sources/raster_source.hpp>    // Raster tile sources

// Source types for map data
#include <mbgl/style/sources/vector_source.hpp>    // Vector tile sources
#include <mbgl/style/sources/raster_source.hpp>    // Raster tile sources

// Rendering logic components (NOT OpenGL context)
#include <mbgl/renderer/render_orchestrator.hpp>   // Layer rendering orchestration
#include <mbgl/renderer/render_layer.hpp>          // Individual layer rendering
#include <mbgl/renderer/bucket.hpp>                // Geometry data buckets
#include <mbgl/programs/program.hpp>               // Shader program management

#include <glk/modelUpload.hpp>


namespace mbgl {

using Size = mbgl::Size;
using LatLng = mbgl::LatLng;
using CameraOptions = mbgl::CameraOptions;
using LatLngBounds = mbgl::LatLngBounds;

using MapboxLayer = mbgl::style::Layer;
using MapboxSource = mbgl::style::Source;
using MapboxStyle = mbgl::style::Style;

class SimpleMapSnapshotter {
public:
    SimpleMapSnapshotter(Size size, float pixelRatio, const std::string& accessToken, const std::string& Photorealistic_3D_api_key);
    ~SimpleMapSnapshotter();
    
    float meters_per_tile_ = 1.0f;   // 1 tile == meters_per_tile_ meters (world units)
    uint8_t geometry_zoom_ = 15;     // keep what you actually use for visible tiles

    // =============================================================
    // step1: Mapbox style and camera management methods
    // =============================================================
    void setStyleURL(const std::string& styleURL);
    void setCameraOptions(const CameraOptions& camera);
    using Callback = std::function<void(std::exception_ptr, std::vector<uint8_t>, int, int)>;
    void snapshot(Callback callback);
    bool isTextureReady() const { return map_texture_ready_; }
    bool isReady() const { return map_texture_ready_ && !loading_; }
    bool isLoading() const { return loading_; }
    bool hasPendingTexture() const { return has_pending_texture_; }

    // =============================================================
    // step2: Geographic data management methods
    // =============================================================
    void setMapBounds(const LatLngBounds& bounds);
    LatLngBounds getMapBounds() const { return current_bounds_; }
    void updateMapBoundsFromCamera();

    // =============================================================
    // step3: Tile management for 3D rendering
    // =============================================================
    struct TileID {
        uint8_t z;   // Zoom level
        uint32_t x;  // Tile X coordinate  
        uint32_t y;  // Tile Y coordinate
        
        bool operator==(const TileID& other) const {
            return z == other.z && x == other.x && y == other.y;
        }
    };
    
    struct TileData {
        TileID id;
        GLuint texture_id = 0;
        bool loaded = false;
        glm::vec3 world_position;  // 3D position in world space
        float tile_size = 1.0f;    // Size in world units
    };
    
    std::vector<TileID> calculateVisibleTiles(const LatLngBounds& bounds, uint8_t zoom);
    void loadTile(const TileID& tile);
    std::string buildTileURL(const TileID& tile, const std::string& tileset = "mapbox.satellite");
    
    // =============================================================
    // step4: Geometry processing (Mapbox-style)
    // =============================================================
    
    // Geometry types from Mapbox Vector Tiles
    enum class GeometryType {
        POINT = 1,      // POIs, labels, symbols
        LINESTRING = 2, // Roads, borders, paths  
        LINE = 2,       // Alias for LINESTRING (streets/roads)
        POLYGON = 3     // Buildings, water, land areas
    };
    
    // Vertex data structure for 3D map geometry
    struct MapVertex {
        glm::vec3 position;    // 3D world position
        glm::vec2 texCoord;    // Texture coordinates
        glm::vec3 normal;      // For 3D lighting
        glm::vec4 color;       // Per-vertex color
        float elevation;       // Height data for extrusion
    };
    
    // Geometry buffer for different layer types
    struct GeometryBuffer {
        std::vector<MapVertex> vertices;
        std::vector<uint32_t> indices;
        GeometryType type;
        GLuint vao = 0;
        GLuint vbo = 0;
        GLuint ebo = 0;
        bool uploaded = false;
    };
    
    // Building geometry for 3D extrusion (like Mapbox buildings)
    struct BuildingGeometry {
        std::vector<glm::vec2> footprint;  // Building outline
        float height = 0.0f;               // Building height
        float base = 0.0f;                 // Building base height

        glm::vec4 roof_color = glm::vec4(0.718f, 0.745f, 0.80f, 1.0f);  // Fixed roof color
        glm::vec4 wall_color = glm::vec4(0.58f, 0.627f, 0.702f, 1.0f);  // Fixed wall color
    };

    glm::vec4 hexToRGBA(const std::string& hexColor, float alpha);

    // Road geometry for 3D roads
    struct RoadGeometry {
        std::vector<glm::vec2> centerline;  // 0..1
        float width = 0.0f;                 // tile fraction (px * invExtent)
        float invExtent = 1.0f;             // <-- NEW
        glm::vec4 color = {0.4f,0.4f,0.4f,1.0f};

        float zOffset;      // NEW
        bool isBridge;      // NEW
        bool isTunnel;      // NEW
        int bridgeLayer;    // NEW: for multi-level bridges
    };
    
    // Green area geometry for parks, forests, etc.
    struct GreenAreaGeometry {
        std::vector<glm::vec2> footprint;   // 0..1 in tile space
        std::string type;                   // "park", "forest", "grass", etc.
        glm::vec4 color = {0.2f, 0.8f, 0.3f, 1.0f};  // Default green color
        float height = 0.0f;                // Height for 3D effect (optional)
    };
    
    // Geometry processing methods
    GeometryBuffer generateTileGeometry(const TileID& tile);
    GeometryBuffer createBuildingGeometry(const std::vector<BuildingGeometry>& buildings, const TileID& tile);
    GeometryBuffer createRoadGeometry(const std::vector<RoadGeometry>& roads, const TileID& tile);
    GeometryBuffer createGreenAreaGeometry(const std::vector<GreenAreaGeometry>& greenAreas, const TileID& tile);
    GeometryBuffer createTerrainGeometry(const TileID& tile, const std::vector<float>& heightData);

    
    void uploadGeometryToGPU(GeometryBuffer& buffer);
    void cleanupGeometry(GeometryBuffer& buffer);
    
    // Geometry generation helpers
    // ✅ NEW: Real building data from Mapbox vector tiles
    void fetchVectorTileBuildings(const TileID& tile, std::vector<BuildingGeometry>& buildings);
    void parseVectorTileBuildings(const std::vector<uint8_t>& mvtData, const TileID& tile, std::vector<BuildingGeometry>& buildings);
    
    // ✅ NEW: Real green area data from Mapbox vector tiles
    void fetchVectorTileGreenAreas(const TileID& tile, std::vector<GreenAreaGeometry>& greenAreas);
    void parseVectorTileGreenAreas(const std::vector<uint8_t>& mvtData, const TileID& tile, std::vector<GreenAreaGeometry>& greenAreas);

    // Helper functions for real building parsing
    float calculatePolygonArea(const std::vector<glm::vec2>& polygon);
    void generateMinimalBuildings(const TileID& tile, std::vector<BuildingGeometry>& buildings);
    void generateSampleGreenAreas(const TileID& tile, std::vector<GreenAreaGeometry>& greenAreas);
    
    void generateSampleBuildings(const TileID& tile, std::vector<BuildingGeometry>& buildings);
    void fetchVectorTileStreets(const TileID& tile, std::vector<RoadGeometry>& roads);
    void generateRealisticRoads(const TileID& tile, std::vector<RoadGeometry>& roads);
    void parseVectorTileStreets(const std::vector<uint8_t>& mvtData, const TileID& tile, std::vector<RoadGeometry>& roads);
    void createExtrudedPolygon(const std::vector<glm::vec2>& footprint,
                            float height_m, float base_m,
                            const glm::vec4& roof_color, const glm::vec4& wall_color,
                            const glm::vec3& offset,
                            std::vector<MapVertex>& vertices, std::vector<uint32_t>& indices);
    void createRoadMesh(const std::vector<glm::vec2>& centerline, float width,
                       float zOffset, const glm::vec4& color, const glm::vec3& offset,
                       std::vector<MapVertex>& vertices, std::vector<uint32_t>& indices);
    void createGreenAreaMesh(const std::vector<glm::vec2>& footprint, const glm::vec4& color,
                            float height, const glm::vec3& offset,
                            std::vector<MapVertex>& vertices, std::vector<uint32_t>& indices);
    void createTerrainQuad(int x, int y, int resolution, const std::vector<float>& heightData,
                          const glm::vec3& offset, std::vector<MapVertex>& vertices, 
                          std::vector<uint32_t>& indices);
    
    // =============================================================
    // step5: 3D coordinate transformation
    // =============================================================
    glm::vec3 latLngToWorld(const LatLng& coord, float height = 0.0f);
    LatLng worldToLatLng(const glm::vec3& world_pos);
    glm::vec3 getTileWorldPosition(const TileID& tile);
    glm::mat4 getTileTransform(const TileID& tile);
    
    // =============================================================
    // step5: Integrated tile geometry management
    // =============================================================
    void updateTileGeometry();  // Calculate and load visible tile geometry
    void loadTileGeometry(const TileID& tile);  // Load geometry for a specific tile
    void processPendingTileGeometry();  // Upload pending geometry to GPU
    std::vector<TileID> getVisibleTilesForCurrentView();  // Get tiles for current camera
    size_t getLoadedGeometryCount() const { return tile_geometry_.size(); }  // Debug info
    
    // =============================================================
    // step6: Rendering methods
    // =============================================================
    void renderMap(const glm::mat4& projection, const glm::mat4& view);
    void uploadImageToTexture(const std::vector<uint8_t>& imageData, int width, int height);
    void updateTexture(); // Call this from main thread to upload pending texture data
    GLuint getMapTexture() const { return map_texture_id_; }
    bool hasValidTexture() const { return map_texture_id_ != 0 && map_texture_ready_; }
    
    // Rendering mode control
    void setRender3DMode(bool enable) { render_3d_mode_ = enable; }
    bool isRender3DMode() const { return render_3d_mode_; }

    // =============================================================
    // step7: 3D landmark management
    // =============================================================

    struct LandmarkRequest {
        mbgl::LatLng center;
        double radiusM = 150.0;
        std::string name;  // optional
        std::string kind;  // e.g., "cathedral"
    };

    // === Circle overlay state ===
    void initializeCircleShader();
    void renderCircle(const glm::mat4& projection, const glm::mat4& view);
    void setCircleLatLng(double lat_deg, double lng_deg) {
        circle_lat_deg_ = lat_deg;
        circle_lng_deg_ = lng_deg;
    }
    void setCircleVisible(bool enabled) { circle_enabled_ = enabled; }
    void setCircleRadiusMeters(float r) { circle_radius_m_ = std::max(0.0f, r); }
    void setCircleBorderMeters(float b) { circle_border_m_ = std::max(0.0f, b); }
    void setCircleColors(const glm::vec4& fill_rgba, const glm::vec4& edge_rgba) {
        circle_fill_rgba_ = fill_rgba;
        circle_edge_rgba_ = edge_rgba;
    }

    glm::vec2 latLngToWorldMeters(const LatLng& ll);

    void initializeCircleWalls(int segments = 128);
    void renderCircleWalls(const glm::mat4& projection, const glm::mat4& view);

    void renderCircleOverlay(const glm::mat4& projection, const glm::mat4& view);


    class Impl;
    
private:
    // Member variables
    Size size_;
    float pixelRatio_;
    std::string accessToken_;
    std::string current_style_url_;
    std::string Photorealistic_3D_api_key_;

    // Geographic data
    LatLngBounds current_bounds_;
    std::unique_ptr<mbgl::TransformState> transform_state_;
    
    // Camera state for bounds calculations
    LatLng current_center_;
    double current_zoom_ = 1.0;
    CameraOptions camera_; // Store camera options for tile alignment

    // Loading state
    mutable std::atomic<bool> loading_{false};
    mutable std::mutex texture_mutex_;
    mutable std::condition_variable texture_cv_;
    
    // Texture state
    bool map_texture_ready_ = false;
    GLuint map_texture_id_ = 0;
    int texture_width_ = 0;
    int texture_height_ = 0;
    
    // Pending texture data (for main thread upload)
    std::vector<uint8_t> pending_image_data_;
    int pending_width_ = 0;
    int pending_height_ = 0;
    bool has_pending_texture_ = false;
    
    // 3D tile management
    struct TileTextureData {
        std::string tile_key;
        std::vector<uint8_t> image_data;
        TileID tile;
    };
    
    std::unordered_map<std::string, TileData> loaded_tiles_;
    std::queue<TileTextureData> pending_tile_textures_;
    uint8_t current_zoom_level_ = 10;
    glm::vec3 map_center_world_ = glm::vec3(0.0f);
    float world_scale_ = 1000.0f; // Match the map texture scale
    
    // Geometry management
    std::unordered_map<std::string, GeometryBuffer> tile_geometry_;
    std::vector<BuildingGeometry> sample_buildings_;  // For testing
    std::vector<RoadGeometry> sample_roads_;         // For testing
    std::vector<GreenAreaGeometry> sample_green_areas_; // For testing
    
    // Tile bounds for positioning
    uint32_t current_tile_bounds_min_x_ = 0;
    uint32_t current_tile_bounds_max_x_ = 0;
    uint32_t current_tile_bounds_min_y_ = 0;
    uint32_t current_tile_bounds_max_y_ = 0;
    bool tile_bounds_valid_ = false;
    
    // Shader program for 2D map rendering (texture)
    GLuint map_shader_program_ = 0;
    GLuint map_vao_ = 0;
    GLuint map_vbo_ = 0;
    GLuint map_ebo_ = 0;
    
    // Shader uniforms for 2D rendering
    GLint u_projection_ = -1;
    GLint u_view_ = -1;
    GLint u_texture_ = -1;
    GLint u_alpha_ = -1;
    
    // 3D geometry shader program
    GLuint geometry_shader_program_ = 0;
    GLint u_geo_projection_ = -1;
    GLint u_geo_view_ = -1;
    GLint u_geo_model_ = -1;
    GLint u_light_dir_ = -1;
    GLint u_light_color_ = -1;
    GLint u_ambient_color_ = -1;
    GLint u_camera_pos_ = -1;
    GLint u_time_ = -1;
    bool geometry_shaders_initialized_ = false;
    bool render_3d_mode_ = true;  // NEW: Toggle between 2D texture and 3D geometry
    
    // Matrices
    glm::mat4 projection_matrix_;
    glm::mat4 view_matrix_;
    
    // Rendering helpers
    void initializeShaders();
    void initializeGeometry();
    void initializeGeometryShaders();  // NEW: Initialize 3D geometry shaders
    void renderTileGeometry(const glm::mat4& projection, const glm::mat4& view);  // NEW: Render 3D tiles
    void cleanup();
    void loadStyleFromMapbox();
    bool shaders_initialized_ = false;

    // 3D landmark management
    std::thread                    landmark_thread_;
    std::mutex                     landmark_mutex_;
    std::condition_variable        landmark_cv_;
    std::deque<LandmarkRequest>    pending_landmark_requests_;
    std::atomic<bool>              landmark_stop_{false};

    // === Circle overlay state ===
    bool   circle_enabled_   = true;
    double circle_lat_deg_   = 0.0;
    double circle_lng_deg_   = 0.0;
    float  circle_radius_m_  = 3.0f;   // requested radius
    float  circle_border_m_  = 1.0f;   // requested subtle border (ring width)
    float  circle_z_offset_  = 0.0f;  // lift to avoid z-fighting with roads

    glm::vec4 circle_fill_rgba_  = glm::vec4(0.10f, 0.80f, 0.20f, 0.35f); // soft green fill
    glm::vec4 circle_edge_rgba_  = glm::vec4(0.10f, 0.80f, 0.20f, 0.90f); // stronger green edge

    // GL resources
    GLuint circle_shader_program_ = 0;
    GLuint circle_vao_ = 0, circle_vbo_ = 0, circle_ebo_ = 0;

    // Uniform locations
    GLint u_c_proj_ = -1, u_c_view_ = -1, u_c_center_wm_ = -1;
    GLint u_c_radius_m_ = -1, u_c_border_m_ = -1, u_c_z_offset_ = -1;
    GLint u_c_fill_ = -1, u_c_edge_ = -1;

    // Height for the circle (meters)
    float circle_height_m_ = 2.0f; // tweak as you like
    float circle_total_radius_m_ = circle_radius_m_ + circle_border_m_;
    GLint u_c_height_m_ = -1;
    
    // Circle wall program + mesh
    GLuint circle_wall_program_ = 0;
    GLuint circle_wall_vao_ = 0, circle_wall_vbo_ = 0;
    GLint  u_w_proj_=-1, u_w_view_=-1, u_w_center_wm_=-1, u_w_radius_m_=-1, u_w_height_m_=-1, u_w_z_offset_=-1;
    GLint  u_w_color_=-1, u_w_light_dir_=-1, u_w_light_color_=-1, u_w_ambient_color_=-1;
    GLsizei circle_wall_vertex_count_ = 0;   
    
    

    std::unique_ptr<Impl> impl;
        
};

} // namespace mbgl