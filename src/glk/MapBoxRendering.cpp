#include <glk/MapBoxRendering.hpp>
#include <curl/curl.h>
#include <iostream>
#include <thread>
#include <future>
#include <sstream>
#include <iomanip>
#include <stb_image.h>
#include <cmath>
#include <GL/gl3w.h>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <cstdio>
#include <zlib.h>

// Mapbox vector tile includes 
#include <mapbox/vector_tile.hpp>
#include <mbgl/tile/vector_tile_data.hpp>
#include <mbgl/tile/geometry_tile_data.hpp>
#include <mbgl/util/feature.hpp>
#include <mapbox/earcut.hpp>


static constexpr float TILE_SIZE_WS = 1.0f;  // world units per tile
static constexpr int   GRID_SIZE    = 3;
static constexpr float MAP_TOTAL_WS = TILE_SIZE_WS * GRID_SIZE;

inline float snap01(float v, float eps = 2e-4f) {  // slightly larger than before
    if (std::abs(v) < eps) return 0.0f;
    if (std::abs(1.0f - v) < eps) return 1.0f;
    return v;
}

inline glm::vec2 clampBleed(glm::vec2 p, float halfWidthTile) {
    const float bleed = glm::clamp(halfWidthTile * 1.25f, 2e-4f, 1.2e-3f);
    p.x = snap01(p.x);
    p.y = snap01(p.y);
    return glm::clamp(p, glm::vec2(-bleed), glm::vec2(1.0f + bleed));
}

// meters per tile at latitude/zoom (Web Mercator)
inline double metersPerTile(double latDeg, int z) {
    constexpr double R = 6378137.0;
    const double C = 2.0 * M_PI * R;
    const double latRad = latDeg * M_PI / 180.0;
    return std::cos(latRad) * C / double(1u << z);
}

// tile-center latitude in degrees (Web Mercator)
inline double tileCenterLatDeg(uint32_t y, uint8_t z) {
    const double n = M_PI - 2.0 * M_PI * ( (double)y + 0.5 ) / double(1u << z);
    return 180.0 / M_PI * std::atan(std::sinh(n));
}

// =============================================================
// Google Photorealistic 3D Tiles API Helpers
// =============================================================

static const std::unordered_set<std::string> kLandmarkTypes = {
    "cathedral","church","mosque","temple","synagogue",
    "museum","library","theater","opera_house",
    "palace","castle","monument","stadium",
    "university","hospital","city_hall","embassy",
    "attraction","monument"
};

namespace mbgl {

// Implementation class with enhanced functionality
class SimpleMapSnapshotter::Impl {
public:
    Impl(Size size, float pixelRatio, const std::string& accessToken, const std::string& Photorealistic_3D_api_key)
        : size_(size), pixelRatio_(pixelRatio), accessToken_(accessToken), Photorealistic_3D_api_key_(Photorealistic_3D_api_key) {
        std::cout << "🔧 SimpleMapSnapshotter::Impl constructor" << std::endl;
        std::cout << "   Size: " << size_.width << "x" << size_.height << std::endl;
        std::cout << "   Pixel ratio: " << pixelRatio_ << std::endl;
        std::cout << "   Access token: " << accessToken_.substr(0, 20) << "..." << std::endl;
        
        curl_global_init(CURL_GLOBAL_DEFAULT);
        std::cout << "   CURL initialized successfully" << std::endl;
    }
    
    ~Impl() {
        std::cout << "🔧 SimpleMapSnapshotter::Impl destructor" << std::endl;
        curl_global_cleanup();
    }
    
    void setStyleURL(const std::string& styleURL) {
        std::cout << "�� Setting style URL: " << styleURL << std::endl;
        styleURL_ = styleURL;
    }

    void setCameraOptions(const CameraOptions& camera) {
        std::cout << "📷 Setting camera options:" << std::endl;
        
        if (camera.center) {
            std::cout << "   Center: " << camera.center->latitude() << ", " << camera.center->longitude() << std::endl;
        } else {
            std::cout << "   Center: not set" << std::endl;
        }
        
        if (camera.zoom) {
            std::cout << "   Zoom: " << *camera.zoom << std::endl;
        } else {
            std::cout << "   Zoom: not set" << std::endl;
        }
        
        if (camera.pitch) {
            std::cout << "   Pitch: " << *camera.pitch << std::endl;
        } else {
            std::cout << "   Pitch: not set" << std::endl;
        }
        
        if (camera.bearing) {
            std::cout << "   Bearing: " << *camera.bearing << std::endl;
        } else {
            std::cout << "   Bearing: not set" << std::endl;
        }
        
        camera_ = camera;
    }
        
    void snapshot(Callback callback) {
        std::cout << "�� Snapshot requested!" << std::endl;
        
        // Run in background thread
        std::thread([this, callback]() {
            try {
                auto imageData = fetchMapImage();
                std::cout << "✅ Map image fetched successfully!" << std::endl;
                callback(nullptr, imageData, size_.width, size_.height);
            } catch (const std::exception& e) {
                std::cerr << "❌ Snapshot error: " << e.what() << std::endl;
                auto fallbackData = generateFallbackImage();
                callback(std::make_exception_ptr(e), fallbackData, size_.width, size_.height);
            }
        }).detach();
    }
    
    std::vector<uint8_t> generateFallbackImage() {
        std::cout << "🔄 Generating fallback image..." << std::endl;
        
        int width = size_.width;
        int height = size_.height;
        std::vector<uint8_t> fallback(width * height * 3);
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = (y * width + x) * 3;
                fallback[index] = 100;     // R
                fallback[index + 1] = 150; // G
                fallback[index + 2] = 255 - (y * 100 / height); // B (gradient)
            }
        }
        
        return fallback;
    }
    
private:
    std::string buildMapboxURL() {
        std::string url = "https://api.mapbox.com/styles/v1/mapbox/streets-v12/static/";
        
        if (!styleURL_.empty() && styleURL_.find("mapbox://styles/") == 0) {
            std::string styleId = styleURL_.substr(16);
            url = "https://api.mapbox.com/styles/v1/" + styleId + "/static/";
        }
        
        // Check if camera options are set
        if (!camera_.center || !camera_.zoom) {
            std::cout << "⚠️ Camera options not fully set, using default values" << std::endl;
            url += "0,0,1"; // Default to center of world, zoom level 1
        } else {
            // Use center point and zoom level that matches our 3D tile system
            double center_lat = camera_.center->latitude();
            double center_lng = camera_.center->longitude();
            double zoom = *camera_.zoom;
            
            // Use the original zoom level for proper detail (but limit the 3D tiles to 12)
            uint8_t texture_zoom = static_cast<uint8_t>(std::max(10.0, std::min(zoom, 16.0)));
            
            // Use center point and zoom level approach (this works with Mapbox Static API)
            url += std::to_string(center_lng) + "," + std::to_string(center_lat);
            url += "," + std::to_string(texture_zoom);
            
            std::cout << "📍 2D Texture Geographic Area:" << std::endl;
            std::cout << "   Center: " << center_lat << ", " << center_lng << std::endl;
            std::cout << "   Zoom: " << (int)texture_zoom << " (for detailed texture)" << std::endl;
        }
        
        url += "/" + std::to_string(size_.width) + "x" + std::to_string(size_.height);
        url += "@2x?access_token=" + accessToken_;
        
        std::cout << "🌐 Fetching map that matches 3D tiles:" << std::endl;
        std::cout << "   " << url << std::endl;
        
        return url;
    }
    
    std::vector<uint8_t> fetchMapImage() {
        std::string url = buildMapboxURL();
                
        CURL* curl = curl_easy_init();
        if (!curl) {
            throw std::runtime_error("Failed to initialize CURL");
        }
        
        std::vector<uint8_t> imageData;
        std::string responseHeaders;
        
        // CURL options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &imageData);
        curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, HeaderCallback);
        curl_easy_setopt(curl, CURLOPT_HEADERDATA, &responseHeaders);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "MapboxViewer/1.0");
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 5L);
        
        std::cout << "📡 Sending HTTP request..." << std::endl;
        
        CURLcode res = curl_easy_perform(curl);
        
        if (res == CURLE_OK) {
            long http_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
            
            std::cout << "✅ HTTP Response:" << std::endl;
            std::cout << "   Status code: " << http_code << std::endl;
            std::cout << "   Response size: " << imageData.size() << " bytes" << std::endl;
            
            if (http_code != 200) {
                std::cerr << "⚠️ Warning: HTTP status code is " << http_code << std::endl;
                if (http_code >= 400) {
                    throw std::runtime_error("HTTP error: " + std::to_string(http_code));
                }
            }
            
            if (imageData.empty()) {
                throw std::runtime_error("No image data received from API");
            }
            
            validateImageData(imageData);
            
        } else {
            std::string error_msg = "CURL error: " + std::string(curl_easy_strerror(res));
            std::cerr << "❌ " << error_msg << std::endl;
            throw std::runtime_error(error_msg);
        }
        
        curl_easy_cleanup(curl);
        return imageData;
    }
    
    void validateImageData(const std::vector<uint8_t>& imageData) {
        if (imageData.size() >= 4) {
            std::cout << "�� Image validation:" << std::endl;
            printf("   First 4 bytes: %02X %02X %02X %02X\n", 
                   imageData[0], imageData[1], imageData[2], imageData[3]);
            
            if (imageData[0] == 0xFF && imageData[1] == 0xD8) {
                std::cout << "   ✅ Valid JPEG format detected" << std::endl;
            } else if (imageData[0] == 0x89 && imageData[1] == 0x50 && 
                       imageData[2] == 0x4E && imageData[3] == 0x47) {
                std::cout << "   ✅ Valid PNG format detected" << std::endl;
            } else {
                std::cout << "   ❓ Unknown format (proceeding anyway)" << std::endl;
            }
        }
    }
    
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::vector<uint8_t>* userp) {
        size_t totalSize = size * nmemb;
        const uint8_t* data = static_cast<const uint8_t*>(contents);
        userp->insert(userp->end(), data, data + totalSize);
        return totalSize;
    }
    
    static size_t HeaderCallback(char* buffer, size_t size, size_t nitems, std::string* userp) {
        size_t totalSize = size * nitems;
        userp->append(buffer, totalSize);
        return totalSize;
    }
    
    Size size_;
    float pixelRatio_;
    std::string accessToken_;
    std::string styleURL_;
    CameraOptions camera_;
    std::string Photorealistic_3D_api_key_;
    
    using Callback = std::function<void(std::exception_ptr, std::vector<uint8_t>, int, int)>;
};

// =============================================================
// step1: Mapbox style and camera management methods
// =============================================================
SimpleMapSnapshotter::SimpleMapSnapshotter(Size size, float pixelRatio, const std::string& accessToken, const std::string& Photorealistic_3D_api_key)
    : size_(size)
    , pixelRatio_(pixelRatio)
    , accessToken_(accessToken)
    , Photorealistic_3D_api_key_(Photorealistic_3D_api_key)
    , current_bounds_(LatLngBounds::world())  // 🆕 NEW: Initialize with world bounds
    , transform_state_(std::make_unique<mbgl::TransformState>())  // 🆕 NEW: Create TransformState
    , current_center_(LatLng(0, 0))  // 🆕 NEW: Initialize center
    , current_zoom_(1.0)  // 🆕 NEW: Initialize zoom
    , impl(std::make_unique<Impl>(size, pixelRatio, accessToken, Photorealistic_3D_api_key)) {
    
    std::cout << "🔧 SimpleMapSnapshotter constructor" << std::endl;
    
    // Initialize matrices
    projection_matrix_ = glm::mat4(1.0f);
    view_matrix_ = glm::mat4(1.0f);
    
    // 🆕 NEW: Initialize TransformState with size only
    transform_state_->setSize(mbgl::Size{static_cast<uint32_t>(size.width), static_cast<uint32_t>(size.height)});
    
    std::cout << "✅ SimpleMapSnapshotter initialized successfully with geographic data" << std::endl;
    std::cout << "   Initial bounds: world" << std::endl;
    std::cout << "   TransformState created and initialized" << std::endl;


    // Initialize 2D shaders and geometry if needed
    if (!shaders_initialized_) {
        initializeShaders();
        initializeGeometry();
        initializeCircleShader();
        initializeCircleWalls();
    }

    std::cout << "✅ Shaders and geometry initialized successfully" << std::endl;

}

SimpleMapSnapshotter::~SimpleMapSnapshotter() {
    std::cout << "�� Enhanced SimpleMapSnapshotter destructor" << std::endl;
    if (landmark_thread_.joinable()) {
        {
            std::lock_guard<std::mutex> lk(landmark_mutex_);
            landmark_stop_ = true;
        }
        landmark_cv_.notify_all();
        landmark_thread_.join();
    }
}

void SimpleMapSnapshotter::setStyleURL(const std::string& styleURL) {
    impl->setStyleURL(styleURL);
    current_style_url_ = styleURL;
    loadStyleFromMapbox();
}

void SimpleMapSnapshotter::setCameraOptions(const CameraOptions& camera) {
    impl->setCameraOptions(camera);
    
    // Store the camera options for tile calculations
    camera_ = camera;
    
    // �� NEW: Update bounds when camera changes
    if (camera.center && camera.zoom) {
        double lat = camera.center->latitude();
        double lng = camera.center->longitude();
        double zoom = *camera.zoom;
        
        // Store current camera state
        current_center_ = LatLng(lat, lng);
        current_zoom_ = zoom;

        // Snap to the geometry zoom you actually use for tiles (same rule as getVisibleTilesForCurrentView)
        geometry_zoom_ = static_cast<uint8_t>(std::max(8.0, std::min(std::floor(zoom + 1e-6), 15.0)));

        // ✅ 1 tile = meters_per_tile_ meters (world units)
        meters_per_tile_ = static_cast<float>(metersPerTile(lat, geometry_zoom_));
        std::cout << "📏 meters_per_tile_ = " << meters_per_tile_
                  << " @ lat=" << lat << " z=" << int(geometry_zoom_) << "\n";
        
        // Calculate bounds based on zoom level and viewport size
        // This is the same calculation Mapbox uses internally
        double metersPerPixel = 156543.03392 * std::cos(lat * M_PI / 180.0) / std::pow(2.0, zoom);
        double halfWidth = (size_.width / 2.0) * metersPerPixel;
        double halfHeight = (size_.height / 2.0) * metersPerPixel;
        
        // Convert meters to degrees (approximate)
        double degreeWidth = halfWidth / 111320.0;
        double degreeHeight = halfHeight / 110540.0;
        
        // Create bounds
        LatLngBounds newBounds = LatLngBounds::hull(
            LatLng(lat - degreeHeight, lng - degreeWidth),  // Southwest
            LatLng(lat + degreeHeight, lng + degreeWidth)   // Northeast
        );
        
        setMapBounds(newBounds);
        
        std::cout << "✅ Camera options set and bounds updated" << std::endl;
    }
}

void SimpleMapSnapshotter::snapshot(Callback callback) {
    // Set loading state
    loading_ = true;
    map_texture_ready_ = false;
    
    std::cout << "📸 Starting snapshot with loading state: " << (loading_ ? "true" : "false") << std::endl;
    
    // Use the callback pattern
    impl->snapshot([this, callback](std::exception_ptr error, std::vector<uint8_t> imageData, int width, int height) {
        // Clear loading state
        loading_ = false;
        
        if (!error) {
            std::cout << "🎉 Snapshot callback success!" << std::endl;
            std::cout << "   Received image: " << width << "x" << height << std::endl;
            std::cout << "   Data size: " << imageData.size() << " bytes" << std::endl;
            
            // Store image data for main thread upload (OpenGL context required)
            std::lock_guard<std::mutex> lock(texture_mutex_);
            pending_image_data_ = imageData;
            pending_width_ = width;
            pending_height_ = height;
            has_pending_texture_ = true;
            std::cout << "✅ Image data ready for GPU upload on main thread" << std::endl;
            
        } else {
            std::cerr << "❌ Snapshot callback error" << std::endl;
        }
        
        // Call the original callback
        callback(error, imageData, width, height);
    });
}

// =============================================================
// step2: Geographic data management methods
// =============================================================
void SimpleMapSnapshotter::setMapBounds(const LatLngBounds& bounds) {
    std::cout << "🗺️ Setting map bounds:" << std::endl;
    std::cout << "   North: " << bounds.north() << std::endl;
    std::cout << "   South: " << bounds.south() << std::endl;
    std::cout << "   East: " << bounds.east() << std::endl;
    std::cout << "   West: " << bounds.west() << std::endl;
    
    current_bounds_ = bounds;
    
    // The TransformState will be updated when we actually render
    if (transform_state_) {
        // We can only set the size, not the position/zoom
        transform_state_->setSize(mbgl::Size{static_cast<uint32_t>(size_.width), static_cast<uint32_t>(size_.height)});
        
        std::cout << "✅ TransformState size updated" << std::endl;
    }
    
    // �� NEW: Calculate and store the center and zoom for later use
    double centerLat = (bounds.north() + bounds.south()) / 2.0;
    double centerLng = (bounds.east() + bounds.west()) / 2.0;
    
    // Calculate appropriate zoom level based on bounds
    double latDiff = bounds.north() - bounds.south();
    double lngDiff = bounds.east() - bounds.west();
    double maxDiff = std::max(latDiff, lngDiff);
    
    // Convert to zoom level (approximate)
    double zoom = std::log2(360.0 / maxDiff);
    zoom = std::max(0.0, std::min(22.0, zoom)); // Clamp to valid zoom range
    
    std::cout << "   Calculated center: " << centerLat << ", " << centerLng << std::endl;
    std::cout << "   Calculated zoom: " << zoom << std::endl;
    
    
    std::cout << "✅ Map bounds set successfully" << std::endl;
}


void SimpleMapSnapshotter::updateMapBoundsFromCamera() {
    if (!impl) {
        std::cout << "⚠️ Impl not available, cannot update bounds from camera" << std::endl;
        return;
    }
    
    std::cout << "🗺️ Updating map bounds from camera..." << std::endl;
    
    // Get camera from impl (you'll need to add a getter)
    // For now, we'll calculate bounds based on current camera and size
    
    // Calculate bounds based on current view
    if (current_bounds_.north() != 0 || current_bounds_.south() != 0) {
        // We have existing bounds, let's refine them
        double centerLat = (current_bounds_.north() + current_bounds_.south()) / 2.0;
        double centerLng = (current_bounds_.east() + current_bounds_.west()) / 2.0;
        
        // Calculate viewport bounds based on size and zoom
        // This is a simplified calculation - in practice you'd use proper projection math
        double latSpan = (current_bounds_.north() - current_bounds_.south()) / 2.0;
        double lngSpan = (current_bounds_.east() - current_bounds_.west()) / 2.0;
        
        // Adjust for aspect ratio
        double aspectRatio = static_cast<double>(size_.width) / static_cast<double>(size_.height);
        if (aspectRatio > 1.0) {
            // Wider than tall
            lngSpan *= aspectRatio;
        } else {
            // Taller than wide
            latSpan /= aspectRatio;
        }
        
        // Create new bounds
        LatLngBounds newBounds = LatLngBounds::hull(
            LatLng(centerLat - latSpan, centerLng - lngSpan),  // Southwest
            LatLng(centerLat + latSpan, centerLng + lngSpan)   // Northeast
        );
        
        setMapBounds(newBounds);
        
        std::cout << "✅ Map bounds updated from camera view" << std::endl;
    } else {
        std::cout << "⚠️ No existing bounds to update from" << std::endl;
    }
}

// =============================================================
// step3: Tile management for 3D rendering
// =============================================================
std::vector<SimpleMapSnapshotter::TileID> SimpleMapSnapshotter::calculateVisibleTiles(const LatLngBounds& bounds, uint8_t zoom) {
    std::vector<TileID> tiles;
    
    // Web Mercator tile calculation
    auto latToTileY = [](double lat, uint8_t z) -> uint32_t {
        double lat_rad = lat * M_PI / 180.0;
        return static_cast<uint32_t>((1.0 - std::asinh(std::tan(lat_rad)) / M_PI) / 2.0 * (1 << z));
    };
    
    auto lngToTileX = [](double lng, uint8_t z) -> uint32_t {
        return static_cast<uint32_t>((lng + 180.0) / 360.0 * (1 << z));
    };
    
    uint32_t min_x = lngToTileX(bounds.west(), zoom);
    uint32_t max_x = lngToTileX(bounds.east(), zoom);
    uint32_t min_y = latToTileY(bounds.north(), zoom);  // Note: Y is flipped
    uint32_t max_y = latToTileY(bounds.south(), zoom);
    
    std::cout << "🔍 Calculating tiles for zoom " << (int)zoom << std::endl;
    std::cout << "   X range: " << min_x << " to " << max_x << std::endl;
    std::cout << "   Y range: " << min_y << " to " << max_y << std::endl;
    
    for (uint32_t x = min_x; x <= max_x; ++x) {
        for (uint32_t y = min_y; y <= max_y; ++y) {
            tiles.push_back({zoom, x, y});
        }
    }
    
    std::cout << "   Total tiles: " << tiles.size() << std::endl;
    return tiles;
}

std::string SimpleMapSnapshotter::buildTileURL(const TileID& tile, const std::string& tileset) {
    std::string url = "https://api.mapbox.com/v4/" + tileset + "/" + 
                      std::to_string(tile.z) + "/" + 
                      std::to_string(tile.x) + "/" + 
                      std::to_string(tile.y) + "@2x.jpg?access_token=" + accessToken_;
    
    std::cout << "🌐 Tile URL: " << url << std::endl;
    return url;
}

void SimpleMapSnapshotter::loadTile(const TileID& tile) {
    std::string tile_key = std::to_string(tile.z) + "_" + std::to_string(tile.x) + "_" + std::to_string(tile.y);
    
    // Check if already loaded
    if (loaded_tiles_.find(tile_key) != loaded_tiles_.end()) {
        return;
    }
    
    std::cout << "📦 Loading tile: " << tile_key << std::endl;
    
    // Create tile data entry
    TileData tile_data;
    tile_data.id = tile;
    tile_data.world_position = getTileWorldPosition(tile);
    loaded_tiles_[tile_key] = tile_data;
    
    // Load tile asynchronously
    std::thread([this, tile, tile_key]() {
        try {
            std::string url = buildTileURL(tile, "mapbox.satellite");
            
            CURL* curl = curl_easy_init();
            if (!curl) return;
            
            std::vector<uint8_t> imageData;
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, [](void* contents, size_t size, size_t nmemb, std::vector<uint8_t>* userp) -> size_t {
                size_t totalSize = size * nmemb;
                const uint8_t* data = static_cast<const uint8_t*>(contents);
                userp->insert(userp->end(), data, data + totalSize);
                return totalSize;
            });
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &imageData);
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);
            
            CURLcode res = curl_easy_perform(curl);
            curl_easy_cleanup(curl);
            
            if (res == CURLE_OK && !imageData.empty()) {
                // Store for main thread upload
                std::lock_guard<std::mutex> lock(texture_mutex_);
                TileTextureData tile_texture;
                tile_texture.tile_key = tile_key;
                tile_texture.image_data = std::move(imageData);
                tile_texture.tile = tile;
                pending_tile_textures_.push(tile_texture);
                
                std::cout << "✅ Tile loaded: " << tile_key << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "❌ Tile load error: " << e.what() << std::endl;
        }
    }).detach();
}

// =============================================================
// step4: 3D coordinate transformation  
// =============================================================
glm::vec3 SimpleMapSnapshotter::getTileWorldPosition(const TileID& tile) {
    if (!camera_.center || !camera_.zoom) return {0,0,0};

    const uint8_t z = tile.z;
    const int32_t n = 1 << z;

    const double lat = camera_.center->latitude();
    const double lng = camera_.center->longitude();

    // Floating tile coordinates of the CAMERA CENTER
    const double xFloat = (lng + 180.0) / 360.0 * n;
    const double yFloat = (1.0 - std::asinh(std::tan(lat * M_PI / 180.0)) / M_PI) * 0.5 * n;

    // Integer tile of the camera center
    int32_t cx = static_cast<int32_t>(std::floor(xFloat));
    int32_t cy = static_cast<int32_t>(std::floor(yFloat));

    // Fractional position inside that tile (0..1)
    const double fx = xFloat - cx;
    const double fy = yFloat - cy;

    // Shortest wrapped delta in X (same as before)
    int32_t dx = static_cast<int32_t>(tile.x) - cx;
    if (dx >  n/2) dx -= n;
    if (dx < -n/2) dx += n;

    // Y grows south in WebMercator tile space
    const int32_t dy = static_cast<int32_t>(tile.y) - cy;

    // Base tile-to-world translation (each tile = meters_per_tile_)
    float wx = dx * meters_per_tile_;
    float wy = -dy * meters_per_tile_; // keep your +Y=north convention

    // ⬇️ Anchor shift: subtract the camera's sub-tile offset (center (0.5,0.5) → world origin)
    wx -= float((fx - 0.5) * meters_per_tile_);
    wy -= float((0.5 - fy) * meters_per_tile_);

    return glm::vec3(wx, wy, 0.0f);
}

glm::vec2 SimpleMapSnapshotter::latLngToWorldMeters(const LatLng& ll) {
    // Requires camera_.center, geometry_zoom_, meters_per_tile_
    if (!camera_.center || !camera_.zoom) return glm::vec2(0.0f);

    const uint8_t z = geometry_zoom_;
    const int32_t n = 1 << z;

    // Center
    const double clat = camera_.center->latitude();
    const double clng = camera_.center->longitude();
    const double cx = (clng + 180.0) / 360.0 * n;
    const double cy = (1.0 - std::asinh(std::tan(clat * M_PI / 180.0)) / M_PI) * 0.5 * n;

    // Target
    const double tlat = ll.latitude();
    const double tlng = ll.longitude();
    double tx = (tlng + 180.0) / 360.0 * n;
    double ty = (1.0 - std::asinh(std::tan(tlat * M_PI / 180.0)) / M_PI) * 0.5 * n;

    // Wrap X to shortest path
    double dx = tx - cx;
    if (dx >  n/2) dx -= n;
    if (dx < -n/2) dx += n;

    // Y grows south in WebMercator tile space → invert for +Y=north
    const double dy = ty - cy;

    // Convert tiles → meters
    const float wx = float(dx * meters_per_tile_);
    const float wy = float(-dy * meters_per_tile_);
    return {wx, wy};
}


// =============================================================
// step5: Style and layer management methods
// =============================================================
void SimpleMapSnapshotter::loadStyleFromMapbox() {
    // For now, this is a placeholder. We'll implement style loading later
    std::cout << "📜 Loading style (placeholder): " << current_style_url_ << std::endl;
}

// =============================================================
// step4: Geometry processing implementation (Mapbox-style)
// =============================================================
SimpleMapSnapshotter::GeometryBuffer SimpleMapSnapshotter::generateTileGeometry(const TileID& tile) {
    std::cout << "🔺 Generating STREET geometry for tile " << (int)tile.z << "/" << tile.x << "/" << tile.y << std::endl;

    GeometryBuffer buffer;
    buffer.type = GeometryType::LINE; // Changed to LINE type for streets
            
        // ✅ NEW: Fetch REAL building data from Mapbox vector tiles
        std::vector<BuildingGeometry> tile_buildings;
        std::cout << "🏢 Fetching real building data from Mapbox..." << std::endl;
        fetchVectorTileBuildings(tile, tile_buildings);
        
        // ✅ NEW: Fetch REAL road data from Mapbox vector tiles (do this BEFORE building fallback)
        std::vector<RoadGeometry> tile_roads;
        std::cout << "🛣️ Fetching real Mapbox vector tile streets..." << std::endl;
        fetchVectorTileStreets(tile, tile_roads); // Use real vector tiles with proper positioning
        
        // ✅ NEW: Fetch REAL green area data from Mapbox vector tiles
        std::vector<GreenAreaGeometry> tile_green_areas;
        std::cout << "🌳 Fetching real Mapbox vector tile green areas..." << std::endl;
        fetchVectorTileGreenAreas(tile, tile_green_areas);
        
        
        // Fallback to sample buildings if no real data available
        if (tile_buildings.empty()) {
            // Only generate sample buildings if we also have no roads
            if (tile_roads.empty()) {
                std::cout << "⚠️ No real building data and no roads, using sample buildings..." << std::endl;
                try {
                    std::cout << "🏢 Generating sample buildings..." << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "❌ Sample building generation failed: " << e.what() << std::endl;
                    tile_buildings.clear();
                } catch (...) {
                    std::cerr << "❌ Unknown error in sample building generation" << std::endl;
                    tile_buildings.clear();
                }
            } else {
                std::cout << "🛣️ No buildings but has roads - proceeding with road-only tile" << std::endl;
            }
        }
        
        // If still no buildings after fallback AND no roads, we still want to process the tile for terrain
        if (tile_buildings.empty() && tile_roads.empty()) {
            std::cout << "🏞️ No buildings or roads available for tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                      << ", but will still generate terrain" << std::endl;
        }
    
    // Check if this tile has any urban features (buildings, roads, or green areas)
    if (tile_buildings.empty() && tile_roads.empty() && tile_green_areas.empty()) {
        std::cout << "🏞️ Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " appears to be terrain/water only (no buildings, roads, or green areas), but will still generate terrain" << std::endl;
    }
    
    // Log what urban features we found
    if (!tile_buildings.empty() && !tile_roads.empty() && !tile_green_areas.empty()) {
        std::cout << "🏙️ Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " has buildings (" << tile_buildings.size() << "), roads (" << tile_roads.size() 
                  << "), and green areas (" << tile_green_areas.size() << ")" << std::endl;
    } else if (!tile_buildings.empty() && !tile_roads.empty()) {
        std::cout << "🏙️ Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " has both buildings (" << tile_buildings.size() << ") and roads (" << tile_roads.size() << ")" << std::endl;
    } else if (!tile_buildings.empty() && !tile_green_areas.empty()) {
        std::cout << "🏢🌳 Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " has buildings (" << tile_buildings.size() << ") and green areas (" << tile_green_areas.size() << ")" << std::endl;
    } else if (!tile_roads.empty() && !tile_green_areas.empty()) {
        std::cout << "🛣️🌳 Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " has roads (" << tile_roads.size() << ") and green areas (" << tile_green_areas.size() << ")" << std::endl;
    } else if (!tile_buildings.empty()) {
        std::cout << "🏢 Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " has buildings (" << tile_buildings.size() << ") but no roads or green areas" << std::endl;
    } else if (!tile_roads.empty()) {
        std::cout << "🛣️ Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " has roads (" << tile_roads.size() << ") but no buildings or green areas" << std::endl;
    } else if (!tile_green_areas.empty()) {
        std::cout << "🌳 Tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << " has green areas (" << tile_green_areas.size() << ") but no buildings or roads" << std::endl;
    }
    
    // Fallback to sample roads if no real data available
    if (tile_roads.empty()) {
        std::cout << "⚠️ No real road data, using sample roads..." << std::endl;
    }
    
    // Fallback to sample green areas if no real data available
    if (tile_green_areas.empty()) {
        std::cout << "⚠️ No real green area data, using sample green areas..." << std::endl;
    }
    
    // If still no roads after fallback, create minimal roads to prevent crashes
    if (tile_roads.empty()) {
        std::cout << "⚠️ No roads available for tile " << (int)tile.z << "/" << tile.x << "/" << tile.y 
                  << ", creating minimal road network" << std::endl;
        
        // Create a simple cross-shaped road network
        RoadGeometry cross_road;
        cross_road.centerline = {{0.25f, 0.5f}, {0.75f, 0.5f}}; // Horizontal
        cross_road.width = 8.0f;
        cross_road.color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f);
        tile_roads.push_back(cross_road);
        
        RoadGeometry vertical_road;
        vertical_road.centerline = {{0.5f, 0.25f}, {0.5f, 0.75f}}; // Vertical
        vertical_road.width = 8.0f;
        vertical_road.color = glm::vec4(0.3f, 0.3f, 0.3f, 1.0f);
        tile_roads.push_back(vertical_road);
    }
    
    std::cout << "🛣️ Generated " << tile_roads.size() << " roads for tile " 
              << (int)tile.z << "/" << tile.x << "/" << tile.y << std::endl;
    
    // Debug: Check if roads span full tile area (0-1 coordinates)
    for (size_t i = 0; i < std::min(tile_roads.size(), size_t(3)); ++i) {
        if (!tile_roads[i].centerline.empty()) {
            auto& road = tile_roads[i];
            glm::vec2 start = road.centerline.front();
            glm::vec2 end = road.centerline.back();
            std::cout << "🛣️ Road " << i << ": (" << start.x << ", " << start.y 
                      << ") → (" << end.x << ", " << end.y << ") width=" << road.width << std::endl;
        }
    }
    
    // ✅ ENABLED: Create building, road, and/or green area geometry based on what's available
    GeometryBuffer building_buffer, road_buffer, green_area_buffer;
    
    if (!tile_buildings.empty()) {
        building_buffer = createBuildingGeometry(tile_buildings, tile);
    }
    
    if (!tile_roads.empty()) {
        road_buffer = createRoadGeometry(tile_roads, tile);
    }
    
    if (!tile_green_areas.empty()) {
        green_area_buffer = createGreenAreaGeometry(tile_green_areas, tile);
    }

    // Combine geometries for complete rendering
    auto combineGeometries = [&](const std::vector<GeometryBuffer>& buffers, const std::string& description) {
        if (buffers.empty()) return;
        
        size_t total_vertices = 0;
        size_t total_indices = 0;
        
        for (const auto& buf : buffers) {
            total_vertices += buf.vertices.size();
            total_indices += buf.indices.size();
        }
        
        buffer.vertices.reserve(total_vertices);
        buffer.indices.reserve(total_indices);
        
        size_t vertex_offset = 0;
        for (const auto& buf : buffers) {
            // Add vertices
            for (const auto& vertex : buf.vertices) {
                buffer.vertices.push_back(vertex);
            }
            
            // Add indices with offset
            for (const auto& index : buf.indices) {
                buffer.indices.push_back(vertex_offset + index);
            }
            
            vertex_offset += buf.vertices.size();
        }
        
        std::cout << "✅ Generated " << description << " geometry: " << buffer.vertices.size() 
                  << " vertices, " << buffer.indices.size() << " indices" << std::endl;
    };
    
    // Determine which geometries to combine
    std::vector<GeometryBuffer> geometries_to_combine;
    std::string description;
    
    if (!tile_buildings.empty()) geometries_to_combine.push_back(building_buffer);
    if (!tile_roads.empty()) geometries_to_combine.push_back(road_buffer);
    if (!tile_green_areas.empty()) geometries_to_combine.push_back(green_area_buffer);
    
    if (!geometries_to_combine.empty()) {
        // Build description
        std::vector<std::string> parts;
        if (!tile_buildings.empty()) parts.push_back(std::to_string(tile_buildings.size()) + " buildings");
        if (!tile_roads.empty()) parts.push_back(std::to_string(tile_roads.size()) + " roads");
        if (!tile_green_areas.empty()) parts.push_back(std::to_string(tile_green_areas.size()) + " green areas");
        
        description = "MIXED (" + parts[0];
        for (size_t i = 1; i < parts.size(); ++i) {
            description += " + " + parts[i];
        }
        description += ")";
        
        combineGeometries(geometries_to_combine, description);
    }
    
    
    return buffer;
}

SimpleMapSnapshotter::GeometryBuffer
SimpleMapSnapshotter::createBuildingGeometry(const std::vector<BuildingGeometry>& buildings,
                                             const TileID& /*tile*/) {
    GeometryBuffer buffer;
    buffer.type = GeometryType::POLYGON;

    // Keep vertices tile-local; placement happens in render via model translate
    const glm::vec3 offsetZero(0.0f);

    glm::vec4 roof_color = hexToRGBA("#808080", 1.0f);  // roof color from comments (more light) 
    glm::vec4 wall_color = hexToRGBA("#575757", 1.0f);  // walls color from comments (more dark)
    
    for (const auto& b : buildings) {
        createExtrudedPolygon(
            b.footprint, 
            b.height, 
            b.base, 
            roof_color, 
            wall_color,
            offsetZero, 
            buffer.vertices, 
            buffer.indices
        );
    }
    return buffer;
}

SimpleMapSnapshotter::GeometryBuffer
SimpleMapSnapshotter::createRoadGeometry(const std::vector<RoadGeometry>& roads,
                                         const TileID& /*tile*/) {
    GeometryBuffer buffer;
    buffer.type = GeometryType::LINESTRING;

    const glm::vec3 offsetZero(0.0f);

    for (const auto& r : roads) {
        createRoadMesh(r.centerline, r.width, r.zOffset, r.color, offsetZero,
                    buffer.vertices, buffer.indices);
    }
    return buffer;
}

SimpleMapSnapshotter::GeometryBuffer
SimpleMapSnapshotter::createGreenAreaGeometry(const std::vector<GreenAreaGeometry>& greenAreas,
                                             const TileID& /*tile*/) {
    GeometryBuffer buffer;
    buffer.type = GeometryType::POLYGON;

    const glm::vec3 offsetZero(0.0f);

    for (const auto& area : greenAreas) {
        createGreenAreaMesh(area.footprint, area.color, area.height, offsetZero,
                           buffer.vertices, buffer.indices);
    }
    return buffer;
}

SimpleMapSnapshotter::GeometryBuffer
SimpleMapSnapshotter::createTerrainGeometry(const TileID& /*tile*/,
                                            const std::vector<float>& heightData) {
    GeometryBuffer buffer;
    buffer.type = GeometryType::POLYGON;

    const glm::vec3 offsetZero(0.0f);
    const int resolution = 32;

    for (int y = 0; y < resolution - 1; ++y) {
        for (int x = 0; x < resolution - 1; ++x) {
            createTerrainQuad(
                x, y, resolution,
                heightData,
                offsetZero,   // ⬅️ keep terrain tile-local; placement via model translate
                buffer.vertices,
                buffer.indices
            );
        }
    }
    return buffer;
}


void SimpleMapSnapshotter::uploadGeometryToGPU(GeometryBuffer& buffer) {
    if (buffer.uploaded || buffer.vertices.empty()) {
        return;
    }
    
    std::cout << "📤 Uploading geometry to GPU: " << buffer.vertices.size() << " vertices" << std::endl;
    
    // Generate OpenGL objects
    glGenVertexArrays(1, &buffer.vao);
    glGenBuffers(1, &buffer.vbo);
    glGenBuffers(1, &buffer.ebo);
    
    // Bind VAO
    glBindVertexArray(buffer.vao);
    
    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, buffer.vbo);
    glBufferData(GL_ARRAY_BUFFER, buffer.vertices.size() * sizeof(MapVertex), 
                 buffer.vertices.data(), GL_STATIC_DRAW);
    
    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, buffer.indices.size() * sizeof(uint32_t),
                 buffer.indices.data(), GL_STATIC_DRAW);
    
    // Setup vertex attributes
    // Position (location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MapVertex), (void*)offsetof(MapVertex, position));
    glEnableVertexAttribArray(0);
    
    // Texture coordinates (location 1) 
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(MapVertex), (void*)offsetof(MapVertex, texCoord));
    glEnableVertexAttribArray(1);
    
    // Normal (location 2)
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(MapVertex), (void*)offsetof(MapVertex, normal));
    glEnableVertexAttribArray(2);
    
    // Color (location 3)
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(MapVertex), (void*)offsetof(MapVertex, color));
    glEnableVertexAttribArray(3);
    
    // Elevation (location 4)
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(MapVertex), (void*)offsetof(MapVertex, elevation));
    glEnableVertexAttribArray(4);
    
    glBindVertexArray(0);
    buffer.uploaded = true;
    
    std::cout << "✅ Geometry uploaded (VAO: " << buffer.vao << ")" << std::endl;
}

void SimpleMapSnapshotter::cleanupGeometry(GeometryBuffer& buffer) {
    if (buffer.vao != 0) glDeleteVertexArrays(1, &buffer.vao);
    if (buffer.vbo != 0) glDeleteBuffers(1, &buffer.vbo);
    if (buffer.ebo != 0) glDeleteBuffers(1, &buffer.ebo);
    
    buffer.vao = buffer.vbo = buffer.ebo = 0;
    buffer.uploaded = false;
}

// =============================================================
// Geometry generation helper functions
// =============================================================
void SimpleMapSnapshotter::fetchVectorTileStreets(const TileID& tile, std::vector<RoadGeometry>& roads) {
    // Fetch real vector tile data from Mapbox to get actual street coordinates
    // This uses Mapbox GL Native's vector tile parsing infrastructure
    
    roads.clear();
    
    // Build vector tile URL (Mapbox Streets tileset)  
    std::string url = "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/" + 
                     std::to_string((int)tile.z) + "/" + std::to_string(tile.x) + "/" + std::to_string(tile.y) + 
                     ".mvt?access_token=" + accessToken_;
    
    std::cout << "🌐 Fetching vector tile data: " << url << std::endl;
    
    // IMPORTANT: Avoid segfault by using system command instead of CURL in OpenGL thread
    // This is safer for your custom OpenGL context
    std::cout << "📡 Downloading vector tile using safe method..." << std::endl;
    
    // Create a temporary file for the download
    std::string tempFile = "/tmp/tile_" + std::to_string(tile.z) + "_" + 
                          std::to_string(tile.x) + "_" + std::to_string(tile.y) + ".mvt";
    
    // Use wget or curl command line tool (safer than library in OpenGL context)
    std::string command = "curl -s -L -o \"" + tempFile + "\" \"" + url + "\"";
    int result = std::system(command.c_str());
    
    if (result != 0) {
        std::cout << "❌ Failed to download vector tile using system command" << std::endl;
        return;
    }
    
    // Read the downloaded file
    std::ifstream file(tempFile, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "❌ Failed to open downloaded tile file" << std::endl;
        return;
    }
    
    // Get file size
    file.seekg(0, std::ios::end);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    if (size <= 0) {
        std::cout << "❌ Empty or invalid tile file" << std::endl;
        file.close();
        std::remove(tempFile.c_str());
        return;
    }
    
    // Read the data
    std::vector<uint8_t> mvtData(size);
    if (!file.read(reinterpret_cast<char*>(mvtData.data()), size)) {
        std::cout << "❌ Failed to read tile data" << std::endl;
        file.close();
        std::remove(tempFile.c_str());
        return;
    }
    
    file.close();
    std::remove(tempFile.c_str()); // Clean up temp file
    
    std::cout << "✅ Downloaded vector tile: " << mvtData.size() << " bytes" << std::endl;
    
    // Parse the real MVT data
    std::cout << "🔍 Parsing vector tile data..." << std::endl;
    parseVectorTileStreets(mvtData, tile, roads);
    std::cout << "✅ Vector tile parsing completed" << std::endl;
}

void SimpleMapSnapshotter::parseVectorTileStreets(const std::vector<uint8_t>& mvtData,
                                                  const TileID& tile,
                                                  std::vector<RoadGeometry>& roads)
{
    roads.clear();
    if (mvtData.empty()) {
        std::cout << "❌ Empty vector tile data\n";
        return;
    }

    // Guard: we rely on meters_per_tile_ being set by setCameraOptions()
    if (meters_per_tile_ <= 0.0f) {
        // Fallback to tile-center latitude if needed (defensive)
        const double latDeg   = tileCenterLatDeg(tile.y, tile.z);
        const double mPerTile = metersPerTile(latDeg, tile.z);
        if (mPerTile <= 0.0) {
            std::cout << "⚠️ Invalid meters_per_tile_; aborting road parse.\n";
            return;
        }
    }

    // --- Decompress if gzip ---
    std::vector<uint8_t> buf;
    try {
        if (mvtData.size() >= 2 && mvtData[0] == 0x1f && mvtData[1] == 0x8b) {
            z_stream s{};
            s.avail_in = static_cast<uInt>(mvtData.size());
            s.next_in  = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(mvtData.data()));
            if (inflateInit2(&s, 15 + 32) != Z_OK) throw std::runtime_error("gzip init failed");
            const size_t CHUNK = 16384;
            int ret = Z_OK;
            do {
                size_t base = buf.size();
                buf.resize(base + CHUNK);
                s.avail_out = CHUNK;
                s.next_out  = reinterpret_cast<Bytef*>(buf.data() + base);
                ret = inflate(&s, Z_NO_FLUSH);
                if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
                    inflateEnd(&s);
                    throw std::runtime_error("gzip inflate failed");
                }
                buf.resize(base + (CHUNK - s.avail_out));
            } while (ret != Z_STREAM_END);
            inflateEnd(&s);
        } else {
            buf = mvtData;
        }
    } catch (const std::exception& e) {
        std::cout << "❌ Gzip inflate error: " << e.what() << "\n";
        return;
    }

    try {
        // --- Parse MVT ---
        std::string blob(reinterpret_cast<const char*>(buf.data()), buf.size());
        mapbox::vector_tile::buffer vt(blob);
        auto layers = vt.getLayers();

        // Helpers -------------
        auto getNum = [](const std::unordered_map<std::string, mapbox::feature::value>& props,
                         const char* key, float& out)->bool {
            auto it = props.find(key);
            if (it == props.end()) return false;
            const auto& v = it->second;
            if (v.is<double>())      { out = float(v.get<double>());   return true; }
            if (v.is<int64_t>())     { out = float(v.get<int64_t>());  return true; }
            if (v.is<uint64_t>())    { out = float(v.get<uint64_t>()); return true; }
            if (v.is<std::string>()) { try { out = std::stof(v.get<std::string>()); return true; } catch (...) {} }
            return false;
        };

        auto getStr = [](const std::unordered_map<std::string, mapbox::feature::value>& props,
                         const char* key, std::string& out)->bool {
            auto it = props.find(key);
            if (it == props.end()) return false;
            if (it->second.is<std::string>()) { out = it->second.get<std::string>(); return true; }
            return false;
        };

        auto getBool = [](const std::unordered_map<std::string, mapbox::feature::value>& props,
                          const char* key, bool& out)->bool {
            auto it = props.find(key);
            if (it == props.end()) return false;
            const auto& v = it->second;
            if (v.is<bool>())        { out = v.get<bool>(); return true; }
            if (v.is<int64_t>())     { out = (v.get<int64_t>() != 0); return true; }
            if (v.is<uint64_t>())    { out = (v.get<uint64_t>() != 0); return true; }
            if (v.is<std::string>()) {
                std::string s = v.get<std::string>();
                std::transform(s.begin(), s.end(), s.begin(), ::tolower);
                if (s=="true"||s=="yes"||s=="1") { out = true; return true; }
                if (s=="false"||s=="no" ||s=="0") { out = false; return true; }
            }
            return false;
        };

        auto clampf = [](float v, float lo, float hi){ return std::max(lo, std::min(hi, v)); };

        auto normClass = [](std::string c)->std::string {
            std::transform(c.begin(), c.end(), c.begin(), ::tolower);
            if (c == "street_limited") return "street";
            // treat *_link as their base class
            if (c.find("_link") != std::string::npos) {
                auto base = c.substr(0, c.find("_link"));
                return base;
            }
            return c;
        };

        // Defaults (kerb-to-kerb for two-way unless noted)
        auto defaultTotalWidth = [](const std::string& c)->float {
            if (c == "motorway")   return 12.0f; // per carriageway typical (oneway/split)
            if (c == "trunk")      return 10.5f; // per carriageway typical (oneway/split)
            if (c == "primary")    return 9.0f;  // many primary are single carriageway
            if (c == "secondary")  return 8.0f;
            if (c == "tertiary")   return 7.0f;
            if (c == "residential" || c == "street") return 9.0f;  // common urban kerb-to-kerb
            if (c == "service")    return 5.0f;
            if (c == "track")      return 3.5f;
            if (c == "pedestrian") return 4.0f;  // narrower by default
            if (c == "path")       return 2.0f;
            return 6.5f;
        };

        auto laneWidthFor = [](const std::string& c)->float {
            if (c == "motorway" || c == "trunk") return 3.6f;
            if (c == "primary" || c == "secondary" || c == "tertiary") return 3.25f;
            if (c == "residential" || c == "street") return 3.0f;
            if (c == "service" || c == "track") return 2.75f;
            return 3.0f;
        };

        auto clampByClass = [&](const std::string& c, float w)->float {
            if (c == "motorway")    return clampf(w, 8.0f, 14.0f);     // per carriageway
            if (c == "trunk")       return clampf(w, 7.0f, 12.0f);     // per carriageway
            if (c == "primary")     return clampf(w, 7.0f, 12.0f);
            if (c == "secondary")   return clampf(w, 6.0f, 10.0f);
            if (c == "tertiary")    return clampf(w, 5.5f, 9.0f);
            if (c == "residential" || c=="street") return clampf(w, 6.0f, 12.5f);
            if (c == "service")     return clampf(w, 3.5f, 7.0f);
            if (c == "track")       return clampf(w, 2.5f, 6.0f);
            if (c == "pedestrian")  return clampf(w, 2.5f, 8.0f);
            if (c == "path")        return clampf(w, 1.0f, 4.0f);
            return clampf(w, 4.0f, 12.0f);
        };

        // Which layers to read (Mapbox Streets v8)
        static const std::unordered_set<std::string> ROAD_LAYERS = {
            "road", "bridge", "tunnel" // Mapbox Streets v8 names
        };

        size_t added_before = roads.size();

        for (const auto& lp : layers) {
            const std::string lname = lp.first;
            if (!ROAD_LAYERS.count(lname)) continue;

            bool layerImpliesBridge = (lname == "bridge");
            bool layerImpliesTunnel = (lname == "tunnel");

            mapbox::vector_tile::layer layer(lp.second);
            const float extent    = static_cast<float>(layer.getExtent());
            const float invExtent = extent > 0.0f ? 1.0f / extent : 1.0f;

            for (std::size_t i = 0; i < layer.featureCount(); ++i) {
                try {
                    auto fdata = layer.getFeature(i);
                    mapbox::vector_tile::feature feat(fdata, layer);
                    if (feat.getType() != mapbox::vector_tile::GeomType::LINESTRING) continue;

                    // Geometry (tile-local 0..extent)
                    auto geoms = feat.getGeometries<mapbox::vector_tile::points_arrays_type>(1.0f);
                    if (geoms.empty()) continue;

                    const auto props = feat.getProperties();

                    // Class
                    std::string cls = "street";
                    (void)getStr(props, "class", cls);
                    cls = normClass(cls);

                    // Ignore ferries, construction, etc. (lines we don't render as asphalt)
                    if (cls == "ferry" || cls == "construction") continue;

                    // Read hints
                    bool oneway = false;
                    (void)getBool(props, "oneway", oneway); // present for many ways in Streets
                    // Some tiles encode ramps/links via class suffix; we normalized above.
                    bool isLink = false;
                    {
                        std::string rampStr;
                        if (getStr(props, "ramp", rampStr)) {
                            std::string s = rampStr;
                            std::transform(s.begin(), s.end(), s.begin(), ::tolower);
                            isLink = (s=="true"||s=="yes"||s=="1");
                        }
                    }

                    // Bridges/tunnels from props (in addition to layer name)
                    bool isBridge = layerImpliesBridge, isTunnel = layerImpliesTunnel;
                    {
                        std::string structure;
                        if (getStr(props, "structure", structure)) {
                            std::transform(structure.begin(), structure.end(), structure.begin(), ::tolower);
                            if (structure == "bridge")  isBridge = true;
                            if (structure == "tunnel")  isTunnel = true;
                        }
                        (void)getBool(props, "bridge", isBridge);
                        (void)getBool(props, "tunnel", isTunnel);
                    }

                    // Width in meters (kerb-to-kerb for two-way; per-carriageway for one-way/split)
                    float width_m = 0.0f;

                    // 1) If explicit width exists, trust it. OSM `width=*` is actual feature width. :contentReference[oaicite:2]{index=2}
                    float wExplicit = 0.0f;
                    if (getNum(props, "width", wExplicit) && wExplicit > 0.5f) {
                        width_m = wExplicit;
                    }

                    // 2) Otherwise, estimate from lanes and class
                    if (width_m <= 0.0f) {
                        float lanes_total = 0.0f, lanes_fwd = 0.0f, lanes_back = 0.0f;
                        (void)getNum(props, "lanes", lanes_total);            // total lanes both dirs :contentReference[oaicite:3]{index=3}
                        (void)getNum(props, "lanes:forward", lanes_fwd);      // may exist (directional) :contentReference[oaicite:4]{index=4}
                        (void)getNum(props, "lanes:backward", lanes_back);

                        const float lane_w = laneWidthFor(cls);

                        if (cls == "pedestrian" || cls == "path") {
                            // Pedestrian/path: use class defaults; lanes don't apply
                            width_m = defaultTotalWidth(cls);
                        } else {
                            if (oneway) {
                                // Single carriageway represented: use forward/back if present, else total
                                float used = (lanes_fwd > 0.0f ? lanes_fwd :
                                              lanes_back > 0.0f ? lanes_back :
                                              lanes_total > 0.0f ? lanes_total : 0.0f);
                                if (used > 0.0f) {
                                    // add edges/shoulders modestly
                                    float edge = (cls=="motorway"||cls=="trunk") ? 1.2f : 0.6f;
                                    width_m = used * lane_w + 2.0f * edge;
                                }
                            } else {
                                // Two-way single carriageway (most urban roads)
                                if (lanes_total > 0.0f) {
                                    // kerb-to-kerb estimate from total lanes
                                    float edge = (cls=="motorway"||cls=="trunk") ? 1.2f : 0.6f;
                                    width_m = lanes_total * lane_w + 2.0f * edge;
                                } else if (lanes_fwd > 0.0f || lanes_back > 0.0f) {
                                    // fall back to max dir if split tagging present
                                    float used = std::max(lanes_fwd, lanes_back);
                                    float edge = 0.6f;
                                    width_m = used * lane_w * 2.0f + 2.0f * edge; // approximate both dirs
                                }
                            }
                        }

                        // 3) Final fallback to class defaults if still unknown
                        if (width_m <= 0.0f) {
                            // Motorways/trunks in Streets are typically split; defaults were per-carriageway.
                            if (cls == "motorway" || cls == "trunk") {
                                width_m = defaultTotalWidth(cls);
                                if (!oneway) width_m += 2.0f; // hint of median if not split
                            } else {
                                width_m = defaultTotalWidth(cls);
                            }
                        }
                    }

                    // Clamp to realistic ranges by class (prevents giant pedestrian streets, etc.)
                    width_m = clampByClass(cls, width_m);

                    // Links a bit narrower
                    if (isLink && (cls=="motorway"||cls=="trunk"||cls=="primary"||cls=="secondary"))
                        width_m *= 0.9f;

                    // Convert meters → tile fraction expected by createRoadMesh
                    const float width_tile = width_m / meters_per_tile_;

                    glm::vec4 concrete_color = hexToRGBA("#404040", 0.1f);
                    glm::vec4 asphalt_color = hexToRGBA("#404040", 0.1f);

                    auto classColor = [&](const std::string& c)->glm::vec4 {
                        if (c=="motorway"||c=="trunk"||c=="primary"||c=="secondary")
                            return concrete_color;
                        return asphalt_color;
                    };

                    // Visuals
                    const glm::vec4 color = classColor(cls);

                    // Z layering
                    float zOffset = 0.01f; // base
                    if (isTunnel)      zOffset = -0.10f;
                    else if (isBridge) zOffset =  0.30f;
                    else {
                        // Subtle hierarchy
                        float dz = 0.0f;
                        if (cls == "motorway") dz = 0.05f;
                        else if (cls == "trunk") dz = 0.04f;
                        else if (cls == "primary") dz = 0.03f;
                        else if (cls == "secondary") dz = 0.02f;
                        else if (cls == "tertiary") dz = 0.015f;
                        else if (cls == "service") dz = 0.005f;
                        zOffset += dz;
                    }

                    // Emit a RoadGeometry per line part
                    for (const auto& line : geoms) {
                        if (line.size() < 2) continue;
                        RoadGeometry r;
                        r.centerline.reserve(line.size());
                        r.width     = width_tile;
                        r.invExtent = invExtent;
                        r.color     = color;
                        r.zOffset   = zOffset;
                        r.isBridge  = isBridge;
                        r.isTunnel  = isTunnel;

                        for (const auto& p : line) {
                            const float x = static_cast<float>(p.x) * invExtent;
                            const float y = static_cast<float>(p.y) * invExtent;
                            r.centerline.emplace_back(x, y);
                        }
                        roads.push_back(std::move(r));
                    }
                } catch (const std::exception& fe) {
                    std::cout << "⚠️ Road feature error: " << fe.what() << "\n";
                    continue;
                }
            }
        }

        const size_t added = roads.size() - added_before;
        if (added == 0) {
            std::cout << "⚠️ No road features found in vector tile\n";
        } else {
            std::cout << "✅ Extracted " << added << " road features (kerb-to-kerb widths)\n";
        }
    } catch (const std::exception& e) {
        std::cout << "❌ Error parsing vector tile: " << e.what() << "\n";
    }
}


void SimpleMapSnapshotter::createExtrudedPolygon(
    const std::vector<glm::vec2>& footprint,
    float height_m, float base_m,
    const glm::vec4& roof_color, const glm::vec4& wall_color,
    const glm::vec3& offset,
    std::vector<MapVertex>& vertices, std::vector<uint32_t>& indices)
{
    if (footprint.size() < 3) return;

    const uint32_t base_index = static_cast<uint32_t>(vertices.size());

    // Map 0..1 tile coords → world XY in *meters* (Z handled in meters below)
    auto to_world_xy = [&](const glm::vec2& p) -> glm::vec3 {
        return glm::vec3(
            (p.x - 0.5f) * meters_per_tile_,   // X meters
            (0.5f - p.y) * meters_per_tile_,   // Y meters (keep +Y = north)
            0.0f
        );
    };

    // Z is now in **meters** (no scaling by meters_per_tile_)
    const float baseZ      = std::max(0.0f, base_m);
    const float topZ       = baseZ + std::max(0.0f, height_m);
    const float roofStartZ = baseZ + (topZ - baseZ) * 0.8f;   // tweak (e.g. 0.8f) if you like
    const float roofZBias  = 0.001f;

    const size_t n = footprint.size();

    // ----- bottom ring (base) -----
    for (const auto& p01 : footprint) {
        MapVertex v{};
        v.position   = offset + to_world_xy(p01) + glm::vec3(0.0f, 0.0f, baseZ);
        v.texCoord   = p01;
        v.normal     = glm::vec3(0.0f, 0.0f, -1.0f);
        v.color      = wall_color;
        v.elevation  = baseZ;
        vertices.push_back(v);
    }

    // ----- middle ring (where roof slope might start) -----
    for (const auto& p01 : footprint) {
        MapVertex v{};
        v.position   = offset + to_world_xy(p01) + glm::vec3(0.0f, 0.0f, roofStartZ);
        v.texCoord   = p01;
        v.normal     = glm::vec3(0.0f, 0.0f,  1.0f);
        v.color      = wall_color;
        v.elevation  = roofStartZ;
        vertices.push_back(v);
    }

    // ----- top ring (roof) -----
    for (const auto& p01 : footprint) {
        MapVertex v{};
        v.position   = offset + to_world_xy(p01) + glm::vec3(0.0f, 0.0f, topZ + roofZBias);
        v.texCoord   = p01;
        v.normal     = glm::vec3(0.0f, 0.0f,  1.0f);
        v.color      = roof_color;
        v.elevation  = topZ + roofZBias;
        vertices.push_back(v);
    }

    // ----- triangulate base & roof via earcut -----
    using N = double;
    std::vector<std::vector<std::array<N,2>>> poly(1);
    poly[0].reserve(n);
    for (const auto& p01 : footprint) poly[0].push_back({ N(p01.x), N(p01.y) });

    // base face (first ring 0..n-1)
    {
        auto tri = mapbox::earcut<uint32_t>(poly);
        for (uint32_t idx : tri) indices.push_back(base_index + idx);
    }

    // roof face (top ring 2n..3n-1)
    {
        auto tri = mapbox::earcut<uint32_t>(poly);
        const uint32_t topStart = base_index + static_cast<uint32_t>(2 * n);
        for (uint32_t idx : tri) indices.push_back(topStart + idx);
    }

    // ----- walls -----
    // lower walls (base → middle)
    for (size_t i = 0; i < n; ++i) {
        const uint32_t bi = base_index + uint32_t(i);
        const uint32_t bj = base_index + uint32_t((i + 1) % n);
        const uint32_t mi = base_index + uint32_t(n + i);
        const uint32_t mj = base_index + uint32_t(n + ((i + 1) % n));
        indices.push_back(bi); indices.push_back(mi); indices.push_back(bj);
        indices.push_back(bj); indices.push_back(mi); indices.push_back(mj);
    }
    // upper walls (middle → roof)
    for (size_t i = 0; i < n; ++i) {
        const uint32_t mi = base_index + uint32_t(n + i);
        const uint32_t mj = base_index + uint32_t(n + ((i + 1) % n));
        const uint32_t ti = base_index + uint32_t(2 * n + i);
        const uint32_t tj = base_index + uint32_t(2 * n + ((i + 1) % n));
        indices.push_back(mi); indices.push_back(ti); indices.push_back(mj);
        indices.push_back(mj); indices.push_back(ti); indices.push_back(tj);
    }
}


void SimpleMapSnapshotter::createRoadMesh(const std::vector<glm::vec2>& centerline,
                    float width, float zOffset,       // <-- NEW param
                    const glm::vec4& color, const glm::vec3& offset,
                    std::vector<MapVertex>& vertices,
                    std::vector<uint32_t>& indices) 
{
                   

    if (centerline.size() < 2) return;

    const uint32_t base_index = static_cast<uint32_t>(vertices.size());
    const float half_width_tile = 0.5f * width;
    const float v_denom = float(centerline.size() - 1);

    for (size_t i = 0; i < centerline.size(); ++i) {
        const glm::vec2 P  = centerline[i];
        const glm::vec2 Pa = (i > 0) ? centerline[i - 1] : centerline[i];
        const glm::vec2 Pb = (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];

        // robust direction
        glm::vec2 dir = (i == 0) ? (Pb - P) : (i + 1 == centerline.size() ? (P - Pa) : (Pb - Pa));
        const float len = glm::length(dir);
        dir = (len > 1e-6f) ? (dir / len) : glm::vec2(1.0f, 0.0f);
        const glm::vec2 perp(-dir.y, dir.x);

        glm::vec2 stitched = P;

        glm::vec2 left_pos  = P + perp * half_width_tile;
        glm::vec2 right_pos = P - perp * half_width_tile;

        const float v = float(i) / v_denom;

        MapVertex L{}, R{};
        // L.position = offset + glm::vec3((left_pos.x  - 0.5f) * meters_per_tile_,
        //                                 (0.5f - left_pos.y)  * meters_per_tile_, 0.0f);
        L.position = offset + glm::vec3((left_pos.x - 0.5f) * meters_per_tile_,
                                        (0.5f - left_pos.y) * meters_per_tile_, zOffset);

                                    
        L.texCoord  = {0.0f, v};
        L.normal    = {0.0f, 0.0f, 1.0f};
        L.color     = color;
        // L.elevation = 0.01f;
        L.elevation = zOffset;

        // R.position = offset + glm::vec3((right_pos.x - 0.5f) * meters_per_tile_,
        //                                 (0.5f - right_pos.y) * meters_per_tile_, 0.0f);
        R.position = offset + glm::vec3((right_pos.x - 0.5f) * meters_per_tile_,
                                        (0.5f - right_pos.y) * meters_per_tile_, zOffset);
        R.texCoord  = {1.0f, v};
        R.normal    = {0.0f, 0.0f, 1.0f};
        R.color     = color;
        // R.elevation = 0.01f;
        R.elevation = zOffset;

        vertices.push_back(L);
        vertices.push_back(R);
    }

    // triangles
    for (size_t i = 0; i + 1 < centerline.size(); ++i) {
        const uint32_t L0 = base_index + uint32_t(i * 2);
        const uint32_t R0 = L0 + 1;
        const uint32_t L1 = L0 + 2;
        const uint32_t R1 = L0 + 3;
        indices.push_back(L0); indices.push_back(L1); indices.push_back(R0);
        indices.push_back(R0); indices.push_back(L1); indices.push_back(R1);
    }
}

void SimpleMapSnapshotter::createGreenAreaMesh(
    const std::vector<glm::vec2>& footprint,
    const glm::vec4& color, float height,
    const glm::vec3& offset,
    std::vector<MapVertex>& vertices,
    std::vector<uint32_t>& indices)
{
    if (footprint.size() < 3) return;

    const uint32_t base_index = static_cast<uint32_t>(vertices.size());

    // Keep the same world mapping (Y flipped so +Y = north)
    auto to_world_xy = [&](const glm::vec2& p) -> glm::vec3 {
        return glm::vec3((p.x - 0.5f) * meters_per_tile_,
                        (0.5f - p.y) * meters_per_tile_,
                        0.0f);
    };

    // Vertices (flat polygon at `height`)
    for (const auto& p01 : footprint) {
        MapVertex v{};
        v.position  = offset + to_world_xy(p01) + glm::vec3(0.0f, 0.0f, height);
        v.texCoord  = p01;
        v.normal    = glm::vec3(0.0f, 0.0f, 1.0f);
        v.color     = color;
        v.elevation = height;
        vertices.push_back(v);
    }

    // Robust triangulation via earcut (outer ring only)
    using N = double;
    std::vector<std::vector<std::array<N,2>>> poly(1);
    poly[0].reserve(footprint.size());
    for (const auto& p01 : footprint) {
        poly[0].push_back({ N(p01.x), N(p01.y) });
    }

    auto tri = mapbox::earcut<uint32_t>(poly);
    for (uint32_t idx : tri) {
        indices.push_back(base_index + idx);
    }
}

void SimpleMapSnapshotter::createTerrainQuad(int x, int y, int resolution, const std::vector<float>& heightData,
                                            const glm::vec3& offset, std::vector<MapVertex>& vertices, 
                                            std::vector<uint32_t>& indices) {
    uint32_t base_index = vertices.size();
    
    float step = 1.0f / (resolution - 1);
    
    // Create 4 vertices for the quad
    for (int dy = 0; dy <= 1; ++dy) {
        for (int dx = 0; dx <= 1; ++dx) {
            MapVertex vertex;
            
            int grid_x = x + dx;
            int grid_y = y + dy;
            int height_index = grid_y * resolution + grid_x;
            
            float height = (height_index < static_cast<int>(heightData.size())) ? heightData[height_index] : 0.0f;
            
            // Updated coordinate system: X=forward, Y=up, Z=left
            vertex.position = offset + glm::vec3(
                grid_x * step,
                height,
                grid_y * step
            );
            vertex.texCoord = glm::vec2(grid_x * step, grid_y * step);
            vertex.normal = glm::vec3(0.0f, 1.0f, 0.0f); // Will be calculated properly later
            vertex.color = glm::vec4(0.5f, 0.7f, 0.3f, 1.0f); // Green terrain
            vertex.elevation = height;
            vertices.push_back(vertex);
        }
    }
    
    // Create 2 triangles for the quad
    // Triangle 1: (0,0) -> (1,0) -> (0,1)
    indices.push_back(base_index);
    indices.push_back(base_index + 1);
    indices.push_back(base_index + 2);
    
    // Triangle 2: (1,0) -> (1,1) -> (0,1)
    indices.push_back(base_index + 1);
    indices.push_back(base_index + 3);
    indices.push_back(base_index + 2);
}

// =============================================================
// step5: Integrated tile geometry management implementation
// =============================================================

void SimpleMapSnapshotter::updateTileGeometry() {
    std::cout << "🔄 Updating tile geometry for current view..." << std::endl;
    
    // Get visible tiles for current view
    auto visible_tiles = getVisibleTilesForCurrentView();
    
    std::cout << "   Found " << visible_tiles.size() << " visible tiles" << std::endl;
    
    // Store tile bounds for positioning calculations (to avoid recursive calls)
    if (!visible_tiles.empty()) {
        uint32_t min_x = visible_tiles[0].x, max_x = visible_tiles[0].x;
        uint32_t min_y = visible_tiles[0].y, max_y = visible_tiles[0].y;
        
        for (const auto& tile : visible_tiles) {
            min_x = std::min(min_x, tile.x);
            max_x = std::max(max_x, tile.x);
            min_y = std::min(min_y, tile.y);
            max_y = std::max(max_y, tile.y);
        }
        
        // Store bounds in member variables
        current_tile_bounds_min_x_ = min_x;
        current_tile_bounds_max_x_ = max_x;
        current_tile_bounds_min_y_ = min_y;
        current_tile_bounds_max_y_ = max_y;
        tile_bounds_valid_ = true;
        
        std::cout << "📍 Tile bounds: X(" << min_x << "-" << max_x 
                  << ") Y(" << min_y << "-" << max_y << ")" << std::endl;
    }
    
    // Load geometry for each visible tile
    for (const auto& tile : visible_tiles) {
        loadTileGeometry(tile);
    }
    
    // Process any pending geometry uploads
    processPendingTileGeometry();
}

static inline int32_t wrapX(int32_t x, uint8_t z) {
    const int32_t n = 1 << z;
    x %= n; if (x < 0) x += n; return x;
}
static inline int32_t clampY(int32_t y, uint8_t z) {
    const int32_t n = 1 << z;
    return std::max(0, std::min(n - 1, y));
}

std::vector<SimpleMapSnapshotter::TileID>
SimpleMapSnapshotter::getVisibleTilesForCurrentView() {
    std::vector<TileID> tiles;
    if (!camera_.center || !camera_.zoom) return tiles;

    const double lat = camera_.center->latitude();
    const double lng = camera_.center->longitude();
    const uint8_t z  = static_cast<uint8_t>(std::max(8.0, std::min(*camera_.zoom, 15.0)));

    const int32_t n  = 1 << z;
    const int32_t cx = int32_t((lng + 180.0) / 360.0 * n);
    const int32_t cy =
        int32_t((1.0 - std::asinh(std::tan(lat * M_PI/180.0))/M_PI) * 0.5 * n);

    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            TileID t{ z,
                      static_cast<uint32_t>(wrapX(cx + dx, z)),
                      static_cast<uint32_t>(clampY(cy + dy, z)) };
            tiles.push_back(t);
        }
    }
    return tiles;
}


void SimpleMapSnapshotter::loadTileGeometry(const TileID& tile) {
    std::string tile_key = std::to_string(tile.z) + "_" + std::to_string(tile.x) + "_" + std::to_string(tile.y);
    
    // Check if geometry already exists
    if (tile_geometry_.find(tile_key) != tile_geometry_.end() && 
        tile_geometry_[tile_key].uploaded) {
        std::cout << "   📦 Tile geometry already loaded: " << tile_key << std::endl;
        return;
    }
    
    std::cout << "🔺 Loading geometry for tile: " << tile_key << std::endl;
    
    // Generate the 3D geometry for this tile
    auto geometry_buffer = generateTileGeometry(tile);
    
    if (!geometry_buffer.vertices.empty()) {
        // Store in our tile geometry map
        tile_geometry_[tile_key] = std::move(geometry_buffer);
        
        std::cout << "✅ Generated geometry for tile " << tile_key 
                  << ": " << tile_geometry_[tile_key].vertices.size() << " vertices, "
                  << tile_geometry_[tile_key].indices.size() << " indices" << std::endl;
    } else {
        std::cout << "⚠️ No geometry generated for tile " << tile_key << std::endl;
    }
}

void SimpleMapSnapshotter::processPendingTileGeometry() {
    std::cout << "📤 Processing pending tile geometry uploads..." << std::endl;
    
    int uploaded_count = 0;
    
    // Upload any geometry that hasn't been uploaded yet
    for (auto& [tile_key, geometry] : tile_geometry_) {
        if (!geometry.uploaded && !geometry.vertices.empty()) {
            std::cout << "   Uploading geometry for tile: " << tile_key << std::endl;
            uploadGeometryToGPU(geometry);
            uploaded_count++;
        }
    }
    
    std::cout << "✅ Uploaded " << uploaded_count << " tile geometries to GPU" << std::endl;
}

// =============================================================
// step6: Rendering implementation methods
// =============================================================

void SimpleMapSnapshotter::initializeShaders() {
    if (shaders_initialized_) return;
    
    std::cout << "🔧 Initializing map rendering shaders..." << std::endl;
    
    // Vertex shader source - simple textured quad
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec2 aTexCoord;
        
        uniform mat4 u_projection;
        uniform mat4 u_view;
        
        out vec2 TexCoord;
        
        void main() {
            gl_Position = u_projection * u_view * vec4(aPos, 1.0);
            TexCoord = aTexCoord;
        }
    )";
    
    // Fragment shader source - simple texture sampling
    const char* fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        
        in vec2 TexCoord;
        uniform sampler2D u_texture;
        uniform float u_alpha;
        
        void main() {
            vec4 texColor = texture(u_texture, TexCoord);
            
            // Handle single-channel textures (GL_RED format)
            // If it's a grayscale image, replicate the red channel to RGB
            if (texColor.g == 0.0 && texColor.b == 0.0 && texColor.a == 0.0) {
                texColor = vec4(texColor.r, texColor.r, texColor.r, 1.0);
            }
            
            FragColor = vec4(texColor.rgb, texColor.a * u_alpha);
        }
    )";
    
    // Compile vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);
    
    // Check vertex shader compilation
    GLint success;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "❌ Vertex shader compilation failed: " << infoLog << std::endl;
        return;
    }
    
    // Compile fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    
    // Check fragment shader compilation
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "❌ Fragment shader compilation failed: " << infoLog << std::endl;
        return;
    }
    
    // Create shader program
    map_shader_program_ = glCreateProgram();
    glAttachShader(map_shader_program_, vertexShader);
    glAttachShader(map_shader_program_, fragmentShader);
    glLinkProgram(map_shader_program_);
    
    // Check program linking
    glGetProgramiv(map_shader_program_, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(map_shader_program_, 512, NULL, infoLog);
        std::cerr << "❌ Shader program linking failed: " << infoLog << std::endl;
        return;
    }
    
    // Clean up shaders
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    // Get uniform locations
    u_projection_ = glGetUniformLocation(map_shader_program_, "u_projection");
    u_view_ = glGetUniformLocation(map_shader_program_, "u_view");
    u_texture_ = glGetUniformLocation(map_shader_program_, "u_texture");
    u_alpha_ = glGetUniformLocation(map_shader_program_, "u_alpha");
    
    std::cout << "✅ Map shaders initialized successfully" << std::endl;
    shaders_initialized_ = true;
}

void SimpleMapSnapshotter::initializeGeometry() {
    std::cout << "🔧 Initializing map geometry..." << std::endl;
    
    // Create a horizontal quad using standard OpenGL coordinates: Z=up, X=forward, Y=left
    // Calculate the world bounds to match the visible tile area perfectly
    float map_width  = MAP_TOTAL_WS;
    float map_height = MAP_TOTAL_WS;
    
    float vertices[] = {
        // positions (OpenGL coords: X=forward, Y=left, Z=up)     // texture coords
        -map_width/2, -map_height/2, 0.0f,   0.0f, 1.0f,   // bottom left (in world space)
         map_width/2, -map_height/2, 0.0f,   1.0f, 1.0f,   // bottom right  
         map_width/2,  map_height/2, 0.0f,   1.0f, 0.0f,   // top right
        -map_width/2,  map_height/2, 0.0f,   0.0f, 0.0f    // top left
    };
    
    unsigned int indices[] = {
        0, 1, 2,   // first triangle
        2, 3, 0    // second triangle
    };
    
    // Generate buffers
    glGenVertexArrays(1, &map_vao_);
    glGenBuffers(1, &map_vbo_);
    glGenBuffers(1, &map_ebo_);
    
    // Bind and setup VAO
    glBindVertexArray(map_vao_);
    
    // Bind and fill VBO
    glBindBuffer(GL_ARRAY_BUFFER, map_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    
    // Bind and fill EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, map_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Texture coordinate attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    
    // Unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    
    std::cout << "✅ Map geometry initialized successfully" << std::endl;
}

void SimpleMapSnapshotter::initializeGeometryShaders() {
    if (geometry_shaders_initialized_) return;
    
    std::cout << "🔧 Initializing 3D geometry shaders..." << std::endl;
    
    // 3D Vertex shader for MapVertex structure
    const char* geometryVertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPosition;     // MapVertex.position
        layout (location = 1) in vec2 aTexCoord;     // MapVertex.texCoord  
        layout (location = 2) in vec3 aNormal;       // MapVertex.normal
        layout (location = 3) in vec4 aColor;        // MapVertex.color
        layout (location = 4) in float aElevation;   // MapVertex.elevation
        
        uniform mat4 u_projection;
        uniform mat4 u_view;
        uniform mat4 u_model;
        uniform vec3 u_light_dir;
        
        out vec3 FragPos;
        out vec3 Normal;
        out vec4 VertexColor;
        out vec2 TexCoord;
        out float Elevation;
        out float LightIntensity;
        
        void main() {
            vec4 worldPos = u_model * vec4(aPosition, 1.0);
            FragPos = worldPos.xyz;
            
            // Transform normal to world space
            Normal = mat3(transpose(inverse(u_model))) * aNormal;
            
            // Pass through vertex attributes
            VertexColor = aColor;
            TexCoord = aTexCoord;
            Elevation = aElevation;
            
            // Simple directional lighting
            vec3 norm = normalize(Normal);
            vec3 lightDir = normalize(-u_light_dir);
            LightIntensity = max(dot(norm, lightDir), 0.3); // 0.3 minimum ambient
            
            gl_Position = u_projection * u_view * worldPos;
        }
    )";
    
    // 3D Fragment shader with lighting and colors
    const char* geometryFragmentShaderSource = R"(
        #version 330 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec4 VertexColor;
        in vec2 TexCoord;
        in float Elevation;
        in float LightIntensity;
        
        uniform vec3 u_light_color;
        uniform vec3 u_ambient_color;
        
        out vec4 FragColor;
        
        void main() {
            // Base color from vertex
            vec3 baseColor = VertexColor.rgb;
            
            // Apply lighting
            vec3 ambient = u_ambient_color * baseColor;
            vec3 diffuse = u_light_color * baseColor * LightIntensity;
            
            // Height-based color variation for buildings
            if (Elevation > 5.0) {
                // Lighter color for higher buildings
                float heightFactor = min(Elevation / 100.0, 1.0);
                baseColor = mix(baseColor, vec3(0.9, 0.9, 1.0), heightFactor * 0.3);
            }
            
            vec3 finalColor = ambient + diffuse;
            FragColor = vec4(finalColor, VertexColor.a);
        }
    )";
    
    // Compile geometry vertex shader
    GLuint geometryVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(geometryVertexShader, 1, &geometryVertexShaderSource, NULL);
    glCompileShader(geometryVertexShader);
    
    GLint success;
    glGetShaderiv(geometryVertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(geometryVertexShader, 512, NULL, infoLog);
        std::cerr << "❌ Geometry vertex shader compilation failed: " << infoLog << std::endl;
        return;
    }
    
    // Compile geometry fragment shader
    GLuint geometryFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(geometryFragmentShader, 1, &geometryFragmentShaderSource, NULL);
    glCompileShader(geometryFragmentShader);
    
    glGetShaderiv(geometryFragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(geometryFragmentShader, 512, NULL, infoLog);
        std::cerr << "❌ Geometry fragment shader compilation failed: " << infoLog << std::endl;
        return;
    }
    
    // Create geometry shader program
    geometry_shader_program_ = glCreateProgram();
    glAttachShader(geometry_shader_program_, geometryVertexShader);
    glAttachShader(geometry_shader_program_, geometryFragmentShader);
    glLinkProgram(geometry_shader_program_);
    
    glGetProgramiv(geometry_shader_program_, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(geometry_shader_program_, 512, NULL, infoLog);
        std::cerr << "❌ Geometry shader program linking failed: " << infoLog << std::endl;
        return;
    }
    
    // Clean up shaders
    glDeleteShader(geometryVertexShader);
    glDeleteShader(geometryFragmentShader);
    
    // Get uniform locations for 3D rendering
    u_geo_projection_ = glGetUniformLocation(geometry_shader_program_, "u_projection");
    u_geo_view_ = glGetUniformLocation(geometry_shader_program_, "u_view");
    u_geo_model_ = glGetUniformLocation(geometry_shader_program_, "u_model");
    u_light_dir_ = glGetUniformLocation(geometry_shader_program_, "u_light_dir");
    u_light_color_ = glGetUniformLocation(geometry_shader_program_, "u_light_color");
    u_ambient_color_ = glGetUniformLocation(geometry_shader_program_, "u_ambient_color");
    
    std::cout << "✅ 3D geometry shaders initialized successfully" << std::endl;
    geometry_shaders_initialized_ = true;
}

void SimpleMapSnapshotter::renderTileGeometry(const glm::mat4& projection, const glm::mat4& view) {
    if (!geometry_shaders_initialized_) {
        initializeGeometryShaders();
    }
    if (geometry_shader_program_ == 0 || tile_geometry_.empty()) {
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glUseProgram(geometry_shader_program_);

    // Shared uniforms
    glUniformMatrix4fv(u_geo_projection_, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(u_geo_view_,       1, GL_FALSE, glm::value_ptr(view));

    const glm::vec3 light_dir(0.3f, -1.0f, 0.2f);
    const glm::vec3 light_color(1.0f, 1.0f, 0.9f);
    const glm::vec3 ambient_color(0.3f, 0.3f, 0.4f);
    glUniform3fv(u_light_dir_,    1, glm::value_ptr(light_dir));
    glUniform3fv(u_light_color_,  1, glm::value_ptr(light_color));
    glUniform3fv(u_ambient_color_,1, glm::value_ptr(ambient_color));

    int rendered_count = 0;

    for (const auto& [tile_key, geometry] : tile_geometry_) {
        if (!geometry.uploaded || geometry.vao == 0) continue;

        // Parse "z_x_y"
        const size_t a = tile_key.find('_');
        const size_t b = tile_key.find('_', a + 1);
        glm::mat4 model(1.0f);

        if (a != std::string::npos && b != std::string::npos) {
            const uint8_t  z = static_cast<uint8_t>(std::stoi(tile_key.substr(0, a)));
            const uint32_t x = static_cast<uint32_t>(std::stoul(tile_key.substr(a + 1, b - a - 1)));
            const uint32_t y = static_cast<uint32_t>(std::stoul(tile_key.substr(b + 1)));

            TileID tile{z, x, y};

            // EXACT integer multiples of TILE_SIZE_WS
            const glm::vec3 tile_world = getTileWorldPosition(tile);
            model = glm::translate(glm::mat4(1.0f), tile_world);

            // if (rendered_count < 3) {
            //     std::cout << "🔍 Tile " << tile_key << " model translate: ("
            //               << tile_world.x << ", " << tile_world.y << ", " << tile_world.z << ")\n";
            // }
        }

        glUniformMatrix4fv(u_geo_model_, 1, GL_FALSE, glm::value_ptr(model));

        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);   // (factor, units) — tweak if needed

        glBindVertexArray(geometry.vao);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(geometry.indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        glDisable(GL_POLYGON_OFFSET_FILL);

        rendered_count++;
    }

    glUseProgram(0);
    glDisable(GL_DEPTH_TEST);

    // std::cout << "✅ Rendered " << rendered_count << " tile geometries\n";
}



void SimpleMapSnapshotter::uploadImageToTexture(const std::vector<uint8_t>& imageData, int width, int height) {
    std::cout << "🖼️ Uploading image to GPU texture..." << std::endl;
    std::cout << "   Image size: " << width << "x" << height << std::endl;
    std::cout << "   Data size: " << imageData.size() << " bytes" << std::endl;
    
    // Decode image using stb_image
    int channels;
    unsigned char* pixels = stbi_load_from_memory(
        imageData.data(), 
        static_cast<int>(imageData.size()), 
        &texture_width_, 
        &texture_height_, 
        &channels, 
        0
    );
    
    if (!pixels) {
        std::cerr << "❌ Failed to decode image data" << std::endl;
        return;
    }
    
    std::cout << "   Decoded: " << texture_width_ << "x" << texture_height_ << " channels: " << channels << std::endl;
    
    // Create OpenGL texture
    if (map_texture_id_ != 0) {
        glDeleteTextures(1, &map_texture_id_);
    }
    
    glGenTextures(1, &map_texture_id_);
    glBindTexture(GL_TEXTURE_2D, map_texture_id_);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Upload pixel data
    GLenum format, internalFormat;
    if (channels == 4) {
        format = GL_RGBA;
        internalFormat = GL_RGBA;
    } else if (channels == 3) {
        format = GL_RGB;
        internalFormat = GL_RGB;
    } else {
        // For single channel (grayscale), use GL_RED in modern OpenGL
        format = GL_RED;
        internalFormat = GL_RED;
    }
    
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, texture_width_, texture_height_, 0, format, GL_UNSIGNED_BYTE, pixels);
    
    // Generate mipmaps
    glGenerateMipmap(GL_TEXTURE_2D);
    
    // Cleanup
    stbi_image_free(pixels);
    glBindTexture(GL_TEXTURE_2D, 0);
    
    std::cout << "✅ Texture uploaded successfully (ID: " << map_texture_id_ << ")" << std::endl;
}

void SimpleMapSnapshotter::updateTexture() {
    std::lock_guard<std::mutex> lock(texture_mutex_);
    if (has_pending_texture_) {
        std::cout << "🔄 Uploading pending texture data on main thread..." << std::endl;
        uploadImageToTexture(pending_image_data_, pending_width_, pending_height_);
        map_texture_ready_ = true;
        has_pending_texture_ = false;
        pending_image_data_.clear(); // Free memory
        std::cout << "✅ Texture upload completed on main thread" << std::endl;
        
        // When texture is ready, also update tile geometry
        std::cout << "🏗️ Triggering tile geometry update..." << std::endl;
        updateTileGeometry();
    }
}

void SimpleMapSnapshotter::renderMap(const glm::mat4& projection, const glm::mat4& view) {
    if (render_3d_mode_) {
        // Render 3D tile geometry
        if (!tile_geometry_.empty()) {
            // std::cout << "🏗️ Rendering 3D tile geometry..." << std::endl;
            renderTileGeometry(projection, view);
        } else {
            std::cout << "⚠️ No 3D geometry loaded yet, falling back to 2D" << std::endl;
            render_3d_mode_ = false; // Temporarily switch to 2D
        }
    }
    
    // Always render 2D texture as background (for now)
    if (hasValidTexture()) {
        // std::cout << "🗺️ Rendering 2D map texture as background..." << std::endl;
        
        
        if (map_shader_program_ == 0 || map_vao_ == 0) {
            return; // Failed to initialize
        }
        
        // Enable depth testing and render as background
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        
        // Enable blending for transparency
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        // Use 2D shader program
        glUseProgram(map_shader_program_);
        
        // Set uniforms (now using world space, not screen space)
        glUniformMatrix4fv(u_projection_, 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(u_view_, 1, GL_FALSE, glm::value_ptr(view));
        glUniform1f(u_alpha_, 0.8f); // Semi-transparent so 3D geometry shows through
        
        // Bind texture
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, map_texture_id_);
        glUniform1i(u_texture_, 0);
        
        // Draw quad
        glBindVertexArray(map_vao_);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
        
        // Cleanup
        glUseProgram(0);
        glDisable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
    }

    // renderCircleOverlay(projection, view);

}

// =============================================================
// ✅ NEW: Real building data from Mapbox vector tiles
// =============================================================
void SimpleMapSnapshotter::fetchVectorTileBuildings(const TileID& tile, std::vector<BuildingGeometry>& buildings) {
    // Fetch real building data from Mapbox vector tiles
    // This uses the same infrastructure as street fetching but for building layers
    
    buildings.clear();
    
    // Build vector tile URL for Mapbox Streets tileset (includes building data)
    std::string url = "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/" + 
                     std::to_string((int)tile.z) + "/" + std::to_string(tile.x) + "/" + std::to_string(tile.y) + 
                     ".mvt?access_token=" + accessToken_;
    
    std::cout << "🏢 Fetching building data from: " << url << std::endl;
    
    // Use the same safe download method as streets
    std::string tempFile = "/tmp/buildings_" + std::to_string(tile.z) + "_" + 
                          std::to_string(tile.x) + "_" + std::to_string(tile.y) + ".mvt";
    
    std::string command = "curl -s -L -o \"" + tempFile + "\" \"" + url + "\"";
    int result = std::system(command.c_str());
    
    if (result != 0) {
        std::cout << "❌ Failed to download building tile data" << std::endl;
        return;
    }
    
    // Read the downloaded file
    std::ifstream file(tempFile, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "❌ Failed to open building tile file" << std::endl;
        return;
    }
    
    std::vector<uint8_t> mvtData((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    
    // Clean up temp file
    std::remove(tempFile.c_str());
    
    if (mvtData.empty()) {
        std::cout << "❌ Empty building tile data" << std::endl;
        return;
    }
    
    // Parse the building data from the vector tile
    parseVectorTileBuildings(mvtData, tile, buildings);
    
    std::cout << "✅ Fetched " << buildings.size() << " real buildings from Mapbox" << std::endl;
}

void SimpleMapSnapshotter::fetchVectorTileGreenAreas(const TileID& tile, std::vector<GreenAreaGeometry>& greenAreas) {
    // Fetch real green area data from Mapbox vector tiles
    // This uses the same infrastructure as building and street fetching but for green area layers
    
    greenAreas.clear();
    
    // Build vector tile URL for Mapbox Streets tileset (includes green area data)
    std::string url = "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/" + 
                     std::to_string((int)tile.z) + "/" + std::to_string(tile.x) + "/" + std::to_string(tile.y) + 
                     ".mvt?access_token=" + accessToken_;
    
    std::cout << "🌳 Fetching green area data from: " << url << std::endl;
    
    // Use the same safe download method as buildings and streets
    std::string tempFile = "/tmp/green_areas_" + std::to_string(tile.z) + "_" + 
                          std::to_string(tile.x) + "_" + std::to_string(tile.y) + ".mvt";
    
    std::string command = "curl -s -L -o \"" + tempFile + "\" \"" + url + "\"";
    int result = std::system(command.c_str());
    
    if (result != 0) {
        std::cout << "❌ Failed to download green area tile data" << std::endl;
        return;
    }
    
    // Read the downloaded file
    std::ifstream file(tempFile, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "❌ Failed to open green area tile file" << std::endl;
        return;
    }
    
    std::vector<uint8_t> mvtData((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    
    // Clean up temp file
    std::remove(tempFile.c_str());
    
    if (mvtData.empty()) {
        std::cout << "❌ Empty green area tile data" << std::endl;
        return;
    }
    
    // Parse the green area data from the vector tile
    parseVectorTileGreenAreas(mvtData, tile, greenAreas);
    
    std::cout << "✅ Fetched " << greenAreas.size() << " real green areas from Mapbox" << std::endl;
}

void SimpleMapSnapshotter::parseVectorTileBuildings(
    const std::vector<uint8_t>& mvtData,
    const TileID& tile,
    std::vector<BuildingGeometry>& buildings)
{
    buildings.clear();
    if (mvtData.empty()) { std::cout << "❌ Empty building vector tile data\n"; return; }

    auto snap01f = [](float v, float eps = 1e-5f) {
        if (std::abs(v) < eps) return 0.0f;
        if (std::abs(1.0f - v) < eps) return 1.0f;
        return v;
    };

    try {
        // ---- decompress if gzip ----
        std::vector<uint8_t> buf;
        if (mvtData.size() >= 2 && mvtData[0] == 0x1f && mvtData[1] == 0x8b) {
            z_stream s{}; s.avail_in = (uInt)mvtData.size();
            s.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(mvtData.data()));
            if (inflateInit2(&s, 15 + 32) != Z_OK) throw std::runtime_error("gzip init failed");
            const size_t CHUNK = 16384;
            int ret = Z_OK;
            do {
                size_t base = buf.size();
                buf.resize(base + CHUNK);
                s.avail_out = CHUNK;
                s.next_out  = reinterpret_cast<Bytef*>(buf.data() + base);
                ret = inflate(&s, Z_NO_FLUSH);
                if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
                    inflateEnd(&s);
                    throw std::runtime_error("gzip inflate failed");
                }
                buf.resize(base + (CHUNK - s.avail_out));
            } while (ret != Z_STREAM_END);
            inflateEnd(&s);
        } else {
            buf = mvtData;
        }

        // ---- parse MVT ----
        std::string blob(reinterpret_cast<const char*>(buf.data()), buf.size());
        mapbox::vector_tile::buffer vt(blob);
        auto layers = vt.getLayers();

        bool foundAny = false;

        for (const auto& lp : layers) {
            if (lp.first != "building") continue;

            try {
                mapbox::vector_tile::layer layer(lp.second);
                const float extent    = static_cast<float>(layer.getExtent());
                const float invExtent = extent > 0.0f ? 1.0f / extent : 1.0f;

                for (std::size_t i = 0; i < layer.featureCount(); ++i) {
                    try {
                        auto fdata = layer.getFeature(i);
                        mapbox::vector_tile::feature feat(fdata, layer);
                        if (feat.getType() != mapbox::vector_tile::GeomType::POLYGON) continue;

                        BuildingGeometry b;

                        // ---- read heights (meters) ----
                        auto props = feat.getProperties();
                        auto getFloatProp = [&](const char* key, float& out)->bool {
                            auto it = props.find(key);
                            if (it == props.end()) return false;
                            const auto& v = it->second;
                            if (v.is<double>())      { out = float(v.get<double>());   return true; }
                            if (v.is<int64_t>())     { out = float(v.get<int64_t>());  return true; }
                            if (v.is<uint64_t>())    { out = float(v.get<uint64_t>()); return true; }
                            if (v.is<std::string>()) { try { out = std::stof(v.get<std::string>()); return true; } catch(...) {} }
                            return false;
                        };

                        float h_m = 12.0f, base_m = 0.0f, tmp;
                        if (getFloatProp("height", tmp) || getFloatProp("render_height", tmp)) h_m = tmp;
                        if (getFloatProp("levels", tmp)) h_m = std::max(h_m, tmp * 3.0f);   // prefer explicit height, but derive if only levels present
                        if (getFloatProp("min_height", tmp)) base_m = std::max(tmp, 0.0f);

                        b.base   = std::max(0.0f, base_m);          // meters
                        b.height = std::max(0.0f, h_m - base_m);    // meters (top - base)

                        // ---- footprint (normalize to 0..1 tile space) ----
                        auto geoms = feat.getGeometries<mapbox::vector_tile::points_arrays_type>(1.0f);
                        if (!geoms.empty() && !geoms[0].empty()) {
                            const auto& outer = geoms[0];
                            b.footprint.reserve(outer.size());
                            for (const auto& p : outer) {
                                float x = snap01f(static_cast<float>(p.x) * invExtent);
                                float y = snap01f(static_cast<float>(p.y) * invExtent);
                                b.footprint.emplace_back(x, y);
                            }
                        }

                        // basic validation
                        if (b.footprint.size() >= 3) {
                            float area = calculatePolygonArea(b.footprint);
                            if (area > 0.0001f) {
                                buildings.push_back(std::move(b));
                                foundAny = true;
                            }
                        }
                    } catch (const std::exception& fe) {
                        std::cout << "⚠️ Building feature error: " << fe.what() << "\n";
                    }
                }

                if (foundAny) return;

            } catch (const std::exception& le) {
                std::cout << "⚠️ Error processing building layer: " << le.what() << "\n";
            }
        }

        if (!foundAny) std::cout << "⚠️ No building features found\n";

    } catch (const std::exception& e) {
        std::cout << "❌ Error parsing building tile data: " << e.what() << "\n";
        buildings.clear();
    }
}


void SimpleMapSnapshotter::parseVectorTileGreenAreas(const std::vector<uint8_t>& mvtData,
                                                     const TileID& tile,
                                                     std::vector<GreenAreaGeometry>& greenAreas) {
    greenAreas.clear();

    if (mvtData.empty()) {
        std::cout << "❌ Empty green area vector tile data" << std::endl;
        return;
    }

    auto snap01f = [](float v, float eps = 1e-5f) {
        if (std::abs(v) < eps) return 0.0f;
        if (std::abs(1.0f - v) < eps) return 1.0f;
        return v;
    };

    std::cout << "🔍 Parsing green area data from vector tile (" << mvtData.size() << " bytes)" << std::endl;

    try {
        // --- decompress (if gzip) ---
        std::vector<uint8_t> decompressedData;
        if (mvtData.size() >= 2 && mvtData[0] == 0x1f && mvtData[1] == 0x8b) {
            std::cout << "🗜️ Detected gzip compression, decompressing..." << std::endl;

            z_stream stream{};
            stream.avail_in = static_cast<uInt>(mvtData.size());
            stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(mvtData.data()));

            if (inflateInit2(&stream, 15 + 32) != Z_OK) {
                throw std::runtime_error("Failed to initialize gzip decompression");
            }

            const size_t CHUNK = 16384;
            std::vector<uint8_t> out; out.reserve(mvtData.size() * 2);

            int ret = Z_OK;
            do {
                size_t base = out.size();
                out.resize(base + CHUNK);
                stream.avail_out = CHUNK;
                stream.next_out = reinterpret_cast<Bytef*>(out.data() + base);
                ret = inflate(&stream, Z_NO_FLUSH);
                if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
                    inflateEnd(&stream);
                    throw std::runtime_error("Gzip decompression failed");
                }
                out.resize(base + (CHUNK - stream.avail_out));
            } while (ret != Z_STREAM_END);

            inflateEnd(&stream);
            decompressedData.swap(out);
            std::cout << "✅ Decompressed " << mvtData.size()
                      << " bytes to " << decompressedData.size() << " bytes" << std::endl;
        } else {
            std::cout << "📦 Data is not gzip compressed, using as-is" << std::endl;
            decompressedData = mvtData;
        }

        // --- parse MVT ---
        std::string blob(reinterpret_cast<const char*>(decompressedData.data()), decompressedData.size());
        mapbox::vector_tile::buffer vt(blob);
        auto layers = vt.getLayers();
        std::cout << "📋 Found " << layers.size() << " layers in vector tile" << std::endl;

        bool foundAny = false;

        // Look for green area layers (parks, forests, grass, etc.)
        std::vector<std::string> greenAreaLayerNames = {
            "landuse", "park", "forest", "grass", "green", "leisure", "natural"
        };

        for (const auto& layerPair : layers) {
            const std::string& layerName = layerPair.first;
            
            // Check if this layer contains green areas
            bool isGreenAreaLayer = false;
            for (const auto& greenName : greenAreaLayerNames) {
                if (layerName.find(greenName) != std::string::npos) {
                    isGreenAreaLayer = true;
                    break;
                }
            }
            
            if (!isGreenAreaLayer) continue;

            try {
                mapbox::vector_tile::layer layer(layerPair.second);
                const float extent    = static_cast<float>(layer.getExtent());
                const float invExtent = extent > 0.0f ? 1.0f / extent : 1.0f;

                std::cout << "🌳 Green area layer \"" << layerName << "\": " << layer.featureCount() 
                          << " features, extent=" << extent << std::endl;

                for (std::size_t i = 0; i < layer.featureCount(); ++i) {
                    try {
                        auto fdata = layer.getFeature(i);
                        mapbox::vector_tile::feature feature(fdata, layer);
                        
                        // Only process polygon features for green areas
                        if (feature.getType() != mapbox::vector_tile::GeomType::POLYGON) continue;

                        GreenAreaGeometry greenArea;

                        // --- read properties ---
                        auto props = feature.getProperties();
            
                        
                        // Check for specific green area types
                        auto getStringProp = [&](const char* key) -> std::string {
                            auto it = props.find(key);
                            if (it == props.end()) return "";
                            const auto& v = it->second;
                            if (v.is<std::string>()) return v.get<std::string>();
                            return "";
                        };
                        
                        std::string areaClass = getStringProp("class");
                        std::string areaType = getStringProp("type");

                        glm::vec4 color = hexToRGBA("#e0e0e0", 1.0f);
                        // Filter for parks and green areas only
                        bool isPark = false;
                        
                        if (areaClass == "park") {
                            isPark = true;
                            // Different colors for different park types
                            if (areaType == "park") {
                                color = hexToRGBA("#e0e0e0", 1.0f); // Standard park green
                            } else if (areaType == "recreation_ground") {
                                color = hexToRGBA("#e0e0e0", 1.0f); // Recreation ground
                            } else if (areaType == "playground") {
                                color = hexToRGBA("#e0e0e0", 1.0f); // Playground
                            } else {
                                color = hexToRGBA("#e0e0e0", 1.0f); // Default park green
                            }
                        } else if (areaClass == "scrub") {
                            isPark = true;
                            color = hexToRGBA("#e0e0e0", 1.0f); // Scrub/natural area
                        } else if (areaClass == "pitch") {
                            isPark = true;
                            color = hexToRGBA("#e0e0e0", 1.0f); // Sports field green
                        }
                        
                        // Skip non-park features
                        if (!isPark) {
                            continue;
                        }
                        
                        greenArea.type = areaType;
                        greenArea.color = color;

                        // --- geometry: outer ring only (holes ignored here) ---
                        auto geom = feature.getGeometries<mapbox::vector_tile::points_arrays_type>(1.0f);
                        if (!geom.empty() && !geom[0].empty()) {
                            const auto& outer = geom[0];
                            greenArea.footprint.reserve(outer.size());
                            for (const auto& p : outer) {
                                float x = snap01f(static_cast<float>(p.x) * invExtent);
                                float y = snap01f(static_cast<float>(p.y) * invExtent);
                                greenArea.footprint.emplace_back(x, y);
                            }
                        }

                        // --- validate ---
                        if (greenArea.footprint.size() >= 3) {
                            float area = calculatePolygonArea(greenArea.footprint);
                            if (area > 0.0001f) { // Minimum area threshold
                                greenAreas.push_back(std::move(greenArea));
                                foundAny = true;
                            }
                        }
                    } catch (const std::exception& fe) {
                        std::cout << "⚠️ Error parsing green area feature " << i << ": " << fe.what() << std::endl;
                        continue;
                    }
                }

                std::cout << "✅ Parsed " << greenAreas.size() << " green areas from layer \"" << layerName << "\"" << std::endl;
                if (foundAny) return; // Found green areas, we're done

            } catch (const std::exception& le) {
                std::cout << "⚠️ Error processing green area layer " << layerName << ": " << le.what() << std::endl;
                continue;
            }
        }

        if (!foundAny) {
            std::cout << "⚠️ No green area features found in vector tile" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cout << "❌ Error parsing green area tile data: " << e.what() << std::endl;
        greenAreas.clear();
    }
}

// Helper function to calculate polygon area
float SimpleMapSnapshotter::calculatePolygonArea(const std::vector<glm::vec2>& polygon) {
    if (polygon.size() < 3) return 0.0f;
    
    float area = 0.0f;
    for (size_t i = 0; i < polygon.size(); i++) {
        size_t j = (i + 1) % polygon.size();
        area += polygon[i].x * polygon[j].y;
        area -= polygon[j].x * polygon[i].y;
    }
    return std::abs(area) * 0.5f;
}


void SimpleMapSnapshotter::cleanup() {
    std::cout << "🧹 Cleaning up map rendering resources..." << std::endl;
    
    if (map_texture_id_ != 0) {
        glDeleteTextures(1, &map_texture_id_);
        map_texture_id_ = 0;
    }
    
    if (map_vao_ != 0) {
        glDeleteVertexArrays(1, &map_vao_);
        map_vao_ = 0;
    }
    
    if (map_vbo_ != 0) {
        glDeleteBuffers(1, &map_vbo_);
        map_vbo_ = 0;
    }
    
    if (map_ebo_ != 0) {
        glDeleteBuffers(1, &map_ebo_);
        map_ebo_ = 0;
    }
    
    if (map_shader_program_ != 0) {
        glDeleteProgram(map_shader_program_);
        map_shader_program_ = 0;
    }
    
    if (geometry_shader_program_ != 0) {
        glDeleteProgram(geometry_shader_program_);
        geometry_shader_program_ = 0;
    }
    
    // Cleanup tile geometry
    for (auto& [tile_key, geometry] : tile_geometry_) {
        cleanupGeometry(geometry);
    }
    tile_geometry_.clear();
    
    map_texture_ready_ = false;
    shaders_initialized_ = false;
    geometry_shaders_initialized_ = false;

    if (circle_shader_program_ != 0) {
        glDeleteProgram(circle_shader_program_);
        circle_shader_program_ = 0;
    }
    if (circle_vao_ != 0) glDeleteVertexArrays(1, &circle_vao_);
    if (circle_vbo_ != 0) glDeleteBuffers(1, &circle_vbo_);
    if (circle_ebo_ != 0) glDeleteBuffers(1, &circle_ebo_);
    circle_vao_ = circle_vbo_ = circle_ebo_ = 0;

    if (circle_wall_program_){ glDeleteProgram(circle_wall_program_); circle_wall_program_=0; }
    if (circle_wall_vbo_) { glDeleteBuffers(1,&circle_wall_vbo_); circle_wall_vbo_=0; }
    if (circle_wall_vao_) { glDeleteVertexArrays(1,&circle_wall_vao_); circle_wall_vao_=0; }
    
    std::cout << "✅ Cleanup completed" << std::endl;
}

// Helper function to convert hex color to RGBA (0-1 range)
glm::vec4 SimpleMapSnapshotter::hexToRGBA(const std::string& hexColor, float alpha = 1.0f) {
    std::string hex = hexColor;
    
    // Remove '#' if present
    if (hex[0] == '#') {
        hex = hex.substr(1);
    }
    
    // Ensure we have 6 characters for RGB
    if (hex.length() != 6) {
        return glm::vec4(1.0f, 1.0f, 1.0f, alpha); // Default to white if invalid
    }
    
    // Convert hex to RGB values (0-255)
    int r = std::stoi(hex.substr(0, 2), nullptr, 16);
    int g = std::stoi(hex.substr(2, 2), nullptr, 16);
    int b = std::stoi(hex.substr(4, 2), nullptr, 16);
    
    // Convert to 0-1 range
    return glm::vec4(r / 255.0f, g / 255.0f, b / 255.0f, alpha);
}

void SimpleMapSnapshotter::initializeCircleShader() {
    if (circle_shader_program_ != 0) return;

    // Vertex: expands a unit square into world-space meters around u_center_wm
    const char* vs = R"(
        #version 330 core
        layout (location = 0) in vec2 aUnit;    // [-1,+1] square
        uniform mat4 u_projection;
        uniform mat4 u_view;
        uniform vec3 u_center_wm;               // world meters anchor (x,y), z unused
        uniform float u_radius_m;
        uniform float u_border_m;
        uniform float u_z_offset;
        uniform float u_height_m;

        out vec2 v_local_m; // local offset in meters in the ground plane

        void main() {
            float halfSize = u_radius_m + u_border_m;
            vec2 local = aUnit * halfSize;      // meters in ground plane
            v_local_m = local;

            vec3 world = vec3(u_center_wm.xy + local, u_z_offset + u_height_m);
            gl_Position = u_projection * u_view * vec4(world, 1.0);
        }
    )";

    // Fragment: fill circle (r) + border ring (r..r+border) with smooth edges
    const char* fs = R"(
        #version 330 core
        in vec2 v_local_m;
        uniform float u_radius_m;
        uniform float u_border_m;
        uniform vec4 u_fill_rgba;
        uniform vec4 u_edge_rgba;

        out vec4 FragColor;

        void main() {
            float d = length(v_local_m);          // meters from center
            float aa = max(fwidth(d), 0.001);     // anti-alias in metric domain

            // Fill mask: 1 inside radius, smooth edge on boundary
            float fill = 1.0 - smoothstep(u_radius_m - aa, u_radius_m + aa, d);

            // Ring mask: between r and r+border, smooth on both sides
            float inner = smoothstep(u_radius_m - aa, u_radius_m + aa, d);
            float outer = 1.0 - smoothstep(u_radius_m + u_border_m - aa, u_radius_m + u_border_m + aa, d);
            float ring = inner * outer;

            vec4 col = u_fill_rgba * fill + u_edge_rgba * ring;

            // Discard fully transparent fragments (nice for blending)
            if (col.a <= 0.001) discard;
            FragColor = col;
        }
    )";

    auto compile = [](GLenum type, const char* src) -> GLuint {
        GLuint sh = glCreateShader(type);
        glShaderSource(sh, 1, &src, nullptr);
        glCompileShader(sh);
        GLint ok = GL_FALSE; glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[1024]; glGetShaderInfoLog(sh, 1024, nullptr, log);
            std::cerr << "❌ Circle shader compile error: " << log << std::endl;
            glDeleteShader(sh);
            return 0;
        }
        return sh;
    };

    GLuint v = compile(GL_VERTEX_SHADER,   vs);
    GLuint f = compile(GL_FRAGMENT_SHADER, fs);
    if (!v || !f) { if (v) glDeleteShader(v); if (f) glDeleteShader(f); return; }

    circle_shader_program_ = glCreateProgram();
    glAttachShader(circle_shader_program_, v);
    glAttachShader(circle_shader_program_, f);
    glLinkProgram(circle_shader_program_);
    GLint linked = GL_FALSE;
    glGetProgramiv(circle_shader_program_, GL_LINK_STATUS, &linked);
    if (!linked) {
        char log[1024]; glGetProgramInfoLog(circle_shader_program_, 1024, nullptr, log);
        std::cerr << "❌ Circle program link error: " << log << std::endl;
        glDeleteProgram(circle_shader_program_);
        circle_shader_program_ = 0;
    }
    glDeleteShader(v);
    glDeleteShader(f);
    if (!circle_shader_program_) return;

    // Uniforms
    u_c_proj_      = glGetUniformLocation(circle_shader_program_, "u_projection");
    u_c_view_      = glGetUniformLocation(circle_shader_program_, "u_view");
    u_c_center_wm_ = glGetUniformLocation(circle_shader_program_, "u_center_wm");
    u_c_radius_m_  = glGetUniformLocation(circle_shader_program_, "u_radius_m");
    u_c_border_m_  = glGetUniformLocation(circle_shader_program_, "u_border_m");
    u_c_z_offset_  = glGetUniformLocation(circle_shader_program_, "u_z_offset");
    u_c_fill_      = glGetUniformLocation(circle_shader_program_, "u_fill_rgba");
    u_c_edge_      = glGetUniformLocation(circle_shader_program_, "u_edge_rgba");
    u_c_height_m_ = glGetUniformLocation(circle_shader_program_, "u_height_m");

    // A tiny quad: [-1,-1],[+1,-1],[+1,+1],[-1,+1] (two triangles)
    const GLfloat quad2D[8] = {
        -1.f,-1.f,   1.f,-1.f,   1.f, 1.f,   -1.f, 1.f
    };
    const GLuint  idx[6] = {0,1,2, 2,3,0};

    glGenVertexArrays(1, &circle_vao_);
    glGenBuffers(1, &circle_vbo_);
    glGenBuffers(1, &circle_ebo_);
    glBindVertexArray(circle_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, circle_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad2D), quad2D, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, circle_ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(idx), idx, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    std::cout << "✅ Circle shader initialized" << std::endl;
}


void SimpleMapSnapshotter::initializeCircleWalls(int segments) {
    if (circle_wall_program_ != 0) return;

    // --- shader sources ---
    const char* vs = R"(
        #version 330 core
        layout (location = 0) in vec3 aAttr; // (dirx, diry, topFlag)
        uniform mat4 u_projection, u_view;
        uniform vec3 u_center_wm;
        uniform float u_radius_m, u_height_m, u_z_offset;

        out vec3 v_normal;  // world-space normal for simple lighting
        void main() {
            vec2 dir = normalize(aAttr.xy);
            float t  = aAttr.z;               // 0 bottom, 1 top
            vec3 pos = vec3(u_center_wm.xy + u_radius_m * dir, u_z_offset + t * u_height_m);
            v_normal = vec3(dir, 0.0);        // radial normal
            gl_Position = u_projection * u_view * vec4(pos, 1.0);
        }
    )";

    const char* fs = R"(
        #version 330 core
        in vec3 v_normal;
        uniform vec4  u_color;
        uniform vec3  u_light_dir;    // world space
        uniform vec3  u_light_color;  // e.g., (1,1,0.9)
        uniform vec3  u_ambient_color;// e.g., (0.3,0.3,0.4)
        out vec4 FragColor;
        void main() {
            vec3 n = normalize(v_normal);
            float ndl = max(dot(n, normalize(-u_light_dir)), 0.15); // keep some ambient
            vec3 lit = u_ambient_color * u_color.rgb + ndl * u_light_color * u_color.rgb;
            FragColor = vec4(lit, u_color.a);
        }
    )";

    auto compile = [](GLenum type, const char* src)->GLuint{
        GLuint sh = glCreateShader(type);
        glShaderSource(sh, 1, &src, nullptr);
        glCompileShader(sh);
        GLint ok=GL_FALSE; glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
        if(!ok){ char log[1024]; glGetShaderInfoLog(sh,1024,nullptr,log);
            std::cerr << "❌ circle wall shader compile: " << log << std::endl;
            glDeleteShader(sh); return 0; }
        return sh;
    };

    GLuint v = compile(GL_VERTEX_SHADER, vs);
    GLuint f = compile(GL_FRAGMENT_SHADER, fs);
    circle_wall_program_ = glCreateProgram();
    glAttachShader(circle_wall_program_, v);
    glAttachShader(circle_wall_program_, f);
    glLinkProgram(circle_wall_program_);
    glDeleteShader(v); glDeleteShader(f);
    GLint linked=GL_FALSE; glGetProgramiv(circle_wall_program_, GL_LINK_STATUS, &linked);
    if(!linked){ char log[1024]; glGetProgramInfoLog(circle_wall_program_,1024,nullptr,log);
        std::cerr << "❌ circle wall link: " << log << std::endl;
        glDeleteProgram(circle_wall_program_); circle_wall_program_=0; return; }

    // uniforms
    u_w_proj_       = glGetUniformLocation(circle_wall_program_, "u_projection");
    u_w_view_       = glGetUniformLocation(circle_wall_program_, "u_view");
    u_w_center_wm_  = glGetUniformLocation(circle_wall_program_, "u_center_wm");
    u_w_radius_m_   = glGetUniformLocation(circle_wall_program_, "u_radius_m");
    u_w_height_m_   = glGetUniformLocation(circle_wall_program_, "u_height_m");
    u_w_z_offset_   = glGetUniformLocation(circle_wall_program_, "u_z_offset");
    u_w_color_      = glGetUniformLocation(circle_wall_program_, "u_color");
    u_w_light_dir_  = glGetUniformLocation(circle_wall_program_, "u_light_dir");
    u_w_light_color_= glGetUniformLocation(circle_wall_program_, "u_light_color");
    u_w_ambient_color_=glGetUniformLocation(circle_wall_program_, "u_ambient_color");

    // --- unit circle strip VBO: (dirx, diry, topFlag) ---
    std::vector<glm::vec3> verts; verts.reserve((segments+1)*2);
    for (int i=0;i<=segments;++i){
        float a = (float(i)/segments) * 2.0f * float(M_PI);
        float cx = std::cos(a), sy = std::sin(a);
        verts.emplace_back(cx, sy, 0.0f); // bottom
        verts.emplace_back(cx, sy, 1.0f); // top
    }
    circle_wall_vertex_count_ = GLsizei(verts.size());

    glGenVertexArrays(1, &circle_wall_vao_);
    glGenBuffers(1, &circle_wall_vbo_);
    glBindVertexArray(circle_wall_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, circle_wall_vbo_);
    glBufferData(GL_ARRAY_BUFFER, verts.size()*sizeof(glm::vec3), verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    std::cout << "✅ Circle walls initialized (" << segments << " segments)\n";
}



void SimpleMapSnapshotter::renderCircle(const glm::mat4& projection, const glm::mat4& view) {
    if (!circle_enabled_ || circle_shader_program_ == 0 || circle_vao_ == 0) return;
    if (!camera_.center || !camera_.zoom || meters_per_tile_ <= 0.0f) return;

    // Where to draw in world meters (relative to current camera center)
    glm::vec2 center_wm = latLngToWorldMeters(LatLng(circle_lat_deg_, circle_lng_deg_));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glUseProgram(circle_shader_program_);
    glUniformMatrix4fv(u_c_proj_, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(u_c_view_, 1, GL_FALSE, glm::value_ptr(view));
    glUniform3f(u_c_center_wm_, center_wm.x, center_wm.y, 0.0f);
    glUniform1f(u_c_radius_m_,  circle_radius_m_);
    glUniform1f(u_c_border_m_,  circle_border_m_);
    glUniform1f(u_c_z_offset_,  circle_z_offset_);
    glUniform1f(u_c_height_m_,  circle_height_m_);
    glUniform4fv(u_c_fill_, 1, glm::value_ptr(circle_fill_rgba_));
    glUniform4fv(u_c_edge_, 1, glm::value_ptr(circle_edge_rgba_));

    glBindVertexArray(circle_vao_);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glUseProgram(0);
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
}


void SimpleMapSnapshotter::renderCircleWalls(const glm::mat4& projection, const glm::mat4& view) {
    if (!circle_enabled_ || circle_wall_program_==0 || circle_wall_vao_==0) return;
    if (!camera_.center || !camera_.zoom || meters_per_tile_<=0.0f) return;
    if (circle_height_m_ <= 0.0f) return;

    glm::vec2 center_wm = latLngToWorldMeters(LatLng(circle_lat_deg_, circle_lng_deg_));

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glUseProgram(circle_wall_program_);
    glUniformMatrix4fv(u_w_proj_, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(u_w_view_, 1, GL_FALSE, glm::value_ptr(view));
    glUniform3f(u_w_center_wm_, center_wm.x, center_wm.y, 0.0f);
    glUniform1f(u_w_radius_m_,  circle_total_radius_m_);
    glUniform1f(u_w_height_m_,  circle_height_m_);
    glUniform1f(u_w_z_offset_,  circle_z_offset_);
    glUniform4fv(u_w_color_, 1, glm::value_ptr(circle_edge_rgba_));

    // simple lighting consistent with your 3D shader
    const glm::vec3 light_dir(0.3f, -1.0f, 0.2f);
    const glm::vec3 light_color(1.0f, 1.0f, 0.9f);
    const glm::vec3 ambient_color(0.3f, 0.3f, 0.4f);
    glUniform3fv(u_w_light_dir_,     1, glm::value_ptr(light_dir));
    glUniform3fv(u_w_light_color_,   1, glm::value_ptr(light_color));
    glUniform3fv(u_w_ambient_color_, 1, glm::value_ptr(ambient_color));

    glBindVertexArray(circle_wall_vao_);
    // We uploaded vertices in strip order already: bottom, top, bottom, top, ...
    glDrawArrays(GL_TRIANGLE_STRIP, 0, circle_wall_vertex_count_);
    glBindVertexArray(0);

    glUseProgram(0);
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
}

void SimpleMapSnapshotter::renderCircleOverlay(const glm::mat4& projection, const glm::mat4& view) {
    // guards inside these ensure one-time init
    initializeCircleShader();
    initializeCircleWalls();
    renderCircle(projection, view);       // top disc (with border, at height)
    renderCircleWalls(projection, view);  // vertical wall (cylinder side)
}



} // namespace mbgl


// change the color height of the building in this function 
// : createExtrudedPolygon you can change the color height 0.0f to 1.0f 
// in the roofStartZ = topZ * 0.8f 