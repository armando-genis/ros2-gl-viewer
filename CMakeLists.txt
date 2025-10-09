
cmake_minimum_required(VERSION 3.10)
project(ros2-gl-viewer)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glm REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Freetype REQUIRED)
find_package(GLUT   REQUIRED)
find_package(JPEG REQUIRED)
find_package(PNG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ICU REQUIRED icu-uc icu-i18n)
pkg_check_modules(LIBUV REQUIRED libuv)

# ROS2 dependencies - without using ament
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# load 3d graphics libraries (Assimp)
find_package(assimp REQUIRED) 
include_directories(${ASSIMP_INCLUDE_DIRS})

# Find gl3w
set(GL3W_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/gl3w)
include_directories(${GL3W_DIR}/include)

# Set ImGui directory
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)
include_directories(${IMGUI_DIR})
set(IMGUI_IMPL_DIR ${IMGUI_DIR}/backends)
include_directories(${IMGUI_IMPL_DIR})

# Lanelet2
set(LANELET2_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Rosless-Lanelet2")
list(APPEND CMAKE_MODULE_PATH "${LANELET2_ROOT}/cmake")
add_subdirectory("${LANELET2_ROOT}" "${CMAKE_BINARY_DIR}/lanelet2_build")

# Cgltf library
set(CGLTF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/cgltf)
set(STB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
include_directories(${CGLTF_DIR})
include_directories(${STB_DIR})

# ===============================================
# MAPBOX GL NATIVE CONFIGURATION (UPDATED)
# ===============================================

# Mapbox GL Native - Use pre-built libraries
set(MAPBOX_GL_NATIVE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mapbox-gl-native)
set(MAPBOX_GL_NATIVE_BUILD_DIR ${MAPBOX_GL_NATIVE_DIR}/build)

# Include Mapbox GL Native headers
include_directories(${MAPBOX_GL_NATIVE_DIR}/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/src)
include_directories(${MAPBOX_GL_NATIVE_DIR}/qt)
include_directories(${MAPBOX_GL_NATIVE_DIR}/platform)
include_directories(${MAPBOX_GL_NATIVE_DIR}/platform/default/include)  # This is where map_snapshotter.hpp is
include_directories(${MAPBOX_GL_NATIVE_DIR}/platform/default/include/mbgl/storage)

# Include Mapbox vendor dependencies that are needed for headers
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/optional)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/variant/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/geometry.hpp/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/geojson.hpp/include)
## RapidJSON headers live in mapbox-base/extras/rapidjson/include in this vendored tree.
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/extras/rapidjson/include)

# Backwards-compatible fallback: some mapbox-gl-native versions put rapidjson under
# vendor/mapbox-base/deps/rapidjson/include — add it only if the directory exists.
if(EXISTS "${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/rapidjson/include")
  include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/rapidjson/include)
endif()
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/expected-lite/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/mapbox-base/deps/shelf-pack-cpp/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/vector-tile/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/protozero/include)
include_directories(${MAPBOX_GL_NATIVE_DIR}/vendor/earcut.hpp/include)

# Function to find all Mapbox libraries
function(find_mapbox_libraries MAPBOX_BUILD_DIR RESULT_VAR)
    set(FOUND_LIBS)
    
    # Core libraries (these should exist)
    set(CORE_LIBS
        "mbgl-core"
        "mbgl-vendor-csscolorparser" 
        "mbgl-vendor-nunicode"
        "mbgl-vendor-parsedate"
        "mbgl-vendor-sqlite"
    )
    
    # Platform and additional libraries (MapSnapshotter is likely here)
    set(PLATFORM_LIBS
        "mbgl-platform-default"
        "mbgl-platform"
        "mbgl-headless"
        "mbgl-filesource"
        "mbgl-renderer"
        "mbgl-storage"
        "mbgl-style"
        "mbgl-text"
        "mbgl-gfx"
        "mbgl-annotation"
        "mbgl-geometry"
        "mbgl-algorithm"
        "mbgl-util"
        "mbgl-vendor-protozero"
        "mbgl-vendor-eternal"
        "mbgl-vendor-polylabel"
    )
    
    # Check all possible library names
    foreach(LIB_NAME ${CORE_LIBS} ${PLATFORM_LIBS})
        set(LIB_PATH "${MAPBOX_BUILD_DIR}/lib${LIB_NAME}.a")
        if(EXISTS "${LIB_PATH}")
            message(STATUS "Found Mapbox library: ${LIB_PATH}")
            list(APPEND FOUND_LIBS "${LIB_PATH}")
            
            # Check if this library contains MapSnapshotter symbols (optional check)
            execute_process(
                COMMAND bash -c "nm '${LIB_PATH}' 2>/dev/null | grep -q 'MapSnapshotter' && echo 'HAS_SYMBOLS' || echo 'NO_SYMBOLS'"
                OUTPUT_VARIABLE SYMBOL_CHECK
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
            )
            if(SYMBOL_CHECK STREQUAL "HAS_SYMBOLS")
                message(STATUS "  ✓ Contains MapSnapshotter symbols: ${LIB_PATH}")
            endif()
        endif()
    endforeach()
    
    set(${RESULT_VAR} ${FOUND_LIBS} PARENT_SCOPE)
endfunction()

# Find all available Mapbox libraries
find_mapbox_libraries(${MAPBOX_GL_NATIVE_BUILD_DIR} MAPBOX_LIBRARIES)

# Check if we found the core library at minimum
set(MAPBOX_CORE_LIB ${MAPBOX_GL_NATIVE_BUILD_DIR}/libmbgl-core.a)
if(NOT EXISTS ${MAPBOX_CORE_LIB})
    message(FATAL_ERROR "Mapbox GL Native core library not found at ${MAPBOX_CORE_LIB}")
endif()

# Check if we found any libraries
if(NOT MAPBOX_LIBRARIES)
    message(FATAL_ERROR "No Mapbox GL Native libraries found in ${MAPBOX_GL_NATIVE_BUILD_DIR}")
else()
    list(LENGTH MAPBOX_LIBRARIES MAPBOX_LIB_COUNT)
    message(STATUS "Found ${MAPBOX_LIB_COUNT} Mapbox libraries total")
endif()

# ===============================================
# END MAPBOX CONFIGURATION
# ===============================================


# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${FREETYPE_INCLUDE_DIRS}

    # ROS2 dependencies
    ${rclcpp_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
)

set(GLK_SOURCES
  src/glk/colormap.cpp
  src/glk/frame_buffer.cpp
  src/glk/glsl_shader.cpp
  src/glk/lines.cpp
  src/glk/mesh.cpp
  src/glk/pointcloud_buffer.cpp
  src/glk/loaders/ply_loader.cpp
  src/glk/primitives/primitives.cpp
  src/glk/TextRendering.cpp
  src/glk/PathRendering.cpp
  src/glk/CloudRendering.cpp
  src/glk/PclLoader.cpp
  src/glk/modelUpload.cpp
  src/glk/LaneletLoader.cpp
  src/glk/ThickLines.cpp
  src/glk/MapBoxRendering.cpp
)

# List all source files in guik directory
set(GUIK_SOURCES
  src/guik/camera_control.cpp
  src/guik/gl_canvas.cpp
  src/guik/imgui_application.cpp
  src/guik/model_control.cpp
  src/guik/projection_control.cpp
)

add_executable(pointcloud_viewer
  src/main.cpp
  ${GLK_SOURCES}
  ${GUIK_SOURCES}
  ${GL3W_DIR}/src/gl3w.c
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_demo.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

# Add compile definitions for the libraries
target_compile_definitions(pointcloud_viewer PRIVATE
    CGLTF_IMPLEMENTATION
    # STB_IMAGE_IMPLEMENTATION
)

target_include_directories(pointcloud_viewer PRIVATE
  ${LIBUV_INCLUDE_DIRS}
)


# ===============================================
# LINKING SECTION (UPDATED)
# ===============================================
ament_target_dependencies(pointcloud_viewer
  rclcpp
  pcl_conversions
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Link libraries
target_link_libraries(pointcloud_viewer
  # OpenGL and GUI libraries
  ${OPENGL_LIBRARIES}
  glfw
  dl  # For gl3w dynamic loading
  ${FREETYPE_LIBRARIES}
  ${GLUT_LIBRARY}
  

  # Lanelet2
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  
  # Point cloud and math libraries
  ${PCL_LIBRARIES}
  
  # Link ALL found Mapbox libraries (this should include the one with MapSnapshotter)
  ${MAPBOX_LIBRARIES}
  
  # System dependencies for Mapbox
  ${LIBUV_LIBRARIES}
  ${JPEG_LIBRARIES}
  ${PNG_LIBRARIES}
  ${ZLIB_LIBRARIES}
  ${ICU_LIBRARIES}
  curl
  
  # System libraries
  GL
  pthread
  m
)


# Make sure the application finds the shader files
add_custom_command(TARGET pointcloud_viewer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_SOURCE_DIR}/data
  ${CMAKE_BINARY_DIR}/data
)

# Copy the icon file from src to build directory
add_custom_command(TARGET pointcloud_viewer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  ${CMAKE_SOURCE_DIR}/src/appicon.png
  ${CMAKE_BINARY_DIR}/appicon.png
  COMMENT "Copying appicon.png to build directory"
)

# Copy the Perlin noise texture for cloud rendering
add_custom_command(TARGET pointcloud_viewer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
  ${CMAKE_SOURCE_DIR}/src/perlin256.png
  ${CMAKE_BINARY_DIR}/perlin256.png
  COMMENT "Copying perlin256.png to build directory"
)

# Install the executable
install(TARGETS pointcloud_viewer
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

