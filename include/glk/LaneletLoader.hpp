#pragma once

#include <iostream>
#include <vector>
#include <GL/gl3w.h>
#include <string>
#include <mutex>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include <glk/glsl_shader.hpp>

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

class LaneletLoader
{
private:
    // color for the terminals
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    // lanelet2 map varianles
    bool has_lanelet2_map_setted_ = false;
    std::string map_path_;
    bool has_lanelet2_map_ = false;
    std::string lanelet_frame_id_ = "map";  // Frame ID for TF transform
    GLuint map_lines_vao_ = 0, map_lines_vbo_ = 0;
    size_t map_lines_count_ = 0;
    std::vector<Eigen::Vector3f> map_lines_;
    std::vector<std::vector<Eigen::Vector3f> > crosswalk_polygons_;

    GLuint crosswalk_vao_ = 0;
    GLuint crosswalk_vbo_ = 0;
    size_t crosswalk_lines_count_ = 0;
    std::vector<Eigen::Vector3f> crosswalk_lines_;

    std::vector<Eigen::Vector3f> crosswalk_tris_;
    size_t crosswalk_tris_count_ = 0;

    size_t stripe_lines_count_ = 0;
    std::vector<Eigen::Vector3f> stripe_lines_;
    GLuint stripe_vao_ = 0;
    GLuint stripe_vbo_ = 0;

    std::vector<Eigen::Vector3f> stripe_tris_;
    size_t stripe_tris_count_ = 0;

public:
    LaneletLoader(/* args */);
    ~LaneletLoader();

    void loadLanelet2Map(const std::string &path);
    void setFrameId(const std::string &frame_id) { lanelet_frame_id_ = frame_id; }
    void mapProcessing(lanelet::LaneletMapPtr &t_map);
    void mapLines(glk::GLSLShader &shader, std::mutex &tf_mutex, const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);
    void crosswalks(glk::GLSLShader &shader, std::mutex &tf_mutex, const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);
    void stripes(glk::GLSLShader &shader, std::mutex &tf_mutex, const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms);
    bool isLoaded() const;
};
