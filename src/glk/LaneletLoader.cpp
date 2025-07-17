#include <glk/LaneletLoader.hpp>
#include <iostream>

LaneletLoader::LaneletLoader(/* args */)
{
}

LaneletLoader::~LaneletLoader()
{
}

void LaneletLoader::renderlanelet(glk::GLSLShader &shader, const std::unordered_map<std::string, Eigen::Isometry3f> &frame_transforms)
{
    if (map_lines_count_ > 0 && map_lines_vao_ != 0)
    {
        shader.set_uniform("color_mode", 1);                                                 // flat color
        shader.set_uniform("material_color", Eigen::Vector4f(0.459f, 0.576f, 0.686f, 0.8f)); // yellow
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        shader.set_uniform("model_matrix", T.matrix());
        glBindVertexArray(map_lines_vao_);
        glDrawArrays(GL_LINES, 0, GLsizei(map_lines_count_));
        glBindVertexArray(0);
    }

    if (crosswalk_tris_count_ > 0 && crosswalk_vao_ != 0)
    {
        // flat color mode
        shader.set_uniform("color_mode", 1);
        // RGBA: here white, but you could pick e.g. (1,1,0,1) for yellow zebra stripes
        shader.set_uniform("material_color", Eigen::Vector4f(0.545f, 0.627f, 0.643f, 1.0f));
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        shader.set_uniform("model_matrix", T.matrix());

        glBindVertexArray(crosswalk_vao_);
        glDrawArrays(GL_TRIANGLES, 0, GLsizei(crosswalk_tris_count_));
        glBindVertexArray(0);
    }

    if (stripe_tris_count_ > 0)
    {
        shader.set_uniform("color_mode", 1); // flat color
        shader.set_uniform("material_color", Eigen::Vector4f(0.918f, 0.918f, 0.918f, 1.0f));
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        shader.set_uniform("model_matrix", T.matrix());
        glBindVertexArray(stripe_vao_);
        glDrawArrays(GL_TRIANGLES, 0, GLsizei(stripe_tris_count_));
        glBindVertexArray(0);
    }
}

bool LaneletLoader::loadLanelet2Map(const std::string &path)
{
    std::cout << "Loading Lanelet2 map from: " << path << std::endl;
    lanelet::Origin origin({49, 8.4});
    lanelet::projection::LocalCartesianProjector projector(origin);
    try
    {
        lanelet::LaneletMapPtr map = lanelet::load(path, projector);
        std::cout << green << "Loaded Lanelet2 map with " << map->laneletLayer.size() << " lanelets." << reset << std::endl;
        map_path_ = path;

        for (auto &point : map->pointLayer)
        {
            point.x() = point.attribute("local_x").asDouble().value();
            point.y() = point.attribute("local_y").asDouble().value();
        }
        mapProcessing(map);
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << red << "Failed to load Lanelet2 map: " << e.what() << reset << std::endl;
        return false;
    }
}

void LaneletLoader::mapProcessing(lanelet::LaneletMapPtr &t_map)
{

    int crosswalk_count = 0;
    int road_element_count = 0;
    float z_offset = 0.08f;
    map_lines_.clear();
    stripe_lines_.clear();
    crosswalk_lines_.clear();
    stripe_tris_.clear();
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
                // polygon.emplace_back(point.x(), point.y(), point.z());
                polygon.emplace_back(point.x(), point.y(), max_z); // Use max_z for all points

            // Right bound (reverse)
            for (int i = ll.rightBound().size() - 1; i >= 0; --i)
                // polygon.emplace_back(ll.rightBound()[i].x(), ll.rightBound()[i].y(), ll.rightBound()[i].z());
                polygon.emplace_back(ll.rightBound()[i].x(), ll.rightBound()[i].y(), max_z); // Use max_z for all points

            // Close the loop
            polygon.push_back(polygon.front());
            crosswalk_polygons_.push_back(polygon);

            // print one point of the crosswalk polygon
            std::cout << "Crosswalk polygon point: "
                      << polygon[0].x() << ", "
                      << polygon[0].y() << ", "
                      << polygon[0].z() << std::endl;

            crosswalk_tris_.clear();
            for (auto &poly : crosswalk_polygons_)
            {
                // fan-triangulate around poly[0]:
                for (size_t i = 1; i + 1 < poly.size(); ++i)
                {
                    crosswalk_tris_.push_back(poly[0]);
                    crosswalk_tris_.push_back(poly[i]);
                    crosswalk_tris_.push_back(poly[i + 1]);
                }
            }
            crosswalk_tris_count_ = crosswalk_tris_.size();

            // zebra stripes
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
                        start_left.z() = max_z + z_offset; // Use the maximum z from the left bound
                        start_left_set = true;
                    }
                    if (accumulated_length + segment_length > end_dist && !end_left_set)
                    {
                        double ratio = (end_dist - accumulated_length) / segment_length;
                        end_left.x() = ll.leftBound()[i].x() + ratio * (ll.leftBound()[i + 1].x() - ll.leftBound()[i].x());
                        end_left.y() = ll.leftBound()[i].y() + ratio * (ll.leftBound()[i + 1].y() - ll.leftBound()[i].y());
                        end_left.z() = max_z + z_offset; // Use the maximum z from the left bound
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
                        start_right.z() = max_z + z_offset; // Use the maximum z from the right bound
                        start_right_set = true;
                    }
                    if (accumulated_length + segment_length > end_dist && !end_right_set)
                    {
                        double ratio = (end_dist - accumulated_length) / segment_length;
                        end_right.x() = ll.rightBound()[i].x() + ratio * (ll.rightBound()[i + 1].x() - ll.rightBound()[i].x());
                        end_right.y() = ll.rightBound()[i].y() + ratio * (ll.rightBound()[i + 1].y() - ll.rightBound()[i].y());
                        end_right.z() = max_z + z_offset; // Use the maximum z from the right bound
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
                }

                if (stripe_polygon.size() >= 3)
                {
                    // triangulate this one stripe:
                    for (size_t j = 1; j + 1 < stripe_polygon.size(); ++j)
                    {
                        stripe_tris_.push_back(stripe_polygon[0]);
                        stripe_tris_.push_back(stripe_polygon[j]);
                        stripe_tris_.push_back(stripe_polygon[j + 1]);
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
    glBufferData(
        GL_ARRAY_BUFFER,
        crosswalk_tris_count_ * sizeof(Eigen::Vector3f),
        crosswalk_tris_.data(),
        GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, // attribute 0 = position
        3, GL_FLOAT, GL_FALSE,
        sizeof(Eigen::Vector3f),
        (void *)0);
    glBindVertexArray(0);

    std::cout << blue << "----> Number of crosswalk lines: " << crosswalk_lines_count_ << reset << std::endl;

    // --- after your zebra‐stripe generation loop ---
    stripe_tris_count_ = stripe_tris_.size();
    // upload to GPU
    if (stripe_vao_ == 0)
    {
        glGenVertexArrays(1, &stripe_vao_);
        glGenBuffers(1, &stripe_vbo_);
    }
    glBindVertexArray(stripe_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, stripe_vbo_);
    glBufferData(
        GL_ARRAY_BUFFER,
        stripe_tris_count_ * sizeof(Eigen::Vector3f),
        stripe_tris_.data(),
        GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE,
        sizeof(Eigen::Vector3f),
        (void *)0);
    glBindVertexArray(0);

    std::cout << blue << "----> Number of stripe lines: " << stripe_lines_count_ << reset << std::endl;
    std::cout << blue << "----> Number of stripe triangles: " << stripe_tris_count_ << reset << std::endl;
}
