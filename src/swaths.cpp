#include "rerun/archetypes/geo_points.hpp"
#include "rerun/archetypes/points3d.hpp"
#include "rerun/recording_stream.hpp"
#include <cmath>
#include <ctime>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/string.hpp>

#include <rerun/blueprint/archetypes/map_background.hpp>
#include <rerun/blueprint/archetypes/view_blueprint.hpp>
#include <rerun/blueprint/components/map_provider.hpp>

#include "farmbot_holodeck/color.hpp"

#include "farmbot_interfaces/msg/lines.hpp"

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
using namespace rerun::demo;
using namespace std::chrono_literals;
using namespace std::placeholders;

class PoseNode {
  private:
    std::string namespace_;
    rclcpp::Node::SharedPtr node;
    std::string tcp;
    std::string color;
    std::shared_ptr<rerun::RecordingStream> rec;

    rclcpp::Subscription<farmbot_interfaces::msg::Lines>::SharedPtr headlands_subscriber, swaths_subscriber;

    std::vector<rerun::Color> colors;

    std::vector<std::array<float, 3>> loc_border_positions_;
    std::vector<rerun::LatLon> geo_border_positions_;
    std::vector<std::vector<std::array<float, 3>>> loc_swath_positions_;
    std::vector<std::vector<rerun::LatLon>> geo_swath_positions_;

  public:
    PoseNode(rclcpp::Node::SharedPtr node) : node(node) {
        RCLCPP_INFO(node->get_logger(), "Rerun for pose created");
        tcp = node->get_parameter_or<std::string>("tcp", "127.0.0.1:9876");
        color = node->get_parameter_or<std::string>("color", "#ff0000");

        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        rec = std::make_shared<rerun::RecordingStream>("farmbot", "space");
        RCLCPP_INFO(node->get_logger(), "Connecting to %s", tcp.c_str());
        auto _one = rec->connect_tcp(tcp);

        if (_one.is_err()) {
            RCLCPP_ERROR(node->get_logger(), "Could not connect to %s", tcp.c_str());
            return;
        }

        RCLCPP_INFO(node->get_logger(), "Spawning viewer for SWATHS node");

        colors.push_back(holodeck::hexToColor(color));

        if (rec->spawn().is_err()) {
            RCLCPP_WARN(node->get_logger(), "Could not spawn viewer");
        }

        swaths_subscriber = node->create_subscription<farmbot_interfaces::msg::Lines>(
            "pln/swaths", 10, std::bind(&PoseNode::swaths_callback, this, _1));

        headlands_subscriber = node->create_subscription<farmbot_interfaces::msg::Lines>(
            "pln/headland", 10, std::bind(&PoseNode::headlands_callback, this, _1));
    }

    ~PoseNode() { rclcpp::shutdown(); }

  private:
    void swaths_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
        loc_swath_positions_.clear();
        geo_swath_positions_.clear();
        // RCLCPP_INFO(node->get_logger(), "Swaths callback");
        for (auto swath : msg->lines) {
            std::vector<std::array<float, 3>> points;
            float x0 = static_cast<float>(swath.loc_line[0].x);
            float y0 = static_cast<float>(swath.loc_line[0].y);
            // float z0 = static_cast<float>(swath.loc_line[0].z);
            float z0 = 0.0;
            float x1 = static_cast<float>(swath.loc_line[1].x);
            float y1 = static_cast<float>(swath.loc_line[1].y);
            // float z1 = static_cast<float>(swath.loc_line[1].z);
            float z1 = 0.0;
            points.push_back({x0, y0, z0});
            points.push_back({x1, y1, z1});
            loc_swath_positions_.push_back(points);

            // std::vector<rerun::LatLon> one_line;
            // double lat0 = swath.geo_line[0].x;
            // double lon0 = swath.geo_line[0].y;
            // one_line.push_back({lat0, lon0});
            // double lat1 = swath.geo_line[1].x;
            // double lon1 = swath.geo_line[1].y;
            // one_line.push_back({lat1, lon1});
            // geo_swath_positions_.push_back(one_line);
        }
        rec->log_static("world/map/" + namespace_ + "/pln/swaths",
                        rerun::Transform3D(rerun::components::Translation3D(.0, .0, .0),
                                           rerun::Quaternion::from_wxyz(1.0, 0.0, 0.0, 0.0)));
        rec->log_static("world/map/" + namespace_ + "/pln/swaths",
                        rerun::LineStrips3D(loc_swath_positions_).with_colors(colors).with_radii({{0.2f}}));
        //
        // std::vector<rerun::components::GeoLineString> linestring;
        // for (auto latlon : geo_swath_positions_) {
        //     auto string = rerun::components::GeoLineString::from_lat_lon(latlon);
        //     linestring.push_back(string);
        // }
        // rec->log_static("world/map/" + namespace_ + "/pln/swaths",
        //                 rerun::GeoLineStrings(linestring).with_colors(colors).with_radii({{0.2f}}));
    }

    void headlands_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
        // RCLCPP_INFO(node->get_logger(), "Headlands callback");
        geo_border_positions_.clear();
        loc_border_positions_.clear();
        for (auto line : msg->lines) {
            float x = static_cast<float>(line.loc_line[0].x);
            float y = static_cast<float>(line.loc_line[0].y);
            // float z = static_cast<float>(line.loc_line[0].z);
            float z = 0.0;
            loc_border_positions_.push_back({x, y, z});
            // float lat = static_cast<float>(line.geo_line[0].x);
            // float lon = static_cast<float>(line.geo_line[0].y);
            // geo_border_positions_.push_back({lat, lon});
        }
        rec->log_static("world/map/" + namespace_ + "/pln/headland",
                        rerun::Transform3D(rerun::components::Translation3D(.0, .0, .0),
                                           rerun::Quaternion::from_wxyz(1.0, 0.0, 0.0, 0.0)));
        auto border__ = rerun::components::LineStrip3D(loc_border_positions_);
        rec->log_static("world/map/" + namespace_ + "/pln/headland",
                        rerun::LineStrips3D(border__).with_colors(colors).with_radii({{0.2f}}));

        // auto linestring = rerun::components::GeoLineString::from_lat_lon(geo_border_positions_);
        // rec->log_static("world/map/" + namespace_ + "/pln/headland",
        //                 rerun::GeoLineStrings(linestring).with_colors(colors).with_radii({{0.2f}}));
    }

    float delta_distance(std::array<float, 3> pos1, std::array<float, 3> pos2) {
        auto dx = pos1[0] - pos2[0];
        auto dy = pos1[1] - pos2[1];
        auto dz = pos1[2] - pos2[2];
        auto distance = sqrt(dx * dx + dy * dy + dz * dz);
        RCLCPP_INFO(node->get_logger(), "Distance: %f", distance);
        return distance;
    }

    float delta_distance(rerun::LatLon loc1, rerun::LatLon loc2) {
        auto dx = loc1.latitude() - loc2.latitude();
        auto dy = loc1.longitude() - loc2.longitude();
        auto distance = sqrt(dx * dx + dy * dy);
        RCLCPP_INFO(node->get_logger(), "Distance: %f", distance);
        return distance;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    rclcpp::NodeOptions options_0;
    options_0.allow_undeclared_parameters(true);
    options_0.automatically_declare_parameters_from_overrides(true);
    auto node_0 = rclcpp::Node::make_shared("rerun", options_0);
    auto parser = PoseNode(node_0);
    executor.add_node(node_0);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
