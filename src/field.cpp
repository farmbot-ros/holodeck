#include "rerun/archetypes/geo_line_strings.hpp"
#include "rerun/archetypes/geo_points.hpp"
#include "rerun/archetypes/line_strips2d.hpp"
#include "rerun/archetypes/line_strips3d.hpp"
#include "rerun/archetypes/points3d.hpp"
#include "rerun/archetypes/transform3d.hpp"
#include "rerun/collection.hpp"
#include "rerun/components/geo_line_string.hpp"
#include "rerun/components/lat_lon.hpp"
#include "rerun/components/line_strip3d.hpp"
#include "rerun/recording_stream.hpp"
#include <cmath>
#include <cstdint>
#include <ctime>
#include <farmbot_interfaces/msg/line.hpp>
#include <farmbot_interfaces/msg/lines.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shared_mutex>
#include <std_msgs/msg/string.hpp>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include <vector>

using namespace rerun::demo;
using namespace std::chrono_literals;
using namespace std::placeholders;

class PoseNode {
  private:
    std::string namespace_;
    rclcpp::Node::SharedPtr node;
    std::string tcp;
    std::random_device rd;
    std::shared_ptr<rerun::RecordingStream> rec;

    bool got_border_, got_headlands_, got_swaths_;
    std::vector<std::array<float, 3>> loc_border_positions_;
    std::vector<rerun::LatLon> geo_border_positions_;
    std::vector<std::vector<std::array<float, 3>>> loc_headland_positions_, geo_headland_positions_;
    std::vector<std::vector<std::array<float, 3>>> loc_swath_positions_, geo_swath_positions_;

    rclcpp::Subscription<farmbot_interfaces::msg::Lines>::SharedPtr headlands_subscriber, swaths_subscriber,
        border_subscriber;

    // timer publish
    rclcpp::TimerBase::SharedPtr timer;

  public:
    PoseNode(rclcpp::Node::SharedPtr node) : node(node) {
        // echo::info("Rerun for pose created");
        RCLCPP_INFO(node->get_logger(), "Rerun for pose created");
        tcp = node->get_parameter_or<std::string>("tcp", "127.0.0.1:9876");

        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        rec = std::make_shared<rerun::RecordingStream>("farmbot", "space");
        RCLCPP_INFO(node->get_logger(), "Connecting to %s", tcp.c_str());
        auto _one = rec->connect_tcp(tcp);

        if (rec->spawn().is_err()) {
            // echo::warn("Could not spawn viewer");
            RCLCPP_WARN(node->get_logger(), "Could not spawn viewer");
        }

        // headlands_subscriber = node->create_subscription<farmbot_interfaces::msg::Lines>(
        //     "/field/headlands", 10, std::bind(&Beacon::headlands_callback, this, _1));

        swaths_subscriber = node->create_subscription<farmbot_interfaces::msg::Lines>(
            "/field/swaths", 10, std::bind(&PoseNode::swaths_callback, this, _1));

        border_subscriber = node->create_subscription<farmbot_interfaces::msg::Lines>(
            "/field/border", 10, std::bind(&PoseNode::border_callback, this, _1));

        std::vector<rerun::Position3D> points;
        points.push_back(rerun::Position3D(.0, .0, .0));
        //
    }

    ~PoseNode() { rclcpp::shutdown(); }

  private:
    // void headlands_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
    //     if (!got_headlands_) {
    //         for (auto polygon : msg->polygons) {
    //             std::vector<std::array<float, 3>> points;
    //             for (auto point : polygon.polygon.points) {
    //                 points.push_back({point.x, point.y, 0});
    //             }
    //             headland_positions_.push_back(points);
    //         }
    //     }
    //     // RCLCPP_INFO(node->get_logger(), "Publishing headlands");
    //     rec->log_static("world/map/field/headland",
    //                     rerun::Transform3D(rerun::components::Translation3D(.0, .0, .0),
    //                                        rerun::Quaternion::from_wxyz(1.0, 0.0, 0.0, 0.0)));
    //     rec->log_static("world/map/field/headland",
    //                     rerun::LineStrips3D(headland_positions_).with_colors({{158, 142,
    //                     158}}).with_radii({{0.2f}}));
    //
    //     got_headlands_ = true;
    // }
    //
    void swaths_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
        if (!got_swaths_) {
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
            }
        }
        RCLCPP_INFO(node->get_logger(), "Publishing %lu swaths", loc_swath_positions_.size());
        rec->log_static("world/map/field/swaths", rerun::Transform3D(rerun::components::Translation3D(.0, .0, .0),
                                                                     rerun::Quaternion::from_wxyz(1.0, 0.0, 0.0, 0.0)));
        rec->log_static("world/map/field/swaths",
                        rerun::LineStrips3D(loc_swath_positions_).with_colors({{158, 142, 158}}).with_radii({{0.2f}}));
        // got_swaths_ = true;
    }

    void border_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
        if (!got_border_) {
            for (auto line : msg->lines) {
                float x = static_cast<float>(line.loc_line[0].x);
                float y = static_cast<float>(line.loc_line[0].y);
                // float z = static_cast<float>(line.loc_line[0].z);
                float z = 0.0;
                loc_border_positions_.push_back({x, y, z});
                float lat = static_cast<float>(line.geo_line[0].x);
                float lon = static_cast<float>(line.geo_line[0].y);
                geo_border_positions_.push_back({lat, lon});
            }
            // RCLCPP_INFO(node->get_logger(), "Publishing border");
            rec->log_static("world/map/field/border",
                            rerun::Transform3D(rerun::components::Translation3D(.0, .0, .0),
                                               rerun::Quaternion::from_wxyz(1.0, 0.0, 0.0, 0.0)));
            auto border__ = rerun::components::LineStrip3D(loc_border_positions_);
            rec->log_static("world/map/field/border",
                            rerun::LineStrips3D(border__).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

            auto linestring = rerun::components::GeoLineString::from_lat_lon(geo_border_positions_);
            rec->log_static("world/map/field/border",
                            rerun::GeoLineStrings(linestring).with_colors({{0, 0, 255}}).with_radii({{0.2f}}));

            got_border_ = true;
        }
    }

    // void border_callback(const farmbot_interfaces::msg::Lines::SharedPtr msg) {
    //     if (!got_border_) {
    //         for (auto point : msg->polygon.points) {
    //             border_positions_.push_back({point.x, point.y, 0});
    //         }
    //     }
    //     // RCLCPP_INFO(node->get_logger(), "Publishing border");
    //     rec->log_static("world/map/field/border", rerun::Transform3D(rerun::components::Translation3D(.0, .0,
    //     .0),
    //                                                                  rerun::Quaternion::from_wxyz(1.0, 0.0, 0.0,
    //                                                                  0.0)));
    //     auto border__ = rerun::components::LineStrip3D(border_positions_);
    //     rec->log_static("world/map/field/border",
    //                     rerun::LineStrips3D(border__).with_colors({{158, 142, 158}}).with_radii({{0.2f}}));
    //     got_border_ = true;
    // }
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
