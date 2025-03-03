#include "rerun/archetypes/geo_line_strings.hpp"
#include "rerun/archetypes/geo_points.hpp"
#include "rerun/archetypes/line_strips3d.hpp"
#include "rerun/archetypes/points3d.hpp"
#include "rerun/archetypes/transform3d.hpp"
#include "rerun/components/geo_line_string.hpp"
#include "rerun/components/lat_lon.hpp"
#include "rerun/recording_stream.hpp"
#include <cmath>
#include <cstdint>
#include <ctime>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shared_mutex>
#include <std_msgs/msg/string.hpp>

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
using namespace rerun::demo;
using namespace std::chrono_literals;
using namespace std::placeholders;

class Beacon {
  private:
    std::string namespace_;
    rclcpp::Node::SharedPtr node;
    std::string tcp;
    std::random_device rd;
    int trajectory;
    std::shared_ptr<rerun::RecordingStream> rec;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robo_pose;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pose;
    std::vector<rerun::Color> colors;

    int counter_pos = 0;
    int counter_loc = 0;
    int every = 100;
    std::vector<std::array<float, 3>> positions;
    std::vector<rerun::LatLon> locations;

  public:
    Beacon(rclcpp::Node::SharedPtr node) : node(node) {
        // echo::info("Rerun for pose created");
        RCLCPP_INFO(node->get_logger(), "Rerun for pose created");
        tcp = node->get_parameter_or<std::string>("tcp", "127.0.0.1:9876");
        trajectory = node->get_parameter_or<int>("trajectory", 100);
        RCLCPP_INFO(node->get_logger(), "Trajectory length: %d", trajectory);

        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        rec = std::make_shared<rerun::RecordingStream>("farmbot", "space");
        RCLCPP_INFO(node->get_logger(), "Connecting to %s", tcp.c_str());
        auto _one = rec->connect_tcp(tcp);

        std::mt19937 gen(rd());
        std::uniform_int_distribution<uint8_t> dis(0, 255);
        colors.push_back(rerun::Color(dis(gen), dis(gen), dis(gen)));

        if (rec->spawn().is_err()) {
            // echo::warn("Could not spawn viewer");
            RCLCPP_WARN(node->get_logger(), "Could not spawn viewer");
        }

        robo_pose = node->create_subscription<nav_msgs::msg::Odometry>(
            "loc/odom", 10, std::bind(&Beacon::robo_pose_callback, this, _1));

        gps_pose = node->create_subscription<sensor_msgs::msg::NavSatFix>(
            "loc/fix", 10, std::bind(&Beacon::robo_gps_callback, this, _1));
    }

    ~Beacon() { rclcpp::shutdown(); }

  private:
    void robo_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        counter_pos++;
        // echo::info("Robo pose callback");
        RCLCPP_INFO(node->get_logger(), "Robo pose callback");
        auto x = float(msg->pose.pose.position.x);
        auto y = float(msg->pose.pose.position.y);
        auto z = float(msg->pose.pose.position.z);

        std::vector<rerun::Position3D> points;
        points.push_back(rerun::Position3D(x, y, z));

        rec->log_static("world/map/" + namespace_, rerun::Points3D(points).with_colors(colors).with_radii({2.f}));

        if (counter_pos % every == 0) {
            positions.push_back({x, y, z});
            auto linestrip = rerun::components::LineStrip3D(positions);
            rec->log_static("world/map/" + namespace_ + "/trajectory",
                            rerun::LineStrips3D(linestrip).with_colors({colors}));
        }
    }

    void robo_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        counter_loc++;
        // echo::info("Robo gps callback");
        RCLCPP_INFO(node->get_logger(), "Robo gps callback");
        auto lat = float(msg->latitude);
        auto lon = float(msg->longitude);

        std::vector<rerun::LatLon> locators;
        locators.push_back(rerun::LatLon(lat, lon));

        rec->log_static("world/map/" + namespace_,
                        rerun::GeoPoints::from_lat_lon(locators).with_colors(colors).with_radii({2.f}));

        // float delta = delta_distance(rerun::LatLon(lat, lon), locations.back());

        if (counter_loc % every == 0) {
            locations.push_back(locators.back());
            auto linesting = rerun::components::GeoLineString::from_lat_lon(locations);
            rec->log_static("world/map/" + namespace_ + "/trajectory",
                            rerun::GeoLineStrings(linesting).with_colors({colors}));
        }
    }

    float delta_distance(std::array<float, 3> pos1, std::array<float, 3> pos2) {
        auto dx = pos1[0] - pos2[0];
        auto dy = pos1[1] - pos2[1];
        auto dz = pos1[2] - pos2[2];
        auto distance = sqrt(dx * dx + dy * dy + dz * dz);
        // echo::info("Distance: %f", distance);
        RCLCPP_INFO(node->get_logger(), "Distance: %f", distance);

        return distance;
    }

    float delta_distance(rerun::LatLon loc1, rerun::LatLon loc2) {
        auto dx = loc1.latitude() - loc2.latitude();
        auto dy = loc1.longitude() - loc2.longitude();
        auto distance = sqrt(dx * dx + dy * dy);
        // echo::info("Distance: %f", distance);
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
    auto parser = Beacon(node_0);
    executor.add_node(node_0);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
