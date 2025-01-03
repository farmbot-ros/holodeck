#include "rerun/archetypes/geo_points.hpp"
#include "rerun/archetypes/points3d.hpp"
#include "rerun/archetypes/transform3d.hpp"
#include "rerun/components/lat_lon.hpp"
#include "rerun/recording_stream.hpp"
#include <rclcpp/publisher.hpp>
#include <shared_mutex>
#include <random>
#include <ctime>
#include <cstdint>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <rclcpp/rclcpp.hpp>


#include <spdlog/spdlog.h>
namespace echo = spdlog;

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
        std::shared_ptr<rerun::RecordingStream> rec;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robo_pose;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pose;
        std::vector<uint8_t> color;

   public:
        Beacon(rclcpp::Node::SharedPtr node) : node(node) {
            echo::info("Beacon created");
            tcp = node->get_parameter_or<std::string>("tcp", "127.0.0.1:9876");

            namespace_ = node->get_namespace();
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_ = namespace_.substr(1);
            }

            rec = std::make_shared<rerun::RecordingStream>("farmbot", "space");
            auto _ = rec->connect_tcp(tcp);
            rec->disable_timeline("rec");

            std::mt19937 gen(rd());
            std::uniform_int_distribution<uint8_t> dis(0, 255);
            color = { dis(gen), dis(gen), dis(gen)};

            RCLCPP_INFO(node->get_logger(), "Color: %d %d %d", color[0], color[1], color[2]);

            if (rec->spawn().is_err()){
                echo::warn("Could not spawn viewer");
            }


            robo_pose = node->create_subscription<nav_msgs::msg::Odometry>(
                "loc/odom", 10, std::bind(&Beacon::robo_pose_callback, this, _1));

            gps_pose = node->create_subscription<sensor_msgs::msg::NavSatFix>(
                "gnss/fix", 10, std::bind(&Beacon::robo_gps_callback, this, _1));
        }

        ~Beacon() {
            rclcpp::shutdown();
        }

    private:
        void robo_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
            echo::info("Robo pose callback");
            // RCLCPP_INFO(node->get_logger(), "Robo pose callback");

            std::vector<rerun::Position3D> points;
            std::vector<rerun::Color> colors;
            points.push_back(rerun::Position3D(.0, .0, .0));
            colors.push_back(rerun::Color(color[0], color[1], color[2]));

            rec->log_static(
                "world/map/"+namespace_,
                rerun::Transform3D(
                    rerun::components::Translation3D(
                        msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z
                    ),
                    rerun::Quaternion::from_wxyz(
                        msg->pose.pose.orientation.w,
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z
                    )
                )
            );
            rec->log_static(
                "world/map/"+namespace_+"/loc",
                rerun::Points3D(points).with_colors(colors).with_radii({0.5f}))
            ;
        }

        void robo_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const {
            // RCLCPP_INFO(node->get_logger(), "Robo gps callback");
            echo::info("Robo gps callback");
            auto lat = msg->latitude;
            auto lon = msg->longitude;

            rec->log_static(
                namespace_+"/gps",
                rerun::GeoPoints::from_lat_lon({{lat, lon}})
                    .with_radii(rerun::Radius::ui_points(10.0f))
                    .with_colors(rerun::Color(color[0], color[1], color[2]))
            );


        }
};



int main(int argc, char** argv) {
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
