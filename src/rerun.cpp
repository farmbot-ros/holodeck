#include "rerun/archetypes/geo_points.hpp"
#include "rerun/archetypes/points3d.hpp"
#include "rerun/archetypes/transform3d.hpp"
#include "rerun/recording_stream.hpp"
#include <rclcpp/publisher.hpp>
#include <shared_mutex>
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
        std::shared_ptr<rerun::RecordingStream> rec;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robo_pose;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pose;

        rerun::Color color = {
            static_cast<uint8_t>(rand() % 255),
            static_cast<uint8_t>(rand() % 255),
            static_cast<uint8_t>(rand() % 255)
        };

        // rclcpp::Subscription<std_msgs::msg::Str
        // rcl


   public:
        Beacon(rclcpp::Node::SharedPtr node) : node(node) {
            echo::info("Beacon created");


            namespace_ = node->get_namespace();
            if (!namespace_.empty() && namespace_[0] == '/') {
                namespace_ = namespace_.substr(1); // Remove leading slash
            }

            rec = std::make_shared<rerun::RecordingStream>("farmbot", "space");
            rec->disable_timeline("rec");



            // rec->log("world/map"+namespace_, rerun::Transform3D

            if(rec->spawn().is_err()){
                echo::error("Failed to spawn viewer");
            }


            robo_pose = node->create_subscription<nav_msgs::msg::Odometry>(
                "loc/odom", 10, std::bind(&Beacon::robo_pose_callback, this, _1));

        }

        ~Beacon() {
            rclcpp::shutdown();
        }

    private:

        void robo_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
            std::vector<rerun::Position3D> points;
            std::vector<rerun::Color> colors;

            echo::info("Robo pose callback");
            rerun::Position3D pos = {.0, .0, .0};
            points.push_back(pos);
            colors.push_back(color);

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
            auto lat = msg->latitude;
            auto lon = msg->latitude;

            rerun::LatLon lat_lon = {lat, lon};
            auto point = rerun::GeoPoints::from_lat_lon(lat_lon);

            // std::vector<rerun::GeoPoints> points;
            // points.push_back(point);

            // rec->log("gps", rerun::GeoPoints(points));
        }
};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    auto rec_ptr = std::make_shared<rerun::RecordingStream>("holodeck");
    // rec_ptr->connect_tcp("10.92.227.1:9876");

    if(rec_ptr->spawn().is_err()){
        echo::error("Failed to spawn viewer");
        return 1;
    }

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
