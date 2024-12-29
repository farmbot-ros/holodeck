#include "rerun/recording_stream.hpp"
#include <rclcpp/publisher.hpp>
#include <shared_mutex>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
        rclcpp::Node::SharedPtr node;
        std::shared_ptr<rerun::RecordingStream> rec;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robo_pose;

        // rclcpp::Subscription<std_msgs::msg::Str
        // rcl


   public:
        Beacon(rclcpp::Node::SharedPtr node, std::shared_ptr<rerun::RecordingStream> rec = nullptr) : node(node), rec(rec) {
            echo::info("Beacon created");

            robo_pose = node->create_subscription<nav_msgs::msg::Odometry>(
                "/robo0/loc/odom", 10, std::bind(&Beacon::robo_pose_callback, this, _1));


            send_data();
        }

        ~Beacon() {
            rclcpp::shutdown();
        }

    private:

        void robo_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
            echo::info("Robo pose callback");
            // std::lock_guard<std::shared_mutex> lock(mutex_);
            // if (publisher_->get_subscription_count() > 0) {
            //     auto message = std_msgs::msg::String();
            //     message.data = "Hello, world! " + std::to_string(count_++);
            //     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            //     publisher_->publish(message);
            // }
        }

        void send_data(){
            echo::info("Sending data");
            // Create some data using the `grid` utility function.
            std::vector<rerun::Position3D> points = grid3d<rerun::Position3D, float>(-10.f, 10.f, 10);
            std::vector<rerun::Color> colors = grid3d<rerun::Color, uint8_t>(0, 255, 10);
            // Log the "my_points" entity with our data, using the `Points3D` archetype.
            rec->log("my_points", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));
        }


};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    auto rec_ptr = std::make_shared<rerun::RecordingStream>("rerun_example_cpp");
    // rec_ptr->connect_tcp("10.92.227.1:9876");

    if(rec_ptr->spawn().is_err()){
        echo::error("Failed to spawn viewer");
        return 1;
    }

    rclcpp::NodeOptions options_0;
    options_0.allow_undeclared_parameters(true);
    options_0.automatically_declare_parameters_from_overrides(true);
    auto node_0 = rclcpp::Node::make_shared("rerun", options_0);
    auto parser = Beacon(node_0, rec_ptr);
    executor.add_node(node_0);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}

// #include <rerun.hpp>
// #include <rerun/demo_utils.hpp>

// using namespace rerun::demo;

// int main() {
//     // Create a new `RecordingStream` which sends data over TCP to the viewer process.
//     const auto rec = rerun::RecordingStream("rerun_example_cpp");
//     // Try to spawn a new viewer instance.
//     rec.spawn().exit_on_failure();

//     // Create some data using the `grid` utility function.
//     std::vector<rerun::Position3D> points = grid3d<rerun::Position3D, float>(-10.f, 10.f, 10);
//     std::vector<rerun::Color> colors = grid3d<rerun::Color, uint8_t>(0, 255, 10);

//     // Log the "my_points" entity with our data, using the `Points3D` archetype.
//     rec.log("my_points", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));
// }
