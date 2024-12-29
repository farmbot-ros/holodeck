#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>


#include <spdlog/spdlog.h>
namespace echo = spdlog;

#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
using namespace rerun::demo;

class Beacon {
    private:
        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        // rcl


   public:
        Beacon(rclcpp::Node::SharedPtr node) : node(node) {
            echo::info("Beacon created");
        }

        void send_data(){
            echo::info("Sending data");
            const auto rec = rerun::RecordingStream("rerun_example_cpp");
            // Create some data using the `grid` utility function.
            std::vector<rerun::Position3D> points = grid3d<rerun::Position3D, float>(-10.f, 10.f, 10);
            std::vector<rerun::Color> colors = grid3d<rerun::Color, uint8_t>(0, 255, 10);
            // Log the "my_points" entity with our data, using the `Points3D` archetype.
            rec.log("my_points", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));
        }

        ~Beacon() {
            rclcpp::shutdown();
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
    parser.send_data();
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
