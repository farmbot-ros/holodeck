#include "farmbot_interfaces/msg/beacon.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rerun/archetypes/ellipsoids3d.hpp"
#include "rerun/recording_stream.hpp"
#include <rerun.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class CapabilitiesNode {
  private:
    rclcpp::Node::SharedPtr node;
    std::shared_ptr<rerun::RecordingStream> rec;
    float counter = 0.;
    std::string namespace_;
    std::string tcp;

    rclcpp::Subscription<farmbot_interfaces::msg::Beacon>::SharedPtr beacon_sub;

  public:
    ~CapabilitiesNode() { rclcpp::shutdown(); }
    CapabilitiesNode(rclcpp::Node::SharedPtr node) : node(node) {
        namespace_ = node->get_namespace();
        if (!namespace_.empty() && namespace_[0] == '/') {
            namespace_ = namespace_.substr(1);
        }

        tcp = node->get_parameter_or<std::string>("tcp", "127.0.0.1:9876");

        beacon_sub = node->create_subscription<farmbot_interfaces::msg::Beacon>(
            "beacon/rci", 10, std::bind(&CapabilitiesNode::beacon_callback, this, _1));

        rec = std::make_shared<rerun::RecordingStream>("farmbot", "space");
        RCLCPP_INFO(node->get_logger(), "Connecting to %s", tcp.c_str());
        auto _one = rec->connect_tcp(tcp);

        if (_one.is_err()) {
            RCLCPP_ERROR(node->get_logger(), "Could not connect to %s", tcp.c_str());
            return;
        }
        if (rec->spawn().is_err()) {
            RCLCPP_WARN(node->get_logger(), "Could not spawn viewer");
        }
    }

    void beacon_callback(const farmbot_interfaces::msg::Beacon::SharedPtr msg) {
        RCLCPP_INFO(node->get_logger(), "Beacon callback");
        if (counter > 10.0) {
            counter = 0.;
        }
        counter = counter + 0.1;
        // auto ellipse = rerun::Ellipsoids3D::from_radii({{1.f, 1.f, 1.f}});
        rec->log_static("world/map/" + msg->name + "/beacon/" + std::to_string(counter),
                        rerun::Ellipsoids3D::from_radii({1.f}));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

    rclcpp::NodeOptions options_0;
    options_0.allow_undeclared_parameters(true);
    options_0.automatically_declare_parameters_from_overrides(true);
    auto node_0 = rclcpp::Node::make_shared("rerun", options_0);
    auto parser = CapabilitiesNode(node_0);
    executor.add_node(node_0);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
