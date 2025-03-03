#include "rerun/archetypes/geo_line_strings.hpp"
#include "rerun/archetypes/geo_points.hpp"
#include "rerun/archetypes/line_strips2d.hpp"
#include "rerun/archetypes/line_strips3d.hpp"
#include "rerun/archetypes/points3d.hpp"
#include "rerun/archetypes/transform3d.hpp"
#include "rerun/components/geo_line_string.hpp"
#include "rerun/components/lat_lon.hpp"
#include "rerun/components/line_strip3d.hpp"
#include "rerun/recording_stream.hpp"
#include <cmath>
#include <cstdint>
#include <ctime>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <shared_mutex>
#include <std_msgs/msg/string.hpp>

#include <farmbot_interfaces/msg/polygon_array.hpp>
#include <farmbot_interfaces/msg/swaths.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include <vector>

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

    std::vector<std::array<float, 3>> border_positions_;
    bool got_border_ = false;
    std::vector<std::vector<std::array<float, 3>>> headland_positions_;
    bool got_headlands_ = false;
    std::vector<std::vector<std::array<float, 3>>> swaths_positions_;
    bool got_swaths_ = false;

    rclcpp::Subscription<farmbot_interfaces::msg::PolygonArray>::SharedPtr headlands_subscriber;
    rclcpp::Subscription<farmbot_interfaces::msg::Swaths>::SharedPtr swaths_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr border_subscriber;

    // timer publish
    rclcpp::TimerBase::SharedPtr timer;

  public:
    Beacon(rclcpp::Node::SharedPtr node) : node(node) {
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

        // timer publish
        timer = node->create_wall_timer(1s, std::bind(&Beacon::publish, this));

        headlands_subscriber = node->create_subscription<farmbot_interfaces::msg::PolygonArray>(
            "pln/headlands", 10, std::bind(&Beacon::headlands_callback, this, _1));

        swaths_subscriber = node->create_subscription<farmbot_interfaces::msg::Swaths>(
            "pln/swaths", 10, std::bind(&Beacon::swaths_callback, this, _1));

        border_subscriber = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "pln/border", 10, std::bind(&Beacon::border_callback, this, _1));
    }

    ~Beacon() { rclcpp::shutdown(); }

  private:
    void headlands_callback(const farmbot_interfaces::msg::PolygonArray::SharedPtr msg) {
        if (!got_headlands_) {
            for (auto polygon : msg->polygons) {
                std::vector<std::array<float, 3>> points;
                for (auto point : polygon.polygon.points) {
                    points.push_back({point.x, point.y, 0});
                }
                headland_positions_.push_back(points);
            }
        }
        RCLCPP_INFO(node->get_logger(), "Publishing headlands");
        for (auto points : headland_positions_) {
            auto headland__ = rerun::components::LineStrip3D(points);
            rec->log_static("world/map/" + namespace_ + "/field/headland",
                            rerun::LineStrips3D(headland__).with_colors({{0, 0, 255}}).with_radii({{0.1f}}));
        }
        got_headlands_ = true;
    }

    void swaths_callback(const farmbot_interfaces::msg::Swaths::SharedPtr msg) {
        // swaths_ = *msg;
        // got_swaths_ = true;
    }

    void border_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
        if (!got_border_) {
            for (auto point : msg->polygon.points) {
                border_positions_.push_back({point.x, point.y, 0});
            }
        }
        RCLCPP_INFO(node->get_logger(), "Publishing border");
        auto border__ = rerun::components::LineStrip3D(border_positions_);
        rec->log_static("world/map/" + namespace_ + "/field/border",
                        rerun::LineStrips3D(border__).with_colors({{0, 0, 255}}).with_radii({{0.1f}}));
        got_border_ = true;
    }

    void publish() {}
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
