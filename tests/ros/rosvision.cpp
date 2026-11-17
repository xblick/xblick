// An rclcpp node with typed vision_msgs subscriptions for tests/test_ros_live.py: rmw_zenoh_cpp deserialises each
// sample into the C++ message, so a sample that reaches the callback has the layout ROS expects. Built by the test
// with g++ against /opt/ros/jazzy (with ros-jazzy-vision-msgs), without ament.
//   rosvision 2d|3d <topic> <seconds>   prints one line per message, then "TOTAL <n>"
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::vector<std::string> a(argv + 1, argv + argc);
    if (a.size() < 3 || (a[0] != "2d" && a[0] != "3d")) {
        std::fprintf(stderr, "usage: see the file's head\n");
        rclcpp::shutdown();
        return 1;
    }
    auto node = std::make_shared<rclcpp::Node>("rosvision");
    int n = 0;
    rclcpp::SubscriptionBase::SharedPtr sub;
    if (a[0] == "2d") {
        sub = node->create_subscription<vision_msgs::msg::Detection2DArray>(
            a[1], 10, [&](const vision_msgs::msg::Detection2DArray& m) {
                std::printf("D2 %d frame=%s stamp=%d.%09u n=%zu", ++n, m.header.frame_id.c_str(), m.header.stamp.sec,
                            m.header.stamp.nanosec, m.detections.size());
                for (auto& d : m.detections) {
                    std::printf(" [id=%s", d.id.c_str());
                    for (auto& r : d.results)
                        std::printf(" %s:%.3f", r.hypothesis.class_id.c_str(), r.hypothesis.score);
                    std::printf(" box=%.2f,%.2f,%.2f,%.2f]", d.bbox.center.position.x, d.bbox.center.position.y,
                                d.bbox.size_x, d.bbox.size_y);
                }
                std::printf("\n");
                std::fflush(stdout);
            });
    } else {
        sub = node->create_subscription<vision_msgs::msg::Detection3DArray>(
            a[1], 10, [&](const vision_msgs::msg::Detection3DArray& m) {
                std::printf("D3 %d frame=%s stamp=%d.%09u n=%zu", ++n, m.header.frame_id.c_str(), m.header.stamp.sec,
                            m.header.stamp.nanosec, m.detections.size());
                for (auto& d : m.detections) {
                    std::printf(" [id=%s", d.id.c_str());
                    for (auto& r : d.results)
                        std::printf(" %s:%.3f cov0=%.3f", r.hypothesis.class_id.c_str(), r.hypothesis.score,
                                    r.pose.covariance[0]);
                    auto& b = d.bbox;
                    std::printf(" box=%.2f,%.2f,%.2f q=%.2f,%.2f,%.2f,%.2f size=%.2f,%.2f,%.2f]", b.center.position.x,
                                b.center.position.y, b.center.position.z, b.center.orientation.x,
                                b.center.orientation.y, b.center.orientation.z, b.center.orientation.w, b.size.x,
                                b.size.y, b.size.z);
                }
                std::printf("\n");
                std::fflush(stdout);
            });
    }
    auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::stod(a[2]));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(10ms);
    }
    std::printf("TOTAL %d\n", n);
    rclcpp::shutdown();
    return 0;
}
