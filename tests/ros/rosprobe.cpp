// A ROS 2 node on rmw_zenoh for tests/test_ros_live.py, standing in for ros2 topic and ros2 param (which need the
// ROS install's Python): built by the test with g++ against /opt/ros/jazzy, without ament.
//   rosprobe sub <topic> <type> <seconds> [transient]        prints "GOT <n> <bytes> <first 48 bytes hex>" per sample
//   rosprobe pub <topic> <type> <cdr hex> <count> [transient]  publishes the CDR bytes count times, 0.2 s apart
//   rosprobe params <node> list | get <name> | set <name> <int>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

static std::string hex(const uint8_t* p, size_t n)
{
    std::string s;
    char b[3];
    for (size_t i = 0; i < n; ++i) {
        std::snprintf(b, sizeof b, "%02x", p[i]);
        s += b;
    }
    return s;
}

static std::vector<uint8_t> unhex(const std::string& h)
{
    std::vector<uint8_t> v;
    for (size_t i = 0; i + 1 < h.size(); i += 2)
        v.push_back(static_cast<uint8_t>(std::stoi(h.substr(i, 2), nullptr, 16)));
    return v;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::vector<std::string> a(argv + 1, argv + argc);
    auto node = std::make_shared<rclcpp::Node>("rosprobe");
    int rc = 0;
    if (a.size() >= 4 && a[0] == "sub") {
        rclcpp::QoS qos(10);
        if (a.size() > 4 && a[4] == "transient") qos = rclcpp::QoS(100).transient_local();
        int n = 0;
        auto on = [&](std::shared_ptr<const rclcpp::SerializedMessage> m) {
            auto& r = m->get_rcl_serialized_message();
            std::printf("GOT %d %zu %s\n", ++n, r.buffer_length,
                        hex(r.buffer, std::min<size_t>(48, r.buffer_length)).c_str());
            std::fflush(stdout);
        };
        auto sub = node->create_generic_subscription(a[1], a[2], qos, on);
        auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::stod(a[3]));
        while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(10ms);
        }
        std::printf("TOTAL %d\n", n);
    } else if (a.size() >= 5 && a[0] == "pub") {
        rclcpp::QoS qos(10);
        if (a.size() > 5 && a[5] == "transient") qos = rclcpp::QoS(1).transient_local();
        auto pub = node->create_generic_publisher(a[1], a[2], qos);
        auto bytes = unhex(a[3]);
        std::this_thread::sleep_for(1s);
        for (int i = 0; i < std::stoi(a[4]); ++i) {
            rclcpp::SerializedMessage m(bytes.size());
            auto& r = m.get_rcl_serialized_message();
            std::copy(bytes.begin(), bytes.end(), r.buffer);
            r.buffer_length = bytes.size();
            pub->publish(m);
            std::this_thread::sleep_for(200ms);
        }
        std::printf("SENT %s\n", a[4].c_str());
        if (a.size() > 5 && a[5] == "transient")
            std::this_thread::sleep_for(std::chrono::seconds(30));  // stays for late joiners
    } else if (a.size() >= 3 && a[0] == "params") {
        auto client = std::make_shared<rclcpp::SyncParametersClient>(node, a[1]);
        if (!client->wait_for_service(10s)) {
            std::printf("NOSERVICE\n");
            rc = 2;
        } else if (a[2] == "list") {
            for (auto& name : client->list_parameters({}, 0).names) std::printf("NAME %s\n", name.c_str());
        } else if (a[2] == "get" && a.size() >= 4) {
            for (auto& p : client->get_parameters({a[3]}))
                std::printf("VALUE %s %s\n", p.get_name().c_str(), p.value_to_string().c_str());
        } else if (a[2] == "set" && a.size() >= 5) {
            for (auto& r : client->set_parameters({rclcpp::Parameter(a[3], std::stol(a[4]))}))
                std::printf("SET %d %s\n", r.successful ? 1 : 0, r.reason.c_str());
        }
    } else {
        std::fprintf(stderr, "usage: see the file's head\n");
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
