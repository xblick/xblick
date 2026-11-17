// ros_gid.hpp: the gid of a ROS entity against rmw_zenoh_cpp 0.2.13's own: the token of ROS 2 Jazzy's talker's
// /chatter publisher and the gid in the attachment of one of its samples (captured on domain 87, tests/test_ros.py).
#include <cstdio>
#include <string>

#include "xblick/ros_gid.hpp"

int main()
{
    const std::string token =
        "@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/11/MP/%/%/talker/%chatter/std_msgs::msg::dds_::String_/"
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,7:,:,:,,";
    const xblick::ros::Gid expected = {0x31, 0xb1, 0xbb, 0x10, 0xe2, 0xfd, 0xa6, 0x8e,
                                       0x3b, 0x70, 0x18, 0xe2, 0x16, 0x54, 0x07, 0x48};
    const xblick::ros::Gid got = xblick::ros::gid(token);
    if (got != expected)
    {
        std::printf("FAIL: the gid of the talker's publisher:");
        for (const auto b : got)
        {
            std::printf(" %02x", b);
        }
        std::printf("\n");
        return 1;
    }
    std::printf("ros gid: ok\n");
    return 0;
}
