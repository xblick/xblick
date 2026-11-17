// The gid of a ROS 2 entity as rmw_zenoh_cpp 0.2.13 makes it: XXH3-128 of the entity's liveliness token key, its low
// 64 bits then its high 64 bits, each little endian (rmw_zenoh's liveliness_utils, Entity::gid). ROS writes it into the
// attachment of every sample, service request and reply (ros.hpp). XXH3 comes from the vendored xxHash 0.8.3
// (cpp/third_party/xxhash, BSD 2-Clause, licenses/xxhash-LICENSE). Part of xblick::core (no Zenoh), so that the Python
// module offers it too (_core.ros_gid).
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

namespace xblick::ros
{
    inline constexpr std::size_t kGidSize = 16;
    using Gid = std::array<std::uint8_t, kGidSize>;

    // The gid of the entity whose liveliness token key is `token_key`.
    Gid gid(std::string_view token_key);
}  // namespace xblick::ros
