// rmw_zenoh's gid: XXH3-128 of a token key (ros_gid.hpp).
#include "xblick/ros_gid.hpp"

#include <cstring>

#define XXH_INLINE_ALL  // xxHash's functions compiled into this file only, none exported
#include "xxhash.h"

namespace xblick::ros
{
    Gid gid(std::string_view token_key)
    {
        const XXH128_hash_t hash = XXH3_128bits(token_key.data(), token_key.size());
        Gid out{};
        for (std::size_t i = 0; i < 8; ++i)  // little endian whatever the host's order, as rmw_zenoh on x86 and arm
        {
            out[i] = static_cast<std::uint8_t>(hash.low64 >> (8 * i));
            out[8 + i] = static_cast<std::uint8_t>(hash.high64 >> (8 * i));
        }
        return out;
    }
}  // namespace xblick::ros
