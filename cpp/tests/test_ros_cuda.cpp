// ros_cuda.hpp: the descriptor message against one cuda_buffer_backend 0.1.2 sent (ROS 2 Lyrical, rmw_zenoh_cpp
// 0.10.6, captured on <key>/_buf/<gid>; the same bytes as in tests/test_ros_cuda.py), the registry without one, and
// the link without a reader in a registry (backends part, no plan, no pool).
#include <cstdio>
#include <cstring>
#include <optional>
#include <stdexcept>

#include <unistd.h>
#include <string>
#include <vector>

#include "xblick/ros_cuda.hpp"

namespace
{
    std::vector<std::uint8_t> from_hex(const std::string& text)
    {
        std::vector<std::uint8_t> out;
        for (std::size_t i = 0; i + 1 < text.size(); i += 2)
        {
            out.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(i, 2), nullptr, 16)));
        }
        return out;
    }
}  // namespace

int main()
{
    const auto stock = from_hex(
        "00010000c8f4076bcbb52c040600000073303a31360000003000000040000000050000007267623800000000c0000000ffffffff"
        "32534f52050000006375646100000000002400000000000002000000680000000000000001000000f9453e000000000000002000"
        "000000001d0000002f746d702f637564615f766d6d5f343038313134355f302e736f636b0000000000000000d42f82926d34419e"
        "f9453e00000000000100000000000000ff0f00000000000000000000000000000000000000000000000000000000000000000000"
        "000000000000000000000000");
    xblick::ros_cuda::ImageHeader header{0x6B07F4C8, 0x042CB5CB, "s0:16", 48, 64, "rgb8", 0, 192};
    xblick::ros_cuda::Descriptor d;
    d.size = 9216;
    d.device_id = 0;
    d.pid = 4081145;
    d.block_id = 0;
    d.block_size = 2 * 1024 * 1024;
    d.socket_path = "/tmp/cuda_vmm_4081145_0.sock";
    std::memcpy(&d.uid, stock.data() + stock.size() - 72, 8);
    std::memcpy(d.event.data(), stock.data() + stock.size() - 64, 64);
    const auto payload = xblick::ros_cuda::image_payload(header, d);
    if (payload != stock)
    {
        std::printf("FAIL: the descriptor message differs from the stock publisher's (%zu bytes, %zu expected)\n",
                    payload.size(), stock.size());
        return 1;
    }
    if (xblick::ros_cuda::registry_lookup("/nonexistent-xblick-test", {}).has_value())
    {
        std::printf("FAIL: a registry where there is none\n");
        return 1;
    }
    namespace rc = xblick::ros_cuda;
    // the backends part and the plans without a registry entry: nothing goes to the buffer keys yet
    const auto backends = rc::backends_of("backends:cpu:;cuda:meta");
    if (backends.size() != 2 or backends.at("cpu") != "" or backends.at("cuda") != "meta")
    {
        std::printf("FAIL: backends_of\n");
        return 1;
    }
    try
    {
        rc::Link missing("/t/image_raw", 0, "/nonexistent-xblick-test", "/tmp");
        std::printf("FAIL: a link with a sockets directory that is none\n");
        return 1;
    }
    catch (const std::invalid_argument& e)
    {
        if (std::string(e.what()).find("XBLICK_ROS_CUDA_SOCKETS") == std::string::npos)
        {
            std::printf("FAIL: %s\n", e.what());
            return 1;
        }
    }
    char empty[] = "/tmp/xblick-test-link-XXXXXX";  // a directory without a registry, removed below
    if (::mkdtemp(empty) == nullptr)
    {
        std::printf("FAIL: mkdtemp\n");
        return 1;
    }
    std::optional<rc::Link> made;
    made.emplace("/t/image_raw", 0, empty, empty);
    rc::Link& link = *made;
    link.add("cpu", rc::Reader{{1}, std::string("backends:cpu:"), false, 1, "/cpu"});
    if (link.plan())
    {
        std::printf("FAIL: a plan without a reader that accepts CUDA buffers\n");
        return 1;
    }
    link.add("gpu", rc::Reader{{2}, std::string("backends:cuda:"), false, 10, "/gpu"});
    if (link.plan())  // not in the registry (none in that directory): the CPU form on the key
    {
        std::printf("FAIL: a plan for a reader that is in no registry\n");
        return 1;
    }
    if (link.remove("gpu") != std::array<std::uint8_t, 16>{2} or link.remove("gpu"))
    {
        std::printf("FAIL: remove\n");
        return 1;
    }
    const auto stats = link.stats();
    if (stats.sent != 0 or stats.blocks != 0)
    {
        std::printf("FAIL: a pool before the first GPU reader\n");
        return 1;
    }
    made.reset();
    ::rmdir(empty);
    std::printf("ok\n");
    return 0;
}
