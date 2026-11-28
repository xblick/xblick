// ros_cuda.hpp: the descriptor message against one cuda_buffer_backend 0.1.2 sent (ROS 2 Lyrical, rmw_zenoh_cpp
// 0.10.6, captured on <key>/_buf/<gid>; the same bytes as in tests/test_ros_cuda.py), the registry without one, and
// the link without a reader in a registry (backends part, no plan, no pool). Without a GPU as well: a reader whose
// queue is deeper than the pool, a registry of another layout, a socket directory with too long a name, the registry
// a receiver makes as the backend makes it (its semaphore compared with one of glibc's sem_open), its close.
#include <fcntl.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "xblick/ring.hpp"
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

    std::vector<std::uint8_t> read_file(const std::string& path)
    {
        std::ifstream in(path, std::ios::binary);
        return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    }

    // the backend's registry (layout of version 1, as tests/test_ros_cuda.py's write_registry) with one subscription
    void write_registry(const std::string& path, const std::array<std::uint8_t, 16>& gid, std::uint32_t uid)
    {
        std::vector<std::uint8_t> data(272 + 512 * 296, 0);
        const std::uint32_t version = 1, max_entries = 512, count = 1;
        std::memcpy(data.data(), &version, 4);
        std::memcpy(data.data() + 260, &max_entries, 4);
        std::memcpy(data.data() + 264, &count, 4);
        const std::uint64_t instance = (static_cast<std::uint64_t>(::getpid()) << 32) | 1;
        std::memcpy(data.data() + 272, &instance, 8);
        std::memcpy(data.data() + 280, gid.data(), 16);
        data[272 + 24] = 1;  // a subscription
        std::memcpy(data.data() + 272 + 32, &uid, 4);
        data[272 + 36] = 1;  // active
        data[272 + 37] = 1;  // ipc capable
        std::ofstream(path, std::ios::binary).write(reinterpret_cast<const char*>(data.data()),
                                                    static_cast<std::streamsize>(data.size()));
    }

    bool any_note(const std::vector<xblick::ros_cuda::Note>& notes, const std::string& part)
    {
        for (const auto& n : notes)
        {
            if (n.text.find(part) != std::string::npos)
            {
                return true;
            }
        }
        return false;
    }

    int fail(const char* what)
    {
        std::printf("FAIL: %s\n", what);
        return 1;
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

    // the other direction: the stock message parsed back, the CPU form is none; a receiver writes nothing before a
    // publisher is decided, and refuses a descriptor of a process it did not decide
    const auto parsed = rc::parse_image_payload(stock.data(), stock.size());
    if (not parsed or parsed->header.frame_id != "s0:16" or parsed->header.step != 192 or
        parsed->descriptor.pid != d.pid or parsed->descriptor.block_size != d.block_size or
        parsed->descriptor.socket_path != d.socket_path or parsed->descriptor.uid != d.uid or
        parsed->descriptor.event != d.event)
    {
        std::printf("FAIL: the stock descriptor message parsed\n");
        return 1;
    }
    const std::vector<std::uint8_t> cpu_form = from_hex("0001000001000000020000000200000066000000010000000200000006"
                                                        "0000006d6f6e6f38000000020000000200000061620000");
    if (rc::parse_image_payload(cpu_form.data(), cpu_form.size()))
    {
        std::printf("FAIL: the CPU form parsed as a buffer\n");
        return 1;
    }
    try
    {
        rc::Receiver missing("/t/image_raw", 0, "/nonexistent-xblick-test", {});
        std::printf("FAIL: a receiver with a shm directory that is none\n");
        return 1;
    }
    catch (const std::invalid_argument&)
    {
    }
    const std::string registry = std::string(empty) + "/" + rc::registry_name();
    const std::string semaphore = std::string(empty) + "/sem." + rc::registry_name() + "_mtx";
    {
        // no ROS process with the backend has run (no registry): the receiver makes the registry and its semaphore
        // as the backend makes them, and enters itself before its token, as a stock subscription with no publisher
        rc::Receiver receiver("/t/image_raw", 0, empty, {3});
        receiver.add_publisher("pub", {4}, "/pub");
        receiver.enter();
        if (not receiver.stats().registered or not any_note(receiver.notes(), "made the CUDA buffer backend's "
                                                                              "registry"))
        {
            return fail("a receiver without a registry did not make one");
        }
        const auto entry = rc::registry_lookup(empty, {3});
        if (not entry or entry->entity_type != rc::kSubscription or entry->device_id != 0 or not entry->ipc_capable or
            entry->uid != ::getuid() or entry->pid() != ::getpid() or entry->name != "/t/image_raw")
        {
            return fail("the entry in the registry the receiver made");
        }
        const mode_t mask = ::umask(0);
        ::umask(mask);
        struct stat r{}, s{};
        if (::stat(registry.c_str(), &r) != 0 or r.st_size != 272 + 512 * 296 or (r.st_mode & 0777) != (0666 & ~mask) or
            ::stat(semaphore.c_str(), &s) != 0 or s.st_size != 32 or (s.st_mode & 0777) != (0666 & ~mask))
        {
            return fail("the registry's or the semaphore's size or mode is not the backend's");
        }
        const auto head = read_file(registry);
        std::uint32_t version = 0, max_entries = 0, count = 0;
        std::memcpy(&version, head.data(), 4);
        std::memcpy(&max_entries, head.data() + 260, 4);
        std::memcpy(&count, head.data() + 264, 4);
        char host[256] = {};
        ::gethostname(host, sizeof(host) - 1);
        if (version != 1 or max_entries != 512 or count != 1 or
            std::string(reinterpret_cast<const char*>(head.data() + 4)) != host)
        {
            return fail("the registry's head is not the backend's");
        }
        // the semaphore as glibc's sem_open (the backend's call) leaves it: one of its own, in /dev/shm, compared
        const std::string probe = "/xblick-test-sem-" + std::to_string(::getpid());
        sem_t* made = ::sem_open(probe.c_str(), O_CREAT | O_EXCL, 0666, 1);
        if (made == SEM_FAILED)
        {
            return fail("sem_open of the test's own semaphore");
        }
        const auto glibc = read_file("/dev/shm/sem." + probe.substr(1));
        ::sem_close(made);
        ::sem_unlink(probe.c_str());
        if (glibc.size() != 32 or read_file(semaphore) != glibc)
        {
            return fail("the semaphore differs from one of glibc's sem_open");
        }
        // a publisher not decided: its descriptors are refused; after close() nothing is taken (stale)
        if (receiver.take(parsed->descriptor) or receiver.stats().refused != 1)
        {
            return fail("a receiver without a decided publisher");
        }
        receiver.close();
        receiver.close();
        if (rc::registry_lookup(empty, {3}) or receiver.take(parsed->descriptor) or receiver.stats().stale != 1)
        {
            return fail("a closed receiver");
        }
    }
    {
        // a registry there already (version 1, an entry): kept as it is, the subscription entered beside it
        rc::Receiver receiver("/t/image_raw", 0, empty, {5});
        receiver.enter();
        if (not receiver.stats().registered or any_note(receiver.notes(), "made the") or
            not rc::registry_lookup(empty, {5}))
        {
            return fail("a receiver with a registry there");
        }
    }
    ::unlink(registry.c_str());
    ::unlink(semaphore.c_str());

    // a reader whose queue the pool cannot hold back (depth + 1 + spare blocks > max_blocks) gets the CPU form, said
    // once; and a pool never takes a reuse count it could not reach
    write_registry(registry, {6}, ::getuid());
    {
        rc::Link deep("/t/deep", 0, empty, empty);
        deep.add("deep", rc::Reader{{6}, std::string("backends:cuda:"), false, 100, "/deep"});
        if (deep.plan() or not any_note(deep.notes(), "keeps the last 100 messages, more than the 64 blocks"))
        {
            return fail("a reader with a queue deeper than the pool");
        }
        rc::ExportPool pool(0, empty, empty, 4);
        try
        {
            pool.acquire(1000, 4);
            return fail("a pool that takes a reuse count of its max_blocks");
        }
        catch (const xblick::RingError&)
        {
        }
    }
    // a registry of another layout (too short): every reader the CPU form, said once, and publishing goes on
    {
        std::ofstream(registry, std::ios::binary | std::ios::trunc).write("short", 5);
        rc::Link odd("/t/odd", 0, empty, empty);
        odd.add("gpu", rc::Reader{{6}, std::string("backends:cuda:"), false, 1, "/gpu"});
        odd.add("gpu2", rc::Reader{{7}, std::string("backends:cuda:"), false, 1, "/gpu2"});
        if (odd.plan() or odd.notes().size() != 1 or odd.plan() or not odd.notes().empty())
        {
            return fail("a registry of another layout");
        }
        std::ofstream(registry, std::ios::binary | std::ios::trunc);  // 0 bytes: being made, nobody yet
        if (rc::registry_lookup(empty, {6}))
        {
            return fail("a registry of 0 bytes");
        }
    }
    ::unlink(registry.c_str());
    // a socket directory whose sockets' names would not fit: said when the link or the pool is made
    {
        const std::string longer = std::string(empty) + "/" + std::string(80, 'd');
        ::mkdir(longer.c_str(), 0700);
        try
        {
            rc::Link link("/t/long", 0, longer, empty);
            return fail("a link with too long a socket directory");
        }
        catch (const std::invalid_argument& e)
        {
            if (std::string(e.what()).find("XBLICK_ROS_CUDA_SOCKETS") == std::string::npos)
            {
                return fail("the line about too long a socket directory");
            }
        }
        try
        {
            rc::ExportPool pool(0, longer, empty);
            return fail("a pool with too long a socket directory");
        }
        catch (const xblick::RingError&)
        {
        }
        ::rmdir(longer.c_str());
    }
    if (::rmdir(empty) != 0)
    {
        return fail("files left in the test's directory");
    }
    std::printf("ok\n");
    return 0;
}
