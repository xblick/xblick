#include "xblick/ros_cuda.hpp"

#include <cuda.h>
#include <dirent.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <tuple>

#include "xblick/cuda_ring.hpp"
#include "xblick/ring.hpp"
#include "xblick/ros2_cdr_msgs.hpp"

namespace xblick::ros_cuda
{
    namespace
    {
        // ------------------------------------------------------------------------------------- driver API
        // The functions the pool needs, looked up as the CUDA ring looks up its own (cuda_ring.cpp): libcuda.so.1 at
        // run time, CUDA 12.0 ABI. A table of its own keeps the ring's file as it is.

        constexpr int kDriverAbi = 12000;

#define XBLICK_POOL_FUNCTIONS(X)                                                                                    \
    X(cuInit)                                                                                                        \
    X(cuGetErrorName)                                                                                                \
    X(cuDeviceGet)                                                                                                   \
    X(cuDeviceGetAttribute)                                                                                          \
    X(cuDevicePrimaryCtxRetain)                                                                                      \
    X(cuDevicePrimaryCtxRelease)                                                                                     \
    X(cuCtxPushCurrent)                                                                                              \
    X(cuCtxPopCurrent)                                                                                               \
    X(cuMemGetAllocationGranularity)                                                                                 \
    X(cuMemCreate)                                                                                                   \
    X(cuMemRelease)                                                                                                  \
    X(cuMemAddressReserve)                                                                                           \
    X(cuMemAddressFree)                                                                                              \
    X(cuMemMap)                                                                                                      \
    X(cuMemUnmap)                                                                                                    \
    X(cuMemSetAccess)                                                                                                \
    X(cuMemExportToShareableHandle)

        struct Driver
        {
#define XBLICK_MEMBER(name) decltype(&::name) name = nullptr;
            XBLICK_POOL_FUNCTIONS(XBLICK_MEMBER)
#undef XBLICK_MEMBER
            std::string error;
        };

        Driver load_driver()
        {
            Driver d;
            void* lib = ::dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
            if (lib == nullptr)
            {
                d.error = std::string("cannot load the CUDA driver libcuda.so.1: ") + ::dlerror();
                return d;
            }
            using GetProcAddress = CUresult (*)(const char*, void**, int, cuuint64_t, CUdriverProcAddressQueryResult*);
            auto get = reinterpret_cast<GetProcAddress>(::dlsym(lib, "cuGetProcAddress_v2"));
            if (get == nullptr)
            {
                d.error = "the CUDA driver has no cuGetProcAddress_v2 (needs a driver for CUDA 12 or newer)";
                return d;
            }
#define XBLICK_LOOKUP(name)                                                                                           \
    {                                                                                                                 \
        void* fn = nullptr;                                                                                           \
        CUdriverProcAddressQueryResult status{};                                                                      \
        if (get(#name, &fn, kDriverAbi, CU_GET_PROC_ADDRESS_DEFAULT, &status) != CUDA_SUCCESS or fn == nullptr)       \
        {                                                                                                             \
            d.error = "the CUDA driver lacks " #name;                                                                 \
            return d;                                                                                                 \
        }                                                                                                             \
        d.name = reinterpret_cast<decltype(d.name)>(fn);                                                              \
    }
            XBLICK_POOL_FUNCTIONS(XBLICK_LOOKUP)
#undef XBLICK_LOOKUP
            if (d.cuInit(0) != CUDA_SUCCESS)
            {
                d = Driver{};
                d.error = "cuInit failed (no usable CUDA device?)";
            }
            return d;
        }

        const Driver& driver()
        {
            static const Driver d = load_driver();
            if (not d.error.empty())
            {
                throw RingError(d.error);
            }
            return d;
        }

        void check(CUresult result, const char* what)
        {
            if (result == CUDA_SUCCESS)
            {
                return;
            }
            const char* name = nullptr;
            driver().cuGetErrorName(result, &name);
            throw RingError(std::string(what) + ": " + (name ? name : "?"));
        }

#define CU(call) check(driver().call, #call)

        class ContextGuard
        {
        public:
            explicit ContextGuard(CUcontext context) { CU(cuCtxPushCurrent(context)); }
            ~ContextGuard()
            {
                CUcontext popped = nullptr;
                driver().cuCtxPopCurrent(&popped);
            }
        };

        std::string errno_text() { return std::strerror(errno); }

        // ------------------------------------------------------------------------------------- the registry
        // host_endpoint_manager.cpp of the backend: EndpointRegistry { uint32 version; char hostname[256]; uint32
        // max_entries; uint32 num_entries; EndpointEntry entries[512]; }, the entry as below (x86-64 and arm64 ABI).

        struct EndpointEntry
        {
            std::uint64_t instance_id;
            std::uint8_t gid[16];
            std::uint8_t entity_type;
            std::int32_t device_id;
            std::uint32_t uid;
            bool active;
            bool ipc_capable;
            std::uint8_t padding[2];
            char name[256];
        };
        constexpr std::size_t kMaxEndpoints = 512;
        struct EndpointRegistry
        {
            std::uint32_t version;
            char hostname[256];
            std::uint32_t max_entries;
            std::uint32_t num_entries;
            EndpointEntry entries[kMaxEndpoints];
        };
        static_assert(sizeof(EndpointEntry) == 296 and offsetof(EndpointEntry, device_id) == 28 and
                      offsetof(EndpointEntry, active) == 36 and offsetof(EndpointEntry, name) == 40);
        static_assert(offsetof(EndpointRegistry, entries) == 272 and sizeof(EndpointRegistry) == 272 + 512 * 296);

        std::uint32_t hash_string(const std::string& text)  // the backend's, for the registry's name
        {
            std::uint32_t hash = 0;
            for (char c : text)
            {
                hash = hash * 31 + static_cast<std::uint32_t>(c);
            }
            return hash;
        }

        std::string hostname()
        {
            char buffer[256] = {};
            if (::gethostname(buffer, sizeof(buffer) - 1) != 0)
            {
                throw RingError("gethostname: " + errno_text());
            }
            return buffer;
        }

        // ------------------------------------------------------------------------------------- files
        // A pool lists what it created in <socket_dir>/xblick_cuda_<pid>.files (its boot id, pid namespace and start
        // time first), so that a later process can remove what a crashed one left, and nothing else.

        constexpr const char* kFilePrefix = "xblick_cuda_";
        constexpr const char* kListSuffix = ".files";

        std::string pid_namespace()
        {
            char target[64] = {};
            const ssize_t n = ::readlink("/proc/self/ns/pid", target, sizeof(target) - 1);
            return n > 0 ? std::string(target, static_cast<std::size_t>(n)) : std::string();
        }

        bool alive_as(int pid, std::uint64_t start_time)
        {
            try
            {
                return process_start_time(pid) == start_time;
            }
            catch (const RingError&)
            {
                return false;  // no such process
            }
        }

        // the file at path, if it is of this user and of the given type (S_IFSOCK, S_IFREG)
        bool ours(const std::string& path, mode_t type)
        {
            struct stat s{};
            return ::lstat(path.c_str(), &s) == 0 and s.st_uid == ::getuid() and (s.st_mode & S_IFMT) == type;
        }

        void remove_stale(const std::string& socket_dir)
        {
            DIR* dir = ::opendir(socket_dir.c_str());
            if (dir == nullptr)
            {
                return;
            }
            const std::string boot = host_id(), ns = pid_namespace();
            std::vector<std::string> lists;
            while (dirent* e = ::readdir(dir))
            {
                const std::string name = e->d_name;
                if (name.rfind(kFilePrefix, 0) == 0 and name.size() > std::strlen(kListSuffix) and
                    name.compare(name.size() - std::strlen(kListSuffix), std::string::npos, kListSuffix) == 0)
                {
                    lists.push_back(socket_dir + "/" + name);
                }
            }
            ::closedir(dir);
            for (const std::string& list : lists)
            {
                if (not ours(list, S_IFREG))
                {
                    continue;
                }
                std::ifstream in(list);
                std::string their_boot, their_ns, line;
                int pid = 0;
                std::uint64_t start = 0;
                if (not(in >> their_boot >> their_ns >> pid >> start) or pid == ::getpid())
                {
                    continue;  // not one of ours, or being written
                }
                // gone for certain: an earlier boot, or this boot and namespace with the pid no longer that process
                const bool gone = their_boot != boot or (their_ns == ns and not alive_as(pid, start));
                if (not gone)
                {
                    continue;
                }
                const std::string record_prefix = "cuda_vmm_" + std::to_string(pid) + "_";
                while (std::getline(in, line))
                {
                    std::istringstream fields(line);
                    std::string kind, path;
                    if (not(fields >> kind >> path))
                    {
                        continue;
                    }
                    const std::string base = path.substr(path.rfind('/') + 1);
                    if (kind == "socket" and base.rfind(kFilePrefix + std::to_string(pid) + "_", 0) == 0 and
                        ours(path, S_IFSOCK))
                    {
                        ::unlink(path.c_str());
                    }
                    else if (kind == "record" and base.rfind(record_prefix, 0) == 0 and ours(path, S_IFREG))
                    {
                        ::unlink(path.c_str());
                    }
                }
                ::unlink(list.c_str());
            }
        }

        std::size_t round_up(std::size_t value, std::size_t multiple)
        {
            return (value + multiple - 1) / multiple * multiple;
        }

        std::uint32_t next_block_id()  // unique in the process: readers never forget a block id
        {
            static std::atomic<std::uint32_t> next{0};
            return next.fetch_add(1);
        }

        sockaddr_un address_of(const std::string& name, bool abstract, socklen_t* length)
        {
            sockaddr_un address{};
            address.sun_family = AF_UNIX;
            if (name.size() + 1 > sizeof(address.sun_path))
            {
                throw RingError("socket name too long: " + name);
            }
            const std::size_t at = abstract ? 1 : 0;  // abstract: a NUL byte first, no file
            std::memcpy(address.sun_path + at, name.data(), name.size());
            *length = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + at + name.size() + (abstract ? 0 : 1));
            return address;
        }

        int listen_on(const std::string& name, bool abstract)
        {
            const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
            if (fd < 0)
            {
                throw RingError("socket: " + errno_text());
            }
            socklen_t length = 0;
            const sockaddr_un address = address_of(name, abstract, &length);
            if (::bind(fd, reinterpret_cast<const sockaddr*>(&address), length) != 0 or ::listen(fd, 16) != 0)
            {
                const std::string why = errno_text();
                ::close(fd);
                throw RingError("cannot listen on " + std::string(abstract ? "@" : "") + name + ": " + why);
            }
            if (not abstract and ::chmod(name.c_str(), 0600) != 0)  // peers are checked anyway (SO_PEERCRED)
            {
                const std::string why = errno_text();
                ::close(fd);
                ::unlink(name.c_str());
                throw RingError("chmod " + name + ": " + why);
            }
            return fd;
        }

        std::uint64_t now_us()
        {
            return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        }
    }  // namespace

    // ----------------------------------------------------------------------------------------------- payload

    std::vector<std::uint8_t> image_payload(const ImageHeader& h, const Descriptor& d)
    {
        ros2_cdr::CdrWriter w;
        w.reserve(256 + h.frame_id.size() + d.socket_path.size());
        w.i32(h.sec);
        w.u32(h.nanosec);
        w.string(h.frame_id);
        w.u32(h.height);
        w.u32(h.width);
        w.string(h.encoding);
        w.u8(h.is_bigendian);
        w.u32(h.step);
        w.u32(kMarker);
        w.u32(kMarkerRos2);
        w.string(kBackend);
        w.u64(d.size);
        w.string(kElementType);
        w.i32(d.device_id);
        w.u8(d.use_ipc ? 1 : 0);
        w.i32(d.pid);
        w.u32(d.block_id);
        w.u64(d.block_size);
        w.string(d.socket_path);
        w.u64(d.uid);
        for (std::uint8_t b : d.event)
        {
            w.u8(b);
        }
        return w.take();
    }

    // ---------------------------------------------------------------------------------------------- registry

    std::string registry_name()
    {
        return "ros2_hem_d0_h" + std::to_string(hash_string(hostname()));
    }

    std::optional<RegistryEntry> registry_lookup(const std::string& shm_dir, const std::array<std::uint8_t, 16>& gid)
    {
        const std::string path = shm_dir + "/" + registry_name();
        const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (fd < 0)
        {
            return std::nullopt;  // no ROS process with the backend on this host (in this /dev/shm)
        }
        struct stat s{};
        if (::fstat(fd, &s) != 0 or static_cast<std::size_t>(s.st_size) < sizeof(EndpointRegistry))
        {
            ::close(fd);
            throw RingError(path + ": " + std::to_string(s.st_size) + " bytes, not the " +
                            std::to_string(sizeof(EndpointRegistry)) + " of the CUDA buffer backend's registry "
                            "(version 1): another backend version");
        }
        void* map = ::mmap(nullptr, sizeof(EndpointRegistry), PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (map == MAP_FAILED)
        {
            throw RingError("mmap " + path + ": " + errno_text());
        }
        const auto* registry = static_cast<const EndpointRegistry*>(map);
        std::optional<RegistryEntry> found;
        const std::uint32_t version = registry->version;
        const std::uint32_t count = std::min<std::uint32_t>(registry->num_entries, kMaxEndpoints);
        if (version != 1)
        {
            ::munmap(map, sizeof(EndpointRegistry));
            if (version == 0)
            {
                return std::nullopt;  // being made
            }
            throw RingError(path + ": registry version " + std::to_string(version) + ", this program knows 1");
        }
        for (std::uint32_t i = 0; i < count; ++i)
        {
            const EndpointEntry& e = registry->entries[i];
            if (std::memcmp(e.gid, gid.data(), 16) != 0 or not e.active)
            {
                continue;
            }
            RegistryEntry r;
            r.instance_id = e.instance_id;
            std::memcpy(r.gid.data(), e.gid, 16);
            r.entity_type = e.entity_type;
            r.device_id = e.device_id;
            r.uid = e.uid;
            r.active = e.active;
            r.ipc_capable = e.ipc_capable;
            r.name.assign(e.name, ::strnlen(e.name, sizeof(e.name)));
            found = r;
            break;
        }
        ::munmap(map, sizeof(EndpointRegistry));
        return found;
    }

    // ---------------------------------------------------------------------------------------------- the pool

    namespace
    {
        // IPCMetadata of the backend (cuda_memory_pool.hpp): 24 bytes
        struct Record
        {
            std::atomic<std::int32_t> refcount;
            std::atomic<std::uint64_t> uid;
            std::atomic<std::uint64_t> publish_us;
        };
        static_assert(sizeof(Record) == 24 and offsetof(Record, uid) == 8 and offsetof(Record, publish_us) == 16);
        static_assert(std::atomic<std::int32_t>::is_always_lock_free and
                      std::atomic<std::uint64_t>::is_always_lock_free);

        struct Block
        {
            std::uint32_t id = 0;
            std::size_t size = 0;
            CUmemGenericAllocationHandle allocation{};
            CUdeviceptr base = 0;
            int fd = -1;               // the exported handle, handed to readers
            std::string socket_path;   // file socket and abstract socket of this name
            int file_listen = -1;
            int abstract_listen = -1;
            std::string record_path;
            Record* record = nullptr;
            std::uint64_t published_us = 0;  // our clock: when it was last published (0: never)
            std::uint64_t published_frame = 0;  // the pool's publish count then
            std::uint64_t uid = 0;           // the uid in its record now (0 while written)
            bool retired = false;            // of an older frame size: freed once nobody holds it
        };
    }  // namespace

    struct ExportPool::Impl
    {
        int device;
        std::string socket_dir, shm_dir;
        std::uint32_t max_blocks;
        std::uint64_t hold_limit_us;
        std::uint64_t min_reuse_us;
        std::uint64_t frames = 0;  // publishes so far
        CUdevice cu_device{};
        CUcontext context{};
        bool retained = false;
        std::size_t granularity = 0;
        CUmemAllocationProp prop{};
        std::string list_path;
        std::string list_head;
        std::mt19937_64 random{std::random_device{}()};

        std::mutex mutex;  // blocks, between the pool and its server thread
        std::vector<std::unique_ptr<Block>> blocks;
        std::atomic<std::uint64_t> dropped{0}, expired{0};
        std::atomic<std::uint32_t> accepts{0}, refused{0};
        std::atomic<bool> stop{false};
        std::thread server;
        // file descriptors of released blocks (listening sockets, exported handles): closed by the server thread
        // after its poll, so that their numbers are not taken by another file while it may still use them
        std::vector<int> closing;
        int wake[2] = {-1, -1};  // a pipe: a new block's sockets are polled at once

        // the list of files, with those of extra (a block being made: listed before its files exist, so that a
        // crash leaves nothing unlisted)
        void write_list(const Block* extra = nullptr)
        {
            const std::string temporary = list_path + ".tmp";
            {
                std::ofstream out(temporary, std::ios::trunc);
                out << list_head;
                for (const auto& b : blocks)
                {
                    out << "socket " << b->socket_path << "\nrecord " << b->record_path << "\n";
                }
                if (extra != nullptr)
                {
                    out << "socket " << extra->socket_path << "\nrecord " << extra->record_path << "\n";
                }
                if (not out)
                {
                    throw RingError("cannot write " + temporary);
                }
            }
            if (::rename(temporary.c_str(), list_path.c_str()) != 0)
            {
                throw RingError("rename " + temporary + ": " + errno_text());
            }
        }

        void start()
        {
            if (retained)
            {
                return;
            }
            const Driver& d = driver();
            (void)d;
            CU(cuDeviceGet(&cu_device, device));
            int vmm = 0, posix_fd = 0;
            CU(cuDeviceGetAttribute(&vmm, CU_DEVICE_ATTRIBUTE_VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED, cu_device));
            CU(cuDeviceGetAttribute(&posix_fd, CU_DEVICE_ATTRIBUTE_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR_SUPPORTED,
                                    cu_device));
            if (not vmm or not posix_fd)
            {
                throw RingError("CUDA device " + std::to_string(device) + " does not support virtual memory "
                                "management with POSIX file descriptors");
            }
            CU(cuDevicePrimaryCtxRetain(&context, cu_device));
            retained = true;
            prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
            prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
            prop.location.id = cu_device;
            prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
            CU(cuMemGetAllocationGranularity(&granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM));
            if (::pipe2(wake, O_CLOEXEC | O_NONBLOCK) != 0)
            {
                throw RingError("pipe: " + errno_text());
            }
            std::ostringstream head;
            head << host_id() << "\n" << pid_namespace() << "\n" << ::getpid() << "\n"
                 << process_start_time(::getpid()) << "\n";
            list_head = head.str();
            list_path = socket_dir + "/" + kFilePrefix + std::to_string(::getpid()) + kListSuffix;
            write_list();
            server = std::thread([this] { serve(); });
        }

        Block& make(std::size_t size)
        {
            auto b = std::make_unique<Block>();
            b->id = next_block_id();
            b->size = size;
            const std::string pid = std::to_string(::getpid());
            b->socket_path = socket_dir + "/" + kFilePrefix + pid + "_" + std::to_string(b->id) + ".sock";
            b->record_path = shm_dir + "/cuda_vmm_" + pid + "_" + std::to_string(b->id);
            Block& block = *b;
            try
            {
                write_list(&block);
                ContextGuard guard(context);
                CU(cuMemCreate(&block.allocation, size, &prop, 0));
                CU(cuMemAddressReserve(&block.base, size, granularity, 0, 0));
                CU(cuMemMap(block.base, size, 0, block.allocation, 0));
                CUmemAccessDesc access{};
                access.location = prop.location;
                access.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
                CU(cuMemSetAccess(block.base, size, &access, 1));
                CU(cuMemExportToShareableHandle(&block.fd, block.allocation, CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR,
                                                0));
                const int rfd = ::open(block.record_path.c_str(), O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0600);
                if (rfd < 0)
                {
                    throw RingError("cannot create " + block.record_path + ": " + errno_text() +
                                    (errno == EEXIST ? " (another process of this pid number writes there: a "
                                                       "container with a pid namespace of its own and this /dev/shm?)"
                                                     : ""));
                }
                const bool sized = ::ftruncate(rfd, sizeof(Record)) == 0;
                void* map = sized ? ::mmap(nullptr, sizeof(Record), PROT_READ | PROT_WRITE, MAP_SHARED, rfd, 0)
                                  : MAP_FAILED;
                ::close(rfd);
                if (map == MAP_FAILED)
                {
                    const std::string why = errno_text();
                    ::unlink(block.record_path.c_str());
                    throw RingError("cannot map " + block.record_path + ": " + why);
                }
                block.record = static_cast<Record*>(map);  // zeros: count 0, uid 0
                block.file_listen = listen_on(block.socket_path, false);
                block.abstract_listen = listen_on(block.socket_path, true);
            }
            catch (...)
            {
                release(block);
                write_list();
                throw;
            }
            {
                std::lock_guard lock(mutex);  // the server thread polls its sockets from now on
                blocks.push_back(std::move(b));
            }
            const char one = 1;
            [[maybe_unused]] auto ignored = ::write(wake[1], &one, 1);
            return block;
        }

        void close_later(int& fd)
        {
            if (fd >= 0)
            {
                if (server.joinable())
                {
                    closing.push_back(fd);
                }
                else
                {
                    ::close(fd);
                }
                fd = -1;
            }
        }

        // frees a block and removes its files (with the mutex held)
        void release(Block& b)
        {
            if (b.file_listen >= 0)
            {
                ::unlink(b.socket_path.c_str());
            }
            close_later(b.file_listen);
            close_later(b.abstract_listen);
            if (b.record != nullptr)
            {
                ::munmap(b.record, sizeof(Record));
                b.record = nullptr;
                ::unlink(b.record_path.c_str());
            }
            close_later(b.fd);
            if (b.base != 0)
            {
                ContextGuard guard(context);
                driver().cuMemUnmap(b.base, b.size);
                driver().cuMemAddressFree(b.base, b.size);
                b.base = 0;
            }
            if (b.allocation != 0)
            {
                driver().cuMemRelease(b.allocation);  // readers that imported it keep the memory until they unmap
                b.allocation = 0;
            }
        }

        // the block may be written: nobody holds it (uid set to 0 first, so that a reader that takes a count
        // meanwhile finds the uid changed and drops its message) and no reader's queue can hold a message of it
        // (reuse_frames publishes or more, and min_reuse_us, since its own), or its hold is older than the limit
        bool take(Block& b, std::uint64_t now, std::uint64_t reuse_frames)
        {
            if (b.record->refcount.load() > 0)
            {
                if (b.published_us == 0 or now - b.published_us < hold_limit_us)
                {
                    return false;
                }
                b.record->uid.store(0);
                b.uid = 0;
                expired.fetch_add(1);
                return true;
            }
            if (b.published_us != 0 and (frames - b.published_frame < reuse_frames or
                                         now - b.published_us < min_reuse_us))
            {
                return false;
            }
            const std::uint64_t before = b.record->uid.load();
            b.record->uid.store(0);
            if (b.record->refcount.load() > 0)  // a reader came in before the store: it may read the old frame
            {
                b.record->uid.store(before);
                return false;
            }
            b.uid = 0;
            return true;
        }

        void serve()
        {
            while (not stop.load())
            {
                std::vector<pollfd> polled{{wake[0], POLLIN, 0}};
                std::vector<int> fds{-1};
                {
                    std::lock_guard lock(mutex);
                    for (const auto& b : blocks)
                    {
                        if (b == nullptr)
                        {
                            continue;
                        }
                        for (int listen_fd : {b->file_listen, b->abstract_listen})
                        {
                            if (listen_fd >= 0)
                            {
                                polled.push_back({listen_fd, POLLIN, 0});
                                fds.push_back(b->fd);
                            }
                        }
                    }
                }
                if (::poll(polled.data(), polled.size(), 200) <= 0)
                {
                    for (pollfd& p : polled)
                    {
                        p.revents = 0;  // nothing to accept; the deferred closes below still run
                    }
                }
                if (polled[0].revents & POLLIN)
                {
                    char drain[16];
                    while (::read(wake[0], drain, sizeof(drain)) > 0)
                    {
                    }
                }
                std::lock_guard lock(mutex);  // a block is not released while its fd is sent
                for (std::size_t i = 1; i < polled.size(); ++i)
                {
                    if (not(polled[i].revents & POLLIN) or
                        std::find(closing.begin(), closing.end(), polled[i].fd) != closing.end())
                    {
                        continue;  // released meanwhile
                    }
                    const int connection = ::accept4(polled[i].fd, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
                    if (connection < 0)
                    {
                        continue;
                    }
                    ucred peer{};
                    socklen_t length = sizeof(peer);
                    if (::getsockopt(connection, SOL_SOCKET, SO_PEERCRED, &peer, &length) == 0 and
                        peer.uid == ::getuid())
                    {
                        char byte = 'X';
                        iovec io{&byte, 1};
                        alignas(cmsghdr) char control[CMSG_SPACE(sizeof(int))] = {};
                        msghdr message{};
                        message.msg_iov = &io;
                        message.msg_iovlen = 1;
                        message.msg_control = control;
                        message.msg_controllen = sizeof(control);
                        cmsghdr* c = CMSG_FIRSTHDR(&message);
                        c->cmsg_level = SOL_SOCKET;
                        c->cmsg_type = SCM_RIGHTS;
                        c->cmsg_len = CMSG_LEN(sizeof(int));
                        std::memcpy(CMSG_DATA(c), &fds[i], sizeof(int));
                        if (::sendmsg(connection, &message, MSG_NOSIGNAL) == 1)
                        {
                            accepts.fetch_add(1);
                        }
                    }
                    else
                    {
                        refused.fetch_add(1);  // a peer of another user gets nothing
                    }
                    ::close(connection);
                }
                for (int fd : closing)
                {
                    ::close(fd);
                }
                closing.clear();
            }
        }
    };

    ExportPool::ExportPool(int device, std::string socket_dir, std::string shm_dir, std::uint32_t max_blocks,
                           double hold_limit_s, double min_reuse_s)
        : impl_(std::make_unique<Impl>())
    {
        if (max_blocks == 0)
        {
            throw RingError("an export pool needs at least one block");
        }
        if (not(hold_limit_s > 0) or not(min_reuse_s >= 0))
        {
            throw RingError("the hold limit of an export pool is a time > 0 s, its minimum reuse time >= 0 s");
        }
        for (const std::string* dir : {&socket_dir, &shm_dir})
        {
            struct stat s{};
            if (::stat(dir->c_str(), &s) != 0 or not S_ISDIR(s.st_mode))
            {
                throw RingError(*dir + ": no such directory (the export pool's sockets and records go there)");
            }
        }
        Impl& m = *impl_;
        m.device = device;
        m.socket_dir = std::move(socket_dir);
        m.shm_dir = std::move(shm_dir);
        m.max_blocks = max_blocks;
        m.hold_limit_us = static_cast<std::uint64_t>(hold_limit_s * 1e6);
        m.min_reuse_us = static_cast<std::uint64_t>(min_reuse_s * 1e6);
        remove_stale(m.socket_dir);
    }

    ExportPool::~ExportPool()
    {
        Impl& m = *impl_;
        m.stop.store(true);
        if (m.server.joinable())
        {
            m.server.join();
        }
        {
            std::lock_guard lock(m.mutex);
            for (auto& b : m.blocks)
            {
                m.release(*b);
            }
            m.blocks.clear();
            for (int fd : m.closing)
            {
                ::close(fd);
            }
            m.closing.clear();
        }
        for (int fd : m.wake)
        {
            if (fd >= 0)
            {
                ::close(fd);
            }
        }
        if (not m.list_path.empty())
        {
            ::unlink(m.list_path.c_str());
        }
        if (m.retained)
        {
            driver().cuDevicePrimaryCtxRelease(m.cu_device);
        }
    }

    int ExportPool::acquire(std::uint64_t nbytes, std::uint32_t reuse_frames)
    {
        Impl& m = *impl_;
        m.start();
        const std::size_t size = round_up(nbytes == 0 ? 1 : nbytes, m.granularity);
        const std::uint64_t now = now_us();
        int best = -1;
        std::uint32_t of_size = 0;
        {
            std::lock_guard lock(m.mutex);
            // blocks of another size: retired, freed when nobody holds them (or their hold expired)
            for (std::size_t i = 0; i < m.blocks.size();)
            {
                Block& b = *m.blocks[i];
                if (b.size != size)
                {
                    b.retired = true;
                }
                if (b.retired and m.take(b, now, reuse_frames))
                {
                    m.release(b);
                    m.blocks.erase(m.blocks.begin() + static_cast<std::ptrdiff_t>(i));
                    m.write_list();
                    continue;
                }
                ++i;
            }
            // of the right size: the one published longest ago that may be written (the longest time for a sample
            // that is queued in a reader but not taken yet)
            std::vector<std::size_t> order;
            for (std::size_t i = 0; i < m.blocks.size(); ++i)
            {
                if (not m.blocks[i]->retired)
                {
                    ++of_size;
                    order.push_back(i);
                }
            }
            std::sort(order.begin(), order.end(), [&m](std::size_t a, std::size_t b) {
                return m.blocks[a]->published_us < m.blocks[b]->published_us;
            });
            for (std::size_t i : order)
            {
                if (m.take(*m.blocks[i], now, reuse_frames))
                {
                    best = static_cast<int>(i);
                    break;
                }
            }
        }
        if (best >= 0)
        {
            return best;
        }
        if (of_size < std::min<std::uint64_t>(m.max_blocks, std::uint64_t{reuse_frames} + 1 + kSpareBlocks))
        {
            m.make(size);
            return static_cast<int>(m.blocks.size() - 1);
        }
        m.dropped.fetch_add(1);
        return -1;
    }

    std::uint64_t ExportPool::block_ptr(int index) const
    {
        const Impl& m = *impl_;
        if (index < 0 or static_cast<std::size_t>(index) >= m.blocks.size())
        {
            throw RingError("export pool: no block " + std::to_string(index));
        }
        return m.blocks[static_cast<std::size_t>(index)]->base;
    }

    Descriptor ExportPool::publish(int index, std::uint64_t nbytes)
    {
        Impl& m = *impl_;
        if (index < 0 or static_cast<std::size_t>(index) >= m.blocks.size())
        {
            throw RingError("export pool: no block " + std::to_string(index));
        }
        Block& b = *m.blocks[static_cast<std::size_t>(index)];
        if (nbytes > b.size)
        {
            throw RingError("export pool: " + std::to_string(nbytes) + " bytes in a block of " +
                            std::to_string(b.size));
        }
        std::uint64_t uid = 0;
        while (uid == 0)
        {
            uid = m.random();
        }
        b.uid = uid;
        b.published_us = now_us();
        b.published_frame = ++m.frames;
        b.record->publish_us.store(b.published_us);
        b.record->uid.store(uid);
        Descriptor d;
        d.size = nbytes;
        d.device_id = m.device;
        d.use_ipc = true;
        d.pid = ::getpid();
        d.block_id = b.id;
        d.block_size = b.size;
        d.socket_path = b.socket_path;
        d.uid = uid;
        return d;
    }

    std::uint64_t ExportPool::dropped() const { return impl_->dropped.load(); }
    std::uint64_t ExportPool::expired() const { return impl_->expired.load(); }
    std::uint32_t ExportPool::accepts() const { return impl_->accepts.load(); }
    std::uint32_t ExportPool::refused() const { return impl_->refused.load(); }
    int ExportPool::device() const { return impl_->device; }
    const std::string& ExportPool::socket_dir() const { return impl_->socket_dir; }
    const std::string& ExportPool::shm_dir() const { return impl_->shm_dir; }

    std::uint32_t ExportPool::blocks() const
    {
        std::lock_guard lock(impl_->mutex);
        return static_cast<std::uint32_t>(impl_->blocks.size());
    }

    // ------------------------------------------------------------------------------------- the link to readers

    bool enabled_by_environment()
    {
        const char* value = std::getenv(kEnableEnv);
        if (value == nullptr or std::string(value) == "1")
        {
            return true;
        }
        if (std::string(value) == "0")
        {
            return false;
        }
        throw std::invalid_argument(std::string(kEnableEnv) + "='" + value +
                                    "': 0 (no GPU link to ROS readers) or 1 (the link, the default)");
    }

    namespace
    {
        std::string env_or(const char* name, const char* fallback)
        {
            const char* value = std::getenv(name);
            return value != nullptr and *value != '\0' ? std::string(value) : std::string(fallback);
        }
    }  // namespace

    std::string socket_dir_from_environment() { return env_or(kSocketsEnv, kDefaultSocketDir); }
    std::string shm_dir_from_environment() { return env_or(kShmEnv, kDefaultShmDir); }

    std::map<std::string, std::string> backends_of(const std::string& part)
    {
        std::map<std::string, std::string> out;
        const std::string_view prefix = "backends:";
        std::string_view rest(part);
        if (rest.substr(0, prefix.size()) == prefix)
        {
            rest.remove_prefix(prefix.size());
        }
        while (not rest.empty())
        {
            const std::size_t end = rest.find(';');
            const std::string_view item = rest.substr(0, end);
            if (not item.empty())
            {
                const std::size_t colon = item.find(':');
                out[std::string(item.substr(0, colon))] =
                    colon == std::string_view::npos ? std::string() : std::string(item.substr(colon + 1));
            }
            if (end == std::string_view::npos)
            {
                break;
            }
            rest.remove_prefix(end + 1);
        }
        return out;
    }

    namespace
    {
        constexpr unsigned kNvidiaMajor = 195;  // /dev/nvidia<minor>; 254 and 255 are nvidia-modeset and nvidiactl
        constexpr unsigned kGpuMinors = 254;
        constexpr const char* kCudaBackend = "cuda";
        constexpr const char* kCpuBackends = "backends:cpu:";  // ros.hpp's kCpuBackends
        constexpr const char* kGpuEnv[] = {"CUDA_VISIBLE_DEVICES", "CUDA_DEVICE_ORDER"};

        std::map<std::string, std::string> environment_of(int pid)
        {
            std::ifstream in("/proc/" + std::to_string(pid) + "/environ", std::ios::binary);
            if (not in)
            {
                throw RingError("cannot read /proc/" + std::to_string(pid) + "/environ: " + errno_text());
            }
            std::map<std::string, std::string> out;
            std::string item;
            while (std::getline(in, item, '\0'))
            {
                const std::size_t eq = item.find('=');
                if (not item.empty())
                {
                    out[item.substr(0, eq)] = eq == std::string::npos ? std::string() : item.substr(eq + 1);
                }
            }
            return out;
        }

        bool maps_registry(int pid, const std::string& registry)
        {
            std::ifstream in("/proc/" + std::to_string(pid) + "/maps");
            if (not in)
            {
                throw RingError("cannot read /proc/" + std::to_string(pid) + "/maps: " + errno_text());
            }
            std::string line;
            const std::string tail = "/" + registry;
            while (std::getline(in, line))
            {
                while (not line.empty() and std::isspace(static_cast<unsigned char>(line.back())))
                {
                    line.pop_back();
                }
                if (line.size() >= tail.size() and line.compare(line.size() - tail.size(), tail.size(), tail) == 0)
                {
                    return true;
                }
            }
            return false;
        }

        std::string trim(std::string s)
        {
            const auto first = s.find_first_not_of(" \t\r\n");
            if (first == std::string::npos)
            {
                return "";
            }
            return s.substr(first, s.find_last_not_of(" \t\r\n") - first + 1);
        }

        std::string lower_hex(std::string s)
        {
            std::string out;
            for (char c : s)
            {
                if (c != '-')
                {
                    out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                }
            }
            return out;
        }
    }  // namespace

    std::set<int> gpu_minors(const std::string& pid)
    {
        const std::string base = "/proc/" + pid + "/fd";
        DIR* dir = ::opendir(base.c_str());
        if (dir == nullptr)
        {
            throw RingError("cannot read " + base + ": " + errno_text());
        }
        std::set<int> minors;
        while (dirent* e = ::readdir(dir))
        {
            if (e->d_name[0] == '.')
            {
                continue;
            }
            struct stat s{};
            if (::stat((base + "/" + e->d_name).c_str(), &s) != 0)
            {
                continue;  // closed meanwhile
            }
            if (S_ISCHR(s.st_mode) and major(s.st_rdev) == kNvidiaMajor and minor(s.st_rdev) < kGpuMinors)
            {
                minors.insert(static_cast<int>(minor(s.st_rdev)));
            }
        }
        ::closedir(dir);
        return minors;
    }

    std::optional<int> minor_of(const std::string& uuid)
    {
        const std::string want = lower_hex(uuid);
        const std::string root = "/proc/driver/nvidia/gpus";
        DIR* dir = ::opendir(root.c_str());
        if (dir == nullptr)
        {
            return std::nullopt;
        }
        std::vector<std::string> buses;
        while (dirent* e = ::readdir(dir))
        {
            if (e->d_name[0] != '.')
            {
                buses.emplace_back(e->d_name);
            }
        }
        ::closedir(dir);
        for (const std::string& bus : buses)
        {
            std::ifstream in(root + "/" + bus + "/information");
            std::map<std::string, std::string> fields;
            std::string line;
            while (std::getline(in, line))
            {
                const std::size_t colon = line.find(':');
                if (colon != std::string::npos)
                {
                    fields[line.substr(0, colon)] = line.substr(colon + 1);
                }
            }
            std::string theirs = trim(fields["GPU UUID"]);
            if (theirs.rfind("GPU-", 0) == 0)
            {
                theirs = theirs.substr(4);
            }
            if (not theirs.empty() and lower_hex(theirs) == want and fields.contains("Device Minor"))
            {
                return std::stoi(trim(fields["Device Minor"]));
            }
        }
        return std::nullopt;
    }

    namespace
    {
        double seconds_now()
        {
            return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        struct KnownReader
        {
            Reader token;
            std::optional<std::map<std::string, std::string>> backends;
            std::optional<bool> importable;  // none: not decided yet (not in the registry)
            int device_id = 0;               // its CUDA ordinal (the descriptor's device_id)
            double seen = seconds_now();
            double checked = -1e9;
            bool said = false;

            bool wants_cuda() const { return backends and backends->contains(kCudaBackend); }
            bool cpu_only() const { return token.backends == std::string(kCpuBackends); }
        };
    }  // namespace

    struct Link::Impl
    {
        std::string topic;
        int device = 0;
        std::string socket_dir, shm_dir;
        std::uint32_t max_blocks = kMaxBlocks;
        double hold_limit_s = kHoldLimitS, min_reuse_s = kMinReuseS;
        mutable std::mutex mutex;  // readers, notes
        std::map<std::string, KnownReader> readers;
        std::vector<Note> notes;
        mutable std::mutex minor_mutex;  // own_minor (importable runs inside plan's lock and alone)
        mutable std::optional<int> own_minor;
        std::unique_ptr<ExportPool> pool;  // made for the first reader that gets descriptors (the publisher's thread)
        std::atomic<std::uint64_t> sent{0};
        bool said_drop = false;

        void say(KnownReader& reader, const std::string& text)
        {
            if (not reader.said)
            {
                reader.said = true;
                notes.push_back({true, topic + ": ROS reader " + reader.token.node +
                                           " accepts CUDA buffers, gets the CPU form: " + text});
            }
        }

        void decide(KnownReader& reader, double now, const Link& link)
        {
            if (reader.importable or now - reader.checked < kRecheckS)
            {
                return;
            }
            reader.checked = now;
            const std::optional<RegistryEntry> entry = registry_lookup(shm_dir, reader.token.gid);
            if (not entry)
            {
                if (now - reader.seen > kUnknownS)
                {
                    say(reader, "it is not in the CUDA buffer backend's registry in " + shm_dir +
                                    " (another host, another /dev/shm, or no backend plugin)");
                }
                return;
            }
            std::pair<bool, std::string> decision;
            if (reader.token.keep_all)
            {
                decision = {false, "it keeps all messages (KEEP_ALL), more than any pool of blocks holds"};
            }
            else
            {
                decision = link.importable(*entry);
            }
            reader.importable = decision.first;
            if (decision.first)
            {
                reader.device_id = entry->device_id;
                notes.push_back({false, topic + ": ROS reader " + reader.token.node + " gets the frames as CUDA buffers"});
            }
            else
            {
                say(reader, decision.second);
            }
        }
    };

    Link::Link(std::string topic, int device, std::string socket_dir, std::string shm_dir, std::uint32_t max_blocks,
               double hold_limit_s, double min_reuse_s)
        : impl_(std::make_unique<Impl>())
    {
        for (const auto& [what, path, env] : {std::tuple{"sockets", &socket_dir, kSocketsEnv},
                                              std::tuple{"shm", &shm_dir, kShmEnv}})
        {
            struct stat s{};
            if (::stat(path->c_str(), &s) != 0 or not S_ISDIR(s.st_mode))
            {
                throw std::invalid_argument("GPU link to ROS readers: " + *path + " (" + what + ") is no directory (" +
                                            env + ")");
            }
        }
        Impl& m = *impl_;
        m.topic = std::move(topic);
        m.device = device;
        m.socket_dir = std::move(socket_dir);
        m.shm_dir = std::move(shm_dir);
        m.max_blocks = max_blocks;
        m.hold_limit_s = hold_limit_s;
        m.min_reuse_s = min_reuse_s;
    }

    Link::~Link() = default;

    void Link::add(const std::string& token_key, const Reader& reader)
    {
        KnownReader known;
        known.token = reader;
        if (reader.backends)
        {
            known.backends = backends_of(*reader.backends);
        }
        std::lock_guard lock(impl_->mutex);
        impl_->readers.insert_or_assign(token_key, std::move(known));
    }

    std::optional<std::array<std::uint8_t, 16>> Link::remove(const std::string& token_key)
    {
        std::lock_guard lock(impl_->mutex);
        const auto found = impl_->readers.find(token_key);
        if (found == impl_->readers.end())
        {
            return std::nullopt;
        }
        const auto gid = found->second.token.gid;
        impl_->readers.erase(found);
        return gid;
    }

    std::optional<Plan> Link::plan()
    {
        Impl& m = *impl_;
        const double now = seconds_now();
        std::lock_guard lock(m.mutex);
        bool any_cuda = false;
        for (const auto& [key, r] : m.readers)
        {
            any_cuda = any_cuda or r.wants_cuda();
            if (not r.backends)
            {
                return std::nullopt;  // Jazzy or an older xblick: the key, for everyone
            }
        }
        if (not any_cuda)
        {
            return std::nullopt;
        }
        Plan plan;
        for (auto& [key, r] : m.readers)
        {
            if (r.wants_cuda())
            {
                m.decide(r, now, *this);
            }
        }
        for (const auto& [key, r] : m.readers)
        {
            if (r.wants_cuda() and r.importable.value_or(false))
            {
                plan.gpu.emplace_back(r.token.gid, r.device_id);
                plan.depth = std::max(plan.depth, static_cast<std::uint32_t>(std::max(r.token.depth, 0)));
            }
            else if (r.cpu_only())
            {
                plan.cpu_group = true;
            }
            else
            {
                plan.cpu.push_back(r.token.gid);
            }
        }
        if (plan.gpu.empty())
        {
            return std::nullopt;
        }
        return plan;
    }

    std::vector<std::pair<std::array<std::uint8_t, 16>, std::vector<std::uint8_t>>> Link::gpu_messages(
        const Plan& plan, CudaRingWriter& ring, std::uint32_t slot, std::uint64_t nbytes, const ImageHeader& header)
    {
        Impl& m = *impl_;
        if (not m.pool)
        {
            m.pool = std::make_unique<ExportPool>(m.device, m.socket_dir, m.shm_dir, m.max_blocks, m.hold_limit_s,
                                                  m.min_reuse_s);
        }
        const int index = m.pool->acquire(nbytes, plan.depth);
        if (index < 0)
        {
            if (not m.said_drop)
            {
                m.said_drop = true;
                std::ostringstream text;
                text << m.topic << ": every block of the GPU link is held or queued by ROS readers; frames for them "
                     << "are dropped until one is free (hold limit " << m.hold_limit_s << " s)";
                std::lock_guard lock(m.mutex);
                m.notes.push_back({true, text.str()});
            }
            return {};
        }
        ring.copy_slot(slot, m.pool->block_ptr(index), nbytes);
        Descriptor descriptor = m.pool->publish(index, nbytes);
        std::map<int, std::vector<std::uint8_t>> payloads;  // per reader's ordinal
        std::vector<std::pair<std::array<std::uint8_t, 16>, std::vector<std::uint8_t>>> out;
        for (const auto& [gid, device_id] : plan.gpu)
        {
            auto found = payloads.find(device_id);
            if (found == payloads.end())
            {
                descriptor.device_id = device_id;
                found = payloads.emplace(device_id, image_payload(header, descriptor)).first;
            }
            out.emplace_back(gid, found->second);
        }
        m.sent += out.size();
        return out;
    }

    std::vector<Note> Link::notes()
    {
        std::lock_guard lock(impl_->mutex);
        return std::exchange(impl_->notes, {});
    }

    LinkStats Link::stats() const
    {
        const Impl& m = *impl_;
        LinkStats s;
        s.sent = m.sent.load();
        if (m.pool)
        {
            s.dropped = m.pool->dropped();
            s.expired = m.pool->expired();
            s.blocks = m.pool->blocks();
            s.accepts = m.pool->accepts();
            s.refused = m.pool->refused();
        }
        return s;
    }

    const std::string& Link::socket_dir() const { return impl_->socket_dir; }
    const std::string& Link::shm_dir() const { return impl_->shm_dir; }

    std::pair<bool, std::string> Link::importable(const RegistryEntry& entry) const
    {
        const Impl& m = *impl_;
        if (entry.entity_type != kSubscription)
        {
            return {false, "its registry entry is no subscription"};
        }
        if (entry.uid != ::getuid())
        {
            return {false, "it runs as Linux user " + std::to_string(entry.uid) + ", this program as " +
                               std::to_string(::getuid())};
        }
        if (not entry.ipc_capable)
        {
            return {false, "its backend cannot import CUDA buffers (no IPC-capable memory pool)"};
        }
        const int pid = entry.pid();
        std::set<int> theirs;
        std::map<std::string, std::string> env;
        try
        {
            if (not maps_registry(pid, registry_name()))
            {
                throw RingError("the process there does not map the registry");
            }
            theirs = gpu_minors(std::to_string(pid));
            env = environment_of(pid);
        }
        catch (const RingError& error)
        {
            return {false, "its process " + std::to_string(pid) + " cannot be seen from here (" + error.what() +
                               "): it needs this program's pid namespace"};
        }
        std::optional<int> own;
        {
            std::lock_guard lock(m.minor_mutex);
            if (not m.own_minor)
            {
                m.own_minor = minor_of(cuda_device_uuid(m.device));
            }
            own = m.own_minor;
        }
        if (not own)
        {
            return {false, "this program cannot tell its own GPU's device file (/proc/driver/nvidia/gpus)"};
        }
        if (theirs.size() == 1)
        {
            if (*theirs.begin() != *own)
            {
                return {false, "it uses GPU /dev/nvidia" + std::to_string(*theirs.begin()) + ", this program /dev/nvidia" +
                                   std::to_string(*own)};
            }
            return {true, ""};
        }
        bool same_numbering = theirs == gpu_minors("self");
        for (const char* name : kGpuEnv)
        {
            const char* mine = std::getenv(name);
            const auto found = env.find(name);
            const bool theirs_set = found != env.end();
            same_numbering = same_numbering and theirs_set == (mine != nullptr) and
                             (not theirs_set or found->second == mine);
        }
        if (not same_numbering)
        {
            return {false, "it sees several GPUs, numbered otherwise than here (its CUDA_VISIBLE_DEVICES and "
                           "CUDA_DEVICE_ORDER, its GPUs): the backend compares CUDA ordinals only"};
        }
        if (entry.device_id != m.device)
        {
            return {false, "it uses CUDA device " + std::to_string(entry.device_id) + ", this program " +
                               std::to_string(m.device)};
        }
        return {true, ""};
    }
}  // namespace xblick::ros_cuda
