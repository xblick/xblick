// The GPU link to ROS 2 readers: ROS 2's CUDA buffer backend (cuda_buffer_backend of ros2/rosidl_buffer_backends,
// experimental upstream, no version field on the wire) spoken without ROS libraries, so that a ROS subscription that
// accepts it gets an image as a CUDA buffer instead of its bytes (docs/concepts/gpu-frames.md).
//
// The CUDA ring stays as it is. While such a reader is there, the publisher copies each frame on the GPU into a block
// of a small export pool (ExportPool) and sends the block's descriptor in place of the image's data. A block is one
// cuMemCreate allocation of exactly the size it is announced with (whole multiples of the allocation granularity):
// the reader maps a handle from offset 0 only, and the driver refuses a partial mapping.
//
// What the backend's reader does with a descriptor (cuda_buffer_ipc_manager.cpp of the backend):
//   * it opens the record /dev/shm/cuda_vmm_<pid>_<block id> (IPCMetadata: int32 refcount at 0, uint64 uid at 8,
//     uint64 publish time in us at 16), adds one to the count and compares the uid with the descriptor's (0 there:
//     no check); another uid: the block was rewritten, it drops the message;
//   * on a block it has not seen, it connects to the Unix socket the descriptor names and takes one byte with the
//     block's file descriptor (SCM_RIGHTS): release 0.1.2 a socket file at that path, the current source an abstract
//     socket of that name; each block is served on both (one name), so either reader finds it. It keeps every block
//     it has seen mapped until it exits, so a block id is never used again for another block;
//   * when it lets go of the message, it takes one off the count. Nothing else comes back.
// The count goes up only when the reader takes the message, not when it arrives: a message still in the reader's
// queue holds nothing. A block is therefore rewritten only
//   * when its count is 0 (uid 0 while the writer checks and writes, so a reader that comes in between drops the
//     message), and
//   * when at least as many frames as the readers' queues hold (reuse_frames: their QoS depth) have been published
//     since, and the minimum reuse time has passed: a queued message's block is not touched. A message that a
//     reader takes with the wrong uid is dropped by the backend; a node that then reads it anyway (Isaac ROS 5.1's
//     ResizeNode) aborts;
// or when the last publish of the block is longer ago than the hold limit (a reader that crashed never takes its
// count back, and one without access to the record holds invisibly). The pool grows to reuse_frames + 1 +
// kSpareBlocks blocks of one size at most (and never beyond its max_blocks). When no block is free, the frame does
// not go to the GPU readers (counted in dropped()); the writer never waits.
//
// The socket serves only peers of this process's Linux user (SO_PEERCRED), as the CUDA ring's.
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace xblick
{
    class CudaRingWriter;
}

namespace xblick::ros_cuda
{
    inline constexpr double kHoldLimitS = 2.0;       // a block held longer since its last publish counts as free
    inline constexpr double kMinReuseS = 0.1;        // a block is not rewritten sooner after its publish (backend's)
    inline constexpr std::uint32_t kSpareBlocks = 4;  // blocks beyond the readers' queues, for messages they hold
    inline constexpr std::uint32_t kMaxBlocks = 64;  // blocks of one frame size at most, whatever the queues
    inline constexpr const char* kDefaultSocketDir = "/tmp";     // the backend's (release 0.1.2) socket files
    inline constexpr const char* kDefaultShmDir = "/dev/shm";    // where POSIX shm_open puts the records
    inline constexpr std::uint32_t kMarker = 0xFFFFFFFF;         // first word in place of a uint8[] field
    inline constexpr std::uint32_t kMarkerRos2 = 0x524F5332;     // second word: "ROS2" as little endian uint32
    inline constexpr const char* kBackend = "cuda";
    inline constexpr const char* kElementType = "h";             // typeid(uint8_t).name() with GCC and Clang

    // CudaBufferDescriptor (cuda_buffer_backend_msgs), field for field.
    struct Descriptor
    {
        std::uint64_t size = 0;
        std::int32_t device_id = 0;  // the publisher's CUDA ordinal; the reader compares it with its own
        bool use_ipc = true;
        std::int32_t pid = 0;
        std::uint32_t block_id = 0;
        std::uint64_t block_size = 0;
        std::string socket_path;
        std::uint64_t uid = 0;
        std::array<std::uint8_t, 64> event{};  // all zero: no event, the block is complete when it is sent
    };

    // The fields of sensor_msgs/Image before its data.
    struct ImageHeader
    {
        std::int32_t sec = 0;
        std::uint32_t nanosec = 0;
        std::string frame_id;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::string encoding;
        std::uint8_t is_bigendian = 0;
        std::uint32_t step = 0;
    };

    // sensor_msgs/Image in CDR with the descriptor in place of its data: the marker words, the backend's name, then
    // the descriptor's fields, aligned as one CDR stream (rosidl_typesupport_fastrtps' buffer serialization).
    std::vector<std::uint8_t> image_payload(const ImageHeader& header, const Descriptor& descriptor);

    // An entry of the backend's host registry (/dev/shm/ros2_hem_d0_h<hash of the host name>, written by every ROS
    // process that loads the backend; read here, never written).
    struct RegistryEntry
    {
        std::uint64_t instance_id = 0;  // the process id in the upper 32 bits
        std::array<std::uint8_t, 16> gid{};
        std::uint8_t entity_type = 0;   // 0 publisher, 1 subscription
        std::int32_t device_id = -1;    // the CUDA ordinal of the process
        std::uint32_t uid = 0;          // its Linux user
        bool active = false;
        bool ipc_capable = false;
        std::string name;
        int pid() const { return static_cast<int>(instance_id >> 32); }
    };
    inline constexpr std::uint8_t kSubscription = 1;

    // The registry's file name below shm_dir (domain 0: the backend uses no other).
    std::string registry_name();
    // The active entry of gid in the registry below shm_dir; nullopt without one or without the registry. RingError
    // for a registry of another layout than version 1 (the backend changed: the link cannot tell who is local).
    std::optional<RegistryEntry> registry_lookup(const std::string& shm_dir, const std::array<std::uint8_t, 16>& gid);

    // The blocks for one device. Nothing is allocated before the first acquire(). Not thread-safe (one publisher).
    class ExportPool
    {
    public:
        // device: the CUDA ordinal of the frames (the CUDA ring's). socket_dir: where the socket files go (the
        // reader must reach them at the same path); shm_dir: where the records go (the reader's /dev/shm). Removes
        // what an earlier process left there that provably was such a pool and is gone (its list of files, same
        // boot or an earlier one, same pid namespace, its pid no longer that process).
        explicit ExportPool(int device, std::string socket_dir = kDefaultSocketDir,
                            std::string shm_dir = kDefaultShmDir, std::uint32_t max_blocks = kMaxBlocks,
                            double hold_limit_s = kHoldLimitS, double min_reuse_s = kMinReuseS);
        ~ExportPool();  // removes every file and block (readers keep their mappings until they let go)
        ExportPool(const ExportPool&) = delete;
        ExportPool& operator=(const ExportPool&) = delete;

        // A block of nbytes that no reader holds and whose last publish is reuse_frames publishes back or more,
        // its uid 0 until publish(); -1 when there is none and the pool has as many blocks as it may (counted in
        // dropped()). A frame of another size retires the blocks of the old one (freed once nobody holds them).
        int acquire(std::uint64_t nbytes, std::uint32_t reuse_frames);
        // The device pointer of a block (of this device, in this process's primary context).
        std::uint64_t block_ptr(int index) const;
        // The frame is in the block (nbytes, complete on the GPU): a new uid in its record, and its descriptor.
        Descriptor publish(int index, std::uint64_t nbytes);

        std::uint64_t dropped() const;   // frames no block was free for
        std::uint64_t expired() const;   // blocks rewritten after the hold limit although a count was up
        std::uint32_t blocks() const;    // blocks allocated now
        std::uint32_t accepts() const;   // file descriptors handed to readers
        std::uint32_t refused() const;   // connections of another Linux user refused
        int device() const;
        const std::string& socket_dir() const;
        const std::string& shm_dir() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    // ------------------------------------------------------------------------------------- the link to readers
    // What a publisher of images in a CUDA ring needs to give ROS readers that accept CUDA buffers the frames as such:
    // the readers of its topic, a decision per reader, the plan of each message and the descriptors. The Zenoh part
    // (the readers' tokens, the buffer keys) is the language's: ros.hpp's GpuLink, Python's ros_cuda.GpuLink.
    //
    // A reader gets descriptors only when the backend could import them, else the CPU form:
    //   * it is in the backend's registry below shm_dir under its gid, as a subscription of this Linux user that can
    //     import (the backend's own rule): so on this host, with this /dev/shm;
    //   * its process is one this program can see (/proc/<pid> maps that registry: the same pid namespace);
    //   * it uses this GPU: with one GPU open in it, that GPU (by the device files /dev/nvidia<minor>); with several,
    //     the same CUDA_VISIBLE_DEVICES and CUDA_DEVICE_ORDER as this program and the same ordinal (the backend
    //     compares ordinals only);
    //   * it keeps the last so many messages (KEEP_LAST), not all: the pool holds up to its depth + 1 + kSpareBlocks.
    // While at least one reader gets descriptors, a message goes on the buffer keys only, as a Lyrical publisher sends
    // it: <key>/_buf_cpu for "backends:cpu:" readers, <key>/_buf/<gid> in CPU form for the other buffer-aware ones.
    // A reader without a backends part (ROS 2 Jazzy, an older xblick) reads the key only: while one is there, every
    // reader gets the CPU form on the key (no plan).

    inline constexpr const char* kEnableEnv = "XBLICK_ROS_CUDA";           // 0: no link; 1 or unset: the link
    inline constexpr const char* kSocketsEnv = "XBLICK_ROS_CUDA_SOCKETS";  // where the sockets go
    inline constexpr const char* kShmEnv = "XBLICK_ROS_CUDA_SHM";          // the readers' /dev/shm as seen here
    inline constexpr double kRecheckS = 0.5;  // a reader not (yet) in the registry is looked up again after this long
    inline constexpr double kUnknownS = 3.0;  // ... and said once after this long

    // false when XBLICK_ROS_CUDA is 0, true when it is 1 or unset; std::invalid_argument for any other value.
    bool enabled_by_environment();
    // XBLICK_ROS_CUDA_SOCKETS, else kDefaultSocketDir; XBLICK_ROS_CUDA_SHM, else kDefaultShmDir.
    std::string socket_dir_from_environment();
    std::string shm_dir_from_environment();

    // The backends of a token's 14th part ("backends:cpu:;cuda:" -> {cpu: "", cuda: ""}).
    std::map<std::string, std::string> backends_of(const std::string& part);
    // The GPUs (minor numbers of /dev/nvidia<N>) a process has open ("self": this one). RingError when its file
    // descriptors cannot be read (another user, gone).
    std::set<int> gpu_minors(const std::string& pid);
    // The minor number of the GPU of a UUID (32 hex digits) from /proc/driver/nvidia/gpus; none if none says so.
    std::optional<int> minor_of(const std::string& uuid);

    // A ROS subscription of the topic, as its liveliness token says it (parsed by the caller).
    struct Reader
    {
        std::array<std::uint8_t, 16> gid{};
        std::optional<std::string> backends;  // the token's 14th part ("backends:..."); none without one
        bool keep_all = false;                // KEEP_ALL history
        int depth = 0;                        // its KEEP_LAST depth
        std::string node;                     // its node, fully qualified (for the lines said about it)
    };

    // Where one message goes while the link is on: descriptors to `gpu` (gid, the CUDA ordinal for the reader's
    // descriptor), the CPU form on <key>/_buf_cpu (cpu_group) and on <key>/_buf/<gid> of `cpu`. depth: the deepest
    // queue of the `gpu` readers.
    struct Plan
    {
        std::vector<std::pair<std::array<std::uint8_t, 16>, int>> gpu;
        bool cpu_group = false;
        std::vector<std::array<std::uint8_t, 16>> cpu;
        std::uint32_t depth = 0;
        bool needs_cpu() const { return cpu_group or not cpu.empty(); }
    };

    // A line to say (warning or not) for the program's log.
    struct Note
    {
        bool warning = false;
        std::string text;
    };

    struct LinkStats
    {
        std::uint64_t sent = 0;  // descriptors sent (one per reader and frame)
        std::uint64_t dropped = 0, expired = 0;
        std::uint32_t blocks = 0, accepts = 0, refused = 0;
    };

    // The readers, the decisions and the pool of one image topic (thread-safe: tokens come from Zenoh's threads).
    class Link
    {
    public:
        // std::invalid_argument when socket_dir or shm_dir is no directory. Nothing allocated before the first frame
        // for a reader that gets descriptors.
        Link(std::string topic, int device, std::string socket_dir, std::string shm_dir,
             std::uint32_t max_blocks = kMaxBlocks, double hold_limit_s = kHoldLimitS,
             double min_reuse_s = kMinReuseS);
        ~Link();  // the pool's files and blocks go
        Link(const Link&) = delete;
        Link& operator=(const Link&) = delete;

        // A reader's token came (again: decided anew), or went (its gid, for its buffer key; none if unknown).
        void add(const std::string& token_key, const Reader& reader);
        std::optional<std::array<std::uint8_t, 16>> remove(const std::string& token_key);
        // Where the next message goes; none: on the topic's key as before (no reader gets descriptors, or one reads
        // the key only). Looks readers up in the registry (every kRecheckS until found).
        std::optional<Plan> plan();
        // The frame in slot `slot` of `ring` (this link's device) into a free block, and its descriptor message for
        // each GPU reader of `plan` (gid, payload). Empty when every block is held (counted, said once): the writer
        // never waits.
        std::vector<std::pair<std::array<std::uint8_t, 16>, std::vector<std::uint8_t>>> gpu_messages(
            const Plan& plan, CudaRingWriter& ring, std::uint32_t slot, std::uint64_t nbytes,
            const ImageHeader& header);
        // The lines to say since the last call.
        std::vector<Note> notes();
        LinkStats stats() const;
        // Whether a reader of this registry entry can import our blocks, and why not.
        std::pair<bool, std::string> importable(const RegistryEntry& entry) const;
        const std::string& socket_dir() const;
        const std::string& shm_dir() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}  // namespace xblick::ros_cuda
