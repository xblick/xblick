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
#include <memory>
#include <optional>
#include <string>
#include <vector>

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
}  // namespace xblick::ros_cuda
