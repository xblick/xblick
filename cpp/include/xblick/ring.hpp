// Shared-memory rings of Durchblick: image buffers handed between processes on one host without a copy through the
// transport. Zenoh carries a small descriptor per frame (segment or endpoint, slot, sequence number); the pixels stay
// in the ring. This header is the normative description of the layout; C++ and Python (xblick._core) use the
// same implementation.
//
// Control block (POSIX shared memory object, 4096 bytes, little endian), shared by both ring kinds:
//
//   offset  size  field
//        0     8  magic "DBLKRING"
//        8     4  version (1)
//       12     4  slot_count (1..kMaxSlots)
//       16     8  slot_size: bytes per slot (host ring: multiple of 4096; CUDA ring: the slot stride)
//       24     8  data_offset: offset of slot 0 in this object (host ring: 4096; CUDA ring: 0, no data here)
//       32     4  kind: 0 = host ring (pixels follow the control block), 1 = CUDA ring (pixels in device memory)
//       36     4  creator_pid: the creating process's id; 0 in a ring of 0.5.0 to 0.5.11 (no creator record)
//       40     8  creator_start: the creator's start time, field 22 of /proc/<pid>/stat (clock ticks after boot)
//       48     8  last_update_ns: wall clock (CLOCK_REALTIME) of the last commit in ns, 0 before the first (atomic)
//       56     8  reserved, zero
//   64 + 64 i    slot i: uint64 seq (atomic), uint64 nbytes, 48 bytes padding (one cache line per slot)
//
// The creator record (offsets 36 to 63) was reserved and zero up to 0.5.11, and readers never look at it, so readers
// of every 0.5 version read every 0.5 ring. Creator lock: the creator holds an exclusive flock(2) on the object for
// its lifetime (the descriptor stays open, close-on-exec), so the kernel releases it however the creator ends.
// Readers never ask for it, and flock is advisory: it keeps nobody from opening or mapping the ring. A ring whose
// lock can be taken therefore has no living creator (`xblick clean`). A process that forks without exec inherits the
// descriptor and keeps the lock, so the ring counts as alive while that child lives.
//
// Sequence lock per slot: seq is even when the slot holds a complete frame and odd while the writer fills it; every
// write adds 2. A descriptor names (slot, seq). A reader that finds seq unchanged after it has read (or copied) the
// slot has read exactly that frame; otherwise the frame was overwritten and is dropped, never shown torn. Readers
// never write to the ring. With slot_count slots a reader has slot_count - 1 frame periods to read a frame.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>

namespace xblick
{
    inline constexpr char kMagic[8] = {'D', 'B', 'L', 'K', 'R', 'I', 'N', 'G'};
    inline constexpr std::uint32_t kVersion = 1;
    inline constexpr std::size_t kControlSize = 4096;
    inline constexpr std::size_t kSlotHeaderOffset = 64;
    inline constexpr std::size_t kSlotHeaderSize = 64;
    inline constexpr std::uint32_t kMaxSlots = (kControlSize - kSlotHeaderOffset) / kSlotHeaderSize;  // 63

    enum class RingKind : std::uint32_t
    {
        Host = 0,
        Cuda = 1,
    };

    class RingError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    struct SlotHeader
    {
        std::atomic<std::uint64_t> seq;
        std::atomic<std::uint64_t> nbytes;  // atomic: a reader may look at it while the writer changes it
        std::uint8_t pad[kSlotHeaderSize - 16];
    };
    static_assert(sizeof(SlotHeader) == kSlotHeaderSize);
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free);

    struct ControlHeader
    {
        char magic[8];
        std::uint32_t version;
        std::uint32_t slot_count;
        std::uint64_t slot_size;
        std::uint64_t data_offset;
        std::uint32_t kind;
        std::uint32_t creator_pid;
        std::uint64_t creator_start;
        std::atomic<std::uint64_t> last_update_ns;
        std::uint8_t reserved[8];
    };
    static_assert(sizeof(ControlHeader) == kSlotHeaderOffset);
    static_assert(offsetof(ControlHeader, creator_pid) == 36 and offsetof(ControlHeader, creator_start) == 40 and
                  offsetof(ControlHeader, last_update_ns) == 48);

    // A mapped shared memory object (shm_open + mmap). The creator keeps the object's descriptor open with the creator
    // lock on it (flock, see above) and unlinks the name when it is destroyed; mappings of readers stay valid until
    // they unmap.
    class Mapping
    {
    public:
        static Mapping create(const std::string& name, std::size_t size);
        static Mapping open_readonly(const std::string& name);
        Mapping() = default;
        Mapping(Mapping&& other) noexcept;
        Mapping& operator=(Mapping&& other) noexcept;
        Mapping(const Mapping&) = delete;
        Mapping& operator=(const Mapping&) = delete;
        ~Mapping();

        std::uint8_t* data() const { return data_; }
        std::size_t size() const { return size_; }
        const std::string& name() const { return name_; }

    private:
        std::string name_;
        std::uint8_t* data_ = nullptr;
        std::size_t size_ = 0;
        bool owner_ = false;
        int fd_ = -1;  // the creator's descriptor, holding the creator lock; -1 for a reader
    };

    // A fresh name "/xblick-<16 hex digits>" for a shared memory object or socket.
    std::string random_name(const std::string& prefix);

    // The writing side of the control block (both ring kinds).
    class ControlWriter
    {
    public:
        ControlWriter(RingKind kind, std::uint32_t slot_count, std::uint64_t slot_size, std::size_t data_bytes);

        // Mark the next slot as being written (seq odd) and return its index.
        std::uint32_t begin();
        // Complete the slot begun last: record nbytes, make seq even, return that seq (the descriptor's).
        std::uint64_t commit(std::uint64_t nbytes);

        const std::string& name() const { return mapping_.name(); }
        std::uint32_t slot_count() const { return header()->slot_count; }
        std::uint64_t slot_size() const { return header()->slot_size; }
        std::uint8_t* data() const { return mapping_.data() + kControlSize; }  // host ring only

    private:
        ControlHeader* header() const { return reinterpret_cast<ControlHeader*>(mapping_.data()); }
        SlotHeader* slot(std::uint32_t i) const;

        Mapping mapping_;
        std::uint32_t next_ = 0;
        std::int64_t open_ = -1;  // slot begun and not committed
    };

    // The reading side of the control block.
    class ControlReader
    {
    public:
        ControlReader(const std::string& name, RingKind kind);

        // True if slot i holds the frame with this seq (call before reading the slot).
        bool acquire(std::uint32_t i, std::uint64_t seq) const;
        // True if slot i still holds that frame (call after reading or copying the slot): the read was not torn.
        bool validate(std::uint32_t i, std::uint64_t seq) const;
        std::uint64_t nbytes(std::uint32_t i) const;

        const std::string& name() const { return mapping_.name(); }
        std::uint32_t slot_count() const { return header()->slot_count; }
        std::uint64_t slot_size() const { return header()->slot_size; }
        const std::uint8_t* data() const { return mapping_.data() + header()->data_offset; }

    private:
        const ControlHeader* header() const { return reinterpret_cast<const ControlHeader*>(mapping_.data()); }
        const SlotHeader* slot(std::uint32_t i) const;

        Mapping mapping_;
    };

    struct Written
    {
        std::uint32_t slot;
        std::uint64_t seq;
    };

    // Host ring: pixels in the same shared memory object as the control block.
    class HostRingWriter
    {
    public:
        HostRingWriter(std::uint32_t slot_count, std::uint64_t slot_size);

        // Copy nbytes from src into the next slot.
        Written write(const void* src, std::uint64_t nbytes);

        // Zero-copy production: loan() returns the next slot to fill in place, commit() publishes it.
        std::uint8_t* loan();
        Written commit(std::uint64_t nbytes);

        // All slots' memory (page-aligned), e.g. to page-lock it for copies from a GPU (CudaRingWriter::pin_host).
        std::span<std::uint8_t> slots() const
        {
            return {control_.data(), static_cast<std::size_t>(control_.slot_count()) * control_.slot_size()};
        }

        const std::string& name() const { return control_.name(); }
        std::uint32_t slot_count() const { return control_.slot_count(); }
        std::uint64_t slot_size() const { return control_.slot_size(); }

    private:
        ControlWriter control_;
        std::int64_t loaned_ = -1;
    };

    class HostRingReader
    {
    public:
        explicit HostRingReader(const std::string& name);

        // Copy the frame (slot, seq) into dst (capacity bytes). False if it was overwritten before or while copying.
        bool read(std::uint32_t slot, std::uint64_t seq, void* dst, std::uint64_t capacity) const;
        // Zero copy: the slot's bytes, valid while validate(slot, seq) is true; check it after use.
        const std::uint8_t* view(std::uint32_t slot) const;
        bool acquire(std::uint32_t slot, std::uint64_t seq) const { return control_.acquire(slot, seq); }
        bool validate(std::uint32_t slot, std::uint64_t seq) const { return control_.validate(slot, seq); }
        std::uint64_t nbytes(std::uint32_t slot) const { return control_.nbytes(slot); }

        const std::string& name() const { return control_.name(); }
        std::uint32_t slot_count() const { return control_.slot_count(); }
        std::uint64_t slot_size() const { return control_.slot_size(); }

    private:
        void check_slot(std::uint32_t slot) const;
        ControlReader control_;
    };

    // The boot id of this host (/proc/sys/kernel/random/boot_id): descriptors carry it, readers compare it, so a ring
    // is never looked up on another machine.
    std::string host_id();

    // The start time of process pid (field 22 of /proc/<pid>/stat, clock ticks after boot): with the pid it tells a
    // process from a later one that got the same pid. Throws RingError if the process does not exist.
    std::uint64_t process_start_time(int pid);
}  // namespace xblick
