// Publishing and receiving one camera stream in C++, with the topics, messages, versions and rings of Python's
// camera.py (protocol.hpp has the wire protocol). A C++ publisher and a Python subscriber (or the other way round)
// work together; tests/test_cpp_interop.py checks every transport.
//
//   zenoh::Session session = xblick::open_session({});      // the PC's Zenoh router
//   xblick::CameraPublisher camera(session, "cam/azurekinect/color",
//                                      {.encoding = "rgb8", .frame_id = "AzureKinectCamera", .source = "robot"});
//   if (camera.subscribed()) camera.publish(xblick::HostImage{pixels, 720, 1280}, capture_ns);
//
//   xblick::ImageSubscriber images(session, "test/camera/color",
//                                      {.transport = xblick::Transport::Auto});
//   if (auto frame = images.latest(true, std::chrono::milliseconds(100))) { use(frame->data(), frame->height, ...); }
//
// Like the Python classes: a representation is produced only while somebody subscribes to it; a subscriber keeps
// the newest sample and decodes it when it is read; a sample of another protocol version raises ProtocolError.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <zenoh.hxx>

#include "xblick/command.hpp"
#include "xblick/cuda_ring.hpp"
#include "xblick/image.hpp"
#include "xblick/msg.hpp"
#include "xblick/pairing.hpp"
#include "xblick/protocol.hpp"
#include "xblick/ring.hpp"
#include "xblick/ros.hpp"

namespace xblick
{
    class ProtocolError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    // ------------------------------------------------------------------------------------------- sessions

    struct SessionOptions
    {
        std::vector<std::string> connect{};  // host, host:port or a Zenoh locator such as tcp/192.0.2.10:7447
        std::vector<std::string> listen{};
        std::string mode = "peer";  // or "client" (through a router)
        bool multicast_scouting = false;
        std::string config_file{};  // a Zenoh JSON5 configuration instead of the options above
        // Neither connect nor listen: the PC's router (127.0.0.1:7447), listening on a free loopback port, as
        // Python's session.py; false: such a session reaches nobody. A peer with connect and no listen listens on a
        // free loopback port as well (peers behind a router reach each other only directly).
        bool local_router = true;
        // The program's name. Set (Options::session_options sets it): open_session says on stderr, as
        // "<program>: ...", when the options make a peer that connects only to other PCs (across_pcs). It is also the
        // program the session declares (empty: the name it was started by).
        std::string program{};
        // The program's version in its declaration (empty: unknown, null in the record).
        std::string version{};
        // The session declares its program on the network (protocol.hpp kSessions, Python's declaration.py): the
        // liveliness token xblick/sessions/<zid>/program while it is open and a queryable there that answers the
        // record (program, version, the core's version, names, host, the whole command line, process id, start time).
        // The command line is visible to everything that reaches the network. false: a session that does not.
        bool declare = true;
        // Take part in ROS 2 as well (ros.hpp): the session is a ROS node of this name on this domain (ros::of), and
        // the publishers and parameters made with it offer themselves to ROS too (docs/integrations/ros2.md). By
        // default as XBLICK_ROS says (ros::from_environment: a part unless it is 0; RosError for a value other than 0
        // and 1); std::nullopt: no part, nothing of ROS is sent. An empty node name: the program's (node_name_of).
        std::optional<ros::Options> ros = ros::from_environment();
    };

    // xblick/sessions/<zid>/program: a session's declaration (SessionOptions::declare).
    std::string declaration_key(const std::string& zid);
    // Adds a camera or node name (test/camera, test/camera/yolox) to the declaration of `session` once; false when
    // the session does not declare. CameraPublisher and PointCloudPublisher add their camera's (base's parent).
    bool add_name(const zenoh::Session& session, std::string name);

    // tcp/<host>:<port>; a full locator (containing '/') is returned unchanged.
    std::string endpoint(const std::string& host, std::uint16_t port = 7447);
    // A session as the options say. A session that Zenoh cannot open throws std::runtime_error with one line that
    // says why (in client mode: no connect endpoint answered, a client needs its router there). Move the session it
    // returns only before the first object is made on it (ros.hpp: the session stays in place).
    zenoh::Session open_session(const SessionOptions& options);

    // A peer that connects only to other PCs and listens on loopback only (Python's session.across_pcs): its connect
    // endpoints (as endpoint() writes them), else none. Behind a Zenoh 1.10 router peers reach each other only
    // directly, so the programs on the other PCs usually cannot reach it; they can when they listen on network
    // addresses, which these options do not show. None in client mode, with a config file, with a connect endpoint
    // on this PC (this_pc) or with a listen endpoint on a network address (not loopback()).
    std::vector<std::string> across_pcs(const SessionOptions& options);
    // Why such a session usually finds nothing, and the three ways out (client mode, a router on this PC, --listen on
    // the network), with its endpoints (Python's across_pcs_text).
    std::string across_pcs_text(const std::vector<std::string>& endpoints);
    // `host` is this PC: localhost, or it resolves to a loopback, the unspecified (0.0.0.0, ::) or an address a socket
    // can bind to (one of this PC's interfaces; nothing is sent). A name that does not resolve is not this PC.
    bool this_pc(const std::string& host);
    // `host` is localhost or resolves to loopback addresses only.
    bool loopback(const std::string& host);

    // The replies to a query (session.get, with `payload` if given) once Zenoh finalized it, or at a deadline of its
    // timeout and max(500 ms, timeout) more if it does not: Zenoh does not always finalize a query by its timeout (seen
    // in CI), and a recv() loop over a FIFO channel would then wait for good (Python's session.replies). A query not
    // finalized by its deadline is counted (queries_not_finalized()) and said once per selector on stderr.
    std::vector<zenoh::Reply> replies(const zenoh::Session& session, const std::string& selector,
                                      std::chrono::milliseconds timeout,
                                      const std::optional<std::vector<std::uint8_t>>& payload = std::nullopt);
    std::uint64_t queries_not_finalized();

    // The replies to a query that only reads (an info record, rays, a node's parameter values), asked again every
    // 250 ms until a first reply comes or `timeout` passed (Python's session.ask): a session just opened knows no
    // queryable yet, and a query sent before ends at once without a reply. Only for reads: an empty result does not
    // prove that no queryable got the query, so a command would run twice.
    std::vector<zenoh::Reply> ask(const zenoh::Session& session, const std::string& selector,
                                  std::chrono::milliseconds timeout,
                                  const std::optional<std::vector<std::uint8_t>>& payload = std::nullopt);

    // A command was not sent: no queryable for its key became known within the timeout. Safe to try again.
    class NotReached : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    // A command was sent once and no answer came within the timeout: it may have been executed, and it was not sent
    // again (Python's session.NoAnswer). A receiver that does not check the command's deadline (any receiver of a
    // command sent without a target) may still execute it later. A type of its own, so that a caller can tell it
    // from NotReached without reading the text.
    class NoAnswer : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    // A command answered by some node(s), while the query was still open at its timeout (the "Timeout" reply,
    // timed_out): another node may have got it and may still execute it, or the answering node held the query after
    // its reply. Not sent again (Python's session.PartlyAnswered).
    class PartlyAnswered : public NoAnswer
    {
    public:
        using NoAnswer::NoAnswer;
    };

    // A command sent and answered by more than one node: each of them executed it. Not sent again (Python's
    // session.SeveralExecuted).
    class SeveralExecuted : public NoAnswer
    {
    public:
        using NoAnswer::NoAnswer;
    };

    // The error reply "Timeout" (Python's session.timed_out). A queryable ends its part of a query when it drops the
    // query object. At the query's timeout, the querying session ends a query that a queryable has not finished yet
    // and adds this reply itself: nothing comes over the network for it. The queryable may still be working on it.
    bool timed_out(const zenoh::Reply& reply);

    // The replies to a command (a parameter set): sent once to each queryable of `selector`, after the session knows
    // one (a querier's matching status). Waits at most `timeout` for it (NotReached: nothing sent), then for the
    // replies up to `timeout` and max(500 ms, timeout) more if Zenoh does not end the query (replies()); an empty
    // result is returned as it is. Never sent again: an empty result does not tell "no queryable got it" from "a
    // queryable got it and gave no reply" (Python's session.ask_once). Two queryables on the key both get it.
    std::vector<zenoh::Reply> ask_once(const zenoh::Session& session, const std::string& selector,
                                       std::chrono::milliseconds timeout,
                                       const std::optional<std::vector<std::uint8_t>>& payload = std::nullopt);

    // ------------------------------------------------------------------- a command's node and deadline (0.5.13)
    // protocol.hpp (kCommandField) has the format; Python's session module the same functions.

    // A command refused by the node it was meant for: its deadline had passed when the node came to execute it. That
    // node did not execute it; not sent again (Python's session.Expired).
    class Expired : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    // The clock of a command's deadline: CLOCK_BOOTTIME in ns (counts a suspend, never set, only slewed by NTP).
    std::int64_t command_clock_ns();

    // The node a command is meant for: its session's Zenoh id, and the least its clock can be ahead of this
    // program's (Python's session.CommandTarget).
    struct CommandTarget
    {
        std::string node;
        std::int64_t offset_ns = 0;
        // `timeout` from now on the node's clock, at the earliest that can be (late only by the clocks' rate
        // difference, at most 0.05 % with NTP, over the time from the read to the node's check).
        // std::invalid_argument for a timeout above 24 h; std::runtime_error ("not sent") for a deadline out of range.
        std::int64_t deadline_ns(std::chrono::milliseconds timeout) const;
    };

    // The target of a command from the one successful reply of the read before it and the time the read ended
    // (command_clock_ns() once it returned); empty when the reply has no attachment of a receiver of 0.5.13 (an older
    // one, or one that does not take part: nothing new is sent or checked). The deadline converted with it comes out
    // early by at most the read's round trip, late only by the clocks' rate difference (deadline_ns).
    // std::runtime_error ("not sent: ...") for an attachment it cannot read.
    std::optional<CommandTarget> target_of(const zenoh::Reply& reply, std::int64_t answered_ns);

    // ask_once, the query naming `to` and its deadline, `timeout` from the moment it is sent, on to's clock. A
    // receiver that checks (CommandQuery) and is not that node does not execute it, nor does that node after the
    // deadline. A receiver that does not check executes it as before; the sender cannot tell which it is until it has
    // an answer (checked()). Without `to` the same as the overload above (nothing attached).
    std::vector<zenoh::Reply> ask_once(const zenoh::Session& session, const std::string& selector,
                                       std::chrono::milliseconds timeout,
                                       const std::optional<std::vector<std::uint8_t>>& payload,
                                       const std::optional<CommandTarget>& to);

    // An error reply's case (protocol.hpp): refused as past the deadline (kExpired), refused by a node the command was
    // not meant for (kNotTarget).
    bool expired(const zenoh::Reply& reply);
    bool not_target(const zenoh::Reply& reply);

    // True when a successful reply to a command sent to `to` says that its node checked node and deadline before
    // executing it; false otherwise (Python's session.checked).
    bool checked(const zenoh::Reply& reply, const std::optional<CommandTarget>& to);

    // What a receiver's replies carry (to a command and to the read before it): its session's Zenoh id `zid`
    // (session.get_zid().to_string()), its clock now and what it checked ("node", "deadline"; none for a read).
    std::string answer_attachment(const std::string& zid, const std::vector<std::string>& checked = {});

    // A receiver's refusal of a command before its effect (CommandQuery::check_node, check_deadline). `schema`:
    // kExpired, kNotTarget or
    // kInvalidCommand; what() the error reply's text.
    class CommandRefused : public std::runtime_error
    {
    public:
        CommandRefused(std::string schema, const std::string& text)
            : std::runtime_error(text), schema(std::move(schema))
        {
        }
        std::string schema;
    };

    // A command as its receiver got it (the query on the key `what`; `zid`: the receiver's session): the node it is
    // meant for and its deadline, as a sender attached them (command.hpp has the rules). A query without them (an older
    // sender, a command without a target) has nothing to check. Construct it inside the code that answers the query
    // with an error reply on any exception: it never throws itself, and no input ends the receiver. Call
    // check_node() on receipt, before anything else (an unreadable attachment, or a command meant for another node),
    // and check_deadline() as late as possible before the command takes effect; reply with answer_attachment(zid,
    // checked()); send a refusal with refuse(). Python's session.CommandQuery.
    class CommandQuery
    {
    public:
        CommandQuery(const zenoh::Query& query, std::string zid, std::string what);
        // CommandRefused (kInvalidCommand, kNotTarget) when it must not be executed by this node
        void check_node();
        // CommandRefused (kExpired, kInvalidCommand for a deadline more than 24 h ahead) when the deadline has passed
        // on this PC's command_clock_ns(); runs check_node() first. Afterwards checked() names both checks.
        void check_deadline();
        const std::vector<std::string>& checked() const { return checked_; }
        const std::string& zid() const { return zid_; }

    private:
        std::string zid_, what_;
        CommandAttachment attachment_;
        std::vector<std::string> checked_;
    };

    // The error reply of a refused command: its text, its case in the encoding ("text/plain;<schema>").
    void refuse(const zenoh::Query& query, const CommandRefused& refused);

    // Topic names (protocol.hpp): "<base>/image_raw" etc.; base is checked against ROS's name rules.
    std::string check_name(const std::string& path);
    std::string stream_base(const std::string& ns, const std::string& camera, const std::string& stream);

    // ----------------------------------------------------------------------------------------- publisher

    // Pixels in device memory (rows without padding), ready on stream (a CUstream; 0 = the legacy default stream).
    struct DeviceImage
    {
        std::uint64_t ptr = 0;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::uint64_t stream = 0;
    };

    struct PublisherOptions
    {
        std::string encoding{};  // ROS encoding of the frames, e.g. rgb8, bgr8, 16UC1
        std::string frame_id{};
        std::string source{};  // e.g. "robot" or "sim": in every frame's attachment
        std::optional<CameraInfo> camera_info{};
        std::string codec = kJpeg;  // of image_raw/compressed
        int jpeg_quality = 85;
        std::uint32_t slots = 32;  // ring slots: a reader has slots - 1 frame periods for a frame (~1 s at 30 Hz)
        std::optional<int> cuda_device{};  // offer image_raw/cuda with a CUDA ring on this device
        // The representations offered, of "raw", "compressed", "shm" and "cuda" (needs cuda_device); empty: all (cuda
        // with cuda_device). A relay (xblick-relay) offers only its host's rings, {"shm", "cuda"}: raw, compressed
        // and camera_info come once, from the source.
        std::vector<std::string> transports{};
        // A relay's: the host id of the publisher whose stream it republishes; in the info record as "relayed_from"
        // (null otherwise). Relays relay no stream whose record has it.
        std::string relayed_from{};
    };

    // A frame of another publisher's stream as that one sent it (a relay): its header (capture stamp, frame_id) and
    // its attachment's seq and source, so that pairing and pose lookups work on the relayed frames as on the source's.
    struct Origin
    {
        Header header;
        std::int64_t seq = 0;
        std::string source;
    };

    class CameraPublisher
    {
    public:
        CameraPublisher(zenoh::Session& session, const std::string& base, PublisherOptions options);
        ~CameraPublisher();
        CameraPublisher(const CameraPublisher&) = delete;
        CameraPublisher& operator=(const CameraPublisher&) = delete;

        // True if anybody takes images of this stream (any representation): render only then.
        bool subscribed() const;
        // Publish one frame captured at stamp_ns (nanoseconds since the epoch; default: now). Returns the topics it
        // went to (empty without subscribers).
        std::vector<std::string> publish(const HostImage& image, std::optional<std::int64_t> stamp_ns = {});
        // A frame in device memory: needs cuda_device; CPU representations download it once.
        std::vector<std::string> publish(const DeviceImage& image, std::optional<std::int64_t> stamp_ns = {});
        // A frame of another publisher (Origin): its header and seq instead of this publisher's own.
        std::vector<std::string> publish(const HostImage& image, const Origin& origin);
        std::vector<std::string> publish(const DeviceImage& image, const Origin& origin);

        // A frame that is a JPEG already (an MJPEG camera): image_raw/compressed sends these bytes instead of encoding
        // the pixels again (needs codec jpeg). Decode the JPEG only if needs_pixels(); otherwise publish_jpeg().
        std::vector<std::string> publish(const HostImage& image, std::optional<std::int64_t> stamp_ns,
                                         std::span<const std::uint8_t> jpeg);
        std::vector<std::string> publish(const DeviceImage& image, std::optional<std::int64_t> stamp_ns,
                                         std::span<const std::uint8_t> jpeg);
        // True if anybody takes the pixels of this stream (raw, shm, cuda), not only the JPEG.
        bool needs_pixels() const;
        // The frame as JPEG only: image_raw/compressed and camera_info. A subscriber to the pixels that appeared
        // since needs_pixels() was asked gets the next frame.
        std::vector<std::string> publish_jpeg(std::span<const std::uint8_t> jpeg,
                                              std::optional<std::int64_t> stamp_ns = {});
        // What <base>/info answers (JSON).
        std::string info_record() const;
        // The GPU link's counters (ros.hpp GpuLink, ros_cuda.hpp): none without the link (no cuda_device, no part in
        // ROS 2, or XBLICK_ROS_CUDA=0).
        std::optional<ros_cuda::LinkStats> ros_gpu_stats() const;

        const std::string& base() const { return base_; }

    private:
        struct Impl;
        std::string base_;
        std::unique_ptr<Impl> impl_;
    };

    // A camera's fixed transforms between its streams' frames, on <camera>/tf_static (protocol.hpp): published once
    // and answered on request for as long as this object lives, as ROS's latched /tf_static. Also answered to
    // zenoh-ext's AdvancedSubscriber (tf_static/@adv/pub/<zid>/**), so that a late transient-local ROS subscriber
    // behind zenoh-bridge-ros2dds gets them.
    class StaticTransformPublisher
    {
    public:
        StaticTransformPublisher(zenoh::Session& session, const std::string& camera, TFMessage transforms);
        StaticTransformPublisher(const StaticTransformPublisher&) = delete;
        StaticTransformPublisher& operator=(const StaticTransformPublisher&) = delete;

        const std::string& key() const { return key_; }

    private:
        std::string key_;
        std::vector<std::uint8_t> payload_;
        std::optional<zenoh::Queryable<void>> queryable_;
        std::optional<zenoh::Queryable<void>> history_queryable_;
        std::unique_ptr<ros::Publisher> ros_;  // --ros: on ROS's /tf_static, kept for late ROS subscribers
    };

    // A camera's fixed transforms from <camera>/tf_static, as Python's StaticTransformSubscriber: those its publisher
    // answers when asked (in a thread of its own, asked again until an answer comes or timeout ends: right after a
    // session connects, a query can end at once without answers) and every later publication (a restarted camera).
    // `ros` (a session with --ros): ROS 2's global /tf_static instead, every publisher's, also those that published
    // before this subscriber started (their caches, transient local), `camera` unused; latest() then holds every
    // transform received, the newest per child frame (as tf2 keeps them).
    class StaticTransformSubscriber
    {
    public:
        StaticTransformSubscriber(zenoh::Session& session, const std::string& camera,
                                  std::chrono::milliseconds timeout = std::chrono::seconds(10), bool ros = false);
        ~StaticTransformSubscriber();
        StaticTransformSubscriber(const StaticTransformSubscriber&) = delete;
        StaticTransformSubscriber& operator=(const StaticTransformSubscriber&) = delete;

        // The newest transforms; empty until they came. Throws ProtocolError for a publisher of another version.
        std::optional<TFMessage> latest() const;
        // How often the transforms came (answers and publications): a change says that latest() may have changed.
        std::uint64_t received() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    // The pose of frame `child` in frame `parent` (p_parent = R p_child + t) through the transforms of a TFMessage,
    // as a row-major rotation and a translation; empty if they do not connect the two frames.
    struct RigidTransform
    {
        std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
        Vector3 translation;
    };
    std::optional<RigidTransform> lookup(const TFMessage& transforms, const std::string& parent,
                                         const std::string& child);

    // A camera's IMU on <camera>/imu (protocol.hpp): one message per sample, dropped under congestion as images are.
    class ImuPublisher
    {
    public:
        ImuPublisher(zenoh::Session& session, const std::string& camera);
        // True if anybody subscribes: read the samples from the device only then, if that costs anything.
        bool subscribed() const;
        void publish(const Imu& sample);

    private:
        zenoh::Publisher publisher_;
        std::unique_ptr<ros::Publisher> ros_;  // --ros: /<camera>/imu
    };

    // How a camera's publishers are doing, on <camera>/diagnostics (protocol.hpp), as ROS's /diagnostics: publish one
    // DiagnosticArray about once a second, its status named after the publisher ("azure_kinect", "body_tracking").
    // Every sample's attachment names the session ("zid"): the nodes of a camera share its diagnostics key.
    class DiagnosticsPublisher
    {
    public:
        DiagnosticsPublisher(zenoh::Session& session, const std::string& camera);
        void publish(const DiagnosticArray& diagnostics);

    private:
        zenoh::Publisher publisher_;
        std::unique_ptr<ros::Publisher> ros_;  // --ros: /<camera>/diagnostics
        std::string zid_;
        std::int64_t seq_ = 0;
    };

    // One ROS type on one key (checked as a topic name), the type's encoding on every sample: for a topic that has no
    // publisher of its own here (e.g. a controller's records, protocol.hpp).
    class MessagePublisher
    {
    public:
        MessagePublisher(zenoh::Session& session, const std::string& key, const protocol::MessageType& type);
        template <class Message>
        void publish(const Message& message)
        {
            put(message.serialize());
        }
        void put(const std::vector<std::uint8_t>& payload);

    private:
        zenoh::Publisher publisher_;
        std::unique_ptr<ros::Publisher> ros_;  // --ros: /<key>, for a ROS type and no command key
    };

    // A camera's moving transforms on <camera>/tf (protocol.hpp), as ROS's /tf: its pose in the world (parent frame
    // "world"), stamped as the image taken from it or, from a robot's state, at that state's time.
    class TransformPublisher
    {
    public:
        TransformPublisher(zenoh::Session& session, const std::string& camera);
        void publish(const TFMessage& transforms);

    private:
        zenoh::Publisher publisher_;
        std::unique_ptr<ros::Publisher> ros_;  // --ros: ROS's /tf
    };

    // A robot's state below its namespace (protocol.hpp): <robot>/joint_states, <robot>/tf, <robot>/odom and
    // <robot>/wrench/<sensor>, one message per state sample, every message sent. A force-torque sensor's key is
    // declared with its first measurement.
    class RobotStatePublisher
    {
    public:
        RobotStatePublisher(zenoh::Session& session, const std::string& robot);
        void joint_states(const JointState& message);
        void tf(const TFMessage& message);
        void odom(const Odometry& message);
        void wrench(const std::string& sensor, const WrenchStamped& message);

    private:
        zenoh::Session& session_;
        std::string robot_;
        zenoh::Publisher joint_states_, tf_, odom_;
        std::map<std::string, zenoh::Publisher> wrenches_;
        // --ros: /<robot>/joint_states, /tf, /<robot>/odom, /<robot>/wrench/<sensor>
        std::unique_ptr<ros::Publisher> ros_joint_states_, ros_tf_, ros_odom_;
        std::map<std::string, std::unique_ptr<ros::Publisher>> ros_wrenches_;
    };

    // ---------------------------------------------------------------------------------------- subscriber

    enum class Transport
    {
        Auto,        // SubscriberOptions::local on the publisher's host, else SubscriberOptions::remote
        Compressed,  // sensor_msgs/CompressedImage (JPEG, PNG or RVL), decoded
        Raw,         // sensor_msgs/Image
        Shm,         // host ring, same host only
        Cuda,        // CUDA ring, same host only; frames in device memory
        // A ROS 2 topic of a ROS publisher, for a session in ROS 2 (ros.hpp): the stream's (/<base>/image_raw, a
        // cloud's /<base>/cloud), or the topic a base with a leading '/' names. Auto takes it for a ROS name, and for
        // a stream of ours that only a ROS publisher of another program offers. Its frames carry no seq, source or
        // publish time.
        Ros,
        // image_transport's compressed topic (<image topic>/compressed) of a ROS name (Transport::Compressed of a
        // base with a leading '/'), or the one Auto takes when a ROS publisher offers only that. Not to be asked for.
        RosCompressed,
        // image_transport's compressedDepth topic (<image topic>/compressedDepth: ROS's compressedDepth plugin, 16-bit
        // or 32FC1 depth, image.hpp), the one Auto takes when a ROS publisher offers neither the image topic nor its
        // compressed one. Not to be asked for.
        RosCompressedDepth,
    };

    const char* to_string(Transport transport);

    struct SubscriberOptions
    {
        Transport transport = Transport::Auto;
        // What Auto takes on the publisher's host: Auto (the publisher's CUDA ring if this process sees the ring's
        // GPU, else Shm), Shm, or Cuda (the CUDA ring where the publisher offers one, else Shm)
        Transport local = Transport::Auto;
        // The GPU this subscriber works on (an ordinal, as CUDA numbers them): the frames of a CUDA ring arrive on
        // it, copied from GPU to GPU if the ring is on another one. Unset: the CPU, so local Auto takes shared memory
        // (a CUDA ring only for a subscriber that works on a GPU), and the frames of a CUDA ring stay on its GPU. A ROS
        // form's frames that a ROS publisher has as CUDA buffers on this GPU arrive on it as well (ros_cuda.hpp
        // Receiver; ros_gpu_stats()), the others as bytes.
        std::optional<int> cuda_device{};
        Transport remote = Transport::Compressed;  // what Auto takes for a publisher on another host (or Raw)
        bool copy = true;                          // false: shm/cuda frames are views into the ring (check is_valid)
        bool camera_info = false;                  // also receive the stream's CameraInfo
        // A ROS topic (/cam/calibration) to read the CameraInfo from while a ROS form is read (camera_info then on);
        // empty: the stream's, or for a ROS form image_transport's beside the image topic (ros::camera_info_topic).
        std::string camera_info_topic{};
        // At most so many frames a second, the newest; the others are skipped before they are decoded or read from
        // their ring (skipped_max_hz()). By their stamps, so that a camera's streams take the same frames. 0: all.
        double max_hz = 0.0;
        // Keep the last so many samples as they came, not decoded, for at(): a partner stream (the depth of a colour
        // frame) read at the stamps of the primary one, so that only the partner frames that are paired are decoded.
        std::size_t keep_recent = 0;
    };

    struct Frame
    {
        Header header;
        std::string encoding;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::uint32_t step = 0;
        std::string key;
        std::int64_t receive_ns = 0;
        std::optional<std::int64_t> seq;
        std::string source;
        std::optional<std::int64_t> publish_ns;

        // Host pixels (every transport but cuda): data() is valid while the frame exists (copies), or while
        // is_valid() holds (shm views).
        const std::uint8_t* data() const { return view != nullptr ? view : pixels.data(); }
        // Device pixels (cuda): a pointer on device `device`.
        std::uint64_t device_ptr = 0;
        int device = -1;

        // False once the publisher has begun overwriting a zero-copy frame; always true for copies.
        bool is_valid() const { return valid ? valid() : true; }

        std::vector<std::uint8_t> pixels;
        const std::uint8_t* view = nullptr;
        std::shared_ptr<const void> owner;  // keeps the payload, a copy's DeviceBuffer or a view's ring alive
        std::function<bool()> valid;

        std::int64_t stamp_ns() const { return header.stamp.to_ns(); }
        // From capture to the put of this frame's sample (its encoding or ring copy in it), on the publisher's clock
        // alone: publish_ns - stamp (empty without publish_ns or stamp).
        std::optional<std::int64_t> age_at_send_ns() const
        {
            if (not publish_ns or stamp_ns() == 0)
            {
                return std::nullopt;
            }
            return *publish_ns - stamp_ns();
        }
    };

    namespace detail
    {
        struct Item;  // a sample as received (stream.hpp)
    }

    class ImageSubscriber
    {
    public:
        ImageSubscriber(zenoh::Session& session, const std::string& base, SubscriberOptions options = {});
        ~ImageSubscriber();
        ImageSubscriber(const ImageSubscriber&) = delete;
        ImageSubscriber& operator=(const ImageSubscriber&) = delete;

        // The newest frame; with consume each frame is returned once. With a timeout, wait that long for a new one.
        // Empty as well for a frame overwritten in its ring before it could be read (counted in stale()).
        std::optional<Frame> latest(bool consume = true,
                                    std::optional<std::chrono::milliseconds> timeout = std::nullopt);
        // The transport in use; empty while Auto has not found the publisher yet.
        std::optional<Transport> transport() const;
        // Why it took that transport, e.g. "cuda (on this subscriber's GPU)"; empty before it chose.
        std::string transport_reason() const;
        std::optional<CameraInfo> camera_info() const;
        std::uint64_t received() const;
        // Frames replaced by a newer one before latest() took them; none with keep_recent (a partner's frames stay
        // available to at() until they leave the kept window).
        std::uint64_t overwritten() const;
        std::uint64_t stale() const;
        std::uint64_t skipped_max_hz() const;  // frames left out for SubscriberOptions::max_hz
        // Of the samples kept (SubscriberOptions::keep_recent): the one nearest to stamp_ns, at most max_offset away,
        // decoded now (the others never are); empty if there is none (yet), or its ring slot was overwritten.
        // partner_of (pairing.hpp) of recent_stamps() says which stamp a primary frame takes, once no nearer can come.
        std::optional<Frame> at(std::int64_t stamp_ns, std::chrono::nanoseconds max_offset);
        std::vector<std::int64_t> recent_stamps() const;  // the samples kept, oldest first, not decoded
        std::uint64_t decoded() const;                     // frames decoded (or read from their ring) and handed out
        // The counters of the CUDA buffers taken from ROS publishers (ros_cuda.hpp Receiver) while a ROS form is read
        // with a cuda_device; none otherwise.
        std::optional<ros_cuda::ReceiverStats> ros_gpu_stats() const;

    private:
        struct Impl;
        std::optional<Frame> decode(const std::shared_ptr<const detail::Item>& held, Transport t);
        std::unique_ptr<Impl> impl_;
    };

    // The info records (JSON text) of the publishers of a stream, as <base>/info answers.
    std::vector<std::string> query_info(zenoh::Session& session, const std::string& base,
                                        std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));

    // An info record read (image streams): who publishes, where, what.
    struct StreamInfo
    {
        std::string kind;  // "image"
        std::string xblick;      // the publisher's version
        std::string host;        // its host id (the boot id)
        std::vector<std::string> transports;
        std::string encoding;
        std::string frame_id;
        std::string source;
        std::string cuda_device_uuid;  // empty: no CUDA ring
        std::string relayed_from;      // empty: not a relay

        bool offers(const std::string& transport) const;
    };
    // The records of query_info, read; ProtocolError for a record of another version.
    std::vector<StreamInfo> stream_infos(zenoh::Session& session, const std::string& base,
                                         std::chrono::milliseconds timeout = std::chrono::milliseconds(1000));
    // (this host's id, to compare with StreamInfo::host: host_id(), ring.hpp)

    // seq, publish_ns and source of a frame sample's attachment (the JSON text Zenoh carries; all empty for none);
    // ProtocolError for an attachment of another version.
    struct FrameMeta
    {
        std::optional<std::int64_t> seq;
        std::optional<std::int64_t> publish_ns;
        std::string source;
    };
    FrameMeta parse_frame_attachment(const std::string& attachment);
}  // namespace xblick
