// ROS 2 on Zenoh in C++: the keys, type names, type hashes, attachments, QoS strings and liveliness tokens of
// rmw_zenoh_cpp 0.2.13 (ROS 2 Jazzy) and 0.10.6 or newer (ROS 2 Lyrical), so that every C++ program takes part in
// ROS 2 as well (by default; --no-ros or XBLICK_ROS=0 turn it off). The same rules, names and bytes as Python's xblick.ros (its module docstring and
// docs/integrations/ros2.md are the reference; tests/test_cpp_ros.py compares the two byte for byte):
//
//   data key   <domain>/<topic without the leading />/<pkg>::msg::dds_::<Name>_/RIHS01_<type hash>
//              (a service: its name, <pkg>::srv::dds_::<Name>_ and the service type's hash)
//   attachment int64 sequence number, int64 source timestamp (ns since the epoch), a length byte 16 and the gid
//              (zenoh-ext's serialization, 33 bytes); a ROS subscriber drops a sample without it
//   tokens     @ros2_lv/<domain>/<zid>/<node id>/<node id>/NN/<enclave>/<namespace>/<node> (a node);
//              @ros2_lv/<domain>/<zid>/<node id>/<entity id>/<MP|MS|SS|SC>/<enclave>/<namespace>/<node>/<topic>/
//              <type>/RIHS01_<hash>/<qos> (an entity), every '/' inside a name written as '%'; Lyrical adds an
//              optional 14th part backends:<name>:<meta>;...
//   gid        XXH3-128 of the entity's token key (ros_gid.hpp)
//   buffers    a type with a uint8[] field, its own or a used type's (buffer_fields()), goes from a Lyrical publisher
//              to a volatile subscription only on <key>/_buf_cpu and <key>/_buf/<subscription gid>, never on the key
//              while it knows every subscription; each message on the key or on these, never on both. A volatile
//              Subscriber of such a type says backends:cpu: and reads <key>/_buf_cpu as well (plain CDR, ROS's
//              attachment). Publishers stay as they are: a Lyrical subscription reads the key.
//
// A session takes part when open_session got SessionOptions::ros (by default from_environment(); Options::
// add_session_options): ros::of(session) is then its Ros, else null. CameraPublisher, PointCloudPublisher, the
// transform, robot state, diagnostics, IMU and message publishers mirror their samples onto ROS's keys while a ROS
// subscriber is there (mirror, mirror_tf), and Parameters serves ROS's parameter services on the program's node. A
// session without a part sends nothing of ROS.
//
// /tf_static (transient local): rmw_zenoh uses zenoh-ext's AdvancedPublisher with a cache, which zenoh-c offers only
// with its unstable API (not in its standalone release). Publisher here stands in for it with the parts a late ROS
// subscriber uses: the cache's liveliness token and queryable <key>/@adv/pub/<zid>/<id>/_ answering the last `depth`
// samples. Not offered: sample sequence numbers and heartbeats (sample-miss detection); a late subscriber may get a
// cached sample twice (its history query and its late-publisher query), which tf2 takes as the same transform.
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <zenoh.hxx>

#include "xblick/protocol.hpp"
#include "xblick/ros_cuda.hpp"
#include "xblick/ros_gid.hpp"

namespace xblick::ros
{
    inline constexpr std::string_view kRmwZenoh = "0.2.13";  // the rmw_zenoh_cpp version followed (ROS 2 Jazzy)
    inline constexpr std::string_view kAdminSpace = "@ros2_lv";
    inline constexpr const char* kDomainEnv = "ROS_DOMAIN_ID";
    inline constexpr int kDefaultDomain = 0;
    inline constexpr std::size_t kAttachmentSize = 33;

    // rmw's QoS enums (rmw/types.h)
    inline constexpr int kSystemDefault = 0, kReliable = 1, kBestEffort = 2;
    inline constexpr int kTransientLocal = 1, kVolatile = 2;
    inline constexpr int kKeepLast = 1, kKeepAll = 2;

    // the 14th token part of a subscription that takes buffer types as plain CDR, and the key below a buffer type's
    // key where Lyrical's publishers send to such subscriptions
    inline constexpr std::string_view kCpuBackends = "backends:cpu:";
    inline constexpr std::string_view kCpuGroup = "_buf_cpu";

    inline constexpr std::string_view kNode = "NN", kPublisher = "MP", kSubscription = "MS", kService = "SS",
                                      kClient = "SC";

    // A name, a type or a value that ROS 2 does not take.
    class RosError : public std::invalid_argument
    {
    public:
        using std::invalid_argument::invalid_argument;
    };

    // ------------------------------------------------------------------------------------------ names and keys

    // `name` as a fully qualified ROS name (/a/b): RosError otherwise, saying why.
    std::string check_ros_name(const std::string& name, const std::string& what = "topic");
    // The ROS topic of one of our keys: "/" + key (RosError for a key that is no ROS name, e.g. one with '-').
    std::string topic_of(const std::string& key);
    std::string check_node_name(const std::string& name);
    // A program's ROS node name by default: every character but a letter, digit or '_' written as '_'.
    std::string node_name_of(const std::string& program);
    // /a/b/c -> (/a/b, c); /c -> (/, c).
    std::pair<std::string, std::string> split_node(const std::string& fully_qualified);
    // A Durchblick name (a camera, test/cam-1) as a ROS namespace, each part as node_name_of writes it (/test/cam_1);
    // "/" for an empty name.
    std::string namespace_of(const std::string& name);
    // (namespace, node) of a Durchblick name as a ROS node, each part as node_name_of writes it: test/cam-1/probe ->
    // (/test/cam_1, probe). RosError for an empty name.
    std::pair<std::string, std::string> node_of(const std::string& name);
    inline constexpr const char* kEnableEnv = "XBLICK_ROS";  // 0: no part in ROS 2; 1 or unset: a part (the default)
    // false when XBLICK_ROS is 0, true when it is 1 or unset; RosError for any other value.
    bool enabled_by_environment();
    inline constexpr std::string_view kClockTopic = "/clock";  // ROS's global clock (use_sim_time)
    inline constexpr int kClockWaitS = 1;  // --ros-clock: a program without its clock's publisher by then is told
    // sensor_msgs/msg/Image -> sensor_msgs::msg::dds_::Image_ (a service: pkg/srv/Name -> pkg::srv::dds_::Name_).
    std::string dds_type(const std::string& type_name);
    std::string type_of_dds(const std::string& dds);
    std::string data_key(int domain, const std::string& name, const std::string& type_name,
                         const std::string& type_hash);
    std::string mangle(std::string name);
    std::string demangle(std::string name);
    // A domain id given as an option, else ROS_DOMAIN_ID, else 0; RosError for one that is no integer >= 0.
    int domain_from(std::optional<std::int64_t> value);
    // A key with a part "command" (what a robot is told): never offered to ROS, whatever its type.
    bool commanding(const std::string& key);

    // ----------------------------------------------------------------------------------------------- attachment

    struct Attachment
    {
        std::int64_t seq = 0;
        std::int64_t source_ns = 0;
        Gid gid{};
    };
    std::vector<std::uint8_t> encode_attachment(std::int64_t seq, std::int64_t source_ns, const Gid& gid);
    // RosError for anything but an rmw_zenoh attachment.
    Attachment decode_attachment(std::span<const std::uint8_t> data);

    // ------------------------------------------------------------------------------------------------------ QoS

    // The QoS of a ROS entity as rmw_zenoh writes it into the token: each field empty when it is rmw_zenoh's
    // default (reliable, volatile, keep last, depth 42, durations infinite, liveliness automatic).
    struct Qos
    {
        int reliability = kReliable;
        int durability = kVolatile;
        int history = kKeepLast;
        int depth = 42;
        std::pair<std::string, std::string> deadline{};
        std::pair<std::string, std::string> lifespan{};
        std::string liveliness{};
        std::pair<std::string, std::string> lease{};

        std::string key() const;
        static Qos parse(const std::string& text);  // RosError for one rmw_zenoh would not read
        bool operator==(const Qos&) const = default;
    };

    inline const Qos kStreamQos{.depth = 1};                                   // our streams
    inline const Qos kTfQos{.depth = 100};                                     // /tf, as tf2's broadcaster
    inline const Qos kTfStaticQos{.durability = kTransientLocal, .depth = 1};  // /tf_static, as tf2's static one
    inline const Qos kTfStaticReadQos{.durability = kTransientLocal, .depth = 100};  // tf2's static listener
    inline const Qos kParametersQos{.depth = 1000};                            // rmw_qos_profile_parameters

    // ---------------------------------------------------------------------------------------------------- tokens

    struct Token
    {
        int domain = 0;
        std::string zid, node_id, entity_id, kind, enclave, ns, node;
        std::optional<std::string> topic, type_name, type_hash;  // an entity's (pkg/msg/Name), not a node's
        std::optional<Qos> qos;
        std::optional<std::string> backends;  // the optional 14th part (rmw_zenoh 0.10.5 and later), "backends:..."

        std::string key() const;
        std::string fully_qualified_node() const;
        static Token parse(const std::string& key);  // RosError for one rmw_zenoh would refuse
    };

    // ------------------------------------------------------------------------------------ type hashes (RIHS01)

    // The RIHS01 hash of every type Durchblick carries to ROS: Python's ros.TYPE_HASHES (ROS 2 Jazzy's own, vision_msgs
    // those of ros-jazzy-vision-msgs 4.1.1), compared by tests/test_cpp_ros.py.
    const std::map<std::string, std::string>& type_hashes();
    std::string hash_of(const std::string& type_name);  // RosError for a type not offered to ROS

    // Every message type of type_hashes(): true when ROS 2 Lyrical's typesupport says it has Buffer fields: Python's
    // ros.BUFFER_FIELDS, compared by tests/test_cpp_ros.py.
    const std::map<std::string, bool>& buffer_fields();
    // The 14th token part of an entity of ours: kCpuBackends for a subscription of a buffer type that is not transient
    // local, else none. RosError for a subscription's message type not in buffer_fields().
    std::optional<std::string> backends_of(std::string_view kind, const std::string& type_name, const Qos& qos);
    // <key>/_buf_cpu
    std::string cpu_group_key(const std::string& key);

    // ---------------------------------------------------------------------------------- taking part on the wire

    // How a session takes part in ROS 2 (--ros-domain, --ros-node, --ros-params-writable, --ros-clock;
    // Options::add_session_options): its node <ns>/<node>; follow_parameters: that name is a default, and the
    // program's first parameters give the node theirs (Ros::parameters_node).
    struct Options
    {
        int domain = kDefaultDomain;
        std::string node{};  // empty: node_name_of(the program), filled in by open_session
        bool params_writable = false;  // ROS clients may set parameters; by default ROS's sets are refused
        std::string ns = "/";
        bool follow_parameters = false;
        std::string clock{};  // --ros-clock: the key of ours that goes to ROS's /clock (empty: none)
    };
    // A program's part when nothing is given (Python's session.default_ros_options): ROS_DOMAIN_ID's domain (else 0),
    // the node the program's name (node_name_of; empty program: left empty) in camera's namespace (namespace_of),
    // renamed after the program's first parameters.
    Options default_options(const std::string& program, const std::string& camera = "");
    // SessionOptions::ros by default: none when XBLICK_ROS is 0, else default_options(""). RosError for an XBLICK_ROS
    // or ROS_DOMAIN_ID that is not understood.
    std::optional<Options> from_environment();
    // The options of a command line's values (Python's session.ros_options_from_args); none with --no-ros or
    // XBLICK_ROS=0 (--ros changes nothing). RosError for --ros-domain, --ros-node, --ros-params-writable or
    // --ros-clock while ROS is off, a domain that is no integer >= 0, a node name ROS does not take (NAME or
    // /namespace/NAME). Without --ros-node: default_options(program, camera).
    std::optional<Options> options_from(bool no_ros, std::optional<std::int64_t> domain,
                                        const std::optional<std::string>& node, bool params_writable,
                                        const std::optional<std::string>& clock, const std::string& program,
                                        const std::string& camera = "");

    // A ROS node: its name, namespace and id.
    struct NodeName
    {
        std::string name;
        std::string ns = "/";
        std::string id;
        std::string fully_qualified() const;
    };

    class Subscriber;
    struct Attachment;

    namespace detail
    {
        using StaticTransform = std::tuple<std::string, std::array<double, 3>, std::array<double, 4>>;
    }

    // A session's part in ROS 2: domain, its program's node, the entity ids (one counter per session, as rmw_zenoh's
    // per context). The program's node carries its publishers, subscribers and its first parameters' services
    // (parameters_node); another node of its fully qualified name in ROS's graph (at the start or later) is said in
    // one line, as ROS's tools only warn. Python's ros.Ros is the same.
    class Ros : public std::enable_shared_from_this<Ros>
    {
    public:
        Ros(std::string zid, const Options& options);
        ~Ros();
        Ros(const Ros&) = delete;
        Ros& operator=(const Ros&) = delete;
        int domain() const { return options_.domain; }
        const std::string& zid() const { return zid_; }
        bool params_writable() const { return options_.params_writable; }
        bool follow_parameters() const { return options_.follow_parameters; }
        const std::string& clock() const { return options_.clock; }  // --ros-clock's key, empty without
        NodeName node() const;  // the program's node, now
        std::string next_id();
        // A new node's name and id (its token: RosNode).
        NodeName add_node(const std::string& name, const std::string& ns = "/");
        // The program's node for the parameters `name` (their Durchblick name) when they are its first: renamed after
        // them (node_of) when its name is a default (follow_parameters), its entities declared anew. None for further
        // parameters, which make a node of their own (RosNode).
        // `session`: the parameters' (open_session's own may have moved since).
        std::optional<NodeName> parameters_node(const std::string& name, zenoh::Session& session);
        void release_parameters();  // the program's parameters closed
        // `retire` is called with the program's first parameters (parameters_node): the node's services without
        // parameters (Parameters::serve_none) give way to theirs.
        void until_parameters(std::function<void()> retire);
        // An entity on the program's node, made as `as_made`: `rename` called with the node's new name whenever it is
        // renamed (at once when that happened meanwhile). The id for leave().
        std::uint64_t enroll(const NodeName& as_made, std::function<void(const NodeName&)> rename);
        void leave(std::uint64_t id);
        void own(const Gid& gid);  // a gid of this session's entities: its own samples
        // One error line on stderr per frame that this program gives two parents on /tf and /tf_static (tf2 takes
        // one parent per frame), and per static frame that another program sends on /tf_static with another parent
        // or transform (read while a static publisher of it lives, mirror_tf). The transforms still go out.
        void check_tf(std::span<const std::uint8_t> payload, bool is_static);
        // open_session: the node's token and the watch for a second node of its name; detach when the session goes.
        void attach(zenoh::Session& session, const std::string& program);
        void detach();
        // a sample of ROS's /tf_static (a static Publisher's companion, mirror_tf): compared with this program's
        void on_static_sample(std::span<const std::uint8_t> payload, const Attachment& attachment);
        // --ros-clock: mirror made the clock key's publisher. Without one within kClockWaitS of attach, one warning
        // line says so (a program may still make it later: no error).
        void clock_published() { clock_published_.store(true); }

    private:
        struct Duplicates;
        void watch_duplicates(zenoh::Session& session);
        void say_static(const std::string& child, const detail::StaticTransform& mine,
                        const detail::StaticTransform& theirs);

        std::string zid_;
        Options options_;
        NodeName node_;
        std::atomic<std::uint64_t> ids_{0};
        mutable std::recursive_mutex graph_mutex_;  // node_, the tokens, the enrolled entities (a rename)
        bool attached_ = false;  // attach: its session open (open_session keeps no pointer to it: it moves)
        std::string program_;
        std::optional<zenoh::LivelinessToken> node_token_;
        std::shared_ptr<Duplicates> duplicates_;
        std::map<std::uint64_t, std::function<void(const NodeName&)>> enrolled_;
        std::uint64_t enrolled_ids_ = 0;
        bool parameters_taken_ = false;
        std::function<void()> retire_;  // until_parameters
        std::set<Gid> own_gids_;
        std::mutex tf_mutex_;
        std::map<std::string, std::string> tf_parents_;
        std::vector<std::string> tf_clashes_;
        std::map<std::string, detail::StaticTransform> static_mine_, static_others_;
        std::set<std::string> static_said_;
        std::atomic<bool> clock_published_{false};
        std::jthread clock_check_;  // last: it ends first (it reads the members above)
    };

    // The part in ROS 2 of `session` (opened by open_session with SessionOptions::ros), null when it takes none.
    std::shared_ptr<Ros> of(const zenoh::Session& session);

    namespace detail
    {
        // open_session: the session takes part (its node's token lives as long as the session); an empty node name
        // becomes node_name_of(program).
        void take_part(zenoh::Session& session, const Options& options, const std::string& program);
    }

    // A node's liveliness token (a node besides the program's own: a node's parameters).
    class RosNode
    {
    public:
        RosNode(zenoh::Session& session, std::shared_ptr<Ros> ros, const std::string& name, const std::string& ns);
        const NodeName& name() const { return name_; }

    private:
        NodeName name_;
        std::optional<zenoh::LivelinessToken> token_;
    };

    // What a ROS entity is: its token, its data key, its gid.
    struct Entity
    {
        Entity(const Ros& ros, const NodeName& node, std::string_view kind, const std::string& name,
               const std::string& type_name, const Qos& qos, std::string entity_id);
        // Its node renamed (Ros::parameters_node): its token, token key and gid of the node's new name.
        void rename(const NodeName& node);
        Token token;
        std::string token_key;
        std::string key;
        Gid gid;
    };

    // A ROS publisher: put() sends the CDR bytes on ROS's key with ROS's attachment. Transient local (/tf_static): its
    // last `depth` samples answered to late subscribers (the header's note).
    class Publisher
    {
    public:
        Publisher(zenoh::Session& session, Ros& ros, const NodeName& node, const std::string& topic,
                  const std::string& type_name, const Qos& qos = kStreamQos);
        ~Publisher();
        Publisher(const Publisher&) = delete;
        Publisher& operator=(const Publisher&) = delete;

        // A ROS subscriber (or any other) of the key is known.
        bool matched() const;
        void put(std::span<const std::uint8_t> payload);
        // The attachment of the next message, its sequence number counted: for a message that goes on keys of its
        // own (GpuLink: the buffer keys of Lyrical's readers), each with this one.
        std::vector<std::uint8_t> attachment();
        const std::string& key() const { return entity_.key; }
        const std::string& topic() const { return *entity_.token.topic; }
        const std::string& type_name() const { return *entity_.token.type_name; }
        const Qos& qos() const { return *entity_.token.qos; }
        const Entity& entity() const { return entity_; }
        // Called with each payload before it goes out (mirror_tf: the frames' parents).
        std::function<void(std::span<const std::uint8_t>)> check;
        // A subscription that lives as long as this publisher (mirror_tf, static: ROS's /tf_static of others).
        std::unique_ptr<Subscriber> companion_;

    private:
        struct Cache;
        void renamed(zenoh::Session& session, const NodeName& node);
        Entity entity_;
        std::weak_ptr<Ros> ros_;  // enrolled: on the program's node
        std::uint64_t enrolled_ = 0;
        std::int64_t seq_ = 0;
        std::mutex mutex_;
        std::optional<zenoh::Publisher> publisher_;
        std::shared_ptr<Cache> cache_;                    // transient local
        std::optional<zenoh::Queryable<void>> cache_queryable_;
        std::optional<zenoh::LivelinessToken> cache_token_;
        std::optional<zenoh::LivelinessToken> token_;     // last: undeclared first
    };

    // ------------------------------------------------------------------------- ROS cameras as ROS names them

    inline constexpr std::string_view kCompressedTopic = "compressed";   // image_transport's <image topic>/compressed
    inline constexpr std::string_view kCameraInfoTopic = "camera_info";  // image_transport's, beside the image topic
    // What reading a ROS topic needs (Python's ros.NEEDS_ROS).
    inline constexpr const char* kNeedsRos =
        "a session that takes part in ROS 2 (the default of open_session and every program; not one opened with "
        "--no-ros, XBLICK_ROS=0 or SessionOptions::ros = std::nullopt)";
    // True for a ROS topic as ROS's tools write it (/image_raw, /resize/image): a name with a leading '/'. A name
    // without one is one of our stream bases (test/camera/color).
    bool is_topic(const std::string& name);
    // /cam/image_raw -> /cam/image_raw/compressed
    std::string compressed_topic(const std::string& image_topic);
    // camera_info in the image topic's namespace: /cam/image_raw -> /cam/camera_info, /image_raw -> /camera_info.
    std::string camera_info_topic(const std::string& image_topic);

    // The publishers in ROS's graph on a domain, as a liveliness subscriber with history keeps their tokens (Python's
    // ros.Graph; here one per subscription that reads or looks for a ROS form). `changed` is called in Zenoh's thread
    // when a publisher came or went. by_xblick(zid): whether a publisher's session declares a program of ours (its
    // declaration's token, a liveliness query kept per session): the ROS topics such a program publishes are copies
    // of its own keys.
    class Graph
    {
    public:
        Graph(zenoh::Session& session, int domain, std::function<void()> changed = {});
        ~Graph();
        Graph(const Graph&) = delete;
        Graph& operator=(const Graph&) = delete;
        // The publishers known now, of `topic` or of all, this session's own left out.
        std::vector<Token> publishers(const std::optional<std::string>& topic = std::nullopt) const;
        bool by_xblick(const std::string& zid);

    private:
        struct State;
        zenoh::Session& session_;
        std::shared_ptr<State> state_;
        std::optional<zenoh::Subscriber<void>> subscriber_;
    };

    // The GPU link of one ROS publisher of images (sensor_msgs/msg/Image), as Python's ros_cuda.GpuLink: a ROS
    // subscription of its topic that accepts ROS 2's CUDA buffer backend (its token says backends:...cuda:) gets the
    // frames that are in a CUDA ring as CUDA buffers instead of their bytes (ros_cuda.hpp has the rules, the pool and
    // the protocol). plan() before each frame (none: everything as before, on the key); send_gpu and send_cpu with
    // that plan. Not thread-safe apart from the tokens (one publisher). The settings: ros_cuda::kEnableEnv,
    // kSocketsEnv, kShmEnv.
    class GpuLink
    {
    public:
        // RosError for another type than sensor_msgs/msg/Image or a sockets or shm directory that is none.
        GpuLink(zenoh::Session& session, Publisher& publisher, int device,
                std::optional<std::string> socket_dir = std::nullopt, std::optional<std::string> shm_dir = std::nullopt);
        ~GpuLink();
        GpuLink(const GpuLink&) = delete;
        GpuLink& operator=(const GpuLink&) = delete;

        std::optional<ros_cuda::Plan> plan();
        // The frame in slot `slot` of `ring` to the GPU readers of `plan`, with `attachment` (Publisher::attachment);
        // the keys it went to (none when every block is held).
        std::vector<std::string> send_gpu(const ros_cuda::Plan& plan, CudaRingWriter& ring, std::uint32_t slot,
                                          std::uint64_t nbytes, const ros_cuda::ImageHeader& header,
                                          const std::vector<std::uint8_t>& attachment);
        // The CPU form (the message's CDR) to the readers of `plan` that take it.
        std::vector<std::string> send_cpu(const ros_cuda::Plan& plan, std::span<const std::uint8_t> payload,
                                          const std::vector<std::uint8_t>& attachment);
        ros_cuda::LinkStats stats() const;

    private:
        struct State;
        std::shared_ptr<zenoh::Publisher> out(const std::string& key);
        std::string buf_key(const std::array<std::uint8_t, 16>& gid) const;
        void say_notes();

        zenoh::Session& session_;
        Publisher& publisher_;
        std::shared_ptr<State> state_;  // the link and the buffer keys' publishers, shared with the token callback
        std::optional<zenoh::Subscriber<void>> tokens_;
    };

    // A ROS subscription: callback(payload, attachment) for each sample, in Zenoh's thread. A sample without ROS's
    // attachment is dropped with one warning line, as ROS drops it. Transient local (/tf_static): also the last
    // `depth` samples of every publisher with a publication cache, those there at the start and those that come
    // later (zenoh-ext's AdvancedSubscriber's history; a sample can come twice). A buffer type (backends_of): also
    // <key>/_buf_cpu, where Lyrical's publishers send it.
    class Subscriber
    {
    public:
        using Callback = std::function<void(std::vector<std::uint8_t>&&, const Attachment&)>;
        Subscriber(zenoh::Session& session, Ros& ros, const NodeName& node, const std::string& topic,
                   const std::string& type_name, Callback callback, const Qos& qos = kStreamQos);
        ~Subscriber();
        Subscriber(const Subscriber&) = delete;
        Subscriber& operator=(const Subscriber&) = delete;
        const std::string& key() const { return entity_.key; }

    private:
        struct State;
        Entity entity_;
        std::weak_ptr<Ros> ros_;
        std::uint64_t enrolled_ = 0;
        std::shared_ptr<State> state_;
        std::optional<zenoh::Subscriber<void>> subscriber_;
        std::optional<zenoh::Subscriber<void>> history_;  // transient local: the publishers' cache tokens
        std::optional<zenoh::Subscriber<void>> cpu_group_;  // a buffer type: <key>/_buf_cpu
        std::optional<zenoh::LivelinessToken> token_;
    };

    // A ROS service: a queryable on ROS's key, declared complete (ROS clients ask with target ALL_COMPLETE), its
    // queries handed to `handler` (with the service, for its reply) in Zenoh's thread; the query is dropped when the
    // handler returns. reply() answers
    // one with ROS's attachment (the request's sequence number and client gid, the reply's time).
    class Service
    {
    public:
        using Handler = std::function<void(const zenoh::Query&, const Service&)>;
        Service(zenoh::Session& session, Ros& ros, const NodeName& node, const std::string& name,
                const std::string& type_name, Handler handler, const Qos& qos = kParametersQos);
        ~Service();
        Service(const Service&) = delete;
        Service& operator=(const Service&) = delete;
        const std::string& key() const { return entity_.key; }
        // The request's (sequence number, source ns, client gid); RosError without a ROS attachment.
        static Attachment request_of(const zenoh::Query& query);
        void reply(const zenoh::Query& query, std::vector<std::uint8_t>&& payload, const Attachment& request) const;

    private:
        Entity entity_;
        std::weak_ptr<Ros> ros_;
        std::uint64_t enrolled_ = 0;
        std::optional<zenoh::Queryable<void>> queryable_;
        std::optional<zenoh::LivelinessToken> token_;
    };

    // A ROS service's reply on `key` with ROS's attachment: the request's sequence number and client gid, the reply's
    // time (Service::reply).
    void reply(const zenoh::Query& query, const std::string& key, std::vector<std::uint8_t>&& payload,
               const Attachment& request);

    // The ROS publisher that mirrors one of our keys (of `type`) on the key's topic when `session` takes part in ROS,
    // else null; on /clock for the session's clock key (--ros-clock; RosError when its type is not
    // rosgraph_msgs/msg/Clock). A command key (commanding), a type without a ROS hash and a key that is no ROS name
    // are not offered: one warning line, null.
    std::unique_ptr<Publisher> mirror(zenoh::Session& session, const std::string& key,
                                      const protocol::MessageType& type, const Qos& qos = kStreamQos);
    // The publisher on ROS's /tf (/tf_static with is_static, transient local) for our key `key` when `session` takes
    // part, else null (null too for a command key, one warning line). Its put checks the frames (Ros::check_tf).
    std::unique_ptr<Publisher> mirror_tf(zenoh::Session& session, bool is_static, const std::string& key);
}  // namespace xblick::ros
