// ROS 2 on Zenoh in C++: the keys, type names, type hashes, attachments, QoS strings and liveliness tokens of
// rmw_zenoh_cpp 0.2.13 (ROS 2 Jazzy, Zenoh 1.10.1), so that a C++ program started with --ros takes part in ROS 2 as
// well. The same rules, names and bytes as Python's xblick.ros (its module docstring and docs/integrations/ros2.md
// are the reference; tests/test_cpp_ros.py compares the two byte for byte):
//
//   data key   <domain>/<topic without the leading />/<pkg>::msg::dds_::<Name>_/RIHS01_<type hash>
//              (a service: its name, <pkg>::srv::dds_::<Name>_ and the service type's hash)
//   attachment int64 sequence number, int64 source timestamp (ns since the epoch), a length byte 16 and the gid
//              (zenoh-ext's serialization, 33 bytes); a ROS subscriber drops a sample without it
//   tokens     @ros2_lv/<domain>/<zid>/<node id>/<node id>/NN/<enclave>/<namespace>/<node> (a node);
//              @ros2_lv/<domain>/<zid>/<node id>/<entity id>/<MP|MS|SS|SC>/<enclave>/<namespace>/<node>/<topic>/
//              <type>/RIHS01_<hash>/<qos> (an entity), every '/' inside a name written as '%'
//   gid        XXH3-128 of the entity's token key (ros_gid.hpp)
//
// A session takes part when open_session got SessionOptions::ros (--ros, Options::add_session_options): ros::of(session)
// is then its Ros, else null. CameraPublisher, PointCloudPublisher, the transform, robot state, diagnostics, IMU and
// message publishers mirror their samples onto ROS's keys while a ROS subscriber is there (mirror, mirror_tf), and
// Parameters serves ROS's parameter services. Without --ros nothing of ROS goes on the wire.
//
// /tf_static (transient local): rmw_zenoh uses zenoh-ext's AdvancedPublisher with a cache, which zenoh-c offers only
// with its unstable API (not in its standalone release). Publisher here stands in for it with the parts a late ROS
// subscriber uses: the cache's liveliness token and queryable <key>/@adv/pub/<zid>/<id>/_ answering the last `depth`
// samples. Not offered: sample sequence numbers and heartbeats (sample-miss detection); a late subscriber may get a
// cached sample twice (its history query and its late-publisher query), which tf2 takes as the same transform.
#pragma once

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <zenoh.hxx>

#include "xblick/protocol.hpp"
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

        std::string key() const;
        std::string fully_qualified_node() const;
        static Token parse(const std::string& key);  // RosError for one rmw_zenoh would refuse
    };

    // ------------------------------------------------------------------------------------ type hashes (RIHS01)

    // The RIHS01 hash of every type Durchblick carries to ROS: Python's ros.TYPE_HASHES (ROS 2 Jazzy's own; vision_msgs
    // computed by Python's message_hash), compared by tests/test_cpp_ros.py.
    const std::map<std::string, std::string>& type_hashes();
    std::string hash_of(const std::string& type_name);  // RosError for a type not offered to ROS

    // ---------------------------------------------------------------------------------- taking part on the wire

    // --ros, --ros-domain, --ros-node, --ros-params-writable, read (Options::add_session_options).
    struct Options
    {
        int domain = kDefaultDomain;
        std::string node{};
        bool params_writable = false;  // ROS clients may set parameters; by default ROS's sets are refused
    };
    // The options of a command line's values; none without --ros. RosError for --ros-domain, --ros-node or
    // --ros-params-writable without --ros, a domain that is no integer >= 0, a node name ROS does not take. The node
    // name defaults to node_name_of(program).
    std::optional<Options> options_from(bool ros, std::optional<std::int64_t> domain,
                                        const std::optional<std::string>& node, bool params_writable,
                                        const std::string& program);

    // A ROS node: its name, namespace and id.
    struct NodeName
    {
        std::string name;
        std::string ns = "/";
        std::string id;
        std::string fully_qualified() const;
    };

    // A session's part in ROS 2 (--ros): domain, its program's node, the entity ids (one counter per session, as
    // rmw_zenoh's per context).
    class Ros
    {
    public:
        Ros(std::string zid, const Options& options);
        int domain() const { return options_.domain; }
        const std::string& zid() const { return zid_; }
        bool params_writable() const { return options_.params_writable; }
        const NodeName& node() const { return node_; }
        std::string next_id();
        // A new node's name and id (its token: RosNode).
        NodeName add_node(const std::string& name, const std::string& ns = "/");
        // One error line on stderr per frame that this program gives two parents on /tf and /tf_static (tf2 takes
        // one parent per frame). The transforms still go out.
        void check_tf(std::span<const std::uint8_t> payload, bool is_static);

    private:
        std::string zid_;
        Options options_;
        NodeName node_;
        std::atomic<std::uint64_t> ids_{0};
        std::mutex tf_mutex_;
        std::map<std::string, std::string> tf_parents_;
        std::vector<std::string> tf_clashes_;
    };

    // The part in ROS 2 of `session` (opened by open_session with SessionOptions::ros), null when it takes none.
    std::shared_ptr<Ros> of(const zenoh::Session& session);

    namespace detail
    {
        // open_session: the session takes part (its node's token lives as long as the session).
        void take_part(zenoh::Session& session, const Options& options);
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
        const std::string& key() const { return entity_.key; }
        const Entity& entity() const { return entity_; }
        // Called with each payload before it goes out (mirror_tf: the frames' parents).
        std::function<void(std::span<const std::uint8_t>)> check;

    private:
        struct Cache;
        Entity entity_;
        std::int64_t seq_ = 0;
        std::mutex mutex_;
        std::optional<zenoh::Publisher> publisher_;
        std::shared_ptr<Cache> cache_;                    // transient local
        std::optional<zenoh::Queryable<void>> cache_queryable_;
        std::optional<zenoh::LivelinessToken> cache_token_;
        std::optional<zenoh::LivelinessToken> token_;     // last: undeclared first
    };

    // A ROS subscription: callback(payload, attachment) for each sample, in Zenoh's thread. A sample without ROS's
    // attachment is dropped with one warning line, as ROS drops it. Transient local (/tf_static): also the last
    // `depth` samples of every publisher with a publication cache, those there at the start and those that come
    // later (zenoh-ext's AdvancedSubscriber's history; a sample can come twice).
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
        std::shared_ptr<State> state_;
        std::optional<zenoh::Subscriber<void>> subscriber_;
        std::optional<zenoh::Subscriber<void>> history_;  // transient local: the publishers' cache tokens
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
        std::optional<zenoh::Queryable<void>> queryable_;
        std::optional<zenoh::LivelinessToken> token_;
    };

    // The ROS publisher that mirrors one of our keys (of `type`) on the key's topic when `session` takes part in ROS,
    // else null. A command key (commanding) and a type without a ROS hash are not offered: one warning line, null.
    // RosError for a key that is no ROS name.
    std::unique_ptr<Publisher> mirror(zenoh::Session& session, const std::string& key,
                                      const protocol::MessageType& type, const Qos& qos = kStreamQos);
    // The publisher on ROS's /tf (/tf_static with is_static, transient local) for our key `key` when `session` takes
    // part, else null (null too for a command key, one warning line). Its put checks the frames (Ros::check_tf).
    std::unique_ptr<Publisher> mirror_tf(zenoh::Session& session, bool is_static, const std::string& key);
}  // namespace xblick::ros
