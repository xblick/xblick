// ROS 2 on rmw_zenoh in C++ (ros.hpp); Python's xblick.ros is the reference.
#include "xblick/ros.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <regex>

#include "xblick/msg.hpp"

namespace xblick::ros
{
    namespace
    {
        const std::regex kTokenName("^[A-Za-z_][A-Za-z0-9_]*$");
        const std::regex kTypeName("^([A-Za-z_][A-Za-z0-9_]*)/(msg|srv)/([A-Za-z_][A-Za-z0-9_]*)$");
        const std::regex kDdsName("^([A-Za-z0-9_]+)::(msg|srv)::dds_::([A-Za-z0-9_]+)_$");

        std::string repr(const std::string& s)  // as Python's repr of a plain string, for the same messages
        {
            return "'" + s + "'";
        }

        std::vector<std::string> split(const std::string& text, char separator)
        {
            std::vector<std::string> out;
            std::size_t start = 0;
            while (true)
            {
                const std::size_t end = text.find(separator, start);
                out.push_back(text.substr(start, end == std::string::npos ? std::string::npos : end - start));
                if (end == std::string::npos)
                {
                    return out;
                }
                start = end + 1;
            }
        }

        std::optional<std::int64_t> integer(const std::string& text)
        {
            std::int64_t v = 0;
            const char* begin = text.data();
            if (not text.empty() and text[0] == '+')
            {
                ++begin;
            }
            const auto [end, error] = std::from_chars(begin, text.data() + text.size(), v);
            if (text.empty() or error != std::errc() or end != text.data() + text.size())
            {
                return std::nullopt;
            }
            return v;
        }

        std::string trimmed(std::string s)
        {
            const auto space = [](unsigned char c) { return std::isspace(c) != 0; };
            s.erase(s.begin(), std::find_if_not(s.begin(), s.end(), space));
            s.erase(std::find_if_not(s.rbegin(), s.rend(), space).base(), s.end());
            return s;
        }

        std::int64_t now_ns()
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                .count();
        }

        void say(const std::string& line)
        {
            std::fprintf(stderr, "xblick: %s\n", line.c_str());
            std::fflush(stderr);
        }
    }  // namespace

    // -------------------------------------------------------------------------------------- names and keys

    std::string check_ros_name(const std::string& name, const std::string& what)
    {
        if (name.empty() or name[0] != '/' or name == "/")
        {
            throw RosError(what + " " + repr(name) + " is no fully qualified ROS name (/a/b)");
        }
        for (const std::string& token : split(name.substr(1), '/'))
        {
            if (not std::regex_match(token, kTokenName))
            {
                throw RosError(what + " " + repr(name) + " is no ROS name: " + repr(token) +
                               " is not letters, digits and _ (not starting with a digit), as ROS requires of each "
                               "part");
            }
        }
        return name;
    }

    std::string topic_of(const std::string& key)
    {
        std::string k = key;
        k.erase(0, k.find_first_not_of('/'));
        k.erase(k.find_last_not_of('/') + 1);
        return check_ros_name("/" + k, "the ROS topic of key");
    }

    std::string check_node_name(const std::string& name)
    {
        if (not std::regex_match(name, kTokenName))
        {
            throw RosError("ROS node name " + repr(name) + ": letters, digits and _, not starting with a digit");
        }
        return name;
    }

    std::string node_name_of(const std::string& program)
    {
        std::string name;
        for (const char c : program)
        {
            const auto u = static_cast<unsigned char>(c);
            if ((u & 0xC0) == 0x80)
            {
                continue;  // a UTF-8 continuation byte: one character is one '_', as in Python
            }
            name += (u < 128 and (std::isalnum(u) or c == '_')) ? c : '_';
        }
        if (name.empty())
        {
            name = "xblick";
        }
        return std::isdigit(static_cast<unsigned char>(name[0])) ? "_" + name : name;
    }

    std::pair<std::string, std::string> split_node(const std::string& fully_qualified)
    {
        check_ros_name(fully_qualified, "node");
        const std::size_t slash = fully_qualified.rfind('/');
        const std::string ns = fully_qualified.substr(0, slash);
        return {ns.empty() ? "/" : ns, fully_qualified.substr(slash + 1)};
    }

    std::string dds_type(const std::string& type_name)
    {
        std::smatch m;
        if (not std::regex_match(type_name, m, kTypeName))
        {
            throw RosError("type " + repr(type_name) + ": <package>/msg/<Name> or <package>/srv/<Name> expected");
        }
        return m[1].str() + "::" + m[2].str() + "::dds_::" + m[3].str() + "_";
    }

    std::string type_of_dds(const std::string& dds)
    {
        std::smatch m;
        if (not std::regex_match(dds, m, kDdsName))
        {
            throw RosError(repr(dds) + " is no rmw_zenoh type name");
        }
        return m[1].str() + "/" + m[2].str() + "/" + m[3].str();
    }

    std::string data_key(int domain, const std::string& name, const std::string& type_name,
                         const std::string& type_hash)
    {
        check_ros_name(name);
        std::string inner = name;
        inner.erase(0, inner.find_first_not_of('/'));
        inner.erase(inner.find_last_not_of('/') + 1);
        return std::to_string(domain) + "/" + inner + "/" + dds_type(type_name) + "/" + type_hash;
    }

    std::string mangle(std::string name)
    {
        std::replace(name.begin(), name.end(), '/', '%');
        return name;
    }

    std::string demangle(std::string name)
    {
        std::replace(name.begin(), name.end(), '%', '/');
        return name;
    }

    int domain_from(std::optional<std::int64_t> value)
    {
        std::string where;
        std::optional<std::int64_t> domain = value;
        if (not value)
        {
            const char* env = std::getenv(kDomainEnv);
            const std::string text = trimmed(env != nullptr ? env : "");
            where = std::string(kDomainEnv) + "=" + (text.empty() ? std::to_string(kDefaultDomain) : repr(text));
            domain = text.empty() ? std::optional<std::int64_t>(kDefaultDomain) : integer(text);
        }
        else
        {
            where = "--ros-domain " + std::to_string(*value);
        }
        if (not domain or *domain < 0 or *domain > 2147483647)
        {
            throw RosError(where + ": a ROS domain id is an integer from 0");
        }
        return static_cast<int>(*domain);
    }

    bool commanding(const std::string& key)
    {
        std::string k = key;
        k.erase(0, k.find_first_not_of('/'));
        k.erase(k.find_last_not_of('/') + 1);
        const std::vector<std::string> parts = split(k, '/');
        return std::find(parts.begin(), parts.end(), std::string(protocol::kCommand)) != parts.end();
    }

    // ---------------------------------------------------------------------------------------------- attachment

    std::vector<std::uint8_t> encode_attachment(std::int64_t seq, std::int64_t source_ns, const Gid& gid)
    {
        std::vector<std::uint8_t> out(kAttachmentSize);
        for (std::size_t i = 0; i < 8; ++i)  // little endian, as zenoh-ext serializes
        {
            out[i] = static_cast<std::uint8_t>(static_cast<std::uint64_t>(seq) >> (8 * i));
            out[8 + i] = static_cast<std::uint8_t>(static_cast<std::uint64_t>(source_ns) >> (8 * i));
        }
        out[16] = static_cast<std::uint8_t>(kGidSize);
        std::copy(gid.begin(), gid.end(), out.begin() + 17);
        return out;
    }

    Attachment decode_attachment(std::span<const std::uint8_t> data)
    {
        if (data.size() != kAttachmentSize or data[16] != kGidSize)
        {
            throw RosError("no rmw_zenoh attachment (" + std::to_string(data.size()) + " bytes; " +
                           std::to_string(kAttachmentSize) + " expected, length byte 16)");
        }
        std::uint64_t seq = 0, stamp = 0;
        for (std::size_t i = 0; i < 8; ++i)
        {
            seq |= static_cast<std::uint64_t>(data[i]) << (8 * i);
            stamp |= static_cast<std::uint64_t>(data[8 + i]) << (8 * i);
        }
        Attachment a;
        a.seq = static_cast<std::int64_t>(seq);
        a.source_ns = static_cast<std::int64_t>(stamp);
        std::copy(data.begin() + 17, data.end(), a.gid.begin());
        return a;
    }

    // ---------------------------------------------------------------------------------------------------- QoS

    std::string Qos::key() const
    {
        const Qos d;
        const std::string r = reliability == d.reliability ? "" : std::to_string(reliability);
        const std::string u = durability == d.durability ? "" : std::to_string(durability);
        const std::string h = history == d.history ? "" : std::to_string(history);
        const std::string n = depth == d.depth ? "" : std::to_string(depth);
        return r + ":" + u + ":" + h + "," + n + ":" + deadline.first + "," + deadline.second + ":" + lifespan.first +
               "," + lifespan.second + ":" + liveliness + "," + lease.first + "," + lease.second;
    }

    Qos Qos::parse(const std::string& text)
    {
        const auto refused = [&] { return RosError(repr(text) + " is no rmw_zenoh QoS"); };
        const std::vector<std::string> parts = split(text, ':');
        if (parts.size() != 6)
        {
            throw refused();
        }
        const std::vector<std::string> history = split(parts[2], ',');
        const std::vector<std::string> deadline = split(parts[3], ',');
        const std::vector<std::string> lifespan = split(parts[4], ',');
        const std::vector<std::string> liveliness = split(parts[5], ',');
        if (history.size() != 2 or deadline.size() != 2 or lifespan.size() != 2 or liveliness.size() != 3)
        {
            throw refused();
        }
        const Qos d;
        const auto field = [&](const std::string& t, int fallback) {
            if (t.empty())
            {
                return fallback;
            }
            const auto v = integer(t);
            if (not v or *v < -2147483648LL or *v > 2147483647LL)
            {
                throw refused();
            }
            return static_cast<int>(*v);
        };
        Qos q;
        q.reliability = field(parts[0], d.reliability);
        q.durability = field(parts[1], d.durability);
        q.history = field(history[0], d.history);
        q.depth = field(history[1], d.depth);
        q.deadline = {deadline[0], deadline[1]};
        q.lifespan = {lifespan[0], lifespan[1]};
        q.liveliness = liveliness[0];
        q.lease = {liveliness[1], liveliness[2]};
        return q;
    }

    // ------------------------------------------------------------------------------------------------- tokens

    std::string Token::key() const
    {
        std::string out = std::string(kAdminSpace) + "/" + std::to_string(domain) + "/" + zid + "/" + node_id + "/" +
                          entity_id + "/" + kind + "/" + mangle(enclave) + "/" + mangle(ns) + "/" + mangle(node);
        if (kind != kNode)
        {
            if (not topic or not type_name or not type_hash or not qos)
            {
                throw std::logic_error("an entity's token needs its topic, type, hash and QoS");
            }
            out += "/" + mangle(*topic) + "/" + dds_type(*type_name) + "/" + *type_hash + "/" + qos->key();
        }
        return out;
    }

    std::string Token::fully_qualified_node() const
    {
        std::string n = ns;
        while (not n.empty() and n.back() == '/')
        {
            n.pop_back();
        }
        return n + "/" + node;
    }

    Token Token::parse(const std::string& key)
    {
        const std::vector<std::string> parts = split(key, '/');
        const bool kind_known = parts.size() > 5 and (parts[5] == kNode or parts[5] == kPublisher or
                                                      parts[5] == kSubscription or parts[5] == kService or
                                                      parts[5] == kClient);
        if (parts.size() < 9 or parts[0] != kAdminSpace or
            std::any_of(parts.begin(), parts.end(), [](const std::string& p) { return p.empty(); }) or not kind_known)
        {
            throw RosError(repr(key) + " is no rmw_zenoh liveliness token");
        }
        const auto domain = integer(parts[1]);
        if (not domain or *domain < -2147483648LL or *domain > 2147483647LL)
        {
            throw RosError(repr(key) + ": domain " + repr(parts[1]));
        }
        Token t;
        t.domain = static_cast<int>(*domain);
        t.zid = parts[2];
        t.node_id = parts[3];
        t.entity_id = parts[4];
        t.kind = parts[5];
        t.enclave = demangle(parts[6]);
        t.ns = demangle(parts[7]);
        t.node = demangle(parts[8]);
        if (t.kind == kNode)
        {
            if (parts.size() != 9)
            {
                throw RosError(repr(key) + ": a node's token has 9 parts");
            }
            return t;
        }
        if (parts.size() != 13)
        {
            throw RosError(repr(key) + ": an entity's token has 13 parts");
        }
        t.topic = demangle(parts[9]);
        t.type_name = type_of_dds(parts[10]);
        t.type_hash = parts[11];
        t.qos = Qos::parse(parts[12]);
        return t;
    }

    // ---------------------------------------------------------------------------------------------- hashes

    const std::map<std::string, std::string>& type_hashes()
    {
        // Python's ros.TYPE_HASHES (ROS 2 Jazzy's share/<package>/<msg|srv>/<Name>.json, vision_msgs computed)
        static const std::map<std::string, std::string> table = {
            {"builtin_interfaces/msg/Time", "RIHS01_b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197"},
            {"diagnostic_msgs/msg/DiagnosticArray",
             "RIHS01_5a8a36efb05fb25070fa0fb3810290c0e6cd4862b54a8fb975a1ee8dc55a333e"},
            {"geometry_msgs/msg/WrenchStamped",
             "RIHS01_8dc3deaf06b2ab281f9f9a742a8961c328ca7cec16e3fd6586d3a5c83fa78f77"},
            {"nav_msgs/msg/Odometry", "RIHS01_3cc97dc7fb7502f8714462c526d369e35b603cfc34d946e3f2eda2766dfec6e0"},
            {"rcl_interfaces/msg/ParameterEvent",
             "RIHS01_043e627780fcad87a22d225bc2a037361dba713fca6a6b9f4b869a5aa0393204"},
            {"rcl_interfaces/srv/DescribeParameters",
             "RIHS01_845b484d71eb0673dae682f2e3ba3c4851a65a3dcfb97bddd82c5b57e91e4cff"},
            {"rcl_interfaces/srv/GetParameterTypes",
             "RIHS01_da199c878688b3e530bdfe3ca8f74cb9fa0c303101e980a9e8f260e25e1c80ca"},
            {"rcl_interfaces/srv/GetParameters",
             "RIHS01_bf9803d5c74cf989a5de3e0c2e99444599a627c7ff75f97b8c05b01003675cbc"},
            {"rcl_interfaces/srv/ListParameters",
             "RIHS01_3e6062bfbb27bfb8730d4cef2558221f51a11646d78e7bb30a1e83afac3aad9d"},
            {"rcl_interfaces/srv/SetParameters",
             "RIHS01_56eed9a67e169f9cb6c1f987bc88f868c14a8fc9f743a263bc734c154015d7e0"},
            {"rcl_interfaces/srv/SetParametersAtomically",
             "RIHS01_0e192ef259c07fc3c07a13191d27002222e65e00ccec653ca05e856f79285fcd"},
            {"rosgraph_msgs/msg/Clock", "RIHS01_692f7a66e93a3c83e71765d033b60349ba68023a8c689a79e48078bcb5c58564"},
            {"sensor_msgs/msg/CameraInfo", "RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275"},
            {"sensor_msgs/msg/CompressedImage",
             "RIHS01_15640771531571185e2efc8a100baf923961a4d15d5569652e6cb6691e8e371a"},
            {"sensor_msgs/msg/Image", "RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47"},
            {"sensor_msgs/msg/Imu", "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b"},
            {"sensor_msgs/msg/JointState", "RIHS01_a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e"},
            {"sensor_msgs/msg/PointCloud2", "RIHS01_9198cabf7da3796ae6fe19c4cb3bdd3525492988c70522628af5daa124bae2b5"},
            {"std_msgs/msg/Float64MultiArray",
             "RIHS01_1025ddc6b9552d191f89ef1a8d2f60f3d373e28b283d8891ddcc974e8c55397f"},
            {"std_msgs/msg/String", "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"},
            {"tf2_msgs/msg/TFMessage", "RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087"},
            {"vision_msgs/msg/Classification",
             "RIHS01_f74db28d0eaef3afb78f48bf2161cd709a1383a0309d3454383b25be11345143"},
            {"vision_msgs/msg/Detection2DArray",
             "RIHS01_214274120007f4a683866481336b70310268cfa107aebe9d594a6acf75bda968"},
            {"vision_msgs/msg/Detection3DArray",
             "RIHS01_793f5eab9595ac1c0da07873cc859db42eae51f0ee76056db8562515d830e018"},
        };
        return table;
    }

    std::string hash_of(const std::string& type_name)
    {
        const auto it = type_hashes().find(type_name);
        if (it == type_hashes().end())
        {
            throw RosError(type_name + ": no ROS 2 type hash known for it, so it is not offered to ROS");
        }
        return it->second;
    }

    // ------------------------------------------------------------------------------------------ taking part

    std::optional<Options> options_from(bool ros, std::optional<std::int64_t> domain,
                                        const std::optional<std::string>& node, bool params_writable,
                                        const std::string& program)
    {
        if (not ros)
        {
            if (domain or node or params_writable)
            {
                throw RosError("--ros-domain, --ros-node and --ros-params-writable need --ros");
            }
            return std::nullopt;
        }
        Options o;
        o.domain = domain_from(domain);
        o.node = check_node_name(node ? *node : node_name_of(program));
        o.params_writable = params_writable;
        return o;
    }

    std::string NodeName::fully_qualified() const
    {
        std::string n = ns;
        while (not n.empty() and n.back() == '/')
        {
            n.pop_back();
        }
        return n + "/" + name;
    }

    Ros::Ros(std::string zid, const Options& options) : zid_(std::move(zid)), options_(options)
    {
        node_ = add_node(options_.node);
    }

    std::string Ros::next_id()
    {
        return std::to_string(ids_.fetch_add(1));
    }

    NodeName Ros::add_node(const std::string& name, const std::string& ns)
    {
        if (ns != "/")
        {
            check_ros_name(ns, "namespace");
        }
        NodeName n{check_node_name(name), ns, ""};
        n.id = next_id();
        return n;
    }

    void Ros::check_tf(std::span<const std::uint8_t> payload, bool is_static)
    {
        TFMessage message;
        try
        {
            message = TFMessage::deserialize(payload);
        }
        catch (const std::exception& e)
        {
            say(std::string(is_static ? "/tf_static" : "/tf") + ": a message that is no TFMessage (" + e.what() + ")");
            return;
        }
        std::lock_guard lock(tf_mutex_);
        for (const auto& t : message.transforms)
        {
            const std::string& child = t.child_frame_id;
            const std::string& parent = t.header.frame_id;
            const auto [it, added] = tf_parents_.emplace(child, parent);
            if (not added and it->second != parent and
                std::find(tf_clashes_.begin(), tf_clashes_.end(), child) == tf_clashes_.end())
            {
                tf_clashes_.push_back(child);
                say("ROS tf: frame " + repr(child) + " has parent " + repr(it->second) + " and parent " +
                    repr(parent) + " in this program; tf2 takes one parent per frame: give each camera and robot "
                    "frame names of its own (--frame-id)");
            }
        }
    }

    namespace
    {
        std::mutex registry_mutex;
        std::map<std::string, std::weak_ptr<Ros>> registry;  // zid -> its part in ROS, while its session is open

        // The program's node: its Ros and its token, kept by the session (released when it closes).
        struct Holder
        {
            std::shared_ptr<Ros> ros;
            std::optional<zenoh::LivelinessToken> token;
        };

        Token node_token(const Ros& ros, const NodeName& node)
        {
            Token t;
            t.domain = ros.domain();
            t.zid = ros.zid();
            t.node_id = node.id;
            t.entity_id = node.id;
            t.kind = std::string(kNode);
            t.enclave = "/";
            t.ns = node.ns;
            t.node = node.name;
            return t;
        }

        // the ids of the stand-ins for zenoh-ext's publication caches: far from zenoh-ext's own counters, which start
        // at 0 per session
        std::atomic<std::uint32_t> cache_ids{3'000'000'000U};
    }  // namespace

    std::shared_ptr<Ros> of(const zenoh::Session& session)
    {
        std::lock_guard lock(registry_mutex);
        const auto found = registry.find(session.get_zid().to_string());
        return found == registry.end() ? nullptr : found->second.lock();
    }

    namespace detail
    {
        void take_part(zenoh::Session& session, const Options& options)
        {
            const std::string zid = session.get_zid().to_string();
            auto holder = std::make_shared<Holder>();
            holder->ros = std::make_shared<Ros>(zid, options);
            const std::string key = node_token(*holder->ros, holder->ros->node()).key();
            holder->token.emplace(session.liveliness_declare_token(zenoh::KeyExpr(key)));
            {
                std::lock_guard lock(registry_mutex);
                registry[zid] = holder->ros;
            }
            // Kept by the session: a background liveliness subscriber on the node's own token, whose drop callback
            // (when the session closes) releases the node's token and its Ros (as the declaration's queryable keeps
            // the declaration's token, declaration.cpp).
            session.liveliness_declare_background_subscriber(
                zenoh::KeyExpr(key), [](const zenoh::Sample&) {},
                [holder, zid]() mutable {
                    {
                        std::lock_guard lock(registry_mutex);
                        registry.erase(zid);
                    }
                    holder->token.reset();
                    holder.reset();
                });
        }
    }  // namespace detail

    RosNode::RosNode(zenoh::Session& session, std::shared_ptr<Ros> ros, const std::string& name, const std::string& ns)
        : name_(ros->add_node(name, ns))
    {
        token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(node_token(*ros, name_).key())));
    }

    Entity::Entity(const Ros& ros, const NodeName& node, std::string_view kind, const std::string& name,
                   const std::string& type_name, const Qos& qos, std::string entity_id)
    {
        token.domain = ros.domain();
        token.zid = ros.zid();
        token.node_id = node.id;
        token.entity_id = std::move(entity_id);
        token.kind = std::string(kind);
        token.enclave = "/";
        token.ns = node.ns;
        token.node = node.name;
        token.topic = check_ros_name(name);
        token.type_name = type_name;
        token.type_hash = hash_of(type_name);
        token.qos = qos;
        token_key = token.key();
        key = data_key(ros.domain(), name, type_name, *token.type_hash);
        if (key.empty() or not std::isdigit(static_cast<unsigned char>(key[0])))  // no key of ours starts so
        {
            throw RosError(repr(key) + " is no rmw_zenoh key");
        }
        gid = ros::gid(token_key);
        if (qos.reliability == kBestEffort)  // zenoh-c sets a publisher's reliability only with its unstable API
        {
            throw RosError(name + ": best-effort ROS entities are not offered by the C++ API");
        }
    }

    namespace
    {
        // rmw_zenoh: blocking congestion control only for reliable keep-all
        zenoh::CongestionControl congestion(const Qos& qos)
        {
            return qos.reliability != kBestEffort and qos.history == kKeepAll ? Z_CONGESTION_CONTROL_BLOCK
                                                                               : Z_CONGESTION_CONTROL_DROP;
        }
    }  // namespace

    struct Publisher::Cache
    {
        std::mutex mutex;
        std::size_t depth = 1;
        std::deque<std::pair<std::vector<std::uint8_t>, std::vector<std::uint8_t>>> samples;  // payload, attachment
    };

    Publisher::Publisher(zenoh::Session& session, Ros& ros, const NodeName& node, const std::string& topic,
                         const std::string& type_name, const Qos& qos)
        : entity_(ros, node, kPublisher, topic, type_name, qos, ros.next_id())
    {
        zenoh::Session::PublisherOptions options;
        options.congestion_control = congestion(qos);
        publisher_.emplace(session.declare_publisher(zenoh::KeyExpr(entity_.key), std::move(options)));
        if (qos.durability == kTransientLocal)
        {
            // the parts of zenoh-ext's AdvancedPublisher (cache, publisher detection) that a late subscriber uses
            // (ros.hpp): its liveliness token and the cache's queryable, both <key>/@adv/pub/<zid>/<id>/_
            cache_ = std::make_shared<Cache>();
            cache_->depth = static_cast<std::size_t>(std::max(qos.depth, 1));
            const std::string cache_key = entity_.key + "/@adv/pub/" + ros.zid() + "/" +
                                          std::to_string(cache_ids.fetch_add(1)) + "/_";
            const std::string data = entity_.key;
            cache_queryable_.emplace(session.declare_queryable(
                zenoh::KeyExpr(cache_key),
                [cache = cache_, data](const zenoh::Query& query) {
                    try
                    {
                        std::lock_guard lock(cache->mutex);
                        for (const auto& [payload, attachment] : cache->samples)
                        {
                            zenoh::Query::ReplyOptions reply;
                            reply.attachment = zenoh::Bytes(attachment);
                            query.reply(zenoh::KeyExpr(data), zenoh::Bytes(payload), std::move(reply));
                        }
                    }
                    catch (const std::exception& e)  // a query that takes no reply on the data key (no _anyke)
                    {
                        say(data + ": a history query not answered: " + e.what());
                    }
                },
                zenoh::closures::none));
            cache_token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(cache_key)));
        }
        token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
    }

    Publisher::~Publisher()
    {
        token_.reset();
        cache_token_.reset();
        cache_queryable_.reset();
        publisher_.reset();
    }

    bool Publisher::matched() const
    {
        return publisher_->get_matching_status().matching;
    }

    void Publisher::put(std::span<const std::uint8_t> payload)
    {
        if (check)
        {
            check(payload);
        }
        std::vector<std::uint8_t> attachment;
        {
            std::lock_guard lock(mutex_);
            attachment = encode_attachment(++seq_, now_ns(), entity_.gid);
        }
        if (cache_)
        {
            std::lock_guard lock(cache_->mutex);
            cache_->samples.emplace_back(std::vector<std::uint8_t>(payload.begin(), payload.end()), attachment);
            while (cache_->samples.size() > cache_->depth)
            {
                cache_->samples.pop_front();
            }
        }
        zenoh::Publisher::PutOptions options;
        options.attachment = zenoh::Bytes(std::move(attachment));
        publisher_->put(zenoh::Bytes(std::vector<std::uint8_t>(payload.begin(), payload.end())), std::move(options));
    }

    Subscriber::Subscriber(zenoh::Session& session, Ros& ros, const NodeName& node, const std::string& topic,
                           const std::string& type_name, Callback callback, const Qos& qos)
        : entity_(ros, node, kSubscription, topic, type_name, qos, ros.next_id())
    {
        if (qos.durability == kTransientLocal)
        {
            throw RosError(topic + ": a transient-local ROS subscription is not offered by the C++ API");
        }
        auto warned = std::make_shared<std::atomic<bool>>(false);
        subscriber_.emplace(session.declare_subscriber(
            zenoh::KeyExpr(entity_.key),
            [callback = std::move(callback), warned, topic](const zenoh::Sample& sample) {
                try
                {
                    const auto a = sample.get_attachment();
                    const std::vector<std::uint8_t> bytes = a ? a->get().as_vector() : std::vector<std::uint8_t>{};
                    Attachment attachment;
                    try
                    {
                        attachment = decode_attachment(bytes);
                    }
                    catch (const RosError& error)
                    {
                        if (not warned->exchange(true))
                        {
                            say(topic + ": a sample dropped, as ROS drops it: " + error.what());
                        }
                        return;
                    }
                    callback(sample.get_payload().as_vector(), attachment);
                }
                catch (const std::exception& e)  // a bad sample must not end the subscription
                {
                    say(topic + ": a sample its reader could not take: " + e.what());
                }
            },
            zenoh::closures::none));
        token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
    }

    Subscriber::~Subscriber()
    {
        token_.reset();
        subscriber_.reset();
    }

    Service::Service(zenoh::Session& session, Ros& ros, const NodeName& node, const std::string& name,
                     const std::string& type_name, Handler handler, const Qos& qos)
        : entity_(ros, node, kService, name, type_name, qos, ros.next_id())
    {
        zenoh::Session::QueryableOptions options;
        options.complete = true;  // ROS clients query with target ALL_COMPLETE (ros.hpp)
        queryable_.emplace(session.declare_queryable(
            zenoh::KeyExpr(entity_.key),
            [this, handler = std::move(handler), name](const zenoh::Query& query) {
                try
                {
                    handler(query, *this);  // entity_ is set: a call during the construction can be answered
                }
                catch (const std::exception& e)  // nothing escapes into Zenoh's thread
                {
                    say(name + ": a ROS call not answered: " + e.what());
                }
                catch (...)
                {
                    say(name + ": a ROS call not answered: an unknown error");
                }
            },
            zenoh::closures::none, std::move(options)));
        token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
    }

    Service::~Service()
    {
        token_.reset();
        queryable_.reset();
    }

    Attachment Service::request_of(const zenoh::Query& query)
    {
        const auto a = query.get_attachment();
        return decode_attachment(a ? a->get().as_vector() : std::vector<std::uint8_t>{});
    }

    void Service::reply(const zenoh::Query& query, std::vector<std::uint8_t>&& payload, const Attachment& request) const
    {
        zenoh::Query::ReplyOptions options;
        options.attachment = zenoh::Bytes(encode_attachment(request.seq, now_ns(), request.gid));
        query.reply(zenoh::KeyExpr(entity_.key), zenoh::Bytes(std::move(payload)), std::move(options));
    }

    std::unique_ptr<Publisher> mirror(zenoh::Session& session, const std::string& key,
                                      const protocol::MessageType& type, const Qos& qos)
    {
        const std::shared_ptr<Ros> ros = of(session);
        if (not ros)
        {
            return nullptr;
        }
        if (commanding(key))
        {
            say(key + ": not offered to ROS (a command key: what a robot is told never goes to ROS)");
            return nullptr;
        }
        const std::string type_name(type.name);
        if (type.version != 0 or not type_hashes().contains(type_name))
        {
            say(key + ": not offered to ROS (" + type.schema() + " has no ROS type hash here)");
            return nullptr;
        }
        return std::make_unique<Publisher>(session, *ros, ros->node(), topic_of(key), type_name, qos);
    }

    std::unique_ptr<Publisher> mirror_tf(zenoh::Session& session, bool is_static, const std::string& key)
    {
        const std::shared_ptr<Ros> ros = of(session);
        if (not ros)
        {
            return nullptr;
        }
        if (commanding(key))
        {
            say(key + ": not offered to ROS (a command key: what a robot is told never goes to ROS)");
            return nullptr;
        }
        auto publisher = std::make_unique<Publisher>(session, *ros, ros->node(), is_static ? "/tf_static" : "/tf",
                                                     std::string(protocol::kTFMessageType.name),
                                                     is_static ? kTfStaticQos : kTfQos);
        publisher->check = [ros, is_static](std::span<const std::uint8_t> payload) { ros->check_tf(payload, is_static); };
        return publisher;
    }
}  // namespace xblick::ros
