// ROS 2 on rmw_zenoh in C++ (ros.hpp); Python's xblick.ros is the reference.
#include "xblick/ros.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <regex>

#include "xblick/camera.hpp"
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

    std::string namespace_of(const std::string& name)
    {
        std::string out;
        for (const std::string& part : split(name, '/'))
        {
            if (not part.empty())
            {
                out += "/" + node_name_of(part);
            }
        }
        return out.empty() ? "/" : out;
    }

    std::pair<std::string, std::string> node_of(const std::string& name)
    {
        std::string n = name;
        n.erase(0, n.find_first_not_of('/'));
        n.erase(n.find_last_not_of('/') + 1);
        const std::size_t slash = n.rfind('/');
        const std::string last = slash == std::string::npos ? n : n.substr(slash + 1);
        if (last.empty())
        {
            throw RosError(repr(name) + ": no node name");
        }
        return {namespace_of(slash == std::string::npos ? "" : n.substr(0, slash)), node_name_of(last)};
    }

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
        throw RosError(std::string(kEnableEnv) + "=" + repr(value) +
                       ": 0 (no part in ROS 2) or 1 (ROS 2 beside our own keys, the default)");
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
            if (backends)
            {
                out += "/" + *backends;
            }
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
        if (parts.size() != 13 and parts.size() != 14)
        {
            throw RosError(repr(key) + ": an entity's token has 13 parts, or 14 with backends");
        }
        if (parts.size() == 14 and not parts[13].starts_with("backends:"))
        {
            throw RosError(repr(key) + ": an entity's 14th token part starts with 'backends:'");
        }
        t.topic = demangle(parts[9]);
        t.type_name = type_of_dds(parts[10]);
        t.type_hash = parts[11];
        t.qos = Qos::parse(parts[12]);
        if (parts.size() == 14)
        {
            t.backends = parts[13];
        }
        return t;
    }

    // ---------------------------------------------------------------------------------------------- hashes

    const std::map<std::string, std::string>& type_hashes()
    {
        // Python's ros.TYPE_HASHES (ROS 2 Jazzy's share/<package>/<msg|srv>/<Name>.json, vision_msgs 4.1.1 included)
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

    const std::map<std::string, bool>& buffer_fields()
    {
        // Python's ros.BUFFER_FIELDS (ROS 2 Lyrical's has_buffer_fields_<pkg>__msg__<Name>())
        static const std::map<std::string, bool> table = {
            {"builtin_interfaces/msg/Time", false},
            {"diagnostic_msgs/msg/DiagnosticArray", false},
            {"geometry_msgs/msg/WrenchStamped", false},
            {"nav_msgs/msg/Odometry", false},
            {"rcl_interfaces/msg/ParameterEvent", false},
            {"rosgraph_msgs/msg/Clock", false},
            {"sensor_msgs/msg/CameraInfo", false},
            {"sensor_msgs/msg/CompressedImage", true},
            {"sensor_msgs/msg/Image", true},
            {"sensor_msgs/msg/Imu", false},
            {"sensor_msgs/msg/JointState", false},
            {"sensor_msgs/msg/PointCloud2", true},
            {"std_msgs/msg/Float64MultiArray", false},
            {"std_msgs/msg/String", false},
            {"tf2_msgs/msg/TFMessage", false},
            {"vision_msgs/msg/Classification", false},
            {"vision_msgs/msg/Detection2DArray", false},
            {"vision_msgs/msg/Detection3DArray", false},
        };
        return table;
    }

    std::optional<std::string> backends_of(std::string_view kind, const std::string& type_name, const Qos& qos)
    {
        if (kind != kSubscription)
        {
            return std::nullopt;
        }
        const auto it = buffer_fields().find(type_name);
        if (it == buffer_fields().end())
        {
            throw RosError(type_name + ": not known whether ROS 2 Lyrical sends it on buffer keys (ros.BUFFER_FIELDS)");
        }
        if (it->second and qos.durability != kTransientLocal)  // rmw_zenoh: no buffer keys when transient local
        {
            return std::string(kCpuBackends);
        }
        return std::nullopt;
    }

    std::string cpu_group_key(const std::string& key)
    {
        return key + "/" + std::string(kCpuGroup);
    }

    // ------------------------------------------------------------------------------------------ taking part

    Options default_options(const std::string& program, const std::string& camera)
    {
        Options o;
        o.domain = domain_from(std::nullopt);
        o.node = program.empty() ? std::string() : node_name_of(program);
        o.ns = camera.empty() ? "/" : namespace_of(camera);
        o.follow_parameters = true;
        return o;
    }

    std::optional<Options> from_environment()
    {
        if (not enabled_by_environment())
        {
            return std::nullopt;
        }
        return default_options("");
    }

    std::optional<Options> options_from(bool no_ros, std::optional<std::int64_t> domain,
                                        const std::optional<std::string>& node, bool params_writable,
                                        const std::optional<std::string>& clock, const std::string& program,
                                        const std::string& camera)
    {
        if (no_ros or not enabled_by_environment())
        {
            if (domain or node or params_writable or clock)
            {
                throw RosError(std::string("--ros-domain, --ros-node, --ros-params-writable and --ros-clock need ROS "
                                           "2, which ") +
                               (no_ros ? "--no-ros" : std::string(kEnableEnv) + "=0") + " turns off");
            }
            return std::nullopt;
        }
        Options o;
        if (node)
        {
            if (not node->empty() and node->front() == '/')
            {
                std::tie(o.ns, o.node) = split_node(*node);
            }
            else
            {
                o.node = check_node_name(*node);
            }
        }
        else
        {
            o = default_options(program, camera);
        }
        o.domain = domain_from(domain);
        o.params_writable = params_writable;
        if (clock)
        {
            std::string c = *clock;
            c.erase(0, c.find_first_not_of('/'));
            c.erase(c.find_last_not_of('/') + 1);
            check_ros_name("/" + c, "--ros-clock");
            o.clock = c;
        }
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
        node_ = add_node(options_.node.empty() ? std::string("xblick") : options_.node, options_.ns);
    }

    Ros::~Ros()
    {
        detach();
    }

    std::string Ros::next_id()
    {
        return std::to_string(ids_.fetch_add(1));
    }

    NodeName Ros::node() const
    {
        std::lock_guard lock(graph_mutex_);
        return node_;
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

    namespace
    {
        std::mutex registry_mutex;
        std::map<std::string, std::weak_ptr<Ros>> registry;  // zid -> its part in ROS, while its session is open

        // Keeps the program's Ros while its session is open (released when it closes).
        struct Holder
        {
            std::shared_ptr<Ros> ros;
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

        using detail::StaticTransform;

        StaticTransform static_of(const TransformStamped& t)
        {
            const auto& tr = t.transform.translation;
            const auto& q = t.transform.rotation;
            return {t.header.frame_id, {tr.x, tr.y, tr.z}, {q.x, q.y, q.z, q.w}};
        }

        constexpr double kTfTolerance = 1e-6;  // two static transforms of one frame that differ less are the same

        // Why two static transforms of one frame clash, or empty (q and -q are the same rotation), as Python's
        std::string static_differs(const StaticTransform& a, const StaticTransform& b)
        {
            if (std::get<0>(a) != std::get<0>(b))
            {
                return "parent " + repr(std::get<0>(b)) + " there, " + repr(std::get<0>(a)) + " here";
            }
            bool same_t = true, same_q = true, same_minus_q = true;
            for (int i = 0; i < 3; ++i)
            {
                same_t = same_t and std::abs(std::get<1>(a)[i] - std::get<1>(b)[i]) <= kTfTolerance;
            }
            for (int i = 0; i < 4; ++i)
            {
                same_q = same_q and std::abs(std::get<2>(a)[i] - std::get<2>(b)[i]) <= kTfTolerance;
                same_minus_q = same_minus_q and std::abs(std::get<2>(a)[i] + std::get<2>(b)[i]) <= kTfTolerance;
            }
            return same_t and (same_q or same_minus_q) ? ""
                                                       : "parent " + repr(std::get<0>(a)) + " in both, another transform";
        }
    }  // namespace

    // The watch for a second node of the program node's name: what its callback compares with, changed by a rename.
    struct Ros::Duplicates
    {
        std::mutex mutex;
        std::string own;   // the program node's token key
        std::string name;  // its fully qualified name
        std::set<std::string> said;
        int domain = 0;

        void seen(const std::string& key)
        {
            std::string name_now;
            {
                std::lock_guard lock(mutex);
                if (key == own)
                {
                    return;
                }
                Token t;
                try
                {
                    t = Token::parse(key);
                }
                catch (const RosError&)  // not a node token rmw_zenoh would take: no node of ROS's graph
                {
                    return;
                }
                if (t.fully_qualified_node() != name or not said.insert(name).second)
                {
                    return;
                }
                name_now = name;
            }
            say("ROS node " + name_now + " is in ROS's graph already (another program, or another session of this one, "
                "on domain " + std::to_string(domain) + "): ROS's tools then show two nodes of one name and reach one "
                "of them; give each program a node name of its own (--ros-node)");
        }
    };

    void Ros::attach(zenoh::Session& session, const std::string& program)
    {
        std::lock_guard lock(graph_mutex_);
        attached_ = true;
        program_ = program;
        node_token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(node_token(*this, node_).key())));
        duplicates_ = std::make_shared<Duplicates>();
        duplicates_->domain = domain();
        duplicates_->own = node_token(*this, node_).key();
        duplicates_->name = node_.fully_qualified();
        // a background subscriber on every node token of the domain (those there now too): Zenoh ends it with the
        // session. A subscriber of ours undeclared while the session closes (its drop callback, take_part) would wait
        // for the closing session for good.
        zenoh::Session::LivelinessSubscriberOptions options;
        options.history = true;
        session.liveliness_declare_background_subscriber(
            zenoh::KeyExpr(std::string(kAdminSpace) + "/" + std::to_string(domain()) + "/*/*/*/" + std::string(kNode) +
                           "/**"),
            [watch = duplicates_](const zenoh::Sample& sample) {
                if (sample.get_kind() == Z_SAMPLE_KIND_PUT)
                {
                    watch->seen(std::string(sample.get_keyexpr().as_string_view()));
                }
            },
            [] {}, std::move(options));
        if (not options_.clock.empty() and not clock_check_.joinable())
        {
            // one line when the clock key has no publisher by then (Python's Ros._check_clock): a key the program
            // never publishes would leave /clock empty without a word
            clock_check_ = std::jthread([this](const std::stop_token& stop) {
                std::mutex m;
                std::condition_variable_any woken;
                std::unique_lock lock(m);
                woken.wait_for(lock, stop, std::chrono::seconds(kClockWaitS), [] { return false; });
                if (not stop.stop_requested() and not clock_published_.load())
                {
                    say("--ros-clock " + options_.clock + ": this program has published no " +
                        options_.clock + " within " + std::to_string(kClockWaitS) + " s, so ROS's /clock stays " +
                        "empty until it does (ROS nodes with use_sim_time wait for it); check the key");
                }
            });
        }
    }

    void Ros::detach()
    {
        clock_check_.request_stop();  // not joined under the mutex: it takes none, and ends at once
        if (clock_check_.joinable() and clock_check_.get_id() != std::this_thread::get_id())
        {
            clock_check_.join();
        }
        std::lock_guard lock(graph_mutex_);
        node_token_.reset();  // a token may go while the session closes (it did before, take_part)
        attached_ = false;
    }

    void Ros::watch_duplicates(zenoh::Session& session)
    {
        {
            std::lock_guard lock(duplicates_->mutex);
            duplicates_->own = node_token(*this, node_).key();
            duplicates_->name = node_.fully_qualified();
        }
        // the nodes of the new name there now: the subscriber's history had those of the old one
        const std::string pattern = std::string(kAdminSpace) + "/" + std::to_string(domain()) + "/*/*/*/" +
                                    std::string(kNode) + "/*/" + mangle(node_.ns) + "/" + mangle(node_.name);
        session.liveliness_get(
            zenoh::KeyExpr(pattern),
            [watch = duplicates_](const zenoh::Reply& reply) {
                if (reply.is_ok())
                {
                    watch->seen(std::string(reply.get_ok().get_keyexpr().as_string_view()));
                }
            },
            [] {});
    }

    std::uint64_t Ros::enroll(const NodeName& as_made, std::function<void(const NodeName&)> rename)
    {
        std::lock_guard lock(graph_mutex_);
        const std::uint64_t id = ++enrolled_ids_;
        if (as_made.ns != node_.ns or as_made.name != node_.name)  // made while the node was renamed
        {
            rename(node_);
        }
        enrolled_.emplace(id, std::move(rename));
        return id;
    }

    void Ros::leave(std::uint64_t id)
    {
        std::lock_guard lock(graph_mutex_);
        enrolled_.erase(id);
    }

    void Ros::own(const Gid& gid)
    {
        std::lock_guard lock(graph_mutex_);
        own_gids_.insert(gid);
    }

    std::optional<NodeName> Ros::parameters_node(const std::string& name, zenoh::Session& session)
    {
        const auto [ns, node] = node_of(name);
        std::lock_guard lock(graph_mutex_);
        if (parameters_taken_)
        {
            return std::nullopt;
        }
        parameters_taken_ = true;
        if (retire_)
        {
            std::exchange(retire_, {})();
        }
        if (options_.follow_parameters and (ns != node_.ns or node != node_.name))
        {
            check_node_name(node);
            if (ns != "/")
            {
                check_ros_name(ns, "namespace");
            }
            node_.ns = ns;
            node_.name = node;
            if (attached_)
            {
                node_token_.reset();
                node_token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(node_token(*this, node_).key())));
                for (auto& [id, rename] : enrolled_)
                {
                    rename(node_);
                }
                watch_duplicates(session);
                if (not program_.empty())
                {
                    std::fprintf(stderr, "%s: in ROS 2 as node %s (the name of its parameters)\n", program_.c_str(),
                                 node_.fully_qualified().c_str());
                    std::fflush(stderr);
                }
            }
        }
        return node_;
    }

    void Ros::until_parameters(std::function<void()> retire)
    {
        std::lock_guard lock(graph_mutex_);
        retire_ = std::move(retire);
    }

    void Ros::release_parameters()
    {
        std::lock_guard lock(graph_mutex_);
        parameters_taken_ = false;
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
        std::vector<std::pair<std::string, std::pair<StaticTransform, StaticTransform>>> compare;
        {
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
                if (is_static)
                {
                    static_mine_[child] = static_of(t);
                    if (const auto other = static_others_.find(child); other != static_others_.end())
                    {
                        compare.push_back({child, {static_mine_[child], other->second}});
                    }
                }
            }
        }
        for (const auto& [child, pair] : compare)
        {
            say_static(child, pair.first, pair.second);
        }
    }

    void Ros::say_static(const std::string& child, const StaticTransform& mine, const StaticTransform& theirs)
    {
        const std::string why = static_differs(mine, theirs);
        {
            std::lock_guard lock(tf_mutex_);
            if (why.empty() or not static_said_.insert(child).second)
            {
                return;
            }
        }
        say("ROS /tf_static: frame " + repr(child) + " is sent by another program too, " + why +
            "; tf2 keeps one of them: give each camera and robot frame names of its own (--frame-id)");
    }

    void Ros::on_static_sample(std::span<const std::uint8_t> payload, const Attachment& attachment)
    {
        {
            std::lock_guard lock(graph_mutex_);
            if (own_gids_.contains(attachment.gid))
            {
                return;
            }
        }
        for (const auto& t : TFMessage::deserialize(payload).transforms)
        {
            const StaticTransform theirs = static_of(t);
            std::optional<StaticTransform> mine;
            {
                std::lock_guard lock(tf_mutex_);
                static_others_[t.child_frame_id] = theirs;
                if (const auto found = static_mine_.find(t.child_frame_id); found != static_mine_.end())
                {
                    mine = found->second;
                }
            }
            if (mine)
            {
                say_static(t.child_frame_id, *mine, theirs);
            }
        }
    }

    std::shared_ptr<Ros> of(const zenoh::Session& session)
    {
        std::lock_guard lock(registry_mutex);
        const auto found = registry.find(session.get_zid().to_string());
        return found == registry.end() ? nullptr : found->second.lock();
    }

    namespace detail
    {
        void take_part(zenoh::Session& session, const Options& options, const std::string& program)
        {
            const std::string zid = session.get_zid().to_string();
            Options o = options;
            if (o.node.empty())
            {
                o.node = node_name_of(program.empty() ? std::string("xblick") : program);
            }
            auto holder = std::make_shared<Holder>();
            holder->ros = std::make_shared<Ros>(zid, o);
            holder->ros->attach(session, program);
            const std::string key = node_token(*holder->ros, holder->ros->node()).key();
            {
                std::lock_guard lock(registry_mutex);
                registry[zid] = holder->ros;
            }
            // Kept by the session: a background liveliness subscriber on the node's own token, whose drop callback
            // (when the session closes) releases the node's tokens and its Ros (as the declaration's queryable keeps
            // the declaration's token, declaration.cpp).
            session.liveliness_declare_background_subscriber(
                zenoh::KeyExpr(key), [](const zenoh::Sample&) {},
                [holder, zid]() mutable {
                    {
                        std::lock_guard lock(registry_mutex);
                        registry.erase(zid);
                    }
                    holder->ros->detach();
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
        token.backends = backends_of(kind, type_name, qos);
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

    void Entity::rename(const NodeName& node)
    {
        token.ns = node.ns;
        token.node = node.name;
        token_key = token.key();
        gid = ros::gid(token_key);
    }

    namespace
    {
        // An entity made on the program's node (its id): enrolled, so that a rename of the node reaches it
        // (Ros::enroll); its gid counted as the session's own.
        template <typename Renamed>
        std::uint64_t enroll_if_program_node(Ros& ros, std::weak_ptr<Ros>& kept, const NodeName& node, const Gid& gid,
                                             Renamed renamed)
        {
            ros.own(gid);
            std::shared_ptr<Ros> shared = ros.weak_from_this().lock();
            if (not shared or node.id != ros.node().id)  // a Ros of no session (a test), or a node of its own
            {
                return 0;
            }
            kept = shared;
            return ros.enroll(node, std::move(renamed));
        }

        void leave(std::weak_ptr<Ros>& kept, std::uint64_t id)
        {
            if (const std::shared_ptr<Ros> ros = kept.lock(); ros and id != 0)
            {
                ros->leave(id);
            }
        }
    }  // namespace

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
        std::atomic<bool> said{false};  // a failed reply said once
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
                    // zenoh-ext's subscriber asks with _anyke, which lets the replies carry the data key; a query
                    // without it could take none of them: not answered (dropped at once), nothing said
                    if (query.get_parameters().find("_anyke") == std::string_view::npos)
                    {
                        return;
                    }
                    decltype(cache->samples) samples;
                    {
                        std::lock_guard lock(cache->mutex);  // copied, then replied: a put is not held up
                        samples = cache->samples;
                    }
                    try
                    {
                        for (const auto& [payload, attachment] : samples)
                        {
                            zenoh::Query::ReplyOptions reply;
                            reply.attachment = zenoh::Bytes(attachment);
                            query.reply(zenoh::KeyExpr(data), zenoh::Bytes(payload), std::move(reply));
                        }
                    }
                    catch (const std::exception& e)  // the session closing: said once per publisher
                    {
                        if (not cache->said.exchange(true))
                        {
                            say(data + ": a history query not answered: " + e.what());
                        }
                    }
                },
                zenoh::closures::none));
            cache_token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(cache_key)));
        }
        token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
        enrolled_ = enroll_if_program_node(ros, ros_, node, entity_.gid, [this, &session, &ros](const NodeName& n) {
            renamed(session, n);
            ros.own(entity_.gid);
        });
    }

    void Publisher::renamed(zenoh::Session& session, const NodeName& node)
    {
        std::lock_guard lock(mutex_);  // put's gid
        entity_.rename(node);
        token_.reset();
        token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
    }

    Publisher::~Publisher()
    {
        leave(ros_, enrolled_);
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
        std::vector<std::uint8_t> attachment = this->attachment();
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

    std::vector<std::uint8_t> Publisher::attachment()
    {
        std::lock_guard lock(mutex_);
        return encode_attachment(++seq_, now_ns(), entity_.gid);
    }

    // ------------------------------------------------------------------------- ROS cameras as ROS names them

    bool is_topic(const std::string& name)
    {
        return not name.empty() and name.front() == '/';
    }

    std::string compressed_topic(const std::string& image_topic)
    {
        return image_topic + "/" + std::string(kCompressedTopic);
    }

    std::string compressed_depth_topic(const std::string& image_topic)
    {
        return image_topic + "/" + std::string(kCompressedDepthTopic);
    }

    std::string camera_info_topic(const std::string& image_topic)
    {
        return image_topic.substr(0, image_topic.rfind('/')) + "/" + std::string(kCameraInfoTopic);
    }

    struct Graph::State
    {
        mutable std::mutex mutex;
        std::map<std::string, Token> tokens;
        std::set<std::string> xblick;  // the zids of the sessions that declared a program of ours
        std::function<void()> changed;
        std::string own_zid;
        std::atomic<bool> closed{false};
    };

    Graph::Graph(zenoh::Session& session, int domain, std::function<void()> changed)
        : session_(session), state_(std::make_shared<State>())
    {
        state_->changed = std::move(changed);
        state_->own_zid = session.get_zid().to_string();
        // the declarations first (the class's comment in ros.hpp): Zenoh calls the callbacks of one link's samples
        // one after the other, and a program's declaration comes before its ROS tokens
        const std::string all = declaration_key("*");
        const std::size_t before = all.find('*'), after = all.size() - before - 1;
        zenoh::Session::LivelinessSubscriberOptions declared;
        declared.history = true;
        declarations_.emplace(session.liveliness_declare_subscriber(
            zenoh::KeyExpr(all),
            [state = state_, before, after](const zenoh::Sample& sample) {
                if (state->closed.load() or sample.get_kind() != Z_SAMPLE_KIND_PUT)
                {
                    return;  // a zid once declared stays ours: the tokens of a program that ends go in no fixed order
                }
                const std::string key(sample.get_keyexpr().as_string_view());
                if (key.size() <= before + after)
                {
                    return;
                }
                const std::string zid = key.substr(before, key.size() - before - after);
                {
                    std::lock_guard lock(state->mutex);
                    if (zid.find('/') != std::string::npos or not state->xblick.insert(zid).second)
                    {
                        return;
                    }
                }
                if (state->changed)
                {
                    state->changed();
                }
            },
            zenoh::closures::none, std::move(declared)));
        const std::string pattern = std::string(kAdminSpace) + "/" + std::to_string(domain) + "/*/*/*/" +
                                    std::string(kPublisher) + "/*/*/*/**";
        zenoh::Session::LivelinessSubscriberOptions options;
        options.history = true;
        subscriber_.emplace(session.liveliness_declare_subscriber(
            zenoh::KeyExpr(pattern),
            [state = state_](const zenoh::Sample& sample) {
                if (state->closed.load())
                {
                    return;
                }
                const std::string key(sample.get_keyexpr().as_string_view());
                {
                    std::lock_guard lock(state->mutex);
                    if (sample.get_kind() == Z_SAMPLE_KIND_PUT)
                    {
                        try
                        {
                            state->tokens.insert_or_assign(key, Token::parse(key));
                        }
                        catch (const RosError&)
                        {
                            return;  // not a token rmw_zenoh would take
                        }
                    }
                    else if (state->tokens.erase(key) == 0)
                    {
                        return;
                    }
                }
                if (state->changed)
                {
                    state->changed();
                }
            },
            zenoh::closures::none, std::move(options)));
    }

    Graph::~Graph()
    {
        state_->closed.store(true);
        subscriber_.reset();
        declarations_.reset();
    }

    std::vector<Token> Graph::publishers(const std::optional<std::string>& topic) const
    {
        std::lock_guard lock(state_->mutex);
        std::vector<Token> out;
        for (const auto& [key, t] : state_->tokens)
        {
            if ((not topic or t.topic == *topic) and t.zid != state_->own_zid)
            {
                out.push_back(t);
            }
        }
        return out;
    }

    bool Graph::by_xblick(const std::string& zid) const
    {
        std::lock_guard lock(state_->mutex);
        return state_->xblick.count(zid) != 0;
    }

    // ------------------------------------------------------------------------------------------- GpuLink

    namespace
    {
        // <key>/_buf/<gid in hex>: where a Lyrical publisher sends to one buffer-aware subscription
        std::string buffer_key(const std::string& key, const std::array<std::uint8_t, 16>& gid)
        {
            static const char* digits = "0123456789abcdef";
            std::string out = key + "/_buf/";
            for (const std::uint8_t b : gid)
            {
                out += digits[b >> 4];
                out += digits[b & 15];
            }
            return out;
        }
    }  // namespace

    struct GpuLink::State
    {
        State(const std::string& topic, int device, std::string socket_dir, std::string shm_dir)
            : link(topic, device, std::move(socket_dir), std::move(shm_dir))
        {
        }
        ros_cuda::Link link;
        std::mutex mutex;  // outs
        std::map<std::string, std::shared_ptr<zenoh::Publisher>> outs;  // buffer key -> its publisher
        std::string key;  // the publisher's data key
    };

    GpuLink::GpuLink(zenoh::Session& session, Publisher& publisher, int device, std::optional<std::string> socket_dir,
                     std::optional<std::string> shm_dir)
        : session_(session), publisher_(publisher)
    {
        if (publisher.type_name() != "sensor_msgs/msg/Image")
        {
            throw RosError(publisher.type_name() + ": the GPU link carries sensor_msgs/msg/Image");
        }
        try
        {
            state_ = std::make_shared<State>(publisher.topic(), device,
                                             socket_dir.value_or(ros_cuda::socket_dir_from_environment()),
                                             shm_dir.value_or(ros_cuda::shm_dir_from_environment()));
        }
        catch (const std::invalid_argument& error)
        {
            throw RosError(error.what());
        }
        state_->key = publisher.key();
        const Token& token = publisher.entity().token;
        const std::string pattern = std::string(kAdminSpace) + "/" + std::to_string(token.domain) + "/*/*/*/" +
                                    std::string(kSubscription) + "/*/*/*/" + mangle(publisher.topic()) + "/" +
                                    dds_type(publisher.type_name()) + "/" + *token.type_hash + "/**";
        zenoh::Session::LivelinessSubscriberOptions options;
        options.history = true;
        std::weak_ptr<State> weak = state_;
        tokens_.emplace(session.liveliness_declare_subscriber(
            zenoh::KeyExpr(pattern),
            [weak](const zenoh::Sample& sample) {
                const std::shared_ptr<State> state = weak.lock();
                if (not state)
                {
                    return;
                }
                const std::string key(sample.get_keyexpr().as_string_view());
                if (sample.get_kind() != Z_SAMPLE_KIND_PUT)
                {
                    if (const auto gid = state->link.remove(key))
                    {
                        std::shared_ptr<zenoh::Publisher> gone;
                        {
                            std::lock_guard lock(state->mutex);
                            const auto found = state->outs.find(buffer_key(state->key, *gid));
                            if (found != state->outs.end())
                            {
                                gone = std::move(found->second);
                                state->outs.erase(found);
                            }
                        }
                    }
                    return;
                }
                try
                {
                    const Token t = Token::parse(key);
                    ros_cuda::Reader reader;
                    reader.gid = gid(key);
                    reader.backends = t.backends;
                    const Qos qos = t.qos.value_or(Qos{});
                    reader.keep_all = qos.history == kKeepAll;
                    reader.depth = qos.depth;
                    reader.node = t.fully_qualified_node();
                    state->link.add(key, reader);
                }
                catch (const RosError&)
                {
                    // not a token rmw_zenoh would take: no reader
                }
            },
            zenoh::closures::none, std::move(options)));
    }

    GpuLink::~GpuLink()
    {
        tokens_.reset();
        std::lock_guard lock(state_->mutex);
        state_->outs.clear();
    }

    std::string GpuLink::buf_key(const std::array<std::uint8_t, 16>& gid) const
    {
        return buffer_key(state_->key, gid);
    }

    std::shared_ptr<zenoh::Publisher> GpuLink::out(const std::string& key)
    {
        std::lock_guard lock(state_->mutex);
        std::shared_ptr<zenoh::Publisher>& found = state_->outs[key];
        if (not found)
        {
            zenoh::Session::PublisherOptions options;
            options.congestion_control = congestion(publisher_.qos());
            found = std::make_shared<zenoh::Publisher>(
                session_.declare_publisher(zenoh::KeyExpr(key), std::move(options)));
        }
        return found;  // shared: a token that goes meanwhile drops only the map's
    }

    void GpuLink::say_notes()
    {
        for (const ros_cuda::Note& note : state_->link.notes())
        {
            say(note.text);
        }
    }

    std::optional<ros_cuda::Plan> GpuLink::plan()
    {
        std::optional<ros_cuda::Plan> plan = state_->link.plan();
        say_notes();
        return plan;
    }

    std::vector<std::string> GpuLink::send_gpu(const ros_cuda::Plan& plan, CudaRingWriter& ring, std::uint32_t slot,
                                               std::uint64_t nbytes, const ros_cuda::ImageHeader& header,
                                               const std::vector<std::uint8_t>& attachment)
    {
        auto messages = state_->link.gpu_messages(plan, ring, slot, nbytes, header);
        say_notes();
        std::vector<std::string> sent;
        for (auto& [gid, payload] : messages)
        {
            const std::string key = buf_key(gid);
            zenoh::Publisher::PutOptions options;
            options.attachment = zenoh::Bytes(attachment);
            out(key)->put(zenoh::Bytes(std::move(payload)), std::move(options));
            sent.push_back(key);
        }
        return sent;
    }

    std::vector<std::string> GpuLink::send_cpu(const ros_cuda::Plan& plan, std::span<const std::uint8_t> payload,
                                               const std::vector<std::uint8_t>& attachment)
    {
        std::vector<std::string> keys;
        if (plan.cpu_group)
        {
            keys.push_back(cpu_group_key(state_->key));
        }
        for (const auto& gid : plan.cpu)
        {
            keys.push_back(buf_key(gid));
        }
        for (const std::string& key : keys)
        {
            zenoh::Publisher::PutOptions options;
            options.attachment = zenoh::Bytes(attachment);
            out(key)->put(zenoh::Bytes(std::vector<std::uint8_t>(payload.begin(), payload.end())), std::move(options));
        }
        return keys;
    }

    ros_cuda::LinkStats GpuLink::stats() const
    {
        return state_->link.stats();
    }

    // What a subscription's callbacks share (Zenoh's threads may still call them while it goes): the reader's
    // callback, and closed, set first by ~Subscriber.
    struct Subscriber::State
    {
        Callback callback;
        std::string topic;
        std::atomic<bool> warned{false};
        std::atomic<bool> closed{false};

        // A sample of the topic or of a publisher's history: to the callback with ROS's attachment, else dropped
        // with one line, as ROS drops it.
        void take(const zenoh::Sample& sample)
        {
            if (closed.load())
            {
                return;
            }
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
                    if (not warned.exchange(true))
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
        }
    };

    Subscriber::Subscriber(zenoh::Session& session, Ros& ros, const NodeName& node, const std::string& topic,
                           const std::string& type_name, Callback callback, const Qos& qos)
        : entity_(ros, node, kSubscription, topic, type_name, qos, ros.next_id()), state_(std::make_shared<State>())
    {
        state_->callback = std::move(callback);
        state_->topic = topic;
        const std::shared_ptr<State> state = state_;
        subscriber_.emplace(session.declare_subscriber(
            zenoh::KeyExpr(entity_.key), [state](const zenoh::Sample& sample) { state->take(sample); },
            zenoh::closures::none));
        if (qos.durability == kTransientLocal)
        {
            // the history of every publisher of the topic with zenoh-ext's publication cache (rmw_zenoh's
            // transient-local publishers, and ours): each one's liveliness token <key>/@adv/pub/<zid>/<id>/<meta>,
            // those there now and those that come later, and a query of that key for its last `depth` samples, as
            // zenoh-ext's AdvancedSubscriber with detect_late_publishers asks it (_max, _anyke). A sample can come
            // twice (live and from the history); its reader takes it again (tf2: the same transform).
            const std::string params = "_max=" + std::to_string(std::max(qos.depth, 1)) + ";_anyke";
            zenoh::Session::LivelinessSubscriberOptions options;
            options.history = true;
            history_.emplace(session.liveliness_declare_subscriber(
                zenoh::KeyExpr(entity_.key + "/@adv/pub/**"),
                [state, &session, params](const zenoh::Sample& token) {
                    if (state->closed.load() or token.get_kind() != Z_SAMPLE_KIND_PUT)
                    {
                        return;
                    }
                    try
                    {
                        session.get(
                            token.get_keyexpr(), params,
                            [state](const zenoh::Reply& reply) {
                                if (reply.is_ok())
                                {
                                    state->take(reply.get_ok());
                                }
                            },
                            [] {});
                    }
                    catch (const std::exception& e)  // the session closing
                    {
                        say(state->topic + ": a publisher's history not asked for: " + e.what());
                    }
                },
                zenoh::closures::none, std::move(options)));
        }
        if (entity_.token.backends == kCpuBackends)
        {
            cpu_group_.emplace(session.declare_subscriber(
                zenoh::KeyExpr(cpu_group_key(entity_.key)),
                [state](const zenoh::Sample& sample) { state->take(sample); }, zenoh::closures::none));
        }
        token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
        enrolled_ = enroll_if_program_node(ros, ros_, node, entity_.gid, [this, &session, &ros](const NodeName& n) {
            entity_.rename(n);
            ros.own(entity_.gid);
            token_.reset();
            token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
        });
    }

    Subscriber::~Subscriber()
    {
        leave(ros_, enrolled_);
        state_->closed.store(true);
        token_.reset();
        history_.reset();
        cpu_group_.reset();
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
        enrolled_ = enroll_if_program_node(ros, ros_, node, entity_.gid, [this, &session, &ros](const NodeName& n) {
            entity_.rename(n);
            ros.own(entity_.gid);
            token_.reset();
            token_.emplace(session.liveliness_declare_token(zenoh::KeyExpr(entity_.token_key)));
        });
    }

    Service::~Service()
    {
        leave(ros_, enrolled_);
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
        ros::reply(query, entity_.key, std::move(payload), request);
    }

    void reply(const zenoh::Query& query, const std::string& key, std::vector<std::uint8_t>&& payload,
               const Attachment& request)
    {
        zenoh::Query::ReplyOptions options;
        options.attachment = zenoh::Bytes(encode_attachment(request.seq, now_ns(), request.gid));
        query.reply(zenoh::KeyExpr(key), zenoh::Bytes(std::move(payload)), std::move(options));
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
        std::string topic;
        std::string k = key;
        k.erase(0, k.find_first_not_of('/'));
        k.erase(k.find_last_not_of('/') + 1);
        if (not ros->clock().empty() and k == ros->clock())
        {
            if (type.version != 0 or type_name != "rosgraph_msgs/msg/Clock")
            {
                throw RosError("--ros-clock " + key + ": its type is " + type.schema() +
                               ", not rosgraph_msgs/msg/Clock");
            }
            topic = std::string(kClockTopic);
            ros->clock_published();
        }
        if (type.version != 0 or not type_hashes().contains(type_name))
        {
            say(key + ": not offered to ROS (" + type.schema() + " has no ROS type hash here)");
            return nullptr;
        }
        if (topic.empty())
        {
            try
            {
                topic = topic_of(key);
            }
            catch (const RosError& error)  // a key that is no ROS name: ours only, the program goes on
            {
                say(key + ": not offered to ROS (" + error.what() + ")");
                return nullptr;
            }
        }
        return std::make_unique<Publisher>(session, *ros, ros->node(), topic, type_name, qos);
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
        if (is_static)
        {
            // ROS's /tf_static of other programs, compared with this program's (Ros::check_tf): read while this
            // publisher lives, so that it goes with an object of the program, before the session
            std::weak_ptr<Ros> weak = ros;
            publisher->companion_ = std::make_unique<Subscriber>(
                session, *ros, ros->node(), "/tf_static", std::string(protocol::kTFMessageType.name),
                [weak](std::vector<std::uint8_t>&& payload, const Attachment& attachment) {
                    if (const std::shared_ptr<Ros> part = weak.lock())
                    {
                        part->on_static_sample(payload, attachment);
                    }
                },
                kTfStaticReadQos);
        }
        return publisher;
    }
}  // namespace xblick::ros
