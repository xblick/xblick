// ros.hpp against bytes captured from rmw_zenoh_cpp 0.2.13 (ROS 2 Jazzy's talker, listener and tf2_ros'
// static_transform_publisher on domain 87) and 0.10.6 (ROS 2 Lyrical's cam2image and showimage on domain 77); the
// same captures as tests/test_ros.py. The comparison with Python's
// xblick.ros case by case is tests/test_cpp_ros.py. The live part (live()) runs on one session that reaches nobody:
// the transient-local history, matched() (a Zenoh subscriber on ** is no ROS subscriber), the program node's rename
// by its first Parameters, the services without parameters over repeated Parameters, and the /tf_static reader that
// the static publishers share. graph_history(): two sessions on the loopback, a Graph made while a program of ours
// runs.
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "xblick/camera.hpp"
#include "xblick/parameters.hpp"
#include "xblick/ros.hpp"

namespace
{
    namespace ros = xblick::ros;
    int failures = 0;

    void check(bool ok, const std::string& what)
    {
        if (not ok)
        {
            ++failures;
            std::printf("FAIL: %s\n", what.c_str());
        }
    }

    template <class F>
    void refused(F f, const std::string& what)
    {
        try
        {
            f();
            check(false, what + ": not refused");
        }
        catch (const ros::RosError&)
        {
        }
    }

    std::vector<std::uint8_t> unhex(const std::string& h)
    {
        std::vector<std::uint8_t> out;
        for (std::size_t i = 0; i + 1 < h.size(); i += 2)
        {
            out.push_back(static_cast<std::uint8_t>(std::stoi(h.substr(i, 2), nullptr, 16)));
        }
        return out;
    }

    const std::string kTalkerChatter =
        "@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/11/MP/%/%/talker/%chatter/std_msgs::msg::dds_::String_/"
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,7:,:,:,,";
    const std::string kListenerChatter =
        "@ros2_lv/87/f54a4cdf922715d7d8be8d57bc314b9b/0/11/MS/%/%/listener/%chatter/std_msgs::msg::dds_::String_/"
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18/::,10:,:,:,,";
    const std::string kTalkerNode = "@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/0/NN/%/%/talker";
    const std::string kTalkerRosout =
        "@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/1/MP/%/%/talker/%rosout/rcl_interfaces::msg::dds_::Log_/"
        "RIHS01_e28ce254ca8abc06abf92773b74602cdbf116ed34fbaf294fb9f81da9f318eac/:1:,1000:,:10,0:,,";
    const std::string kStaticTf =
        "@ros2_lv/87/35014fbe89d585c32f08f47d73e62323/0/11/MP/%/%/static_transform_publisher_QjRTOJWJVsBx8onc/"
        "%tf_static/tf2_msgs::msg::dds_::TFMessage_/"
        "RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087/:1:,1:,:,:,,";
    const std::string kTalkerGet =
        "@ros2_lv/87/78cf7f7f723020cd89f943acf3eb2710/0/2/SS/%/%/talker/%talker%get_parameters/rcl_interfaces::srv::"
        "dds_::GetParameters_/RIHS01_bf9803d5c74cf989a5de3e0c2e99444599a627c7ff75f97b8c05b01003675cbc/::,1000:,:,:,,";
    const std::string kLyricalCam2image =
        "@ros2_lv/77/1b98790c445491809533e78cc3304fd5/0/10/MP/%/%/cam2image/%lyrg%image_raw/sensor_msgs::msg::dds_::"
        "Image_/RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47/::2,10:,:,:,,/backends:cpu:";
    const std::string kLyricalCam2imageGid = "3d71c655915d7801d6885bcb18590281";
    const std::string kLyricalShowimage =
        "@ros2_lv/77/16a25e588aeb51c11d8c2340714f13f/0/10/MS/%/%/showimage/%lyrg%image_raw/sensor_msgs::msg::dds_::"
        "Image_/RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47/::2,10:,:,:,,/backends:cpu:";
    const std::string kChatterKey =
        "87/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
    const std::string kChatterAttachment = "0300000000000000dca5f02d39ace8181031b1bb10e2fda68e3b7018e216540748";

    // ----------------------------------------------------------------------------------------- the live part

    constexpr int kDomain = 211;  // the live part's domain: its session reaches nobody anyway

    // Waits until `done` holds, at most `seconds`.
    template <class F>
    bool soon(F done, int seconds = 5)
    {
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
        while (std::chrono::steady_clock::now() < end)
        {
            if (done())
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return done();
    }

    // What a query's callbacks fill in, and its end (they may come after the wait: they keep it).
    template <class T>
    struct Answers
    {
        std::mutex mutex;
        std::condition_variable cv;
        std::vector<T> items;
        bool done = false;

        void add(T item)
        {
            std::lock_guard lock(mutex);
            items.push_back(std::move(item));
        }
        void end()
        {
            std::lock_guard lock(mutex);
            done = true;
            cv.notify_all();
        }
        std::vector<T> wait(const std::string& what)
        {
            std::unique_lock lock(mutex);
            check(cv.wait_for(lock, std::chrono::seconds(5), [this] { return done; }), what + ": query finalized");
            return items;
        }
    };

    // The liveliness tokens of `pattern` there now.
    std::vector<std::string> tokens(const zenoh::Session& session, const std::string& pattern)
    {
        auto answers = std::make_shared<Answers<std::string>>();
        session.liveliness_get(
            zenoh::KeyExpr(pattern),
            [answers](const zenoh::Reply& reply) {
                if (reply.is_ok())
                {
                    answers->add(std::string(reply.get_ok().get_keyexpr().as_string_view()));
                }
            },
            [answers] { answers->end(); });
        return answers->wait("tokens " + pattern);
    }

    // The names in each ok reply to a ROS client's list_parameters of `node` (fully qualified), every reply kept.
    std::vector<std::vector<std::string>> listed(const zenoh::Session& session, const std::string& node)
    {
        const std::string type = "rcl_interfaces/srv/ListParameters";
        auto answers = std::make_shared<Answers<std::vector<std::string>>>();
        zenoh::Session::GetOptions options;
        options.target = Z_QUERY_TARGET_ALL_COMPLETE;  // as ROS clients ask
        options.consolidation = zenoh::QueryConsolidation(Z_CONSOLIDATION_MODE_NONE);
        options.attachment = zenoh::Bytes(ros::encode_attachment(1, 0, ros::Gid{}));
        options.timeout_ms = 2000;
        session.get(
            zenoh::KeyExpr(ros::data_key(kDomain, node + "/list_parameters", type, ros::hash_of(type))), "",
            [answers](const zenoh::Reply& reply) {
                if (reply.is_ok())
                {
                    answers->add(xblick::ListParametersResponse::deserialize(reply.get_ok().get_payload().as_vector())
                                     .names);
                }
            },
            [answers] { answers->end(); }, std::move(options));
        return answers->wait("list_parameters of " + node);
    }

    // A free TCP port on the loopback (bound once, released).
    int free_port()
    {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        socklen_t size = sizeof(address);
        if (fd < 0 or ::bind(fd, reinterpret_cast<sockaddr*>(&address), size) != 0 or
            ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) != 0)
        {
            std::perror("free_port");
            std::exit(2);
        }
        ::close(fd);
        return ntohs(address.sin_port);
    }

    // A Graph made while a program of ours runs (its tokens in both subscribers' histories), its declarations
    // subscriber's history late (it comes by a query of its own, in no fixed order with the publishers'): the
    // program's ROS publisher is never in the graph as a foreign one; the Graph's query of the declarations holds it
    // back (Python's test_the_graph_holds_publishers_of_its_history_until_the_declarations_are_known).
    void graph_history()
    {
        const std::string endpoint = "tcp/127.0.0.1:" + std::to_string(free_port());
        zenoh::Session ours = xblick::open_session(
            {.listen = {endpoint}, .ros = ros::Options{.domain = kDomain, .node = "ours_history"}});
        zenoh::Session reader = xblick::open_session(
            {.connect = {endpoint}, .ros = ros::Options{.domain = kDomain, .node = "graph_reader"}});
        const std::shared_ptr<ros::Ros> part = ros::of(ours);
        const std::string topic = "/ours_history/image_raw";
        ros::Publisher publisher(ours, *part, part->node(), topic, "sensor_msgs/msg/Image");
        const std::string zid = ours.get_zid().to_string();
        {
            const ros::Graph first(reader, kDomain);
            check(soon([&] { return not first.publishers(topic).empty(); }), "the reader sees the publisher");
        }
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        ros::detail::graph_declarations_dropped_until_ns.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now + std::chrono::seconds(1)).count());
        const ros::Graph graph(reader, kDomain);
        bool seen = false, foreign = false;
        const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < end)
        {
            if (not graph.publishers(topic).empty())
            {
                seen = true;
                foreign = foreign or not graph.by_xblick(zid);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        ros::detail::graph_declarations_dropped_until_ns.store(0);
        check(seen, "the Graph made later has the publisher");
        check(not foreign, "never as a foreign one: held until the declarations are known");
    }

    void live()
    {
        ::unsetenv(ros::kEnableEnv);
        zenoh::Session session = xblick::open_session(
            {.local_router = false,
             .ros = ros::Options{.domain = kDomain, .node = "test_ros", .follow_parameters = true}});
        const std::shared_ptr<ros::Ros> part = ros::of(session);
        check(part != nullptr, "the session takes part in ROS");
        if (not part)
        {
            return;
        }
        const std::string tf = std::string(xblick::protocol::kTFMessageType.name);
        const std::string zid = session.get_zid().to_string();
        const std::string own = std::string(ros::kAdminSpace) + "/" + std::to_string(kDomain) + "/" + zid + "/*/*/";

        // transient local: matched() always (as Python's), and a late subscriber gets every sample of the cache's
        // depth, not one (the history query keeps every reply)
        const ros::Qos history{.durability = ros::kTransientLocal, .depth = 3};
        ros::Publisher cached(session, *part, part->node(), "/history_test", tf, history);
        ros::Publisher plain(session, *part, part->node(), "/plain_test", tf);
        check(cached.matched(), "a transient-local publisher is matched without a subscriber");
        check(not plain.matched(), "a volatile publisher without a subscriber is not");

        // a Zenoh subscriber on ** (a storage, z_sub, a traffic meter) switches no ROS form on; a ROS subscription of
        // the topic and type does (this session's own here), and none once it has gone (Python's
        // test_a_subscriber_on_everything_is_no_ros_subscriber)
        {
            auto everything = session.declare_subscriber(
                zenoh::KeyExpr("**"), [](const zenoh::Sample&) {}, zenoh::closures::none);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            check(not plain.matched(), "a Zenoh subscriber on ** is no ROS subscriber");
            {
                ros::Subscriber reader(session, *part, part->node(), "/plain_test", tf,
                                       [](std::vector<std::uint8_t>&&, const ros::Attachment&) {});
                check(soon([&] { return plain.matched(); }), "a ROS subscription of the topic and type matches");
            }
            check(soon([&] { return not plain.matched(); }), "no longer once that subscription has gone");
        }
        for (std::uint8_t i = 1; i <= 4; ++i)
        {
            const std::vector<std::uint8_t> payload{i};
            cached.put(payload);
        }
        struct Got
        {
            std::mutex mutex;
            std::set<int> payloads;  // each sample's one byte (-1: another size)
        };
        auto got = std::make_shared<Got>();
        {
            ros::Subscriber late(
                session, *part, part->node(), "/history_test", tf,
                [got](std::vector<std::uint8_t>&& payload, const ros::Attachment&) {
                    std::lock_guard lock(got->mutex);
                    got->payloads.insert(payload.size() == 1 ? payload[0] : -1);
                },
                history);
            const auto all = [got] {
                std::lock_guard lock(got->mutex);
                return got->payloads.size() >= 3;
            };
            check(soon(all), "a late transient-local subscriber gets the depth-3 history");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));  // a fourth would have come by now
            std::lock_guard lock(got->mutex);
            check(got->payloads == std::set<int>{2, 3, 4},
                  "the history: the last 3 samples, not the one before them");
        }

        // one /tf_static reader per Ros, shared by the static publishers, gone with the last of them
        const std::string static_readers = own + "MS/%/%/test_ros/%tf_static/**";
        {
            const auto a = ros::mirror_tf(session, true, "t/a/tf_static");
            const auto b = ros::mirror_tf(session, true, "t/b/tf_static");
            check(a and b and a->companion_ and a->companion_ == b->companion_,
                  "two static publishers share one /tf_static reader");
            check(tokens(session, static_readers).size() == 1, "one /tf_static subscription in ROS's graph");
        }
        check(tokens(session, static_readers).empty(), "the /tf_static reader goes with the last static publisher");

        // the program's first Parameters rename its node (follow_parameters): its entities declared anew under the
        // new name; the services without parameters of the old name go
        const std::string old_publisher = own + "MP/%/%/test_ros/%history_test/**";
        const std::string new_publisher = own + "MP/%/%test%cam/probe/%history_test/**";
        check(part->parameterless_sets() == 1, "open_session served the node without parameters");
        check(tokens(session, own + "SS/%/%/test_ros/**").size() == 6, "six services without parameters");
        check(listed(session, "/test_ros") == std::vector<std::vector<std::string>>{{}},
              "the node without parameters answers list_parameters, with no names");
        for (int round = 0; round < 4; ++round)
        {
            const std::string r = " (round " + std::to_string(round) + ")";
            {
                xblick::Parameters parameters(session, "probe", "test/cam");
                parameters.declare("gain", 1.0);
                check(part->node().fully_qualified() == "/test/cam/probe", "the node takes its parameters' name" + r);
                check(*cached.entity().token.topic == "/history_test" and cached.entity().token.node == "probe" and
                          cached.entity().token.ns == "/test/cam",
                      "a publisher made before is renamed" + r);
                check(tokens(session, old_publisher).empty() and tokens(session, new_publisher).size() == 1,
                      "its token goes under the new name" + r);
                check(tokens(session, own + "SS/%/%/test_ros/**").empty(), "the old name's services go" + r);
                check(tokens(session, own + "SS/%/%test%cam/probe/**").size() == 6, "six services of theirs" + r);
                check(listed(session, "/test/cam/probe") == std::vector<std::vector<std::string>>{{"gain"}},
                      "only the Parameters answer while they live" + r);
            }
            check(part->parameterless_sets() == 2, "one set per node name, served again, not made anew" + r);
            check(tokens(session, own + "SS/%/%test%cam/probe/**").size() == 6, "six services without parameters" + r);
            check(listed(session, "/test/cam/probe") == std::vector<std::vector<std::string>>{{}},
                  "after them, the node without parameters answers alone" + r);
        }
    }
}  // namespace

int main()
{
    // keys and names
    check(ros::data_key(87, "/chatter", "std_msgs/msg/String", ros::hash_of("std_msgs/msg/String")) == kChatterKey,
          "data key of /chatter");
    check(ros::dds_type("sensor_msgs/msg/Image") == "sensor_msgs::msg::dds_::Image_", "dds_type");
    check(ros::dds_type("rcl_interfaces/srv/SetParameters") == "rcl_interfaces::srv::dds_::SetParameters_",
          "dds_type of a service");
    check(ros::type_of_dds("tf2_msgs::msg::dds_::TFMessage_") == "tf2_msgs/msg/TFMessage", "type_of_dds");
    refused([] { ros::dds_type("sensor_msgs/Image"); }, "dds_type without msg");
    check(ros::topic_of("test/camera/color/image_raw") == "/test/camera/color/image_raw", "topic_of");
    for (const char* bad : {"rc-visard/color/image_raw", "test/3d/points", "a//b"})
    {
        refused([bad] { ros::topic_of(bad); }, std::string("topic_of ") + bad);
    }
    check(ros::node_name_of("xblick test-camera") == "xblick_test_camera", "node_name_of");
    check(ros::node_name_of("3d") == "_3d", "node_name_of a leading digit");
    check(ros::split_node("/test/camera/boxer") == std::make_pair(std::string("/test/camera"), std::string("boxer")),
          "split_node");
    check(ros::split_node("/talker") == std::make_pair(std::string("/"), std::string("talker")), "split_node /talker");
    check(ros::commanding("robot/controller/arm/command") and ros::commanding("robot/command/joints") and
              not ros::commanding("robot/commands/x") and not ros::commanding("test/camera/color/image_raw"),
          "commanding");

    // the attachment of a talker sample
    const std::vector<std::uint8_t> bytes = unhex(kChatterAttachment);
    const ros::Attachment a = ros::decode_attachment(bytes);
    check(a.seq == 3 and a.source_ns > 1'700'000'000LL * 1'000'000'000LL, "attachment's seq and stamp");
    check(ros::encode_attachment(a.seq, a.source_ns, a.gid) == bytes, "attachment written back");
    check(a.gid == ros::gid(kTalkerChatter), "the sample's gid is its publisher token's gid");
    refused([] { ros::decode_attachment(std::vector<std::uint8_t>{}); }, "an empty attachment");
    refused([&] { ros::decode_attachment(std::span(bytes).first(32)); }, "a short attachment");
    std::vector<std::uint8_t> length = bytes;
    length[16] = 15;
    refused([&] { ros::decode_attachment(length); }, "an attachment with length byte 15");

    // tokens read and written back
    for (const std::string& key : {kTalkerChatter, kListenerChatter, kTalkerNode, kTalkerRosout, kStaticTf, kTalkerGet,
                                   kLyricalCam2image, kLyricalShowimage})
    {
        check(ros::Token::parse(key).key() == key, "token read and written back: " + key);
    }
    const ros::Token t = ros::Token::parse(kStaticTf);
    check(t.domain == 87 and t.kind == "MP" and t.ns == "/" and t.topic == "/tf_static" and
              t.type_name == "tf2_msgs/msg/TFMessage" and t.qos == ros::kTfStaticQos,
          "the parts of /tf_static's token");
    check(ros::Token::parse(kTalkerGet).topic == "/talker/get_parameters", "a service's token");
    check(ros::Token::parse(kTalkerNode).fully_qualified_node() == "/talker", "a node's token");
    for (std::string bad : {std::string("@ros2_lv/87/x"), kTalkerChatter + "/extra"})
    {
        refused([bad] { ros::Token::parse(bad); }, "token " + bad);
    }
    std::string kind = kTalkerChatter;
    kind.replace(kind.find("/MP/"), 4, "/XX/");
    refused([kind] { ros::Token::parse(kind); }, "a token of kind XX");

    // Lyrical's optional 14th part: read and kept, the gid over the whole token; another 14th part or a 15th refused
    const ros::Token lyrical = ros::Token::parse(kLyricalCam2image);
    check(lyrical.kind == "MP" and lyrical.type_name == "sensor_msgs/msg/Image" and lyrical.backends == "backends:cpu:",
          "the parts of cam2image's token");
    check(ros::Token::parse(kLyricalShowimage).backends == ros::kCpuBackends, "showimage's backends");
    check(not ros::Token::parse(kTalkerChatter).backends, "no backends in a Jazzy token");
    const ros::Gid lyrical_gid = ros::gid(kLyricalCam2image);
    check(std::vector<std::uint8_t>(lyrical_gid.begin(), lyrical_gid.end()) == unhex(kLyricalCam2imageGid),
          "cam2image's gid is the one in its samples");
    std::string not_backends = kLyricalCam2image;
    not_backends.replace(not_backends.find("/backends:"), 10, "/b:");
    for (std::string bad : {not_backends, kLyricalCam2image + "/backends:cpu:", kTalkerNode + "/backends:cpu:"})
    {
        refused([bad] { ros::Token::parse(bad); }, "token " + bad);
    }

    // buffer types: a volatile subscription says backends:cpu:, nothing else does; an unknown message type refused
    check(ros::backends_of(ros::kSubscription, "sensor_msgs/msg/Image", ros::kStreamQos) == "backends:cpu:",
          "backends of an Image subscription");
    check(ros::backends_of(ros::kSubscription, "sensor_msgs/msg/PointCloud2", ros::kStreamQos) == "backends:cpu:",
          "backends of a PointCloud2 subscription");
    check(not ros::backends_of(ros::kSubscription, "sensor_msgs/msg/Image",
                               ros::Qos{.durability = ros::kTransientLocal, .depth = 1}),
          "no backends when transient local");
    check(not ros::backends_of(ros::kPublisher, "sensor_msgs/msg/Image", ros::kStreamQos),
          "no backends of a publisher");
    check(not ros::backends_of(ros::kSubscription, "sensor_msgs/msg/CameraInfo", ros::kStreamQos),
          "no backends of CameraInfo");
    check(not ros::backends_of(ros::kService, "rcl_interfaces/srv/GetParameters", ros::kParametersQos),
          "no backends of a service");
    refused([] { ros::backends_of(ros::kSubscription, "sensor_msgs/msg/Range", ros::kStreamQos); },
            "a subscription of unknown buffer status");
    for (const auto& [type, hash] : ros::type_hashes())
    {
        check(type.find("/msg/") == std::string::npos or ros::buffer_fields().contains(type),
              "buffer status of " + type);
    }
    check(ros::cpu_group_key("77/x/sensor_msgs::msg::dds_::Image_/RIHS01_0") ==
              "77/x/sensor_msgs::msg::dds_::Image_/RIHS01_0/_buf_cpu",
          "cpu_group_key");

    // QoS
    check(ros::Qos{.depth = 7}.key() == "::,7:,:,:,,", "QoS depth 7");
    check(ros::kTfStaticQos.key() == ":1:,1:,:,:,,", "QoS of /tf_static");
    check(ros::kParametersQos.key() == "::,1000:,:,:,,", "QoS of parameters");
    check(ros::Qos{}.key() == "::,:,:,:,,", "the default QoS");
    check(ros::Qos::parse(":1:,1000:,:10,0:,,").key() == ":1:,1000:,:10,0:,,", "QoS of /rosout read back");
    refused([] { ros::Qos::parse(":7:,:,:,,"); }, "a QoS without the history's comma");

    // domain
    ::unsetenv(ros::kDomainEnv);
    check(ros::domain_from(std::nullopt) == 0, "domain default");
    ::setenv(ros::kDomainEnv, "42", 1);
    check(ros::domain_from(std::nullopt) == 42, "domain of ROS_DOMAIN_ID");
    check(ros::domain_from(7) == 7, "domain of the option");
    refused([] { ros::domain_from(-1); }, "domain -1");
    ::setenv(ros::kDomainEnv, "abc", 1);
    refused([] { ros::domain_from(std::nullopt); }, "ROS_DOMAIN_ID=abc");
    ::unsetenv(ros::kDomainEnv);

    // the option family's rules (Python's session.ros_options_from_args): on by default, off with --no-ros or
    // XBLICK_ROS=0, the ROS options refused then
    ::unsetenv(ros::kEnableEnv);
    const std::nullopt_t none = std::nullopt;
    refused([] { ros::options_from(true, 3, std::nullopt, false, std::nullopt, "p"); }, "--ros-domain with --no-ros");
    refused([] { ros::options_from(true, std::nullopt, "n", false, std::nullopt, "p"); }, "--ros-node with --no-ros");
    refused([] { ros::options_from(true, std::nullopt, std::nullopt, true, std::nullopt, "p"); },
            "--ros-params-writable with --no-ros");
    refused([] { ros::options_from(true, std::nullopt, std::nullopt, false, "sim/clock", "p"); },
            "--ros-clock with --no-ros");
    check(not ros::options_from(true, none, none, false, none, "p"), "--no-ros: none");
    const auto o = ros::options_from(false, none, none, false, none, "xblick-cpp-camera");
    check(o and o->domain == 0 and o->node == "xblick_cpp_camera" and o->ns == "/" and o->follow_parameters and
              not o->params_writable,
          "the default: on, the program's name, renamed after its parameters");
    const auto c = ros::options_from(false, none, none, false, "/sim/clock", "xblick-yolox", "test/cam-2");
    check(c and c->ns == "/test/cam_2" and c->node == "xblick_yolox" and c->clock == "sim/clock",
          "the camera's namespace, written as ROS takes it; the clock key");
    const auto n = ros::options_from(false, none, "/lab/eye", false, none, "p");
    check(n and n->ns == "/lab" and n->node == "eye" and not n->follow_parameters, "--ros-node /lab/eye");
    refused([] { ros::options_from(false, std::nullopt, "bad-name", false, std::nullopt, "p"); }, "a node name with -");
    ::setenv(ros::kEnableEnv, "0", 1);
    check(not ros::options_from(false, none, none, false, none, "p") and not ros::from_environment(),
          "XBLICK_ROS=0: none");
    refused([] { ros::options_from(false, 3, std::nullopt, false, std::nullopt, "p"); }, "--ros-domain, XBLICK_ROS=0");
    ::setenv(ros::kEnableEnv, "yes", 1);
    refused([] { ros::from_environment(); }, "XBLICK_ROS=yes");
    ::setenv(ros::kEnableEnv, "1", 1);
    check(ros::from_environment().has_value(), "XBLICK_ROS=1");
    ::unsetenv(ros::kEnableEnv);
    check(ros::namespace_of("test/cam-1") == "/test/cam_1" and ros::namespace_of("") == "/", "namespace_of");
    check(ros::node_of("test/cam-1/probe") == std::make_pair(std::string("/test/cam_1"), std::string("probe")) and
              ros::node_of("/probe") == std::make_pair(std::string("/"), std::string("probe")),
          "node_of");

    // hashes
    check(ros::type_hashes().size() == 24, "24 type hashes, as Python's table");
    refused([] { ros::hash_of("xblick_msgs/msg/ShmImage"); }, "a type without a hash");

    live();
    graph_history();

    std::printf(failures ? "%d failure(s)\n" : "ros: ok\n", failures);
    return failures ? 1 : 0;
}
