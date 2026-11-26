// StaticTransformPublisher answers zenoh-ext's AdvancedSubscriber, as zenoh-bridge-ros2dds asks for a late
// transient-local ROS subscriber (<key>/@adv/** with _max and _anyke): once, on the key itself. A query without _anyke
// gets no reply, and Durchblick's own queries (the key, a wildcard below the camera) still get one. The same cases as
// tests/test_camera.py::test_static_transforms_for_zenoh_ext.
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "xblick/camera.hpp"

namespace
{
    int failures = 0;

    void expect(bool condition, const char* what)
    {
        if (not condition)
        {
            std::fprintf(stderr, "FAILED: %s\n", what);
            ++failures;
        }
    }

    // The keys of the ok replies to `key` with `parameters` (xblick::replies takes no parameters), every reply (no
    // consolidation), the payload and encoding of the last one in `payload` and `encoding`. Fails the test when Zenoh
    // does not finalize the query within twice its timeout.
    std::vector<std::string> answered(const zenoh::Session& session, const std::string& key,
                                      const std::string& parameters = "", std::vector<std::uint8_t>* payload = nullptr,
                                      std::string* encoding = nullptr)
    {
        struct State
        {
            std::mutex mutex;
            std::condition_variable cv;
            std::vector<std::string> keys;
            std::vector<std::uint8_t> payload;
            std::string encoding;
            bool done = false;
        };
        auto state = std::make_shared<State>();  // the callbacks may come after the wait: they keep it
        zenoh::Session::GetOptions options;
        options.timeout_ms = 1000;
        options.consolidation = zenoh::QueryConsolidation(Z_CONSOLIDATION_MODE_NONE);
        session.get(
            zenoh::KeyExpr(key), parameters,
            [state](zenoh::Reply& reply) {
                if (reply.is_ok())
                {
                    std::lock_guard lock(state->mutex);
                    state->keys.emplace_back(reply.get_ok().get_keyexpr().as_string_view());
                    state->payload = reply.get_ok().get_payload().as_vector();
                    state->encoding = reply.get_ok().get_encoding().as_string();
                }
            },
            [state] {
                std::lock_guard lock(state->mutex);
                state->done = true;
                state->cv.notify_all();
            },
            std::move(options));
        std::unique_lock lock(state->mutex);
        expect(state->cv.wait_for(lock, std::chrono::seconds(2), [&] { return state->done; }), "query finalized");
        if (payload)
        {
            *payload = state->payload;
        }
        if (encoding)
        {
            *encoding = state->encoding;
        }
        return state->keys;
    }
}

int main()
{
    using namespace xblick;
    zenoh::Session session = open_session({.local_router = false});
    TFMessage transforms;
    TransformStamped t;
    t.header.frame_id = "depth";
    t.child_frame_id = "rgb";
    t.transform.translation.x = -0.032;
    t.transform.rotation.w = 1.0;
    transforms.transforms.push_back(t);
    {
        StaticTransformPublisher publisher(session, "t/cam", transforms);
        const std::string key = publisher.key();
        expect(key == "t/cam/tf_static", "the key");
        std::vector<std::uint8_t> payload;
        std::string encoding;
        const auto bridge = answered(session, key + "/@adv/**", "_max=100;_anyke", &payload, &encoding);
        expect(bridge == std::vector<std::string>{key}, "zenoh-ext's form: one reply, on the key itself");
        expect(payload == transforms.serialize(), "zenoh-ext's form: the transforms");
        expect(encoding == std::string(protocol::kTFMessageType.encoding()), "zenoh-ext's form: the encoding");
        expect(answered(session, key + "/@adv/**", "_max=100").empty(), "without _anyke: no reply");
        expect(answered(session, key) == std::vector<std::string>{key}, "the key: one reply");
        expect(answered(session, "t/cam/**") == std::vector<std::string>{key}, "a wildcard: one reply");
    }
    expect(answered(session, "t/cam/tf_static/@adv/**", "_max=100;_anyke").empty(), "closed: no reply");
    if (failures == 0)
    {
        std::printf("tf_static: all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
