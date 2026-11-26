// A stock ROS 2 Lyrical subscriber of sensor_msgs/Image that accepts the CUDA buffer backend, for
// tests/test_ros_cuda_live.py: for each message one line with the backend the data came by and a check of the test
// pattern on the GPU (byte i = (i * 13 + seed) & 0xFF, seed in frame_id "xb:<seed>"):
//   CUDACHECK <n> frame=<id> size=<bytes> backend=<cuda|cpu> mismatches=<n> OK|FAIL
// Parameters: topic (string), backends (string, "any": acceptable_buffer_backends), hold_ms (int, 0: keep each message
// that long).
#include <cuda_runtime.h>

#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

cudaError_t pattern_check(const void* p, size_t n, uint32_t seed, cudaStream_t s, unsigned long long* bad);

class CudaCheck : public rclcpp::Node
{
public:
    CudaCheck() : Node("cuda_check")
    {
        backends_ = declare_parameter<std::string>("backends", "any");
        hold_ms_ = declare_parameter<int>("hold_ms", 0);
        const std::string topic = declare_parameter<std::string>("topic", "image");
        cudaStreamCreate(&stream_);
        rclcpp::SubscriptionOptions options;
        options.acceptable_buffer_backends = backends_.c_str();
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            topic, 10, [this](sensor_msgs::msg::Image::SharedPtr m) { on_image(std::move(m)); }, options);
        RCLCPP_INFO(get_logger(), "CUDACHECK start pid=%d topic=%s backends=%s", getpid(), topic.c_str(),
                    backends_.c_str());
        if (hold_ms_ > 0)  // let go of held messages on time, also when no new one comes
        {
            timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { release_held(); });
        }
    }

private:
    void on_image(sensor_msgs::msg::Image::SharedPtr m)
    {
        ++count_;
        uint32_t seed = 0;
        if (m->header.frame_id.rfind("xb:", 0) == 0)
        {
            seed = std::stoul(m->header.frame_id.substr(3));
        }
        const std::string backend = m->data.get_backend_type();
        const size_t n = m->data.size();
        unsigned long long bad = 0;
        std::string error;
        try
        {
            if (backend == "cuda" and n > 0)
            {
                const rosidl::Buffer<uint8_t>& data = m->data;
                cuda_buffer_backend::ReadHandle handle = cuda_buffer_backend::from_input_buffer(data, stream_);
                const cudaError_t e = pattern_check(handle.get_ptr(), n, seed, stream_, &bad);
                if (e != cudaSuccess)
                {
                    error = cudaGetErrorString(e);
                }
            }
            else if (backend != "cuda")
            {
                const std::vector<uint8_t>& v = m->data;
                for (size_t i = 0; i < v.size(); ++i)
                {
                    bad += v[i] != static_cast<uint8_t>((i * 13u + seed) & 0xFFu);
                }
            }
        }
        catch (const std::exception& e)
        {
            error = e.what();
        }
        std::printf("CUDACHECK %zu frame=%s size=%zu backend=%s mismatches=%llu %s%s%s\n", count_,
                    m->header.frame_id.c_str(), n, backend.c_str(), bad, (n > 0 and bad == 0 and error.empty())
                    ? "OK" : "FAIL", error.empty() ? "" : " error=", error.c_str());
        std::fflush(stdout);
        if (hold_ms_ > 0)
        {
            held_.emplace_back(std::chrono::steady_clock::now(), m);
        }
    }

    void release_held()
    {
        while (not held_.empty() and std::chrono::steady_clock::now() - held_.front().first >=
               std::chrono::milliseconds(hold_ms_))
        {
            held_.pop_front();
        }
    }

    std::string backends_;
    int hold_ms_ = 0;
    size_t count_ = 0;
    cudaStream_t stream_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::deque<std::pair<std::chrono::steady_clock::time_point, sensor_msgs::msg::Image::SharedPtr>> held_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CudaCheck>());
    rclcpp::shutdown();
    return 0;
}
