// A stock ROS 2 Lyrical publisher of sensor_msgs/Image whose data is a CUDA buffer of the backend (allocate_buffer,
// from_output_buffer), for tests/test_ros_cuda_live.py: the test pattern filled on the GPU by a kernel on the node's
// stream (byte i = (i * 13 + seed) & 0xFF, frame_id "xb:<seed>", seeds from 1000), as an Isaac ROS node fills its
// output. One line per message: CUDAPUB <n> frame=<id> backend=<cuda|cpu>.
// Parameters: topic (string), width, height (int), rate_ms (int, between messages).
#include <cuda_runtime.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

cudaError_t pattern_fill(void* p, size_t n, uint32_t seed, cudaStream_t s);

class CudaPatternPub : public rclcpp::Node
{
public:
    CudaPatternPub() : Node("cuda_pattern_pub")
    {
        const std::string topic = declare_parameter<std::string>("topic", "image");
        width_ = declare_parameter<int>("width", 640);
        height_ = declare_parameter<int>("height", 480);
        cudaStreamCreate(&stream_);
        pub_ = create_publisher<sensor_msgs::msg::Image>(topic, 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(declare_parameter<int>("rate_ms", 50)), [this] { tick(); });
        RCLCPP_INFO(get_logger(), "CUDAPUB start pid=%d topic=%s", getpid(), topic.c_str());
    }

private:
    void tick()
    {
        const size_t n = static_cast<size_t>(width_) * height_ * 3;
        sensor_msgs::msg::Image msg;
        msg.data = cuda_buffer_backend::allocate_buffer(n);
        msg.header.stamp = now();
        msg.header.frame_id = "xb:" + std::to_string(seed_);
        msg.height = height_;
        msg.width = width_;
        msg.encoding = "rgb8";
        msg.step = width_ * 3;
        {
            cuda_buffer_backend::WriteHandle handle = cuda_buffer_backend::from_output_buffer(msg.data, stream_);
            pattern_fill(handle.get_ptr(), n, seed_, stream_);
        }
        if (not rclcpp::ok())
        {
            return;  // SIGINT came: shutting down (a publish now fails), so that the pool's files go at the exit
        }
        try
        {
            pub_->publish(msg);
        }
        catch (const rclcpp::exceptions::RCLError& e)  // shut down meanwhile
        {
            if (rclcpp::ok())
            {
                throw;
            }
            return;
        }
        std::printf("CUDAPUB %u frame=%s backend=%s\n", ++count_, msg.header.frame_id.c_str(),
                    msg.data.get_backend_type().c_str());
        std::fflush(stdout);
        ++seed_;
    }

    int width_ = 0, height_ = 0;
    uint32_t seed_ = 1000, count_ = 0;
    cudaStream_t stream_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CudaPatternPub>());
    rclcpp::shutdown();
    return 0;
}
