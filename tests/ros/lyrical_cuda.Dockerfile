# The image for tests/test_ros_cuda_live.py: ROS 2 Lyrical with rmw_zenoh, the CUDA buffer backend (release) and a
# CUDA compiler for the test's subscriber (tests/ros/cuda_check, built by the test).
#   docker build -t xblick-test-lyrical-cuda -f tests/ros/lyrical_cuda.Dockerfile tests/ros
#   XBLICK_ROS_CUDA_IMAGE=xblick-test-lyrical-cuda uv run pytest tests/test_ros_cuda_live.py
FROM ros:lyrical-ros-base
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      ros-lyrical-cuda-buffer-backend ros-lyrical-cuda-buffer ros-lyrical-cuda-buffer-backend-msgs \
      ros-lyrical-rmw-zenoh-cpp ros-lyrical-sensor-msgs ros-lyrical-rclcpp ros-lyrical-ament-cmake \
      build-essential cmake python3-colcon-common-extensions nvidia-cuda-toolkit \
    && rm -rf /var/lib/apt/lists/*
# cuda_buffer pulls libnvidia-compute-<version> and its libcuda.so.1; the NVIDIA container runtime brings the host
# driver's at start, and a libcuda of another driver version in the image fails. Remove only those files.
RUN dpkg -L $(dpkg -l | awk '/libnvidia-compute-[0-9]+ /{print $2}') \
      | grep -E 'libcuda\.so|libnvidia-ptxjitcompiler|libnvidia-nvvm|libnvidia-gpucomp' | xargs -r rm -f && ldconfig
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
