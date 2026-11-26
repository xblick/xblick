# cuda_check for tests/test_ros_cuda_live.py, built by the test inside the image (colcon).
cmake_minimum_required(VERSION 3.20)
project(xblick_cuda_check LANGUAGES CXX CUDA)

# PTX for compute 90, compiled by the driver for newer GPUs (an older nvcc, e.g. Ubuntu's 12.4, cannot target them)
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 90-virtual)
endif()
set(CMAKE_CXX_STANDARD 17)
# nvcc may use an older g++ than the one that links: keep its libstdc++ out of the link
set(CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES "")
set(CMAKE_CUDA_IMPLICIT_LINK_LIBRARIES "")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_library(pattern STATIC src/pattern.cu)
target_link_libraries(pattern PUBLIC CUDA::cudart)
add_executable(cuda_check src/cuda_check.cpp)
target_link_libraries(cuda_check pattern rclcpp::rclcpp ${sensor_msgs_TARGETS} cuda_buffer::cuda_buffer CUDA::cudart
                      CUDA::cuda_driver)  # cuda_buffer's headers call the driver API
install(TARGETS cuda_check DESTINATION lib/${PROJECT_NAME})
ament_package()
