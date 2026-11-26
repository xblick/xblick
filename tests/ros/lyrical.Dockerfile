# The image for tests/test_ros_lyrical.py: ROS 2 Lyrical with rmw_zenoh and the programs the test starts.
#   docker build -t xblick-test-lyrical -f tests/ros/lyrical.Dockerfile tests/ros
#   XBLICK_ROS_LYRICAL_IMAGE=xblick-test-lyrical uv run pytest tests/test_ros_lyrical.py
FROM ros:lyrical-ros-base
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-lyrical-rmw-zenoh-cpp ros-lyrical-image-tools \
    && rm -rf /var/lib/apt/lists/*
