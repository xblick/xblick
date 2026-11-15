"""Poses at a stamp (poses.py): interpolation, the buffer's rules (exact, between two, gap, never extrapolated), a
robot's frames in the world from <robot>/tf over Zenoh, the transform lookup over a camera's tf_static and tf, and a
fixed camera's pose (--pose)."""

import socket
import time

import numpy as np
import pytest

from xblick import topics
from xblick.camera import StaticTransformPublisher, TransformPublisher
from xblick.msg import Header, Quaternion, TFMessage, Time, Transform, TransformStamped, Vector3
from xblick.poses import (BODY_FROM_OPTICAL, CameraPoses, PoseBuffer, RobotPoses, TransformBuffer, TransformError,
                          fixed_camera_transforms, interpolate, matrix_of, pose_from, quaternion_from_rotation,
                          rotation_from_quaternion)
from xblick.robot import RobotStatePublisher
from xblick.session import open_session


def yaw(angle: float, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = [[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = t
    return m


def test_quaternions_round_trip():
    rng = np.random.default_rng(0)
    for q in rng.normal(size=(20, 4)):
        q /= np.linalg.norm(q)
        r = rotation_from_quaternion(*q)
        back = np.array(quaternion_from_rotation(r))
        assert np.allclose(back, q if q[3] >= 0 else -q)


def test_interpolate():
    assert np.allclose(interpolate(yaw(0.0), yaw(np.pi / 2, (1, 2, 0)), 0.5), yaw(np.pi / 4, (0.5, 1.0, 0.0)))
    # the shorter arc: from +170 deg to -170 deg passes 180
    assert np.allclose(interpolate(yaw(np.radians(170)), yaw(np.radians(-170)), 0.5), yaw(np.pi))
    stack = interpolate(np.stack([yaw(0.0), yaw(1.0)]), np.stack([yaw(1.0), yaw(1.0)]), 0.5)
    assert np.allclose(stack, np.stack([yaw(0.5), yaw(1.0)]))


def test_buffer():
    buffer = PoseBuffer(keep=10, max_gap=100e-9)
    buffer.add(1000, yaw(0.0))
    assert buffer.at(1000)[1] and np.allclose(buffer.at(1000)[0], yaw(0.0))
    assert buffer.at(1050) == (None, False)  # nothing after it yet: it may still come
    buffer.add(1100, yaw(1.0))
    value, final = buffer.at(1050)
    assert final and np.allclose(value, yaw(0.5))
    buffer.add(1300, yaw(1.0))
    assert buffer.at(1200) == (None, True)  # 200 ns apart: more than max_gap
    assert buffer.at(900) == (None, True)  # before the first: never extrapolated
    assert buffer.wait(2000, 0.05) is None  # waited for, nothing came


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def sessions():
    port = free_port()
    a = open_session(listen=[f"127.0.0.1:{port}"])
    b = open_session(connect=[f"127.0.0.1:{port}"])
    yield a, b
    b.close()
    a.close()


def stamped(parent, child, pose, stamp_ns):
    x, y, z, w = quaternion_from_rotation(pose[:3, :3])
    return TransformStamped(Header(Time.from_ns(stamp_ns), parent), child,
                            Transform(Vector3(*pose[:3, 3]), Quaternion(x, y, z, w)))


def test_robot_and_camera_poses_over_zenoh(sessions):
    a, b = sessions
    robot = RobotPoses(b, "robot")
    camera = CameraPoses(b, "robot/camera")
    time.sleep(0.3)  # the subscribers reach the other session
    state = RobotStatePublisher(a, "robot")
    camera_tf = TransformPublisher(a, "robot/camera")
    for stamp, angle in ((1_000_000_000, 0.0), (1_010_000_000, 0.2)):
        state.tf(TFMessage([stamped(topics.WORLD, topics.MAP, yaw(0.5, (2, 1, 0)), stamp),
                            stamped(topics.MAP, topics.ODOM_FRAME, np.eye(4), stamp),
                            stamped(topics.ODOM_FRAME, "root", yaw(angle), stamp),
                            stamped("root", "Neck", yaw(0.0, (0, 0, 1.6)), stamp)]))
        camera_tf.publish([stamped(topics.WORLD, "AzureKinectCamera", yaw(angle, (0, 0, 1.7)), stamp)])
    frames = robot.wait(1_005_000_000, 2.0)
    assert frames is not None and robot.frames == [topics.MAP, topics.ODOM_FRAME, "root", "Neck"]
    assert np.allclose(frames["Neck"], yaw(0.5, (2, 1, 0)) @ yaw(0.1) @ yaw(0.0, (0, 0, 1.6)))
    assert np.allclose(camera.wait(1_005_000_000, 2.0), yaw(0.1, (0, 0, 1.7)))
    assert camera.frame == "AzureKinectCamera"
    assert np.allclose(matrix_of(stamped("a", "b", yaw(0.3, (1, 2, 3)), 0)), yaw(0.3, (1, 2, 3)))
    for closable in (robot, camera, state):
        closable.close()


def test_pose_from_and_the_optical_frame():
    """--pose: ROS's roll-pitch-yaw of the body frame (x forward, z up); a positive pitch looks down. The optical
    frame (z forward, x right, y down) of REP 103."""
    assert np.allclose(pose_from(1, 2, 3, 90, 0, 0), yaw(np.pi / 2, (1, 2, 3)))
    forward = pose_from(0, 0, 1.5, 0, 10, 0)[:3, :3] @ [1.0, 0.0, 0.0]
    assert forward[2] == pytest.approx(-np.sin(np.radians(10)))  # looks down
    assert np.allclose(BODY_FROM_OPTICAL[:3, :3] @ np.eye(3), [[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    # roll, pitch and yaw compose as Rz @ Ry @ Rx
    r = pose_from(0, 0, 0, 30, 20, 10)[:3, :3]
    assert np.allclose(r, pose_from(0, 0, 0, 30, 0, 0)[:3, :3] @ pose_from(0, 0, 0, 0, 20, 0)[:3, :3]
                       @ pose_from(0, 0, 0, 0, 0, 10)[:3, :3])


def test_the_transform_buffer_merges_fixed_and_changing_transforms(sessions):
    """TransformBuffer: a moving camera (world -> head on tf, interpolated at the stamp) whose depth frame hangs off
    it on tf_static (valid at every stamp), composed; a fixed camera's chain world -> body -> optical on tf_static,
    published before the buffer started (asked for). Missing transforms fail with one line naming the frames, never
    an identity."""
    a, b = sessions
    static = StaticTransformPublisher(a, "fixed/camera", fixed_camera_transforms("webcam", pose_from(0, 0, 1.5, 0, 10, 0),
                                                                                 0))
    fixed = TransformBuffer(b, "fixed/camera")
    moving = TransformBuffer(b, "robot/camera")
    time.sleep(0.3)  # the subscribers reach the other session
    head_to_depth = yaw(0.0, (0.0, -0.03, 0.0))
    depth_static = StaticTransformPublisher(a, "robot/camera", [stamped("head", "depth", head_to_depth, 0)])
    tf = TransformPublisher(a, "robot/camera")
    for stamp, angle in ((1_000_000_000, 0.0), (1_010_000_000, 0.2)):
        tf.publish([stamped(topics.WORLD, "head", yaw(angle, (0, 0, 1.7)), stamp)])
    try:
        expected = pose_from(0, 0, 1.5, 0, 10, 0) @ BODY_FROM_OPTICAL
        for stamp in (0, 1_005_000_000, 4 * 10**18):  # fixed: at every stamp
            assert np.allclose(fixed.lookup(topics.WORLD, "webcam_optical", stamp, 2.0), expected)
        assert np.allclose(fixed.lookup("webcam", "webcam_optical", 5, 0.0), BODY_FROM_OPTICAL)
        assert np.allclose(moving.lookup(topics.WORLD, "depth", 1_005_000_000, 2.0),
                           yaw(0.1, (0, 0, 1.7)) @ head_to_depth)
        assert np.allclose(moving.lookup("depth", topics.WORLD, 1_000_000_000, 2.0),
                           np.linalg.inv(yaw(0.0, (0, 0, 1.7)) @ head_to_depth))
        with pytest.raises(TransformError, match=r"no transform from 'color' to 'world' on robot/camera/tf_static "
                                                 r"and robot/camera/tf: no transform names 'color'"):
            moving.lookup(topics.WORLD, "color", 1_005_000_000, 0.1)
        with pytest.raises(TransformError, match=r"robot/camera/tf: no pose of 'head' in 'world' at stamp 2000000000"
                                                 r": none at or after it yet \(never extrapolated\)"):
            moving.lookup(topics.WORLD, "depth", 2_000_000_000, 0.1)
        with pytest.raises(TransformError, match="before the first"):
            moving.lookup(topics.WORLD, "depth", 500_000_000, 1.0)
    finally:
        for closable in (fixed, moving, static, depth_static, tf):
            closable.close()


def test_test_camera_publishes_its_pose():
    """``xblick test-camera --pose``: the images in the optical frame of the body frame --frame-id, the chain on the
    camera's tf_static, answered to a buffer that starts later."""
    import subprocess
    import sys

    port = free_port()
    process = subprocess.Popen([sys.executable, "-m", "xblick.cli", "test-camera", "--namespace", "posed", "--size",
                                "64", "48", "--frame-id", "cam", "--pose", "1", "2", "1.5", "90", "10", "0",
                                "--listen", f"tcp/127.0.0.1:{port}"], stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    session = open_session(connect=[f"127.0.0.1:{port}"])
    transforms = None
    try:
        said = "pose on posed/camera/tf_static: world -> cam (1 2 1.5 90 10 0) -> cam_optical, the images' frame"
        lines = [process.stdout.readline() for _ in range(3)]
        assert any(said in line for line in lines), lines
        transforms = TransformBuffer(session, "posed/camera")
        assert np.allclose(transforms.lookup(topics.WORLD, "cam_optical", 123, 5.0),
                           pose_from(1, 2, 1.5, 90, 10, 0) @ BODY_FROM_OPTICAL)
    finally:
        if transforms is not None:
            transforms.close()
        session.close()
        process.terminate()
        process.communicate(timeout=10)
