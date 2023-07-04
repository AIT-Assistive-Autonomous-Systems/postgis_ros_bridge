# Copyright 2023 AIT - Austrian Institute of Technology GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for PostGisConverter class."""
import pytest
from scipy.spatial.transform import Rotation
from shapely import wkt
from std_msgs.msg import Header

from postgis_ros_bridge.postgis_converter import PostGisConverter

wkb_point_to_point = {
    "POINT (10 20 30)": (10, 20, 30),
    "POINT (0 0 0)": (0, 0, 0),
    "POINT (1 2 3)": (1, 2, 3),
    "POINT (1.1 2.2 3.3)": (1.1, 2.2, 3.3),
    "POINT (1.1 2.2 3.3 4.4)": (1.1, 2.2, 3.3),
    "POINT (1.1 2.2)": (1.1, 2.2, 0),
    "POINT (-1.1 3.0 2.0)": (-1.1, 3.0, 2.0),
    "POINT (1.1 -3.0 2.0)": (1.1, -3.0, 2.0),
    "POINT (1.1 3.0 -2.0)": (1.1, 3.0, -2.0),
    "POINT (-1.1 -3.0 -2.0)": (-1.1, -3.0, -2.0),
}


def test_to_point():
    """Test conversion from WKB to ROS Point."""
    for wkb_point, point in wkb_point_to_point.items():
        ros_point = PostGisConverter.to_point(
            geometry=wkt.loads(wkb_point).wkb, as_hex=False)
        assert ros_point.x == pytest.approx(
            point[0]), f"check x for {wkb_point}"
        assert ros_point.y == pytest.approx(
            point[1]), f"check y for {wkb_point}"
        assert ros_point.z == pytest.approx(
            point[2]), f"check z for {wkb_point}"


def test_to_point_hex():
    """Test conversion from WKB to ROS Point."""
    for wkb_point, point in wkb_point_to_point.items():
        ros_point = PostGisConverter.to_point(
            geometry=wkt.loads(wkb_point).wkb_hex, as_hex=True)
        assert ros_point.x == pytest.approx(
            point[0]), f"check x for {wkb_point}"
        assert ros_point.y == pytest.approx(
            point[1]), f"check y for {wkb_point}"
        assert ros_point.z == pytest.approx(
            point[2]), f"check z for {wkb_point}"

    for wkb_point, point in wkb_point_to_point.items():
        ros_point = PostGisConverter.to_point(
            geometry=wkt.loads(wkb_point).wkb_hex)
        assert ros_point.x == pytest.approx(
            point[0]), f"check x for {wkb_point}"
        assert ros_point.y == pytest.approx(
            point[1]), f"check y for {wkb_point}"
        assert ros_point.z == pytest.approx(
            point[2]), f"check z for {wkb_point}"


def test_to_point_xyz():
    """Test conversion from WKB to tuple of x, y, z."""
    for wkb_point, point in wkb_point_to_point.items():
        point_tuple = PostGisConverter.to_point_xyz(
            geometry=wkt.loads(wkb_point).wkb, as_hex=False)
        assert point_tuple[0] == pytest.approx(
            point[0]), f"check x for {wkb_point}"
        assert point_tuple[1] == pytest.approx(
            point[1]), f"check y for {wkb_point}"
        if len(point_tuple) == 3:
            assert point_tuple[2] == pytest.approx(
                point[2]), f"check z for {wkb_point}"


def test_to_point_xyz_hex():
    """Test conversion from WKB to tuple of x, y, z."""
    for wkb_point, point in wkb_point_to_point.items():
        point_tuple = PostGisConverter.to_point_xyz(
            geometry=wkt.loads(wkb_point).wkb_hex, as_hex=True)
        assert point_tuple[0] == pytest.approx(
            point[0]), f"check x for {wkb_point}"
        assert point_tuple[1] == pytest.approx(
            point[1]), f"check y for {wkb_point}"
        if len(point_tuple) == 3:
            assert point_tuple[2] == pytest.approx(
                point[2]), f"check z for {wkb_point}"
    for wkb_point, point in wkb_point_to_point.items():
        point_tuple = PostGisConverter.to_point_xyz(
            geometry=wkt.loads(wkb_point).wkb_hex)
        assert point_tuple[0] == pytest.approx(
            point[0]), f"check x for {wkb_point}"
        assert point_tuple[1] == pytest.approx(
            point[1]), f"check y for {wkb_point}"
        if len(point_tuple) == 3:
            assert point_tuple[2] == pytest.approx(
                point[2]), f"check z for {wkb_point}"


wkb_point_to_orientation = {
    "POINT (0 0 0)": (Rotation.from_rotvec([0, 0, 0]).as_quat()),
    "POINT (0 0 0.1)": (Rotation.from_rotvec([0, 0, 0.1]).as_quat()),
    "POINT (0.1 0.2 0.3)": (Rotation.from_rotvec([0.1, 0.2, 0.3]).as_quat()),
    "POINT (-0.1 0.2 0.3)": (Rotation.from_rotvec([-0.1, 0.2, 0.3]).as_quat()),
}


def test_to_orientation():
    """Test conversion from WKB to ROS Orientation."""
    for wkb_point, orientation in wkb_point_to_orientation.items():
        ros_orientation = PostGisConverter.to_orientation(
            orientation=wkt.loads(wkb_point).wkb, as_hex=False)
        assert ros_orientation.x == pytest.approx(
            orientation[0]), f"check x for {wkb_point}"
        assert ros_orientation.y == pytest.approx(
            orientation[1]), f"check y for {wkb_point}"
        assert ros_orientation.z == pytest.approx(
            orientation[2]), f"check z for {wkb_point}"
        assert ros_orientation.w == pytest.approx(
            orientation[3]), f"check w for {wkb_point}"


def test_to_orientation_hex():
    """Test conversion from WKB to ROS Orientation."""
    for wkb_point, orientation in wkb_point_to_orientation.items():
        ros_orientation = PostGisConverter.to_orientation(
            orientation=wkt.loads(wkb_point).wkb_hex, as_hex=True)
        assert ros_orientation.x == pytest.approx(
            orientation[0]), f"check x for {wkb_point}"
        assert ros_orientation.y == pytest.approx(
            orientation[1]), f"check y for {wkb_point}"
        assert ros_orientation.z == pytest.approx(
            orientation[2]), f"check z for {wkb_point}"
        assert ros_orientation.w == pytest.approx(
            orientation[3]), f"check w for {wkb_point}"

    for wkb_point, orientation in wkb_point_to_orientation.items():
        ros_orientation = PostGisConverter.to_orientation(
            orientation=wkt.loads(wkb_point).wkb_hex)
        assert ros_orientation.x == pytest.approx(
            orientation[0]), f"check x for {wkb_point}"
        assert ros_orientation.y == pytest.approx(
            orientation[1]), f"check y for {wkb_point}"
        assert ros_orientation.z == pytest.approx(
            orientation[2]), f"check z for {wkb_point}"
        assert ros_orientation.w == pytest.approx(
            orientation[3]), f"check w for {wkb_point}"


wkb_pose_to_pose = {
    ("POINT (10 20 30)", "POINT (0 0 0)"):
        (10, 20, 30, *Rotation.from_rotvec([0, 0, 0]).as_quat()),
    ("POINT (10 20 30)", "POINT (0 0 0.1)"):
        (10, 20, 30, *Rotation.from_rotvec([0, 0, 0.1]).as_quat()),
    ("POINT (10 20 30)", "POINT (0 0 0.1 0.1)"):
        (10, 20, 30, *Rotation.from_rotvec([0, 0, 0.1]).as_quat()),
    ("POINT (0 0 0)", "POINT (0 0.3 0.1)"):
        (0, 0, 0, *Rotation.from_rotvec([0, 0.3, 0.1]).as_quat()),
    ("POINT (-10 0 0)", "POINT (0 0.3 0.1)"):
        (-10, 0, 0, *Rotation.from_rotvec([0, 0.3, 0.1]).as_quat()),
}


def test_to_pose():
    """Test conversion from WKB to ROS Pose."""
    for wkb_points, points in wkb_pose_to_pose.items():
        ros_pose = PostGisConverter.to_pose(geometry=wkt.loads(
            wkb_points[0]).wkb, orientation=wkt.loads(wkb_points[1]).wkb, as_hex=False)
        assert ros_pose.position.x == pytest.approx(
            points[0]), f"check x for {wkb_points}"
        assert ros_pose.position.y == pytest.approx(
            points[1]), f"check y for {wkb_points}"
        assert ros_pose.position.z == pytest.approx(
            points[2]), f"check z for {wkb_points}"
        assert ros_pose.orientation.x == pytest.approx(
            points[3]), f"check x for {wkb_points}"
        assert ros_pose.orientation.y == pytest.approx(
            points[4]), f"check y for {wkb_points}"
        assert ros_pose.orientation.z == pytest.approx(
            points[5]), f"check z for {wkb_points}"
        assert ros_pose.orientation.w == pytest.approx(
            points[6]), f"check w for {wkb_points}"


def test_to_pose_hex():
    """Test conversion from WKB to ROS Pose."""
    for wkb_points, points in wkb_pose_to_pose.items():
        ros_pose = PostGisConverter.to_pose(geometry=wkt.loads(
            wkb_points[0]).wkb_hex, orientation=wkt.loads(wkb_points[1]).wkb_hex, as_hex=True)
        assert ros_pose.position.x == pytest.approx(
            points[0]), f"check x for {wkb_points}"
        assert ros_pose.position.y == pytest.approx(
            points[1]), f"check y for {wkb_points}"
        assert ros_pose.position.z == pytest.approx(
            points[2]), f"check z for {wkb_points}"
        assert ros_pose.orientation.x == pytest.approx(
            points[3]), f"check x for {wkb_points}"
        assert ros_pose.orientation.y == pytest.approx(
            points[4]), f"check y for {wkb_points}"
        assert ros_pose.orientation.z == pytest.approx(
            points[5]), f"check z for {wkb_points}"
        assert ros_pose.orientation.w == pytest.approx(
            points[6]), f"check w for {wkb_points}"

    for wkb_points, points in wkb_pose_to_pose.items():
        ros_pose = PostGisConverter.to_pose(geometry=wkt.loads(
            wkb_points[0]).wkb_hex, orientation=wkt.loads(wkb_points[1]).wkb_hex)
        assert ros_pose.position.x == pytest.approx(
            points[0]), f"check x for {wkb_points}"
        assert ros_pose.position.y == pytest.approx(
            points[1]), f"check y for {wkb_points}"
        assert ros_pose.position.z == pytest.approx(
            points[2]), f"check z for {wkb_points}"
        assert ros_pose.orientation.x == pytest.approx(
            points[3]), f"check x for {wkb_points}"
        assert ros_pose.orientation.y == pytest.approx(
            points[4]), f"check y for {wkb_points}"
        assert ros_pose.orientation.z == pytest.approx(
            points[5]), f"check z for {wkb_points}"
        assert ros_pose.orientation.w == pytest.approx(
            points[6]), f"check w for {wkb_points}"


def test_to_pose_stamped():
    """Test conversion from WKB to ROS PoseStamped."""
    for wkb_points, points in wkb_pose_to_pose.items():
        ros_pose = PostGisConverter.to_pose_stamped(header=Header(), geometry=wkt.loads(
            wkb_points[0]).wkb, orientation=wkt.loads(wkb_points[1]).wkb, as_hex=False)
        assert ros_pose.pose.position.x == pytest.approx(
            points[0]), f"check x for {wkb_points}"
        assert ros_pose.pose.position.y == pytest.approx(
            points[1]), f"check y for {wkb_points}"
        assert ros_pose.pose.position.z == pytest.approx(
            points[2]), f"check z for {wkb_points}"
        assert ros_pose.pose.orientation.x == pytest.approx(
            points[3]), f"check x for {wkb_points}"
        assert ros_pose.pose.orientation.y == pytest.approx(
            points[4]), f"check y for {wkb_points}"
        assert ros_pose.pose.orientation.z == pytest.approx(
            points[5]), f"check z for {wkb_points}"
        assert ros_pose.pose.orientation.w == pytest.approx(
            points[6]), f"check w for {wkb_points}"


def test_to_pose_stamped_hex():
    """Test conversion from WKB to ROS PoseStamped."""
    for wkb_points, points in wkb_pose_to_pose.items():
        ros_pose = PostGisConverter.to_pose_stamped(header=Header(), geometry=wkt.loads(
            wkb_points[0]).wkb_hex, orientation=wkt.loads(wkb_points[1]).wkb_hex, as_hex=True)
        assert ros_pose.pose.position.x == pytest.approx(
            points[0]), f"check x for {wkb_points}"
        assert ros_pose.pose.position.y == pytest.approx(
            points[1]), f"check y for {wkb_points}"
        assert ros_pose.pose.position.z == pytest.approx(
            points[2]), f"check z for {wkb_points}"
        assert ros_pose.pose.orientation.x == pytest.approx(
            points[3]), f"check x for {wkb_points}"
        assert ros_pose.pose.orientation.y == pytest.approx(
            points[4]), f"check y for {wkb_points}"
        assert ros_pose.pose.orientation.z == pytest.approx(
            points[5]), f"check z for {wkb_points}"
        assert ros_pose.pose.orientation.w == pytest.approx(
            points[6]), f"check w for {wkb_points}"

    for wkb_points, points in wkb_pose_to_pose.items():
        ros_pose = PostGisConverter.to_pose_stamped(header=Header(), geometry=wkt.loads(
            wkb_points[0]).wkb_hex, orientation=wkt.loads(wkb_points[1]).wkb_hex)
        assert ros_pose.pose.position.x == pytest.approx(
            points[0]), f"check x for {wkb_points}"
        assert ros_pose.pose.position.y == pytest.approx(
            points[1]), f"check y for {wkb_points}"
        assert ros_pose.pose.position.z == pytest.approx(
            points[2]), f"check z for {wkb_points}"
        assert ros_pose.pose.orientation.x == pytest.approx(
            points[3]), f"check x for {wkb_points}"
        assert ros_pose.pose.orientation.y == pytest.approx(
            points[4]), f"check y for {wkb_points}"
        assert ros_pose.pose.orientation.z == pytest.approx(
            points[5]), f"check z for {wkb_points}"
        assert ros_pose.pose.orientation.w == pytest.approx(
            points[6]), f"check w for {wkb_points}"
