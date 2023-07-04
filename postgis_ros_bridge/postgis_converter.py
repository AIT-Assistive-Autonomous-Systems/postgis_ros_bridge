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

"""PostGIS converter module."""
from typing import Union

import shapely.geometry
from geometry_msgs.msg import (Point, Point32, Polygon, PolygonStamped, Pose,
                               PoseStamped, Quaternion)
from scipy.spatial.transform import Rotation
from shapely import wkb
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class PostGisConverter:
    """PostGIS converter class."""

    @staticmethod
    def load(geometry: Union[bytes, str], as_hex=True):
        """Load a shapely point from well known binary form."""
        return wkb.loads(geometry, hex=as_hex)

    @staticmethod
    def to_point(geometry: Union[bytes, str], as_hex=True) -> Point:
        """Convert a shapely point to a ROS point."""
        if not geometry:
            return Point(x=0.0, y=0.0, z=0.0)
        point = wkb.loads(geometry, hex=as_hex)
        return PostGisConverter.to_point_(point)

    @staticmethod
    def to_point_(point: shapely.geometry.Point) -> Point:
        """Convert a shapely point to a ROS point."""
        return Point(x=point.x, y=point.y, z=point.z if point.has_z else 0.0)

    @staticmethod
    def to_point_xyz(geometry: Union[bytes, str], as_hex=True):
        """Convert a shapely point to a xyz tuple."""
        point = wkb.loads(geometry, hex=as_hex)
        return (point.x, point.y, point.z if point.has_z else 0.0)

    @staticmethod
    def to_orientation(orientation: Union[bytes, str], as_hex=True) -> Quaternion:
        """Convert a orientation as well-known-binary to a ROS Quaternion."""
        # TODO: check quaternion order match with ROS
        if not orientation:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        orientation = wkb.loads(orientation, hex=as_hex)
        return PostGisConverter.to_orientation_(orientation)

    @staticmethod
    def to_orientation_(orientation: shapely.geometry.Point) -> Quaternion:
        """Convert a orientation as shapeley point to a ROS Quaternion."""
        if not orientation:
            return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        quat = Rotation.from_rotvec(
            [orientation.x, orientation.y, orientation.z]).as_quat()
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    @staticmethod
    def to_points_from_line_sring(geometry: shapely.geometry.LineString) -> list[Point]:
        """Convert a shapely LineString to a list of ROS points."""
        return [Point(x=point[0], y=point[1], z=point[2] if geometry.has_z else 0.0)
                for point in geometry.coords]

    @staticmethod
    def to_points_from_polygon(geometry: shapely.geometry.LineString) -> list[Point]:
        """Convert a shapely Polygon (as linestring) to a list of ROS points."""
        return [Point(x=point[0], y=point[1], z=point[2] if geometry.has_z else 0.0)
                for point in geometry.coords]

    @staticmethod
    def to_marker(header: Header, geometry: Union[bytes, str],
                  orientation: Union[bytes, str], as_hex, *args, **kwargs):
        """Convert a shapely geometry to a ROS marker."""
        geometry = PostGisConverter.load(geometry, as_hex=as_hex)
        geometry_orientation = PostGisConverter.load(
            orientation, as_hex=as_hex) if orientation else None

        marker = Marker(header=header, *args, **kwargs)

        if geometry.geom_type == "Point":
            marker.pose = PostGisConverter.to_pose_(
                geometry, geometry_orientation)
        elif geometry.geom_type == "LineString":
            marker.points = PostGisConverter.to_points_from_line_sring(
                geometry)
            marker.type = Marker.LINE_STRIP
        elif geometry.geom_type == "Polygon":
            # TODO untested
            marker.points = [Point(x=point[0], y=point[1], z=point[2] if geometry.has_z else 0.0)
                             for point in geometry.exterior.coords]
            marker.type = Marker.LINE_STRIP
        elif geometry.geom_type == "MultiPolygon":
            # TODO: output only first polygon of multipolygon
            geom = geometry.geoms[0]
            marker.points = [Point(x=point[0], y=point[1], z=point[2]
                                   if geometry.has_z else 0.0) for point in geom.exterior.coords]
            marker.type = Marker.LINE_STRIP
        elif geometry.geom_type == "MultiLineString":
            geom = geometry.geoms[0]
            marker.points = PostGisConverter.to_points_from_line_sring(geom)
            marker.type = Marker.LINE_STRIP
        else:
            raise ValueError(
                f"Unsupported geometry type: {geometry.geom_type}")
        return marker

    @staticmethod
    def to_pose(geometry: Union[bytes, str], orientation: Union[bytes, str], as_hex=True) -> Pose:
        """Convert a point as well-known-binary to a ROS Pose."""
        geometry_position = PostGisConverter.load(geometry, as_hex=as_hex)
        geometry_orientation = PostGisConverter.load(orientation, as_hex=as_hex)
        return PostGisConverter.to_pose_(point=geometry_position,
                                         orientation=geometry_orientation)

    @staticmethod
    def to_pose_(point: shapely.geometry.Point, orientation: shapely.geometry.Point) -> Pose:
        """Convert a point as shapely point to a ROS Pose."""
        return Pose(position=PostGisConverter.to_point_(point),
                    orientation=PostGisConverter.to_orientation_(orientation))

    @staticmethod
    def to_pose_stamped(header: Header, geometry: Union[bytes, str],
                        orientation: Union[bytes, str], as_hex=True) -> PoseStamped:
        """Convert a point as well-known-binary to a ROS PoseStamped."""
        pose = PostGisConverter.to_pose(geometry, orientation, as_hex=as_hex)
        return PoseStamped(header=header, pose=pose)

    @staticmethod
    def to_polygon(geometry: Union[bytes, str], as_hex=True) -> Polygon:
        """Convert a polygon as well-known-binary to a ROS Polygon."""
        polygon = wkb.loads(geometry, hex=as_hex)
        return Polygon(
            points=[Point32(x=point[0], y=point[1], z=point[2] if polygon.has_z else 0.0)
                    for point in polygon.boundary.coords])

    @staticmethod
    def to_polygon_stamped(header: Header,
                           geometry: Union[bytes, str], as_hex=True) -> PolygonStamped:
        """Convert a polygon as well-known-binary to a ROS PolygonStamped."""
        polygon = PostGisConverter.to_polygon(geometry, as_hex=as_hex)
        return PolygonStamped(header=header, polygon=polygon)
