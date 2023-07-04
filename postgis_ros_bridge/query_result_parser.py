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

"""Parser classes for converting query results to ROS messages."""
from abc import ABC, abstractmethod
from typing import Any, Dict, Iterable, Tuple

from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import (PointStamped, Polygon, PolygonStamped,
                               Pose, PoseStamped, Vector3, Quaternion)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sqlalchemy import Result, Row
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from postgis_ros_bridge.postgis_converter import PostGisConverter
from postgis_ros_bridge.geodesic_transform import GeodesicTransform


class QueryResultDefaultParameters:
    """Default parameters for query result parsers."""

    def __init__(self):
        self._rate = None
        self._frame_id = None
        self._geodesic = None

    def declare_params(self) -> Iterable[Tuple[str, Any]]:
        """Declare default parameters for query result parsers."""
        return [
            ('rate', 1.0, ParameterDescriptor()),
            ('frame_id', 'map', ParameterDescriptor()),
            ('geodesic', False, ParameterDescriptor()),
        ]

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Set default parameters for query result parsers."""
        self._rate = params['rate'].value
        self._frame_id = params['frame_id'].value
        self._geodesic = params['geodesic'].value

    @property
    def rate(self):
        """Get rate of query result parser."""
        return self._rate

    @property
    def frame_id(self):
        """Get frame id of query result parser."""
        return self._frame_id

    @property
    def geodesic(self):
        """Get utm transform enabled / disabled of query result parser."""
        return self._geodesic


class QueryResultParser(ABC):
    """Abstract Query Result Parser."""

    TYPE = "AbstractParser"

    @abstractmethod
    def declare_params(self,
                       defaults: QueryResultDefaultParameters) -> Iterable[
                           Tuple[str, Any, ParameterDescriptor]]:
        """Define API of declare_params."""
        return []

    @abstractmethod
    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Define API of set_params."""
        return []

    @abstractmethod
    def parse_result(self, result: Result, time: Time) -> Iterable[Tuple[str, Any]]:
        """Define API of parse_result."""
        return []

    def __repr__(self) -> str:
        return self.TYPE


class StampedTopicParser(QueryResultParser):
    """Base class for parsers which produce a single stamped message topic."""

    TYPE = None

    def __init__(self) -> None:
        """Initialize StampedTopicParser."""
        super().__init__()
        self.frame_id = None
        self.topic = None
        self.geodesic = None
        self.geodesic_transformer = None

    def declare_params(
            self,
            defaults: QueryResultDefaultParameters) -> Iterable[Tuple[str,
                                                                      Any,
                                                                      ParameterDescriptor]]:
        """Implement API of declare_params."""
        return [
            ('frame_id', defaults.frame_id, ParameterDescriptor()),
            ('topic', Parameter.Type.STRING, ParameterDescriptor()),
            ('geodesic', defaults.geodesic, ParameterDescriptor()),
        ]

    def set_geodesic_transformer(self, transformer: GeodesicTransform):
        """Set cartesian transfromer."""
        self.geodesic_transformer = transformer

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params."""
        self.frame_id = params['frame_id'].value
        self.topic = params['topic'].value
        self.geodesic = params['geodesic'].value

        return []

    def get_frame_id(self, elem: Row) -> str:
        """Get frame id of from query."""
        return elem.frame_id if hasattr(elem, 'frame_id') else self.frame_id


class SingleElementParser(StampedTopicParser):
    """Base class for parsers which produce a single stamped message topic."""

    TYPE = None

    @abstractmethod
    def parse_single_element(self, element: Row, time: Time) -> Tuple[str, Any]:
        """Implement API of parse_single_element."""
        return None

    def parse_result(self, result: Result, time: Time) -> Iterable[Tuple[str, Any]]:
        """Implement API of parse_result for single element parsers."""
        return (self.parse_single_element(element, time) for element in result)


class PointResultParser(SingleElementParser):
    """Parser for point results."""

    TYPE = "PointStamped"

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params for point results."""
        return super().set_params(params) + [(self.topic, PointStamped)]

    def parse_single_element(self, element: Row, time: Time) -> Tuple[str, Any]:
        """Implement API of parse_single_element for point results."""
        msg = PointStamped(header=Header(frame_id=self.get_frame_id(element), stamp=time),
                           point=PostGisConverter.to_point(element.geometry))

        if self.geodesic:
            msg.point = self.geodesic_transformer.transform_point(msg.point)

        return (self.topic, msg)

    def __repr__(self) -> str:
        return super().__repr__() + f" (using frame_id: {self.frame_id} and topic: {self.topic})"


class PoseResultParser(SingleElementParser):
    """Parser for pose results."""

    TYPE = "Pose"

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params for pose results."""
        return super().set_params(params) + [(self.topic, Pose)]

    def parse_single_element(self, element: Row, time: Time) -> Tuple[str, Any]:
        """Implement API of parse_single_element for pose results."""
        msg = PostGisConverter.to_pose(element.geometry, element.rotation)

        if self.geodesic_transformer:
            msg.position = self.geodesic_transformer.transform_point(msg.position)

        return (self.topic, msg)

    def __repr__(self) -> str:
        return super().__repr__() + f" (using topic: {self.topic})"


class PoseStampedResultParser(SingleElementParser):
    """Parser for pose stamped results."""

    TYPE = "PoseStamped"

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params for pose stamped results."""
        return super().set_params(params) + [(self.topic, PoseStamped)]

    def parse_single_element(self, element: Row, time: Time) -> Tuple[str, Any]:
        """Implement API of parse_single_element for pose stamped results."""
        msg = PostGisConverter.to_pose_stamped(geometry=element.geometry,
                                               orientation=element.rotation,
                                               header=Header(frame_id=self.get_frame_id(element),
                                                             stamp=time))
        if self.geodesic_transformer:
            # TODO: handle orientation # pylint: disable=fixme
            msg.pose.position = self.geodesic_transformer.transform_point(
                msg.pose.position)
        return (self.topic, msg)

    def __repr__(self) -> str:
        return super().__repr__() + f" (using frame_id: {self.frame_id} and topic: {self.topic})"


class PC2ResultParser(StampedTopicParser):
    """Parser for pointcloud2 results."""

    TYPE = "PointCloud2"

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params for pointcloud2 results."""
        return super().set_params(params) + [(self.topic,  PointCloud2)]

    def parse_result(self, result: Result, time: Time) -> Iterable[Tuple[str, Any]]:
        """Implement API of parse_result for pointcloud2 results."""
        pointcloud_msg = PointCloud2()
        header = Header(frame_id=self.frame_id, stamp=time)
        points = [
            PostGisConverter.to_point_xyz(element.geometry) for element in result.all()
        ]

        if self.geodesic_transformer:
            points = [self.geodesic_transformer.transform_tuple(
                point) for point in points]

        pointcloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        return [(self.topic, pointcloud_msg)]

    def __repr__(self) -> str:
        return super().__repr__() + f" (using frame_id: {self.frame_id} and topic: {self.topic})"


class PolygonResultParser(SingleElementParser):
    """Parser for polygon results."""

    TYPE = "Polygon"

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params for polygon results."""
        return super().set_params(params) + [(self.topic, Polygon)]

    def parse_single_element(self, element: Row, time: Time) -> Tuple[str, Any]:
        """Implement API of parse_single_element for polygon results."""
        msg = PostGisConverter.to_polygon(element.geometry)

        if self.geodesic_transformer:
            msg.points = [self.geodesic_transformer.transform_point32(
                point) for point in msg.points]

        return (self.topic, msg)

    def __repr__(self) -> str:
        return super().__repr__() + f" (using topic: {self.topic})"


class PolygonStampedResultParser(SingleElementParser):
    """Parser for polygon stamped results."""

    TYPE = "PolygonStamped"

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params for polygon stamped results."""
        return super().set_params(params) + [(self.topic, PolygonStamped)]

    def parse_single_element(self, element: Row, time: Time) -> Tuple[str, Any]:
        """Implement API of parse_single_element for polygon stamped results."""
        msg = PostGisConverter.to_polygon_stamped(
            geometry=element.geometry,
            header=Header(frame_id=self.get_frame_id(element),
                          stamp=time))

        if self.geodesic_transformer:
            msg.polygon.points = [self.geodesic_transformer.transform_point32(
                point) for point in msg.polygon.points]

        return (self.topic, msg)

    def __repr__(self) -> str:
        return super().__repr__() + f" (using topic: {self.topic})"


class MarkerResultParser(SingleElementParser):
    """Parser for marker results."""

    TYPE = "Marker"

    def __init__(self):
        super().__init__()
        self.marker_type = None
        self.marker_ns = None
        self.marker_color = None
        self.marker_scale = None
        self.marker_lifetime = None

    def declare_params(
            self,
            defaults: QueryResultDefaultParameters) -> Iterable[Tuple[str, Any,
                                                                      ParameterDescriptor]]:
        """Implement API of declare_params for marker results."""
        return super().declare_params(defaults) + [
            ('marker_type', Parameter.Type.STRING, ParameterDescriptor()),
            ('marker_ns', Parameter.Type.STRING, ParameterDescriptor()),
            ('marker_color', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor()),
            ('marker_scale', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor()),
            ('marker_lifetime', 3, ParameterDescriptor()),
            ('marker_mesh_resource', "", ParameterDescriptor()),
            ('marker_orientation', Parameter.Type.DOUBLE_ARRAY, ParameterDescriptor()),
        ]

    def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
        """Implement API of set_params for marker results."""
        topics = super().set_params(params)
        self.marker_type = params['marker_type'].value
        self.marker_ns = params['marker_ns'].value if params['marker_ns'].value else ''
        self.marker_color = params['marker_color'].value if params['marker_color'].value else [
            1.0, 0.0, 0.0, 1.0]
        self.marker_scale = params['marker_scale'].value if params['marker_scale'].value else [
            0.1, 0.1, 0.1]
        self.marker_lifetime = Duration(sec=params['marker_lifetime'].value)
        self.marker_mesh_resource = params['marker_mesh_resource'].value
        self.marker_orientation = params['marker_orientation'].value
        self.marker_orientation_override = True if self.marker_orientation else False

        return topics + [(self.topic, Marker)]

    def parse_single_element(self, element: Row, time: Time) -> Tuple[str, Any]:
        """Implement API of parse_single_element for marker results."""
        def get_id(elem):
            """Get id of element if it has one, otherwise return 0."""
            return elem.id if hasattr(elem, 'id') else 0

        marker_color = ColorRGBA(r=self.marker_color[0],
                                 g=self.marker_color[1],
                                 b=self.marker_color[2],
                                 a=self.marker_color[3])
        marker_types = {
            "visualization_msgs::Marker::ARROW": Marker.ARROW,
            "visualization_msgs::Marker::CUBE": Marker.CUBE,
            "visualization_msgs::Marker::SPHERE": Marker.SPHERE,
            "visualization_msgs::Marker::CYLINDER": Marker.CYLINDER,
            "visualization_msgs::Marker::MESH_RESOURCE": Marker.MESH_RESOURCE,
        }
        marker_type = marker_types.get(self.marker_type, Marker.ARROW)
        marker_scale = Vector3(
            x=self.marker_scale[0], y=self.marker_scale[1], z=self.marker_scale[2])

        msg = PostGisConverter.to_marker(header=Header(frame_id=self.get_frame_id(element),
                                                       stamp=time),
                                         as_hex=True,
                                         geometry=element.geometry,
                                         orientation=None,
                                         action=Marker.MODIFY,
                                         id=get_id(element),
                                         ns=self.marker_ns,
                                         type=marker_type,
                                         scale=marker_scale,
                                         color=marker_color,
                                         mesh_resource=self.marker_mesh_resource,
                                         lifetime=self.marker_lifetime)

        if self.marker_orientation_override:
            msg.pose.orientation = Quaternion(x=self.marker_orientation[0],
                                              y=self.marker_orientation[1],
                                              z=self.marker_orientation[2],
                                              w=self.marker_orientation[3])

        if self.geodesic_transformer:
            # todo handle orientation
            if msg.type in [Marker.LINE_STRIP, Marker.LINE_LIST, Marker.POINTS]:
                msg.points = [self.geodesic_transformer.transform_point(
                    point) for point in msg.points]
            else:
                msg.pose.position = self.geodesic_transformer.transform_point(
                    msg.pose.position)

        return (self.topic, msg)

    def __repr__(self) -> str:
        return super().__repr__() + f" (using frame_id: {self.frame_id} and topic: {self.topic})"


class BasicStampedArrayParserFactory:
    """Factory for basic stamped array parsers."""

    @staticmethod
    def create_array_parser(cls: SingleElementParser, msg: Any, field: str):
        """Create array parser for given message and field."""
        class ArrayParserMessage(cls):
            """Array parser for given message and field."""

            def __init__(self) -> None:
                super().__init__()
                self._has_header = hasattr(msg, 'header')

            def set_params(self, params: Dict[str, Parameter]) -> Iterable[Tuple[str, Any]]:
                """Implement API of set_params for array parser."""
                super().set_params(params)
                return [(self.topic, msg)]

            def parse_result(self, result: Result, time: Time) -> Iterable[Tuple[str, Any]]:
                """Implement API of parse_result for array parser."""
                # TODO: toplevel header (frame_id, timestamp can be different
                #       from internal elements).
                #       Add addition toplevel param and if not defined use header of first element.
                args = {}

                if self._has_header:
                    args['header'] = Header(
                        frame_id=self.get_frame_id(None), stamp=time)

                args.update({field: [self.parse_single_element(
                    element, time)[1] for element in result]})

                message = msg(**args)
                return [(self.topic, message)]

            def __repr__(self) -> str:
                return f"{super().TYPE}Array "\
                    f"(using frame_id: {self.frame_id} and topic: {self.topic})"

        return ArrayParserMessage
