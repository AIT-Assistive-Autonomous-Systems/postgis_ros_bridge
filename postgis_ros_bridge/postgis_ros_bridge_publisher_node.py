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

"""PostGIS ROS Bridge Publisher Node."""
from functools import partial
from typing import Dict, Tuple
import math

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from geometry_msgs.msg import PoseArray, TransformStamped
from visualization_msgs.msg import MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from postgis_ros_bridge.postgresql_connection import PostgreSQLConnection
from postgis_ros_bridge.query import Query
from postgis_ros_bridge.query_result_parser import (
    BasicStampedArrayParserFactory, MarkerResultParser, PC2ResultParser,
    PointResultParser, PolygonResultParser, PolygonStampedResultParser,
    PoseResultParser, PoseStampedResultParser, QueryResultDefaultParameters,
    QueryResultParser, GeoJSONResultParser)

from postgis_ros_bridge.geodesic_transform import GeodesicTransform


query_parser: Dict[str, QueryResultParser] = {
    q.TYPE: q for q in [
        PointResultParser,
        PoseResultParser,
        PoseStampedResultParser,
        PC2ResultParser,
        MarkerResultParser,
        PolygonResultParser,
        PolygonStampedResultParser,
        GeoJSONResultParser
    ]}

query_parser.update({
    "MarkerArray":
        BasicStampedArrayParserFactory.create_array_parser(
            MarkerResultParser, MarkerArray, "markers"),
    "PoseArray":
        BasicStampedArrayParserFactory.create_array_parser(
            PoseResultParser, PoseArray, "poses"),
})


def euler_to_quaternion(roll: float,
                        pitch: float,
                        yaw: float) -> Tuple[float, float, float, float]:
    """Convert euler angles (roll-pitch-yaw) in radians to quaternion (x,y,z,w)."""
    cos_y = math.cos(yaw * 0.5)
    sin_y = math.sin(yaw * 0.5)
    cos_p = math.cos(pitch * 0.5)
    sin_p = math.sin(pitch * 0.5)
    cos_r = math.cos(roll * 0.5)
    sin_r = math.sin(roll * 0.5)

    quat_w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y
    quat_x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
    quat_y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
    quat_z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y

    return (quat_x, quat_y, quat_z, quat_w)


class PostGisPublisher(Node):
    """PostGIS ROS Bridge Publisher Node."""

    def __init__(self):
        super().__init__(node_name="postgis_ros_publisher")
        self.get_logger().info(f"Starting {self.get_name()}...")

        # automatically_declare_parameters_from_overrides=True
        self.declare_parameters(
            namespace="",
            parameters=[
                ("publish", rclpy.Parameter.Type.STRING_ARRAY),
            ],
        )

        configurations = self.get_parameter("publish").value
        self.get_logger().info(f"Parsing sections: {configurations}")
        self.converter_pubs: Dict[str, Publisher] = dict()
        self.postgresql_connection = PostgreSQLConnection(self)
        self.get_logger().info(
            f"Connected to database via {self.postgresql_connection}")

        # get common default settings
        default_section_name = "query_defaults"
        default_parameters = QueryResultDefaultParameters()
        self.declare_parameters(
            namespace="",
            parameters=list(map(lambda x: (f"{default_section_name}.{x[0]}", x[1], x[2]),
                                default_parameters.declare_params())))
        default_parameters.set_params(
            self.get_parameters_by_prefix(default_section_name))

        self.geodesic_transformer = None
        self.init_geodesic_transformer()

        for config in configurations:
            self.declare_parameters(
                namespace="",
                parameters=[
                    (f"{config}.query", rclpy.Parameter.Type.STRING),
                    (f"{config}.rate", default_parameters.rate),
                    (f"{config}.type", rclpy.Parameter.Type.STRING),
                ],
            )
            query_type = self.get_parameter(f"{config}.type").value
            sql_query = self.get_parameter(f"{config}.query").value
            rate = self.get_parameter(f"{config}.rate").value

            if query_type not in query_parser.keys():
                raise ValueError(
                    f"Type: '{query_type}' is not supported. Supported: {query_parser.keys()}"
                )

            parser = query_parser[query_type]()
            # TODO: namespace bug in rclpy Node declare
            # params name initialized after value query [fixme]
            self.declare_parameters(
                namespace="", parameters=list(
                    map(lambda x: (f"{config}.{x[0]}", x[1], x[2]),
                        parser.declare_params(default_parameters))))
            topics_msgs = parser.set_params(
                self.get_parameters_by_prefix(config))

            if parser.geodesic:
                if self.geodesic_transformer is None:
                    raise ValueError(
                        "Geodesic transform is enabled but no cartesian transform is set")
                parser.set_geodesic_transformer(self.geodesic_transformer)

            query = Query(self.postgresql_connection, sql_query)

            # TODO: probably sensor data qos or later configurable [fixme]
            pubs = [(t, self.create_publisher(m, t, 10))
                    for (t, m) in topics_msgs]
            self.converter_pubs.update(pubs)

            self.get_logger().info(f"Register parser: {str(parser)}")

            if rate > 0:
                self.create_timer(
                    1.0/rate, partial(self.timer_callback, query, parser))
            else:
                self.timer_callback(query, parser)

    def init_geodesic_transformer(self):
        """Initialize cartesian transformation."""
        config = "cartesian_transform"
        self.declare_parameters(
            namespace="",
            parameters=[
                (f"{config}.type", ""),
                (f"{config}.utm_zone", -1),
                (f"{config}.utm_band", ""),
                (f"{config}.lon", -1000.0),
                (f"{config}.lat", -1000.0),
                (f"{config}.yaw_offset", 0.0),
                (f"{config}.broadcast_cartesian_transform", False),
                (f"{config}.world_frame_id", "map"),
                (f"{config}.cartesian_frame_id", "utm"),
                (f"{config}.inplace", False),
            ],
        )

        self.geodesic_transformer = None

        cartesian_type = self.get_parameter(f"{config}.type").value
        # if no cartesian transform type is set, disable cartesian transform
        if cartesian_type == "":
            return

        utm_zone = self.get_parameter(f"{config}.utm_zone").value
        utm_band = self.get_parameter(f"{config}.utm_band").value
        lon = self.get_parameter(f"{config}.lon").value
        lat = self.get_parameter(f"{config}.lat").value
        broadcast = self.get_parameter(f"{config}.broadcast_cartesian_transform").value
        yaw_offset = self.get_parameter(f"{config}.yaw_offset").value
        world_frame_id = self.get_parameter(f"{config}.world_frame_id").value
        cartesian_frame_id = self.get_parameter(f"{config}.cartesian_frame_id").value
        inplace = self.get_parameter(f"{config}.inplace").value

        if (inplace or broadcast) and (lon == -1000.0 or lat == -1000.0):
            raise ValueError("lon and lat must be set if inplace or broadcast is True")

        if cartesian_type == "utm":
            if utm_zone > -1:
                if utm_band == "":
                    raise ValueError("utm_band is not set")

                self.geodesic_transformer = GeodesicTransform.to_utm(
                    utm_zone, utm_band, origin_transform=inplace, origin_lon=lon, origin_lat=lat)
            else:

                if lon == -1000.0 or lat == -1000.0:
                    raise ValueError("lon and lat must be set if no utm_zone/utm_band is set")

                self.geodesic_transformer = GeodesicTransform.to_utm_lonlat(
                    lon, lat, origin_transform=inplace)

        else:
            raise ValueError(
                f"Type: '{cartesian_type}' is not supported. Supported: utm"
            )

        self.get_logger().info(f"Initialized cartesian transform: \
                               {str(self.geodesic_transformer)}")

        if broadcast:
            if inplace:
                raise ValueError("inplace and broadcast can not be both True")

            self.tf_static_broadcaster = StaticTransformBroadcaster(self)

            tf_msg = TransformStamped()

            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = cartesian_frame_id
            tf_msg.child_frame_id = world_frame_id

            easting, northing = self.geodesic_transformer.transform_lonlat(lon, lat)
            tf_msg.transform.translation.x = easting
            tf_msg.transform.translation.y = northing
            tf_msg.transform.translation.z = 0.0

            meridian_convergence = self.geodesic_transformer.meridian_convergence(lon, lat)
            yaw = meridian_convergence + yaw_offset
            quat = euler_to_quaternion(0.0, 0.0, math.radians(yaw))

            tf_msg.transform.rotation.x = quat[0]
            tf_msg.transform.rotation.y = quat[1]
            tf_msg.transform.rotation.z = quat[2]
            tf_msg.transform.rotation.w = quat[3]

            self.get_logger().info(f"Broadcasting cartesian transform\n\
                                    [{tf_msg.header.frame_id}]->[{tf_msg.child_frame_id}]\n\
                                    translation: x={tf_msg.transform.translation.x}\n\
                                                 y={tf_msg.transform.translation.y}\n\
                                                 z={tf_msg.transform.translation.z}\n\
                                    meridian_convergence: {meridian_convergence} deg\n\
                                    yaw_offset: {yaw_offset}  deg")

            self.tf_static_broadcaster.sendTransform(tf_msg)

    def timer_callback(self, query: Query, converter: QueryResultParser):
        """Timer callback for all queries."""
        with query.get_results() as results:
            for topic, message in converter.parse_result(results, self.get_clock().now().to_msg()):
                self.converter_pubs[topic].publish(message)


def main(args=None):
    """ROS main function."""
    rclpy.init(args=args)
    postgis_publisher = PostGisPublisher()
    rclpy.spin(postgis_publisher)

    postgis_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
