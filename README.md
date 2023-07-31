<p align="center">
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/blob/main/LICENSE">
        <img alt="GitHub" src="https://img.shields.io/github/license/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge?label=License">
  </a>
    <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/actions/workflows/linting.yaml">
        <img alt="GitHub Workflow Status (with event)" src="https://img.shields.io/github/actions/workflow/status/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/linting.yaml?label=Linting">
  </a>
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/actions/workflows/ros_humble.yaml">
        <img alt="GitHub Workflow Status (with event)" src="https://img.shields.io/github/actions/workflow/status/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/ros_humble.yaml?label=ROS2%20Humble%20Tests">
  </a>
<a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/actions/workflows/ros_iron.yaml">
        <img alt="GitHub Workflow Status (with event)" src="https://img.shields.io/github/actions/workflow/status/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/ros_iron.yaml?label=ROS2%20Iron%20Tests">
  </a>
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/issues">
      <img alt="GitHub issues" src="https://img.shields.io/github/issues/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge?label=Issues">
  </a>
</p>

<p align="center">
  <a href="https://postgis.net">
        <img height="75" alt="PostGIS" src="https://upload.wikimedia.org/wikipedia/commons/7/7b/Logo_square_postgis.png">
  </a>
  <a href="https://ros.org">
        <img height="75" alt="PostGIS" src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg">
  </a>
</p>

# PostGIS ROS2 Bridge

ROS2 node connecting PostgreSQL databases containing spatial PostGIS data with the ROS world.

## Basic Usage

The publishing node can be started with

````bash
ros2 run postgis_ros_bridge postgis_ros_bridge_publisher_node --ros-args --params-file /PATH_TO/params.yaml
````
The `yaml` file is structured in two fixed sections (postgres and list of publisher), one optional default section, and n=len(list of publish) sections for each query.

The `postgresql`-section with username / password (either as plain text or give the name of an environmen variable holding the password), as well as the hostname, port and the schema to be used:
````yaml
postgresql:
        user: "postgres"
        pass_env: "POSTGRES_PASSWORD" # read password from environment variable (recommended)
        pass: "postgres" # Alternative to pass_env (store password in plaintext (not recommended))
        host: "localhost" 
        port: 5432
        schema: "example_schema"
````
This is followed by a list of query publishers, for the example in `cfg/example.yaml`:
````yaml
publish:
        - query_point
        - query_pose
        - query_pose_array
        - query_pose_stamped
        - query_polygon
        - query_polygon_stamped
        - query_marker
        - query_marker_array
        - query_pointcloud

````
For each of these list elements, there needs to be a section with parameters, depending on the type of desired message beeing published.

The publisher(s) are set up in the defined sections from above. Every section has at least a SQL query that contains a column `geometry`.
Depending on the targeted message type, there is also a special column `rotation`, `frame_id`, and `id`.
Parameters for all sections can be set in a `query_defaults` section, e.g.:
````yaml
query_defaults:
        rate: 10.0
        frame_id: "map"
````

## Geodesic Coordinates (UTM Transform)

For simple use of geodesic coordinates (latitude, longitude) we offer a basic functionality to directly transform  coordinates into a local cartesian coordinate frame (UTM).

All queries where parameter `geodesic=True` are transformed according to the transformation defined in following configuration block: 

```
cartesian_transform:  
    type: "utm"
    # utm_zone: 33
    # utm_band: "N"
    lon: 16.511422682732736
    lat: 47.977274686327114
    broadcast_cartesian_transform: true
    yaw_offset: 0.0 
    cartesian_frame_id: "utm"
    world_frame_id: "map"
```
 
* `type` specify type of cartesian coordinate. Supported: `[utm]`
* `utm_zone`/`utm_band` directly set utm zone and band
* `lat`/`lon` used to auto detect utm zone / band if not directly set 
* `broadcast_cartesian_transform`: broadcast the transformation between `[cartesian_frame_id]-->[world_frame_id]`
* `yaw_offset` manual rotational offset between cartesian and world frame
* `cartesian_frame_id` name of cartesian frame
* `world_frame_id` name of local map frame
* `inplace`: sets `lon`/`lat` as origin of our local map frame and directly transform points into this frame (broadcast must be disabled)
  

## Supported ROS2 Messages
* `geometry_msgs`
  * `PointStamped`
  * `Pose`
  * `PoseArray`
  * `PoseStamped`
  * `Polygon`
  * `PolygonStamped`
* `visualization_msgs`
  * `Marker`
* `sensor_msgs`
  * `PointCloud2`
* `foxglove_msgs`
  * `GeoJSON`

A simple example of to configure each type is listed in the following sections.


### geometry_msgs/msg/PointStamped [(ROS2 Reference)](https://docs.ros2.org/latest/api/geometry_msgs/msg/PointStamped.html)
````yaml
query_point:
        query: "SELECT position AS geometry, 'test_frame_id' AS frame_id FROM landmark;"
        type: "PointStamped" 
        topic: "point"
````
This is a simple example to query the `position` column from a table called `landmark` and rename it to the defined `geometry` keyword.
The frame id is set static for all points to `test_frame_id`. This valued could be fetched from the database as well using a more advanced query. The type is set using the `type` parameter, and the topic to publish the result as `topic`.
The parameters set in the query_defaults (`rate` and `frame_id`) could be set as all here to overwrite the defaults.

### geometry_msgs/msg/Pose [(ROS2 Reference)](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html)
````yaml
query_pose:
        query: "SELECT pose.position AS geometry, pose.rotation_vector AS rotation FROM pose;"
        type: "Pose"
        topic: "pose"
````
Query pose (x,y,z) in column `geometry` and rotation in column `rotation` as scaled euler rotation (see [scipy](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html)) in (roll, pitch, yaw). Each row of the query result is published as `Pose`.

### geometry_msgs/msg/PoseArray [(ROS2 Reference)](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseArray.html)
````yaml
query_pose_array:
        query: "SELECT pose.position AS geometry, pose.rotation_vector AS rotation FROM pose;"
        type: "PoseArray"
        topic: "pose_array"
        frame_id: "test_frame_id"
````
Query pose (x,y,z) in column `geometry` and rotation in column `rotation` as scaled euler rotation (see [scipy](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html)) in (roll, pitch, yaw). All poses of one query result are published as `PoseArray`.

### geometry_msgs/msg/PoseStamped [(ROS2 Reference)](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)
````yaml
query_pose_stamped:
        query: "SELECT pose.position AS geometry, pose.rotation_vector AS rotation, 'test_frame_id' AS frame_id FROM pose;"
        type: "PoseStamped"
        topic: "pose_stamped"
````
Query pose (x,y,z) in column `geometry` and rotation in column `rotation` as scaled euler rotation (see [scipy](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html)) in (roll, pitch, yaw). Each row of the query result is published as `PoseStamped` with frame id and timestamp.

### geometry_msgs/msg/Polygon [(ROS2 Reference)](https://docs.ros2.org/latest/api/geometry_msgs/msg/Polygon.html)
````yaml
query_polygon:
        query: "SELECT ST_MakePolygon( 'LINESTRING(0 0, 5 5, 7.7 10, -1.0 10.0, 0 0)') AS geometry, 'test' AS frame_id;"
        type: "Polygon"
        topic: "polygon"
````
Simple example of a 2D PostGIS polygon (could be fetched from a table too; 3D is also supported). The resulting geometry is published as `Polygon`.

### geometry_msgs/msg/PolygonStamped [(ROS2 Reference)](https://docs.ros2.org/latest/api/geometry_msgs/msg/PolygonStamped.html)
````yaml
query_polygon_stamped:
        query: "SELECT ST_MakePolygon( 'LINESTRING(0 0 0, 5 5 1, 7.7 10 1, -1.0 10.0 0, 0 0 0)') AS geometry, 'test' AS frame_id;"
        type: "PolygonStamped"
        topic: "polygon_stamped"
        frame_id: "test_frame_id"
````
Simple example of a 3D PostGIS polygon (could be fetched from a table too; 2D is also supported). The resulting geometry is published as `PolygonStamped` with frame id and timestamp.

### visualization_msgs/msg/Marker [(ROS2 Reference)](https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html)
````yaml
query_marker:
        query: "SELECT ROW_NUMBER() OVER (ORDER BY pose.id) AS id, pose.position AS geometry, pose.rotation_vector AS rotation, test_frame_id' AS frame_id FROM pose;"
        type: "Marker"
        marker_type: "visualization_msgs::Marker::SPHERE" # marker_type or here | default sphere
        topic: "marker"
````
Example of a marker message generation. As PostGIS does not support 6DoF poses, additionally to the `geometry` column containing the position (x,y,z) of the marker, a second column `rotation` is needed. This column is expected to hold the rotation as scaled euler rotation (see [scipy](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html)) in (roll, pitch, yaw). The `id` field of the ROS2 marker message can be filled for each row using the `id` column.
Implemented marker_types are `ARROW`, `CUBE`, `SPHERE`, and  `CYLINDER` (list of supported types can be extended easily in code, see `postgis_ros_bridge/query_result_parser.py`).

### visualization_msgs/msg/MarkerArray [(ROS2 Reference)](https://docs.ros2.org/galactic/api/visualization_msgs/msg/MarkerArray.html)
````yaml
query_marker_array:
        query: "SELECT ROW_NUMBER() OVER (ORDER BY pose.id) AS id, pose.position AS geometry, pose.rotation_vector AS rotation FROM pose;"
        type: "MarkerArray"
        marker_type: "visualization_msgs::Marker::SPHERE" 
        topic: "marker_array"
        frame_id: "test_frame_id"
````
For suitable message types, there is also the `Array` version implemented. For this example, the type is simply set to `MarkerArray`. In this example, the `frame_id` is set in the parameter section. It is also possible to define the `frame_id` in the query itself or as here, in the defaults. Query result overwrite the value set in the query section, and this parameter overwrites one in the defaults section, i.e. query result -> parameter section -> default value.

### sensor_msgs/msg/PointCloud2 [(ROS2 Reference)](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)
````yaml
query_pointcloud:
        query: "SELECT position AS geometry FROM landmark;"
        type: "PointCloud2"
        frame_id: "test_frame_id"
        topic: "pointcloud"
        rate: 1.0
````
Minimal query to fetch points to be published as pointcloud2. The `rate` parameter overwrites the default value.


### foxglove_msgs/msg/GeoJSON [(ROS2 Reference)](https://foxglove.dev/docs/studio/messages/geo-json)
````yaml
query_building_geojson:
        query: "SELECT json_build_object('type', 'FeatureCollection', 'features', json_agg(json_build_object('type', 'Feature', 'geometry', ST_AsGeoJSON(wkb_geometry)::json))) AS geojson FROM building"
        type: "GeoJSON" 
        topic: "geojson_building"
````
Construct a GeoJSON object via sql query and name it as `geojson`. This type can be visualized with the `Map`-Panel in `foxglove-studio`. 


# Funding
[DE] Die FFG ist die zentrale nationale Förderorganisation und stärkt Österreichs Innovationskraft. Dieses Projekt wird aus Mitteln der FFG gefördert. 

[EN] FFG is the central national funding organization and strengthens Austria's innovative power. This project is funded by the FFG. 

[www.ffg.at](www.ffg.at)

Projekt: [openSCHEMA](https://iktderzukunft.at/de/projekte/open-semantic-collaborative-hierarchical-environment-mapping.php)
