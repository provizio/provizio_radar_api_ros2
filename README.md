# provizio_radar_api_ros2

ROS 2 Driver for Provizio radars. Provides access to the full runtime APIs. Both [DDS-based](https://github.com/provizio/provizio_dds) and [UDP-based](https://github.com/provizio/provizio_radar_api_core) Provizio Radar APIs are supported.
It provides both regular and [managed (lifecycle)](https://design.ros2.org/articles/node_lifecycle.html) ROS 2 nodes.

## APIs Notes

The primary radar API in Provizio radars is [DDS API](https://github.com/provizio/provizio_dds) - it provides full functionality and allows for flexible configuration while being completely functional out-of-box in most environments. For stricter setups, there is also [UDP API](https://github.com/provizio/provizio_radar_api_core) that requires no dynamic memory allocation, multi-threading or using other "unsafe" language features. At the moment it only provides access to the most basic functionality of Provizio radars, but may be extended in the future.

## DDS API Notes

*Isn't applicable when built with `-DPROVIZIO_RADAR_API="udp"`*

[Provizio DDS API](https://github.com/provizio/provizio_dds) is built on top of [eProsima Fast-DDS](https://fast-dds.docs.eprosima.com/en/latest/) and is generally compatible with ROS 2 when it's running using its default [Fast-DDS RMW](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2.html). At the same time the Provizio DDS API itself doesn't respect ROS 2 QoS configurations, isn't compatible with other [ROS 2 RMWs](https://docs.ros.org/en/rolling/How-To-Guides/Working-with-multiple-RMW-implementations.html) and can't be used as a [managed (lifecycle) ROS 2 node](https://design.ros2.org/articles/node_lifecycle.html) due to not being based on ROS 2 in the first place.

This ROS 2 driver then can be leveraged to overcome all of these limitations. In order to do so, all the interaction with the underlying Provizio DDS API is isolated in this driver to a separate shared library (.so) which is [automatically loaded](provizio_radar_api_ros2_dds/src/provizio_dds_container.cpp) with its dependencies into a separate namespace of the node's process. It's required to avoid conflicts with ROS 2 version of eProsima Fast-DDS and works completely transparently for the node's user.

## Building and running the nodes

### Dependencies

- Linux (Ubuntu 20.04+ recommended, but other distributions are also supported)
- C++-17 compatible compiler
- CMake 3.14+
- ROS 2 Humble+

### Building steps

1. [Create a ROS 2 workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
2. In its `src` directory, clone `provizio_radar_api_ros2`.
3. [Source the ROS 2 environment](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html#source-ros-2-environment)
4. From the root of your ROS 2 workspace, build as (it may take a few minutes):

   ```Bash
   export PROVIZIO_RADAR_API="dds" # or "udp"
   export BUILD_SIMPLE_NODE="ON" # or "OFF"
   export BUILD_LIFECYCLE_NODE="ON" # or "OFF"
   colcon build --symlink-install --cmake-args -DPROVIZIO_RADAR_API=${PROVIZIO_RADAR_API} -DBUILD_SIMPLE_NODE=${BUILD_SIMPLE_NODE} -DBUILD_LIFECYCLE_NODE=${BUILD_LIFECYCLE_NODE} -DDISABLE_PROVIZIO_CODING_STANDARDS_CHECKS=ON -DBUILD_TESTING=OFF
   ```

### Running the built nodes

From the root of your ROS 2 workspace:

1. Source the built workspace, f.e. `source install/setup.bash`
2. Run the node:
   `ros2 run provizio_radar_api_ros2 provizio_radar_node [--ros-args -p <parameter>:=<value> ...]`

   or

   `ros2 run provizio_radar_api_ros2 provizio_radar_lifecycle_node [--ros-args -p <parameter>:=<value> ...]`

For more details on supported parameters, please see below.

## Provided Functionality and Config Parameters

### Radar point cloud

Unprocessed radar point clouds.

- Supported with `PROVIZIO_RADAR_API`: `dds`, `udp`.
- Message Type: [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#radar-point-cloud-fields).
- Enable parameter (bool): `publish_radar_pc`. Default: `true`.
- ROS 2 topic name parameter (string): `radar_pc_topic`. Default: `/provizio/radar_point_cloud`.
- [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#radar-point-cloud-fields).

### Super-resolution enhanced radar point cloud

Super-resolution (AI) enhanced radar point clouds.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Message Type: [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#radar-point-cloud-fields).
- Enable parameter (bool): `publish_radar_pc_sr`. Default: `true`.
- ROS 2 topic name parameter (string): `radar_pc_sr_topic`. Default: `/provizio/radar_point_cloud_sr`.

### Radar info

General radar info: timestamp, frame id, current radar range, supported ranges, serial_number (optionally).

- Supported with `PROVIZIO_RADAR_API`: `dds`, `udp`.
- Message Type: [provizio_radar_api_ros2/msg/RadarInfo](msg/RadarInfo.msg).
- Enable parameter (bool): `publish_radar_info`. Default: `true`.
- ROS 2 topic name parameter (string): `radar_info_topic`. Default: `/provizio/radar_info`.

### Set radar(s) range

Service to change the current radar range.

- Supported with `PROVIZIO_RADAR_API`: `dds`, `udp`.
- Service Type: [provizio_radar_api_ros2/srv/SetRadarRange.srv](srv/SetRadarRange.srv).
- Enable parameter (bool): `serve_set_radar_range`. Default: `true`.
- ROS 2 service name parameter (string): `set_radar_range_service`. Default: `/provizio/set_radar_range`.

### Radar-based odometry

Odometry based on radar data.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Message Type: [nav_msgs/msg/Odometry](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html).
- Enable parameter (bool): `publish_radar_odometry`. Default: `true`.
- ROS 2 topic name parameter (string): `radar_odometry_topic`. Default: `/provizio/odometry/radar`.

### Radar-based entities

Entities (objects) detection based on radar data.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Message Type: [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#entities-fields).
- Enable parameter (bool): `publish_entities_radar`. Default: `true`.
- ROS 2 topic name parameter (string): `entities_radar_topic`. Default: `/provizio/entities/radar`.

### Camera-based entities

Entities (objects) detection based on camera data.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Message Type: [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#entities-fields).
- Enable parameter (bool): `publish_entities_camera`. Default: `true`.
- ROS 2 topic name parameter (string): `entities_camera_topic`. Default: `/provizio/entities/camera`.

### Fusion-based entities

Entities (objects) detection based on radar+camera fusion.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Message Type: [sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#entities-fields).
- Enable parameter (bool): `publish_entities_fusion`. Default: `true`.
- ROS 2 topic name parameter (string): `entities_fusion_topic`. Default: `/provizio/entities/fusion`.

### Raw camera frames

Uncompressed camera frames.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Message Type: [sensor_msgs/msg/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html).
- Enable parameter (bool): `publish_camera`. Default: `true`.
- ROS 2 topic name parameter (string): `camera_topic`. Default: `/provizio/camera_raw`.

### Radar freespace polygons

Freespace, i.e. unoccupied or drivable area (polygonal) based on radar data. Other regions types support will be added in the future.

#### PolygonStamped version

Doesn't contain polygon "id" and can only be used for unoccupied regions.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Supports pre-Jazzy versions of ROS 2.
- Message Type: [geometry_msgs/msg/PolygonStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PolygonStamped.html). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#polygonal-freespaces).
- Enable parameter (bool): `publish_radar_freespace`. Default: `true`.
- ROS 2 topic name parameter (string): `radar_freespace_topic`. Default: `/provizio/radar_freespace/stamped`.

#### PolygonInstanceStamped version

Contains polygon "id" and can be used for unoccupied and other type of regions.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Supports Jazzy+ versions of ROS 2.
- Message Type: [geometry_msgs/msg/PolygonInstanceStamped](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PolygonInstanceStamped.msg). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#polygonal-freespaces).
- Enable parameter (bool): `publish_radar_freespace_instance`. Default: `true`.
- ROS 2 topic name parameter (string): `radar_freespace_instance_topic`. Default: `/provizio/radar_freespace/instance_stamped`.

### Camera freespace polygons

Freespace, i.e. unoccupied or drivable area (polygonal) based on camera data. Other regions types support will be added in the future.

#### PolygonStamped version

Doesn't contain polygon "id" and can only be used for unoccupied regions.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Supports pre-Jazzy versions of ROS 2.
- Message Type: [geometry_msgs/msg/PolygonStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PolygonStamped.html). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#polygonal-freespaces).
- Enable parameter (bool): `publish_camera_freespace`. Default: `true`.
- ROS 2 topic name parameter (string): `camera_freespace_topic`. Default: `/provizio/camera_freespace/stamped`.

#### PolygonInstanceStamped version

Contains polygon "id" and can be used for unoccupied and other type of regions.

- Supported with `PROVIZIO_RADAR_API`: `dds`.
- Supports Jazzy+ versions of ROS 2.
- Message Type: [geometry_msgs/msg/PolygonInstanceStamped](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/PolygonInstanceStamped.msg). [Additional details](https://github.com/provizio/provizio_dds_idls/blob/develop/TOPICS.md#polygonal-freespaces).
- Enable parameter (bool): `publish_camera_freespace_instance`. Default: `true`.
- ROS 2 topic name parameter (string): `camera_freespace_instance_topic`. Default: `/provizio/camera_freespace/instance_stamped`.

### General config parameters

#### Common

- `snr_threshold` (float dB). Default: `0`. Sets a filter to only publish radar points with signal-to-noise ratio >= specified value.

#### UDP API

- `max_radars` (int). Default: `6`. The maximum number of Provizio radars in the local network that can be handled by the node. The UDP API implementation needs all the required memory to be allocated in advance. Every radar's context takes over 3Mb of memory, so limiting the number of maximum radars in the setup significantly reduces memory consumption.
- `point_clouds_udp_port` (int16). Default: `7769`. The port number used to send radar point clouds over Provizio UDP protocol.
- `set_range_udp_port` (int16). Default: `7770`. The port number used to set current radar range over Provizio UDP protocol.

#### DDS API

- `provizio_dds_domain_id` (int). Default: `0`. Input [DDS Domain Id](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domain.html).
