# Prefiltered Point Cloud Octomap Updater

This plugin is based on
the [pointcloud_octomap_updater](https://github.com/ros-planning/moveit/tree/master/moveit_ros/perception/pointcloud_octomap_updater),
but without the self-filtering. Therefore, it is intended to be used with a pre-filtered point cloud.

An example of how to use this plugin in the sensor YAML file is shown below:

```yaml
sensors:
  - sensor_plugin: occupancy_map_monitor/PrefilteredPointCloudOctomapUpdater
    point_cloud_topic: /scan_matched_points2
    max_range: 5.0
    min_range: 0.0
    point_subsample: 1
    max_update_rate: 1.0