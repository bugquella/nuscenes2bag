#pragma once

#include "nuscenes2bag/Filesystem.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "nuscenes2bag/PclRadarObject.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


namespace nuscenes2bag {

boost::optional<sensor_msgs::PointCloud2> readRidarFilePointCloud(const fs::path& filePath);

}
