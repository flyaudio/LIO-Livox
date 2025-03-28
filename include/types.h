#pragma once
#include <pcl/point_types.h>

enum class MSG_TYPE : int
{
    CUSTOM = 0,
    POINTCLOUD2 = 1
};

enum class LIDAR_TYPE : int
{
    HORIZON = 0,
    HAP = 1,
    MID360 = 2
};

typedef pcl::PointXYZINormal PointType;