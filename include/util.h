#pragma once

uint32_t _float_as_int(float f);

float _int_as_float(uint32_t i);

void removeLidarDistortion(pcl::PointCloud<PointType>::Ptr& cloud,
    const Eigen::Matrix3d& dRlc, const Eigen::Vector3d& dtlc);

    