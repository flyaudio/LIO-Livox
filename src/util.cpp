    /** \brief transform float to int
  */
uint32_t _float_as_int(float f) {
    union{uint32_t i; float f;} conv{};
    conv.f = f;
    return conv.i;
}

/** \brief transform int to float
*/
float _int_as_float(uint32_t i) {
    union{float f; uint32_t i;} conv{};
    conv.i = i;
    return conv.f;
}


/** \brief Remove Lidar Distortion
  * \param[in] cloud: = map_T_lidar * lidar-points
  * \param[in] dRlc: delta rotation = li_R_li+1
  * \param[in] dtlc: delta displacement = li_t_li+1
  */
void removeLidarDistortion(pcl::PointCloud<PointType>::Ptr& cloud,
    const Eigen::Matrix3d& dRlc, const Eigen::Vector3d& dtlc) {
    for (size_t i = 0; i < cloud->points.size(); i++) {
        float s = cloud->points[i].normal_x;
        Eigen::Quaterniond qlc = Eigen::Quaterniond(dRlc).normalized();
        Eigen::Quaterniond delta_qlc = Eigen::Quaterniond::Identity().slerp(s, qlc).normalized();
        const Eigen::Vector3d delta_Plc = s * dtlc;
        // Eigen::Vector3d startP;
        Eigen::Vector3d startP = delta_qlc * Eigen::Vector3d(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z) + delta_Plc;
        Eigen::Vector3d _po = dRlc.transpose() * (startP - dtlc);

        cloud->points[i].x = _po(0);
        cloud->points[i].y = _po(1);
        cloud->points[i].z = _po(2);
        cloud->points[i].normal_x = 1.0;
    }
}

