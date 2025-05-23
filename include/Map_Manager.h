#pragma once
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <future>
#include "types.h"

extern int g_gridNumX;// 子cube沿宽、高、深度分割个数
extern int g_gridNumY;
extern int g_gridNumZ;
extern int g_gridNum ;//cube的总数量:4851


template <typename PointType>
bool pntToCubeIdx(const PointType& pi, 
    const int& cenDepth, const int& cenWidth, const int& cenHeight,
    std::size_t& cubeIdx) 
{
    int cubeI = int((pi.x + 25.0) / 50.0) + cenDepth;
    int cubeJ = int((pi.y + 25.0) / 50.0) + cenWidth;
    int cubeK = int((pi.z + 25.0) / 50.0) + cenHeight;

    if (pi.x + 25.0 < 0) cubeI--;
    if (pi.y + 25.0 < 0) cubeJ--;
    if (pi.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < g_gridNumX &&
        cubeJ >= 0 && cubeJ < g_gridNumY &&
        cubeK >= 0 && cubeK < g_gridNumZ) {
        cubeIdx = ToIndex(cubeI, cubeJ, cubeK);
        return true;
    }
    return false;
}


template <typename Derived>
bool pntToCubeIdx(const Eigen::Matrix<typename Derived::Scalar,3,1>& pi,
    const int& cenDepth, const int& cenWidth, const int& cenHeight,
    std::size_t& cubeI, std::size_t& cubeJ, std::size_t& cubeK)
{
    int cubeI = int((pi.x() + 25.0) / 50.0) + cenDepth;
    int cubeJ = int((pi.y() + 25.0) / 50.0) + cenWidth;
    int cubeK = int((pi.z() + 25.0) / 50.0) + cenHeight;

    if (pi.x() + 25.0 < 0) cubeI--;
    if (pi.y() + 25.0 < 0) cubeJ--;
    if (pi.z() + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < g_gridNumX &&
        cubeJ >= 0 && cubeJ < g_gridNumY &&
        cubeK >= 0 && cubeK < g_gridNumZ) {
        return true;
    }
    return false;
}



/** \brief transform point pi to the MAP coordinate
 */
template <typename PointType>
void 
pointAssociateToMap(const PointType& pi,
					PointType& po,
					const Eigen::Matrix4d& _transformTobeMapped) {
	po->getVector3fMap() = _transformTobeMapped.topLeftCorner(3,3) * pi->getVector3fMap() + _transformTobeMapped.topRightCorner(3,1);
	po->intensity = pi->intensity;
	po->normal_z = pi->normal_z;
}


/**
 * @deprecated
 */
template <typename PointType>
void 
featureAssociateToMap(const typename pcl::PointCloud<PointType>::ConstPtr& corners,
					const typename pcl::PointCloud<PointType>::ConstPtr& surf,
					const typename pcl::PointCloud<PointType>::ConstPtr& nonFeatures,

					typename pcl::PointCloud<PointType>::Ptr& cornersToMap,
					typename pcl::PointCloud<PointType>::Ptr& surfToMap,
					typename pcl::PointCloud<PointType>::Ptr& nonFeaturesToMap,
					const Eigen::Matrix4d& transformTobeMapped){

	PointType pointSel1; //,pointSel2,pointSel3;
	for (const auto& p : corners->points) {
		pointAssociateToMap(p, pointSel1, transformTobeMapped);
		cornersToMap->push_back(pointSel1);
	}
	for (const auto& p : surf->points) {
		pointAssociateToMap(p, pointSel1, transformTobeMapped);
		surfToMap->push_back(pointSel1);
	}
	for (const auto& p : nonFeatures->points) {
		pointAssociateToMap(p, pointSel1, transformTobeMapped);
		nonFeaturesToMap->push_back(pointSel1);
	}
}


template <typename PointType>
void 
featureAssociateToMap(const pcl::PointCloud<PointType>& corners,
					const pcl::PointCloud<PointType>& surf,
					const pcl::PointCloud<PointType>& nonFeatures,

					pcl::PointCloud<PointType>& cornersToMap,
					pcl::PointCloud<PointType>& surfToMap,
					pcl::PointCloud<PointType>& nonFeaturesToMap,
					Eigen::Matrix4f& map_T_lidar) {

    pcl::transformPointCloud(corners, cornersToMap, map_T_lidar);
    pcl::transformPointCloud(surf, surfToMap, map_T_lidar);
    pcl::transformPointCloud(nonFeatures, nonFeaturesToMap, map_T_lidar);
}


template <typename PointType>
class Feature {
public:
    constexpr std::size_t localWindowSize = 60;
    pcl::PointCloud<PointType>::Ptr grid_[g_gridNum];//存放cube 角特征的数组
    pcl::PointCloud<PointType>::Ptr gridPre_[g_gridNum];
    pcl::PointCloud<PointType>::Ptr window_[localWindowSize];
    pcl::PointCloud<PointType>::Ptr allGrid_;//accumulate all grids
    pcl::VoxelGrid<PointType> downFilter_;
    pcl::KdTreeFLANN<PointType>::Ptr trees_[g_gridNum];
    pcl::KdTreeFLANN<PointType>::Ptr treesPre_[g_gridNum];
}


template <typename PointType>
class MAP_MANAGER{
public:

    std::mutex mtx_MapManager;
    /** \brief constructor of MAP_MANAGER */
    MAP_MANAGER(const float& filter_corner, const float& filter_surf);

    static size_t ToIndex(int i, int j, int k);

    /** \brief add new lidar points to the map
     * \param[in] corners: coner features that need to be added to map
     * \param[in] surfs: surf features that need to be added to map
     * \param[in] map_T_lidar: transform matrix of the lidar pose
     */
    void MapIncrement(const pcl::PointCloud<PointType>::Ptr& corners,
                      const pcl::PointCloud<PointType>::Ptr& surfs,
                      const pcl::PointCloud<PointType>::Ptr& nonFeatures,
                      const Eigen::Matrix4d& map_T_lidar);

    /** \brief retrieve map points according to the lidar pose
     * \param[in] laserCloudCornerFromMap: store coner feature points retrieved from map
     * \param[in] laserCloudSurfFromMap: tore surf feature points retrieved from map
     * \param[in] map_T_lidar: transform matrix of the lidar pose
     */
    void MapMove(const Eigen::Matrix4d& map_T_lidar);


    size_t FindUsedCornerMap(const PointType& p,int a,int b,int c);//三个函数是一样的
    size_t FindUsedSurfMap(const PointType& p,int a,int b,int c);
    size_t FindUsedNonFeatureMap(const PointType& p,int a,int b,int c);

    /**
     * getter
     */
    pcl::KdTreeFLANN<PointType> getCornerKdMap(int i) const {
        return CornerKdMap_last[i];
    }

    pcl::KdTreeFLANN<PointType> getSurfKdMap(int i) const {
        return SurfKdMap_last[i];
    }

    pcl::KdTreeFLANN<PointType> getNonFeatureKdMap(int i) const {
        return NonFeatureKdMap_last[i];
    }

    pcl::PointCloud<PointType>::Ptr get_corner_map() const {
        return laserCloudCornerFromMap;
    }
    pcl::PointCloud<PointType>::Ptr get_surf_map() const {
        return laserCloudSurfFromMap;
    }
    pcl::PointCloud<PointType>::Ptr get_nonfeature_map() const {
        return laserCloudNonFeatureFromMap;
    }

    // int get_map_current_pos() const {//NOT in use
    //     return currentUpdatePos;
    // }

    int get_laserCloudCenWidth_last() const {
      return laserCloudCenWidth_last;
    }

    int get_laserCloudCenHeight_last() const {
      return laserCloudCenHeight_last;
    }

    int get_laserCloudCenDepth_last() const {
      return laserCloudCenDepth_last;
    }

    pcl::PointCloud<PointType> laserCloudSurf_for_match[g_gridNum];
    pcl::PointCloud<PointType> laserCloudCorner_for_match[g_gridNum];
    pcl::PointCloud<PointType> laserCloudNonFeature_for_match[g_gridNum];

private:
    int laserCloudCenWidth = 10;// cube宽、高、深度
    int laserCloudCenHeight = 5;
    int laserCloudCenDepth = 10;

    int laserCloudCenWidth_last = 10;
    int laserCloudCenHeight_last = 5;
    int laserCloudCenDepth_last = 10;

    static const int laserCloudWidth = 21;// 子cube沿宽、高、深度分割个数
    static const int laserCloudHeight = 11;
    static const int laserCloudDepth = 21;
    static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;//cube的总数量:4851
    pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];//存放cube 角特征的数组
    pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];  //存放cube 面特征的数组
    pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureArray[laserCloudNum];
    
    // pcl::PointCloud<PointType>::Ptr laserCloudCornerArrayStack[laserCloudNum];
    // pcl::PointCloud<PointType>::Ptr laserCloudSurfArrayStack[laserCloudNum];
    // pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureArrayStack[laserCloudNum];

    pcl::VoxelGrid<PointType> downSizeFilterCorner;//
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterNonFeature;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;//accumulate all grids
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudNonFeatureFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr laserCloudCornerKdMap[laserCloudNum];//cornorTrees
    pcl::KdTreeFLANN<PointType>::Ptr laserCloudSurfKdMap[laserCloudNum];//surfTrees
    pcl::KdTreeFLANN<PointType>::Ptr laserCloudNonFeatureKdMap[laserCloudNum];//NonFeatTrees

    // pcl::KdTreeFLANN<PointType> CornerKdMap_copy[laserCloudNum];//NOT in use
    // pcl::KdTreeFLANN<PointType> SurfKdMap_copy[laserCloudNum];//NOT in use
    // pcl::KdTreeFLANN<PointType> NonFeatureKdMap_copy[laserCloudNum];//NOT in use

    pcl::KdTreeFLANN<PointType> CornerKdMap_last[laserCloudNum];
    pcl::KdTreeFLANN<PointType> SurfKdMap_last[laserCloudNum];
    pcl::KdTreeFLANN<PointType> NonFeatureKdMap_last[laserCloudNum];

    static const int localMapWindowSize = 60;
    pcl::PointCloud<PointType>::Ptr localCornerMap[localMapWindowSize];
    pcl::PointCloud<PointType>::Ptr localSurfMap[localMapWindowSize];
    pcl::PointCloud<PointType>::Ptr localNonFeatureMap[localMapWindowSize];

    // int localMapID = 0;
    // int currentUpdatePos = 0;
    // int estimatorPos = 0;
};

