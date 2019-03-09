#ifndef MBCUDA_VOXEL_GRID_H_
#define MBCUDA_VOXEL_GRID_H_

#include "mb_cuda/common/point_types.h"
#include "mb_cuda/common/thrust.h"

namespace mb_cuda {

  typedef mb_cuda::PointXYZRGB pointT;
  typedef thrust::device_vector<pointT> deviceCloudT;
  typedef thrust::host_vector<pointT> hostCloudT;

  /**
   * @brief filter--using voxel grid to downsample point cloud
   * @param [in]input--input cloud with device_vector<mb_cuda::PointXYZRGB> type
   * @param [out]output--output cloud with device_vector<mb_cuda::PointXYZRGB> type
   * @param [in]leafsize--the resolution of voxel grids
   */
  void voxel_grid_filter(const deviceCloudT& input,deviceCloudT& output,const float leafsize);

  /**
   * @brief filter--using voxel grid to downsample point cloud
   * @param [in]input--input cloud with host_vector<mb_cuda::PointXYZRGB> type
   * @param [out]output--output cloud with host_vector<mb_cuda::PointXYZRGB> type
   * @param [in]leafsize--the resolution of voxel grids
   */
  void voxel_grid_filter(const hostCloudT& input,hostCloudT& output,const float leafsize);

  /**
   * @brief getMinMax3D--find the minimum and maximum x,y,z coordinates of a point cloud
   * @param [in]input--point cloud with device_vector<mb_cuda::PointXYZRGB> type
   * @param [out]minPt--minPt with mb_cuda::PointXYZRGB type which records the minimum x,y,z coordinates
   * @param [out]maxPt--maxPt with mb_cuda::PointXYZRGB type which records the maximum x,y,z coordinates
   */
  void getMinMax3D(const deviceCloudT& input, pointT& minPt, pointT& maxPt);

  /**
   * @brief getMinMax3D--find the minimum and maximum x,y,z coordinates of a point cloud
   * @param [in]input--point cloud with host_vector<mb_cuda::PointXYZRGB> type
   * @param [out]minPt--minPt with mb_cuda::PointXYZRGB type which records the minimum x,y,z coordinates
   * @param [out]maxPt--maxPt with mb_cuda::PointXYZRGB type which records the maximum x,y,z coordinates
   */
  void getMinMax3D(const hostCloudT& input, pointT& minPt, pointT& maxPt);

}

#endif
