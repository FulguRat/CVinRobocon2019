#ifndef PCL_THRUST_H_
#define PCL_THRUST_H_

#include "mb_cuda/common/point_types.h"
#include "mb_cuda/common/thrust.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cuda.h>
#include <cuda_runtime.h>

namespace mb_cuda
{
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr;
    typedef thrust::host_vector<mb_cuda::PointXYZRGB> thrustCloudT;

  /**
     * @brief convert data from pcl pointcloud to thrust vectors
     * @param pcl_cloud with pointType PointXYZRGB
     * @param thrust_cloud must be thrust::host_vector<>
     */
    void pcl_to_thrust(pclCloudPtr & pcl_cloud, thrustCloudT & thrust_cloud);

    /**
     * @brief convert data from thrust vector to spcl pointcloud
     * @param thrust_cloud
     * @param pcl_cloud
     */
    void thrust_to_pcl(thrustCloudT& thrust_cloud, pclCloudPtr & pcl_cloud);

}

#endif
