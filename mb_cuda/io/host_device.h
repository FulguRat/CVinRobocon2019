#ifndef HOST_DEVICE_H_
#define HOST_DEVICE_H_

#include "mb_cuda/common/point_types.h"
#include "mb_cuda/common/thrust.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace mb_cuda
{

    typedef thrust::host_vector<mb_cuda::PointXYZRGB> hostCloudT;
    typedef thrust::device_vector<mb_cuda::PointXYZRGB> deviceCloudT;

  /**
     * @brief load data from host vector to device vector
     * @param host_cloud with pointType PointXYZRGB
     * @param device_cloud with pointType PointXYZRGB
     */
    void host_to_device(hostCloudT & host_cloud, deviceCloudT & device_cloud);

    /**
     * @brief load data from device vector to host vector
     * @param device_cloud with pointType PointXYZRGB
     * @param host_cloud with pointType PointXYZRGB
     */
    void device_to_host(deviceCloudT & device_cloud, hostCloudT & host_cloud);
}

#endif
