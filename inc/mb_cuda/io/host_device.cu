#include "mb_cuda/io/pcl_thrust.h"

namespace mb_cuda {

  typedef thrust::host_vector<mb_cuda::PointXYZRGB> hostCloudT;
  typedef thrust::device_vector<mb_cuda::PointXYZRGB> deviceCloudT;

  void host_to_device(hostCloudT & host_cloud,
                                          deviceCloudT & device_cloud)
  {
    device_cloud.resize(host_cloud.size());
    thrust::copy(host_cloud.begin(),host_cloud.end(),device_cloud.begin());

  }

  void device_to_host(deviceCloudT & device_cloud,
                                          hostCloudT & host_cloud)
  {
    host_cloud.resize(device_cloud.size());
    thrust::copy(device_cloud.begin(),device_cloud.end(),host_cloud.begin());
  }

}
