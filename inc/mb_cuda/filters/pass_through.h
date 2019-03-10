#ifndef MBCUDA_PASS_THROUGH_H_
#define MBCUDA_PASS_THROUGH_H_

#include "mb_cuda/common/point_types.h"
#include "mb_cuda/common/thrust.h"

namespace mb_cuda {

  typedef thrust::host_vector<mb_cuda::PointXYZRGB> hostCloudT;
  typedef thrust::device_vector<mb_cuda::PointXYZRGB> deviceCloudT;
  /**
   * @brief pass_through_filter: check each point and remove those exceed the limitations
   * @param inputCloud
   * @param outputCloud
   * @param filterFieldName
   * @param minLimit
   * @param maxLimit
   */
  void pass_through_filter(const hostCloudT &inputCloud, hostCloudT & outputCloud,
                           char filterFieldName,
                           float minLimit,
                           float maxLimit);

  /**
   * @brief pass_through_filter
   * @param inputCloud
   * @param outputCloud
   * @param filterFieldName
   * @param minLimit
   * @param maxLimit
   */
  void pass_through_filter(const deviceCloudT &inputCloud, deviceCloudT & outputCloud,
                           char filterFieldName,
                           float minLimit,
                           float maxLimit);

  /**
   * @brief removeNans
   * @param inputCloud
   * @param outputCloud
   */
  void removeNansOrIfs(const deviceCloudT &inputCloud, deviceCloudT & outputCloud);
}
#endif
