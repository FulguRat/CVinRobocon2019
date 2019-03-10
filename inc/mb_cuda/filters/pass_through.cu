#include "mb_cuda/filters/pass_through.h"
#include <thrust/execution_policy.h>
#include <math.h>

namespace mb_cuda {

  /**
   * @brief The isExceedLimit struct
   */
  struct isExceedLimit
  {
    float _minLimit=0,_maxLimit=0;
    char _fieldIndex=2;
    isExceedLimit(float minLimit,float maxLimit,char fieldIndex)
      :_minLimit(minLimit),_maxLimit(maxLimit),_fieldIndex(fieldIndex)
    {}

    inline __host__ __device__
    bool operator () (const mb_cuda::PointXYZRGB& pt) const
    {
      float value;
      if(_fieldIndex==0)
        value=pt.x;
      else if(_fieldIndex==1)
        value=pt.y;
      else
        value=pt.z;

      if(value>=_minLimit && value<=_maxLimit)
        return true;
      else
        return false;
    }
  };

  /**
   * @brief pass_through_filter
   * @param inputCloud
   * @param outputCloud
   * @param filterFieldName
   * @param minLimit
   * @param maxLimit
   */
  void pass_through_filter(const hostCloudT & inputCloud, hostCloudT & outputCloud,
                           char filterFieldName,
                           float minLimit,
                           float maxLimit)
  {
    char fieldIndex=(char)filterFieldName-(char)('x');
    isExceedLimit Checker(minLimit,maxLimit,fieldIndex);

    outputCloud.resize(inputCloud.size());
    hostCloudT::iterator it=thrust::copy_if(thrust::host,inputCloud.begin(),inputCloud.end(),outputCloud.begin(),Checker);
    outputCloud.resize(it-outputCloud.begin());
  }

  /**
   * @brief pass_through_filter
   * @param inputCloud
   * @param outputCloud
   * @param filterFieldName
   * @param minLimit
   * @param maxLimit
   */
  void pass_through_filter(const deviceCloudT & inputCloud, deviceCloudT & outputCloud,
                           char filterFieldName,
                           float minLimit,
                           float maxLimit)
  {
    char fieldIndex=(char)filterFieldName-(char)('x');
    isExceedLimit Checker(minLimit,maxLimit,fieldIndex);

    outputCloud.resize(inputCloud.size());
    deviceCloudT::iterator it=thrust::copy_if(thrust::device,inputCloud.begin(),inputCloud.end(),outputCloud.begin(),Checker);
    outputCloud.resize(it-outputCloud.begin());
  }

  /**
   * @brief The inNan_functor struct
   */
  struct isNan_functor
  {
    __host__ __device__
    bool operator() (const mb_cuda::PointXYZRGB& p)const
    {
      return (!isnan(p.x) && !isnan(p.y) && !isnan(p.z)
              && isfinite(p.x) && isfinite(p.y) && isfinite(p.z));
    }
  };

  /**
   * @brief removeNans
   * @param inputCloud
   * @param outputCloud
   */
  void removeNansOrIfs(const deviceCloudT &inputCloud, deviceCloudT & outputCloud)
  {
    outputCloud.resize(inputCloud.size());
    deviceCloudT::iterator it=thrust::copy_if(thrust::device,inputCloud.begin(),inputCloud.end(),outputCloud.begin(),isNan_functor());
    outputCloud.reserve(it-outputCloud.begin());
  }
}
