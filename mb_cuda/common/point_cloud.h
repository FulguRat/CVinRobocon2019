#ifndef MB_CUDA_POINT_TYPES_H_
#define MB_CUDA_POINT_TYPES_H_

#include "mb_cuda/common/point_types.h"
#include "mb_cuda/common/thrust.h"
#include <boost/shared_ptr.hpp>

namespace mb_cuda {
  /** \brief misnamed class holding a 3x3 matrix */
  struct CovarianceMatrix
  {
    float3 data[3];
  };

}
