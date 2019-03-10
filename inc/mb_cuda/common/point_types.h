#ifndef MB_CUDA_POINT_TYPES_H_
#define MB_CUDA_POINT_TYPES_H_

#include <cuda.h>
#include <cuda_runtime.h>
namespace mb_cuda {

  /**
   * @brief describe the coordinates and color
   * information of a point
   */
  struct PointXYZRGB
  {
    //coordinates
    union
    {
      float3 xyz;
      struct
      {
        float x;
        float y;
        float z;
      };
    };

    //rgb
     union
     {
       uchar3 rgb;
       struct
       {
         unsigned char r;
         unsigned char g;
         unsigned char b;
       };
     };

//    unsigned char alpha;

    inline __host__ __device__ PointXYZRGB operator - (const PointXYZRGB &rhs)
    {
      PointXYZRGB res=*this;
      res.x-=rhs.x;
      res.y-=rhs.y;
      res.z-=rhs.z;
      return (res);
    }

    inline __host__ __device__ PointXYZRGB operator + (const PointXYZRGB &rhs)
    {
      PointXYZRGB res=*this;
      res.x+=rhs.x;
      res.y+=rhs.y;
      res.z+=rhs.z;
      return (res);
    }

    inline __host__ __device__ PointXYZRGB& operator += (const PointXYZRGB &rhs)
    {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
      return (*this);
    }

    inline __host__ __device__ PointXYZRGB& operator -= (const PointXYZRGB &rhs)
    {
      x -= rhs.x;
      y -= rhs.y;
      z -= rhs.z;
      return (*this);
    }

  };
}
#endif
