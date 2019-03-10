#include "mb_cuda/filters/voxel_grid.h"
#include <thrust/transform_reduce.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>
#include <thrust/remove.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>

namespace mb_cuda {

  /**
   * @brief The minmax_pair struct
   */
  struct minmax_tuple
  {
    float3 min_val;
    float3 max_val;
  };

  /**
   * @brief The minmax_unary_op struct
   */
  struct minmax_unary_op:
      public thrust::unary_function<pointT, minmax_tuple>
  {
    __host__ __device__
    minmax_tuple operator () (const pointT& pt) const
    {
      minmax_tuple minmax;
      minmax.min_val.x=pt.x;
      minmax.min_val.y=pt.y;
      minmax.min_val.z=pt.z;

      minmax.max_val.x=pt.x;
      minmax.max_val.y=pt.y;
      minmax.max_val.z=pt.z;
      return minmax;
    }
  };

  /**
   * @brief The minmax_binary_op struct
   */
  struct minmax_binary_op:
      public thrust::binary_function<minmax_tuple,minmax_tuple,minmax_tuple>
  {
    __host__ __device__
    minmax_tuple operator()(const minmax_tuple& p1, const minmax_tuple& p2) const
    {
      minmax_tuple result;
      result.min_val.x=p1.min_val.x>p2.min_val.x ? p2.min_val.x : p1.min_val.x;
      result.min_val.y=p1.min_val.y>p2.min_val.y ? p2.min_val.y : p1.min_val.y;
      result.min_val.z=p1.min_val.z>p2.min_val.z ? p2.min_val.z : p1.min_val.z;

      result.max_val.x=p1.max_val.x<p2.max_val.x ? p2.max_val.x : p1.max_val.x;
      result.max_val.y=p1.max_val.y<p2.max_val.y ? p2.max_val.y : p1.max_val.y;
      result.max_val.z=p1.max_val.z<p2.max_val.z ? p2.max_val.z : p1.max_val.z;

      return result;
    }
  };

  /**
   * @brief getMinMax3D
   * @param input
   * @param minPt
   * @param maxPt
   */
  void getMinMax3D(const deviceCloudT& input, pointT& minPt, pointT& maxPt)
  {
    //initialize default values
    if(input.size()<=0)
      return;
    float minV=std::numeric_limits<float>::min();
    float maxV=std::numeric_limits<float>::max();
    minPt.x=minV;
    minPt.y=minV;
    minPt.z=minV;
    maxPt.x=maxV;
    maxPt.y=maxV;
    maxPt.z=maxV;

    minmax_unary_op unary_op;
    minmax_binary_op binary_op;

    //initialize reduction with the first value
    minmax_tuple initpair=unary_op(input[0]);

    //compute minimum and maximum values
    minmax_tuple result=thrust::transform_reduce(
          input.begin(),input.end(),unary_op,initpair,binary_op);

    minPt.x=result.min_val.x;
    minPt.y=result.min_val.y;
    minPt.z=result.min_val.z;
    maxPt.x=result.max_val.x;
    maxPt.y=result.max_val.y;
    maxPt.z=result.max_val.z;
  }

  /**
   * @brief The index_pair struct records the voxel index and point index of the cloud
   */
  struct index_pair
  {
    int64_t voxel_index;
    int64_t point_index;
  };

  /**
   * @brief The index_indices_pair struct
   * index stors the voxel index
   * indices store the indices of points belonging to the voxel
   */
  struct index_indices_pair
  {
    int64_t voxel_index;
    thrust::device_ptr<int64_t> pt_indices;
//    thrust::device_vector<int> pt_indices;
  };

  /**
   * @brief The computeVoxelIndex_functor struct
   */
  struct computeVoxelIndex_functor
  {
    float leafsize;
//    int min_b_0,min_b_1,min_b_2;
//    int divb_mul_0,divb_mul_1,divb_mul_2;
    thrust::device_ptr<int64_t> min_b_;
    thrust::device_ptr<int64_t> divb_mul_;

    __host__ __device__
    index_pair operator () (const pointT& pt,const int64_t& pt_idx) const
    {
      int64_t ijk0=static_cast<int64_t>(std::floor((pt.x)/leafsize))-min_b_[0];
      int64_t ijk1=static_cast<int64_t>(std::floor((pt.y)/leafsize))-min_b_[1];
      int64_t ijk2=static_cast<int64_t>(std::floor((pt.z)/leafsize))-min_b_[2];
      int64_t idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];

      index_pair ip;
      ip.point_index=pt_idx;
      ip.voxel_index=idx;
      return ip;
    }
  };


  /**
   * @brief The compareIndexPair_functor struct
   */
  struct compareIndexPair_functor
  {
    __host__ __device__
    bool operator() (const index_pair& ip1, const index_pair& ip2)const
    {
      return (ip1.voxel_index<ip2.voxel_index);
    }
  };

  /**
   * @brief The equalToZero_funcor struct
   */
  struct findEqual_funcor:public thrust::binary_function<index_pair,index_pair,int64_t>
  {
    __host__ __device__
    int64_t operator() (const index_pair& idx1, const index_pair& idx2) const
    {
      if(idx1.voxel_index==idx2.voxel_index)
        return -1;
      else
        return idx1.voxel_index;
    }
  };

  /**
   * @brief The isEqualMinusOne struct
   */
  struct isEqualMinusOne
  {
    __host__ __device__
    bool operator() (int64_t idx)
    {
      return idx==-1;
    }
  };

  struct map_voxel_indices_functor :public thrust::binary_function<int64_t,int64_t,int64_t>
  {
    __host__ __device__
    int64_t operator() (const int64_t& voxel_idx, const int64_t & sequence_idx)const
    {
      if(voxel_idx==-1)
        return -1;
      else
        return sequence_idx;
    }
  };


  /**
   * @brief compute_centroid_kernel
   * @param voxel_indices
   * @param len
   * @param full_indices
   * @param pointNum
   * @param point_input
   * @param point_output
   */
  __global__ void compute_centroid_kernel(thrust::device_ptr<int64_t> voxel_indices,
                                          int64_t len,
                                          thrust::device_ptr<index_pair> full_indices,
                                          int64_t pointNum,
                                          thrust::device_ptr<pointT> point_input,
                                          thrust::device_ptr<pointT> point_output
                                          )
  {
    int64_t idx=blockIdx.x*blockDim.x+threadIdx.x;
    if(idx>len)
      return;

    int64_t index=static_cast<int64_t>(voxel_indices[idx]);

    if(index>=pointNum)
      return;

    int64_t pt_idx=static_cast<int64_t>((index_pair(full_indices[index])).point_index);

    if(pt_idx>=pointNum)
      return;

    pointT centroid=point_input[pt_idx];

    int64_t i = index + 1;
    while (i < pointNum &&
           (index_pair(full_indices[i])).voxel_index == (index_pair(full_indices[index])).voxel_index)
    {
      int64_t p_idx=static_cast<int64_t>((index_pair(full_indices[i])).point_index);
      if(p_idx>=pointNum)
        continue;
      pointT pt=point_input[p_idx];
      centroid.x+=pt.x;
      centroid.y+=pt.y;
      centroid.z+=pt.z;

      ++i;
    }

    int64_t dist=i-index;
    if(dist!=0){
        centroid.x /= static_cast<float>(dist);
        centroid.y /= static_cast<float>(dist);
        centroid.z /= static_cast<float>(dist);
      }

    point_output[idx]=centroid;

//    __syncthreads();
  }

  /**
   * @brief voxel_grid_filter
   * @param input
   * @param output
   * @param leafsize
   */
  void voxel_grid_filter(const deviceCloudT& input,deviceCloudT& output,const float leafsize)
  {
    //obtain the minimum and maximum x,y,z coordinates
    pointT min_pt=input[0],max_pt=input[0];
    getMinMax3D(input,min_pt,max_pt);

    //check the leafsize's feasibility
    int64_t dx=static_cast<int64_t>((max_pt.x-min_pt.x)/leafsize);
    int64_t dy=static_cast<int64_t>((max_pt.y-min_pt.y)/leafsize);
    int64_t dz=static_cast<int64_t>((max_pt.z-min_pt.z)/leafsize);
    if( (dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()) )
    {
       std::cerr<<"ERROR: the leafsize is too small, so that the voxel size will overflow"<<std::endl;
       return;
    }

    //compute the minimum and maximum bounding box values
    thrust::device_ptr<int64_t> min_b_=thrust::device_malloc<int64_t>(3);
    thrust::device_ptr<int64_t> max_b_=thrust::device_malloc<int64_t>(3);
    thrust::device_ptr<int64_t> div_b_=thrust::device_malloc<int64_t>(3);
    thrust::device_ptr<int64_t> divb_mul_=thrust::device_malloc<int64_t>(3);

    min_b_[0]=static_cast<int>(std::floor(min_pt.x/leafsize));
    min_b_[1]=static_cast<int>(std::floor(min_pt.y/leafsize));
    min_b_[2]=static_cast<int>(std::floor(min_pt.z/leafsize));
    max_b_[0]=static_cast<int>(std::floor(max_pt.x/leafsize));
    max_b_[1]=static_cast<int>(std::floor(max_pt.y/leafsize));
    max_b_[2]=static_cast<int>(std::floor(max_pt.z/leafsize));

//    std::cout<<"min_pt: "<<min_pt.x<<" "<<min_pt.y<<" "<<min_pt.z<<std::endl;
//    std::cout<<"min_b_: "<<min_b_[0]<<" "<<min_b_[1]<<" "<<min_b_[2]<<std::endl;

    //compute the number of divisions needed along all axis
    div_b_[0]=max_b_[0]-min_b_[0]+1;
    div_b_[1]=max_b_[1]-min_b_[1]+1;
    div_b_[2]=max_b_[2]-min_b_[2]+1;

    divb_mul_[0]=1;
    divb_mul_[1]=div_b_[0];
    divb_mul_[2]=div_b_[0]*div_b_[1];

//    std::cout<<"divb_mul_: "<<divb_mul_[0]<<" "<<divb_mul_[1]<<" "<<divb_mul_[2]<<std::endl;

    //index_vector stores the point index and voxel index of a point
    thrust::device_vector<index_pair> index_vector(input.size());
    //generate an aux vector that stors the indices of point cloud
    thrust::device_vector<int64_t> point_index_vec(input.size());
    thrust::sequence(point_index_vec.begin(),point_index_vec.end());

    //compute the indices of voxel grids
    computeVoxelIndex_functor cvi_functor;
    cvi_functor.leafsize=leafsize;
    cvi_functor.min_b_=min_b_;
    cvi_functor.divb_mul_=divb_mul_;
    thrust::transform(input.begin(),
                      input.end(),
                      point_index_vec.begin(),
                      index_vector.begin(),
                      cvi_functor);

    thrust::device_free(min_b_);
    thrust::device_free(max_b_);
    thrust::device_free(div_b_);
    thrust::device_free(divb_mul_);

    //sort the index_vector
    thrust::sort(index_vector.begin(),index_vector.end(),compareIndexPair_functor());

    //count ouput cells
    //we need to skip all the same, adjacenet idx values
    thrust::device_vector<index_pair> temp_index_vec(index_vector.size()+1);
    thrust::copy(index_vector.begin(),index_vector.end(),temp_index_vec.begin()+1);
    index_pair ip;
    ip.voxel_index=-1;
    ip.point_index=-1;
    temp_index_vec[0]=ip;
    thrust::device_vector<int64_t> nonequal_voxel_indices(index_vector.size());
    thrust::transform(index_vector.begin(),
                      index_vector.end(),
                      temp_index_vec.begin(),
                      nonequal_voxel_indices.begin(),
                      findEqual_funcor());

    thrust::device_vector<int64_t> voxel_indices(index_vector.size());
    thrust::transform(nonequal_voxel_indices.begin(),
                      nonequal_voxel_indices.end(),
                      point_index_vec.begin(),
                      voxel_indices.begin(),
                      map_voxel_indices_functor());

    thrust::device_vector<int64_t>::iterator it=thrust::remove_if(thrust::device,
                                                                  voxel_indices.begin(),
                                                                  voxel_indices.end(),
                                                                  isEqualMinusOne());
    voxel_indices.resize(it-voxel_indices.begin());

    //input paras
    int64_t len=voxel_indices.size();
    int64_t pt_num=input.size();

    thrust::device_ptr<int64_t> voxel_indices_array_ptr=thrust::device_malloc<int64_t>(len);
    thrust::copy(voxel_indices.begin(),voxel_indices.end(),voxel_indices_array_ptr);

    thrust::device_ptr<index_pair> full_indices=thrust::device_malloc<index_pair>(pt_num);
    thrust::copy(thrust::device,index_vector.begin(),index_vector.end(),full_indices);

    thrust::device_ptr<pointT> point_input=thrust::device_malloc<pointT>(pt_num);
    thrust::copy(input.begin(),input.end(),point_input);

    thrust::device_ptr<pointT> point_output=thrust::device_malloc<pointT>(len);


//    //kernel function
    int BLOCK_SIZE=16;
    dim3 threadsPerBlock(16,16);
    int numBlocks= len/(BLOCK_SIZE*BLOCK_SIZE) + ((len % (BLOCK_SIZE*BLOCK_SIZE))==0 ? 0 : 1);
    std::cout<<"num blocks: "<<numBlocks<<std::endl;

    compute_centroid_kernel <<<numBlocks,BLOCK_SIZE*BLOCK_SIZE>>> (voxel_indices_array_ptr,
                                                                   len,
                                                                   full_indices,
                                                                   pt_num,point_input,
                                                                   point_output);
    cudaDeviceSynchronize();

    output.clear();
    output.resize(len);
    thrust::copy_n(thrust::device,point_output,len,output.begin());

    thrust::free(thrust::device,voxel_indices_array_ptr);
    thrust::free(thrust::device,full_indices);
    thrust::free(thrust::device,point_input);
    thrust::free(thrust::device,point_output);
  }

}
