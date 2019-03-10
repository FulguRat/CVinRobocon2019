#define FLANN_USE_CUDA
#include "mb_cuda/filters/statistical_outlier_removal.h"
#include <thrust/execution_policy.h>
#include <thrust/transform_reduce.h>
#include <thrust/remove.h>

namespace mb_cuda {

  /**
   * @brief The pointXYZRGBToXYZ_funtor struct
   */
  struct pointXYZRGBToXYZ_funtor:
      public thrust::unary_function<mb_cuda::PointXYZRGB,float4>
  {
    __host__ __device__
    float4 operator() (const pointT& pt)const
    {
      float4 fp=make_float4(pt.x,pt.y,pt.z,0);
      return fp;
    }
  };

  /**
   * @brief The square struct
   */
  struct square
  {
    __host__ __device__
    float operator()(const float& x) const {
      return x * x;
    }
  };


  /**
   * @brief The isInlier_functor struct
   */
  struct isInlier_functor
  {
    __host__ __device__
    bool operator() (const int& i)const
    {
      return (i==-1);
    }
  };

  /**
   * @brief computeAvgDistanceKernel
   * @param dists
   * @param pt_num
   * @param knn
   * @param avg_dists
   */
  __global__ void computeAvgDistanceKernel(thrust::device_ptr<float> dists,
                                      int pt_num,
                                      int knn,
                                      thrust::device_ptr<float> avg_dists)
  {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx>=pt_num)
      return;

    double dist_sum = 0;
    for (int i = 1; i < knn; ++i)
      dist_sum += sqrt (static_cast<float>(dists[idx * knn + i]));
    avg_dists[idx] = static_cast<float> (dist_sum / (knn - 1));
  }

  /**
   * @brief computeInliers
   * @param distances
   * @param pt_num
   * @param dist_threshold
   * @param inliers
   */
  __global__ void computeInliers(thrust::device_ptr<float> distances,
                                 int pt_num,
                                 double dist_threshold,
                                 thrust::device_ptr<int> inliers)
  {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx>=pt_num)
      return;

    if(distances[idx]<=dist_threshold)
      inliers[idx]=idx;
    else
      inliers[idx]=-1;
  }

  /**
   * @brief statistical_outlier_removal
   * @param input_points
   * @param k
   * @param std_mul
   * @param inliers_indices
   * @param inliers_num
   */
  
  void statistical_outlier_removal(deviceCloudT input_points,
                                   int k,
                                   float std_mul_,
                                   thrust::device_vector<int> inliers_indices,
                                   int& inliers_num)
  {
    // validate the input
    if(input_points.size()<=0)
      return;

    if(k<=1){
        std::cerr<<"the k nearest neighbours should be set as a number larger than 1!"<<std::endl;
        return;
      }

    if(std_mul_==0){
      std::cerr<<"the standard deviation multipler not set"<<std::endl;
      return;
     }

    // go over the points to find the k nearest neighbours and distances of each point
    // input data conversion
    thrust::device_vector<float4> points_vec(input_points.size());
    pointXYZRGBToXYZ_funtor conv_functor;
    thrust::transform(input_points.begin(),
                      input_points.end(),
                      points_vec.begin(),
                      conv_functor);

    float* points_vec_ptr=(float*)thrust::raw_pointer_cast(&(points_vec[0]));
    flann::Matrix<float> points_mat(points_vec_ptr,points_vec.size(), 3, 4 * sizeof(float));

    flann::KDTreeCuda3dIndexParams cparams;
    cparams["input_is_gpu_float4"]=true;
    flann::KDTreeCuda3dIndex<flann::L2<float> >flann_index(points_mat,cparams);
    flann_index.buildIndex();// consumer too much time (about 1500ms)


    // queries results
    int points_num=input_points.size();
    thrust::device_ptr<int> indices_ptr=thrust::device_malloc<int>(k*points_num);
    thrust::device_ptr<float> dists_ptr=thrust::device_malloc<float>(k*points_num);
    int* raw_indices_ptr=(int*)thrust::raw_pointer_cast(indices_ptr);
    float* raw_dists_ptr=(float*)thrust::raw_pointer_cast(dists_ptr);
    flann::Matrix<int> indices_gpu ( raw_indices_ptr, points_num, k );
    flann::Matrix<float> dists_gpu ( raw_dists_ptr, points_num, k );
    flann::SearchParams sparas;
    sparas.matrices_in_gpu_ram=true;

    // apply queries
    flann_index.knnSearchGpu(points_mat,indices_gpu,dists_gpu, k, sparas);

    // mean distance
    thrust::device_ptr<float> d_avgDists_ptr=thrust::device_malloc<float>(points_num);
    int threadPerBlock=256;
    int numOfBlocks=points_num/threadPerBlock + (points_num % threadPerBlock ==0 ? 0 : 1);
    computeAvgDistanceKernel<<< numOfBlocks, threadPerBlock >>>(dists_ptr,
                                                                 points_num,
                                                                 k,
                                                                 d_avgDists_ptr);

    cudaDeviceSynchronize();

    //estimate the mean and the standard devication of the distance vector
    thrust::device_vector<float> distances(points_num);
    thrust::device_vector<float> sq_distances(points_num);
    thrust::copy_n(d_avgDists_ptr,points_num,distances.begin());
    float sum=0;
    sum = thrust::reduce(distances.begin(),distances.end(), (float)0);
    square unary_op;
    thrust::plus<float> binary_op;
    float sq_sum=0;
    sq_sum=thrust::transform_reduce(distances.begin(),distances.end(),unary_op, (float)0, binary_op);

    double mean = sum / static_cast<double>(points_num);
    double variance = (sq_sum - sum * sum / static_cast<double>(points_num)) / (static_cast<double>(points_num) - 1);
    double stddev = sqrt (variance);
    double distance_threshold = mean + std_mul_ * stddev;

    //compute inliers
    thrust::device_ptr<int> inliers=thrust::device_malloc<int>(points_num);
    computeInliers<<< numOfBlocks, threadPerBlock >>>(d_avgDists_ptr,
                                                      points_num,
                                                      distance_threshold,
                                                      inliers);
    cudaDeviceSynchronize();

    //results
    inliers_indices.clear();
    inliers_indices.resize(points_num);
    thrust::copy_n(inliers,points_num,inliers_indices.begin());
    thrust::device_vector<int>::iterator it = thrust::remove_if(inliers_indices.begin(),inliers_indices.end(),isInlier_functor());
    inliers_num=it-inliers_indices.begin();
    inliers_indices.resize(inliers_num);

//    free(indices_ptr);
//    free(dists_ptr);
//    thrust::free(thrust::device,d_dists_ptr);
    thrust::free(thrust::device,indices_ptr);
    thrust::free(thrust::device, dists_ptr);
    thrust::free(thrust::device,d_avgDists_ptr);
    thrust::free(thrust::device,inliers);

  }






  /**
   * @brief statistical_outlier_removal
   * @param dists
   * @param pt_num
   * @param k
   * @param std_mul
   * @param inliers_indices
   * @param inliers_num
   */
  void statistical_outlier_removal(
      thrust::device_ptr<float> dists,
      int pt_num,
      int k,
      float std_mul_,
      thrust::device_vector<int> inliers_indices,
      int& inliers_num)
  {
    int points_num=pt_num;
    // mean distance
    thrust::device_ptr<float> d_avgDists_ptr=thrust::device_malloc<float>(points_num);
    int threadPerBlock=256;
    int numOfBlocks=points_num/threadPerBlock + (points_num % threadPerBlock ==0 ? 0 : 1);
    computeAvgDistanceKernel<<< numOfBlocks, threadPerBlock >>>(dists,
                                                                 points_num,
                                                                 k,
                                                                 d_avgDists_ptr);

    cudaDeviceSynchronize();

    //estimate the mean and the standard devication of the distance vector
    thrust::device_vector<float> distances(points_num);
    thrust::device_vector<float> sq_distances(points_num);
    thrust::copy_n(d_avgDists_ptr,points_num,distances.begin());
    float sum=0;
    sum = thrust::reduce(distances.begin(),distances.end(), (float)0);
    square unary_op;
    thrust::plus<float> binary_op;
    float sq_sum=0;
    sq_sum=thrust::transform_reduce(distances.begin(),distances.end(),unary_op, (float)0, binary_op);

    double mean = sum / static_cast<double>(points_num);
    double variance = (sq_sum - sum * sum / static_cast<double>(points_num)) / (static_cast<double>(points_num) - 1);
    double stddev = sqrt (variance);
    double distance_threshold = mean + std_mul_ * stddev;

    //compute inliers
    thrust::device_ptr<int> inliers=thrust::device_malloc<int>(points_num);
    computeInliers<<< numOfBlocks, threadPerBlock >>>(d_avgDists_ptr,
                                                      points_num,
                                                      distance_threshold,
                                                      inliers);
    cudaDeviceSynchronize();

    //results
    inliers_indices.clear();
    inliers_indices.resize(points_num);
    thrust::copy_n(inliers,points_num,inliers_indices.begin());
    thrust::device_vector<int>::iterator it = thrust::remove_if(inliers_indices.begin(),inliers_indices.end(),isInlier_functor());
    inliers_num=it-inliers_indices.begin();
    inliers_indices.resize(inliers_num);

    thrust::free(thrust::device,d_avgDists_ptr);
    thrust::free(thrust::device,inliers);
  }

}
