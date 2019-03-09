#include "mb_cuda/io/pcl_thrust.h"
#include <iostream>
namespace mb_cuda {

  void pcl_to_thrust(pclCloudPtr & pcl_cloud,
                     thrustCloudT & thrust_cloud)
  {
    thrust_cloud.resize(pcl_cloud->points.size());
    for(int i=0;i<pcl_cloud->points.size();++i){
            thrust_cloud[i].x=pcl_cloud->points[i].x;
            thrust_cloud[i].y=pcl_cloud->points[i].y;
            thrust_cloud[i].z=pcl_cloud->points[i].z;
             thrust_cloud[i].r=pcl_cloud->points[i].r;
             thrust_cloud[i].g=pcl_cloud->points[i].g;
             thrust_cloud[i].b=pcl_cloud->points[i].b;
          }

  }

  void thrust_to_pcl(thrustCloudT & thrust_cloud,
                     pclCloudPtr & pcl_cloud)
  {
   // std::cout << "before resize: " << pcl_cloud->points.size() << std::endl;
    if(pcl_cloud->points.size()>0)
      pcl_cloud->clear();

    pcl_cloud->points.resize(0);

   // std::cout << "after resize: " << pcl_cloud->points.size() << std::endl;
    for(int i=0;i<thrust_cloud.size();++i){
        pcl::PointXYZRGB pt;
        pt.x=thrust_cloud[i].x;
        pt.y=thrust_cloud[i].y;
        pt.z=thrust_cloud[i].z;
         pt.r=thrust_cloud[i].r;
         pt.g=thrust_cloud[i].g;
         pt.b=thrust_cloud[i].b;

        pcl_cloud->points.push_back(pt);
      }



    pcl_cloud->width=pcl_cloud->points.size();
    pcl_cloud->height=1;

  }

}
