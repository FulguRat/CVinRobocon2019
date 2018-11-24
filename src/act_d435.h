#define _CRT_SECURE_NO_WARNINGS

#ifndef ACT_D435_H_
#define ACT_D435_H_

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <opencv2/opencv.hpp>
#include <time.h>

using namespace std;
using namespace rs2;
//using namespace cv;

using pPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class ActD435
{
public:
	ActD435();
	ActD435(const ActD435&) = delete;
	ActD435& operator=(const ActD435&) = delete;
	~ActD435();

	void init(void);
	void update(void);

private:
	pPointCloud pointsToPointCloud(const rs2::points& points);

private:
	rs2::pointcloud  pc;
	rs2::points      points;

	rs2::pipeline    pipe;
	rs2::config      cfg;

	pPointCloud      cloudFiltered;

	pcl::visualization::CloudViewer viewer;

	rs2::frameset    frameSet;
};

#endif