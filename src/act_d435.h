#define _CRT_SECURE_NO_WARNINGS

#ifndef ACT_D435_H_
#define ACT_D435_H_

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include "calc_time.h"
//#include <opencv2/opencv.hpp>

using namespace std;
using namespace rs2;
//using namespace cv;

using pPointCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr;

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
	tuple<uint8_t, uint8_t, uint8_t> getTexcolor(video_frame texture, texture_coordinate texcoords);
	pPointCloud pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color);

private:
	rs2::points     points;
	rs2::pipeline   pipe;
	rs2::config     cfg;

	rs2::align      align;

	pPointCloud     cloudFiltered;

	pcl::visualization::CloudViewer viewer;

	rs2::frameset   frameSet;
	rs2::frameset   alignedFrameSet;
};

#endif