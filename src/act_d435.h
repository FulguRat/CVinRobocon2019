#define _CRT_SECURE_NO_WARNINGS

#ifndef ACT_D435_H_
#define ACT_D435_H_

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace rs2;


typedef pcl::PointXYZRGB 			pointType;
typedef pcl::PointCloud<pointType> 	pointCloud;
typedef pointCloud::Ptr 			pPointCloud;

class ActD435
{
public:
	ActD435();
	ActD435(const ActD435&) = delete;
	ActD435& operator=(const ActD435&) = delete;
	~ActD435();

	void init(void);
	pPointCloud update(void);
	cv::Mat				 color;

private:
	//-- For color-aligned point cloud
	std::tuple<uint8_t, uint8_t, uint8_t> getColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	pPointCloud pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color);

	//-- For point cloud without color
	pPointCloud pointsToPointCloud(const rs2::points& points);


private:
	rs2::pointcloud  rs2Cloud;
	rs2::points      rs2Points;

	rs2::pipeline    pipe;
	rs2::config      cfg;

	rs2::frameset    frameSet;
	rs2::frameset    alignedFrameSet;

	rs2::align       align;
	


	pPointCloud		 cloudByRS2;

	// pcl::visualization::CloudViewer viewer;
};

#endif