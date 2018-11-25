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

using pointType = pcl::PointXYZRGB;
using pointCloud = pcl::PointCloud<pointType>;
using pPointCloud = pointCloud::Ptr;

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
	// For color-aligned point cloud
	std::tuple<uint8_t, uint8_t, uint8_t> getColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	pPointCloud pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color);

	// For point cloud without color
	pPointCloud pointsToPointCloud(const rs2::points& points);

private:
	rs2::pointcloud  pc;
	rs2::points      points;

	rs2::pipeline    pipe;
	rs2::config      cfg;

	rs2::align       align;

	pPointCloud      cloudFiltered;

	pcl::visualization::CloudViewer viewer;

	rs2::frameset    frameSet;
	rs2::frameset    alignedFrameSet;
};

#endif