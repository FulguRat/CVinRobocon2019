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
#include <Eigen/Dense>

#define MODEL			0
#define LEFT_MODEL		0
#define RIGHT_MODEL		1

#define HORIZONAL_FENSE			0
#define VERTICAL_FENSE				1

#define STARTUP_INITIAL				0
#define BEFORE_DUNE_STAGE_1			1
#define BEFORE_DUNE_STAGE_2			2
#define BEFORE_DUNE_STAGE_3			3
#define PASSING_DUNE                4
#define BEFORE_GRASSLAND_STAGE_1	5
#define BEFORE_GRASSLAND_STAGE_2	6
#define PASSING_GRASSLAND_STAGE_1	7
#define PASSING_GRASSLAND_STAGE_2	8
#define CLIMBING_MOUNTAIN           9

#define CAMERA_ARGS_LEFT  { 605.1696f, 606.6738f,    /*Focal Length*/ \
							325.2110f, 243.7405f,    /*Principal Point*/ \
							-0.0829f,    /*Skew*/ \
							0.0937f, -0.0774f,   /*Radial Distortion*/ \
							0.0f, 0.0f   /*Tangential Distortion*/ }

//Arguments of right camera
#define CAMERA_ARGS_RIGHT { 386.5348f, 387.4860f,    /*Focal Length*/ \
							330.0357f, 232.8512f,    /*Principal Point*/ \
							-0.2563f,    /*Skew*/ \
							-0.0305f, 0.2411f,   /*Radial Distortion*/ \
							0.0f, 0.0f   /*Tangential Distortion*/ }

#define ROTATION_MATRIX		 (cv::Mat_<float>(3, 3) << 0.9997f, -0.0209f, 0.0093f, \
													0.0209f, 0.9998f, -0.0014f, \
													-0.0093f, 0.0016f, 1.0000f)

#define TRANSLATION_MATRIX	 (cv::Mat_<float>(3, 1) << -15.2247f, -0.0541f, -0.7736f)

const float cameraYawAngle = 20 * CV_PI / 180;

typedef struct
{
	//inner arguments
	float fx;
	float fy;
	float cx;
	float cy;
	float skew;

	//distortion arguments
	float k1;
	float k2;
	float p1;
	float p2;

} CameraArguments;
typedef struct
{
	float lineAngle[2] = { 0 };
	float distance[2] = { 0 };
	int index[2] = { 0 };
}houghLine;

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
	void imgProcess(void);
	void FindPillarCenter(void);
	//mode 0 由列逆序遍历 1 由列顺序遍历
	void FindFenseCorner(int fenseType,int mode);
	void FindLineCross(void);
	void FindLineEnd(void);
	cv::Point GetCrossPoint(cv::Point pt1, cv::Point pt2, cv::Point pt3, cv::Point pt4);
	cv::Point SetSeedPoint(void);
	void ClimbingMountain(void);
	int ClimbingMountainStageJudge(void);
	float GetDepth(cv::Point2f& pt, cv::Point3f& pt1);
	float GetyawAngle(cv::Point2f& pt1, cv::Point2f& pt2, int fenseType);
	cv::Point3f GetIrCorrdinate(cv::Point2f& pt);

private:
	//-- For color-aligned point cloud
	std::tuple<uint8_t, uint8_t, uint8_t> getColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	pPointCloud pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color);

	//-- For point cloud without color
	pPointCloud pointsToPointCloud(const rs2::points& points);

public:
	vector<vector<cv::Point>> contours;
	vector<vector<cv::Point>> filterContours;
	vector<cv::Vec4i> hierarchy;
	vector<float> groundCoeff;
	cv::Point2f center1 = cv::Point2f(0, 0);
	cv::Point2f center2 = cv::Point2f(0, 0);
	cv::Point2f fenseCorner = cv::Point2f(0, 0);
	cv::Point2f lineEnd = cv::Point2f(0, 0);
	cv::Point2f lineCross = cv::Point2f(0, 0);
	cv::Point3f center1In3D = cv::Point3f(0, 0, 0);
	cv::Point3f center2In3D = cv::Point3f(0, 0, 0);
	cv::Point3f fenseCornerIn3D = cv::Point3f(0, 0, 0);
	cv::Point3f lineEndIn3D = cv::Point3f(0, 0, 0);
	cv::Point3f lineCrossIn3D = cv::Point3f(0, 0, 0);

	cv::Point seedPoint;

	float nowXpos;
	float nowZpos;
	float angle;
	double angleAlpha;
	float robotYawAngle;
	int climbingMountainStage;

	float lineSlope;
	unsigned int status = CLIMBING_MOUNTAIN;

private:

	rs2::pointcloud  rs2Cloud;
	rs2::points      rs2Points;

	rs2::pipeline    pipe;
	rs2::config      cfg;

	rs2::frameset    frameSet;
	rs2::frameset    alignedFrameSet;

	rs2::align       align;

	pPointCloud		 cloudByRS2;

	//opencv module

	CameraArguments argsLeft;
	CameraArguments argsRight;
	cv::Mat intrinsicMatrixLeft;
	cv::Mat intrinsicMatrixRight;
	cv::Mat rotationMatrix;
	cv::Mat translationMatrix;


	vector<cv::Mat> channels;
	cv::Mat srcImage;
	cv::Mat channelR;
	cv::Mat channelH;
	cv::Mat channelS;
	cv::Mat channelB;
	cv::Mat maskImage;
	cv::Mat dstImage;
	cv::Mat dst2Image;
	cv::Mat firstPillarMask;

	// pcl::visualization::CloudViewer viewer;
};

#endif