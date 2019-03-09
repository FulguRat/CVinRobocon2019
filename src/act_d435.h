#define _CRT_SECURE_NO_WARNINGS

#ifndef ACT_D435_H_
#define ACT_D435_H_

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

extern int mode;

#define DEBUG
#define LEFT_MODE		0
#define RIGHT_MODE		1

#define HORIZONAL_FENSE			0
#define VERTICAL_FENSE				1

#define STARTUP_INITIAL				0
#define BEFORE_DUNE_STAGE_1			1
#define BEFORE_DUNE_STAGE_2			2
#define BEFORE_DUNE_STAGE_3			3
#define PASSING_DUNE                4
#define BEFORE_GRASSLAND_STAGE_1	5
#define BEFORE_GRASSLAND_STAGE_2	6
#define UNDER_MOUNTAIN				7
#define BONE_RECOGNITION			8
#define CLIMBING_MOUNTAIN           9
#define REACH_MOUNTAIN				10

#define CAMERA_ARGS_LEFT  { 619.817, 619.787,    /*Focal Length*/ \
							330.33,  242.407,    /*Principal Point*/ \
							-0.0829f,    /*Skew*/ \
							0.0937f, -0.0774f,   /*Radial Distortion*/ \
							0.0f, 0.0f   /*Tangential Distortion*/ }

//Arguments of right camera
#define CAMERA_ARGS_RIGHT { 385.72f,  385.72f,    /*Focal Length*/ \
							324.6954f, 234.4622f,    /*Principal Point*/ \
							-0.2563f,    /*Skew*/ \
							-0.0305f, 0.2411f,   /*Radial Distortion*/ \
							0.0f, 0.0f   /*Tangential Distortion*/ }

#define ROTATION_MATRIX		 (cv::Mat_<float>(3, 3) << 1.0000f, -0.00026f, -0.00503f, \
													0.00026f, 1.0000f, -0.0006f, \
													0.00503f, 0.0006f, 1.0000f)

#define TRANSLATION_MATRIX	 (cv::Mat_<float>(3, 1) << 64.5776f, -0.0505f, 0.0003f)

enum roiFlag {
	midRoi, leftRoi, rightRoi
};

extern enum roiFlag colorFrameRoi;
const float fenseCorner2fenseDist = 1440;
const float lineCross2RopeDist = 860;
const float lineCross2FrontfenseDist = 850;
const float line2BesidefenseDist = 715;
const float lineEnd2BesidefenseDist = 730;
const float mountainDist = 1650;
const float pillar2leftFenseDist = 80;
const float pillarRadius = 80;

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
struct houghLine
{
	float lineAngle = 0;
	float distance = 0;
	float lineSlop = 0;
	int index = 0;

	houghLine& operator= (houghLine& s1)
	{
		lineAngle = s1.lineAngle;
		distance = s1.distance;
		lineSlop = s1.lineSlop;
		index = s1.index;
		return *this;
	}

};

typedef struct houghLine HoughLine;

using namespace std;
using namespace rs2;
using namespace std::chrono;

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
	void FillHoles(cv::Mat& src);
	//mode 0 ����������� 1 ����˳�����
	void FindFenseCorner(int fenseType,int mode);
	int FindPillarCenter (void);
	void FindBinaryThresh(void);
	void FindLineCross(void);
	void FindLineEnd(void);
	cv::Point GetCrossPoint(cv::Point pt1, cv::Point pt2, cv::Point pt3, cv::Point pt4);
	cv::Point SetSeedPoint(void);
	int ClimbingMountainStageJudge(void);
	bool MatchLine(vector<cv::Vec4i>& src, vector<cv::Vec4i>& dst, float angleThresh, float minDistThresh, float maxDistThresh);
	void FindHoughLineCross(void);
	void FindLineCrossCenter(float angleThresh, float minDistThresh, float maxDistThresh);
	void FindHorizonalHoughLine(cv::Mat& src);
	float GetDepth(cv::Point2f& pt, cv::Point3f& pt1);
	float GetyawAngle(const cv::Point2f& pt1, const cv::Point2f& pt2, int fenseType);
	cv::Point3f GetIrCorrdinate(cv::Point2f pt);

private:
	//-- For color-aligned point cloud
	std::tuple<uint8_t, uint8_t, uint8_t> getColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	pPointCloud pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color);

	//-- For point cloud without color
	pPointCloud pointsToPointCloud(const rs2::points& points);

public:
	vector<cv::Vec4i> filterLine;
	vector<float> groundCoeff;
	Eigen::Matrix3f RotatedMatrix;
	cv::Point2f pillarLeftUpPt;
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
	float cameraYawAngle;
	double angleAlpha;
	int climbingMountainStage = 0;
	float pillarPixWidth = 0;
	int houghlineThresh = 50;
	int pillarStatus;
	float pillarHeight;
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



	cv::Mat channelB;
	cv::Mat channelG;
	cv::Mat channelR;

	cv::Mat channelH;
	cv::Mat channelS;
	
	cv::Mat channelL;
	cv::Mat channelA;

	cv::Mat grayImage;
	vector<cv::Mat> channels;
	cv::Mat srcImage;
	cv::Mat maskImage;
	cv::Mat dstImage;
	cv::Mat dst2Image;
	cv::Mat firstPillarMask;

	// pcl::visualization::CloudViewer viewer;
};

#endif
