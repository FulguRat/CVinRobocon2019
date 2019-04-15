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
#include <mutex>
#include <queue>
#include <condition_variable>
#include <thread>
#include "mb_cuda/common/point_types.h"
#include "mb_cuda/io/pcl_thrust.h"
#include "mb_cuda/io/host_device.h"
#include "mb_cuda/filters/pass_through.h"
#include "mb_cuda/filters/voxel_grid.h"
#include "mb_cuda/filters/statistical_outlier_removal.h"
#include <librealsense2/rsutil.h>
extern int mode; 
extern int dbStatus;
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
#define PASSING_GRASSLAND_STAGE_1	7
#define PASSING_GRASSLAND_STAGE_2	8
#define UNDER_MOUNTAIN				9
#define BONE_RECOGNITION			10
#define CLIMBING_MOUNTAIN           11
#define REACH_MOUNTAIN				12
#define WAIT_STATUS                 13

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
const float duneLine2FenseDist = 840;
const float line2LeftDuneDist = 985;
const float line2RightDuneDist = 1015;
const float lineEnd2secondRopeDist = 1620;
const float fenseCorner2fenseDist = 1440;
const float lineCross2RopeDist = 860;
const float lineCross2FrontfenseDist = 850;
const float line2BesidefenseDist = 715;
const float lineEnd2BesidefenseDist = 730;
const float mountainDist = 1650;
const float pillar2leftFenseDist = 80;
const float pillarRadius = 80;
extern float frontDist;
extern float lateralDist;
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
	float intercept = 0;
	float lineSlop = 0;
	int index = 0;

	houghLine& operator= (houghLine& s1)
	{
		lineAngle = s1.lineAngle;
		distance = s1.distance;
		intercept = s1.intercept;
		lineSlop = s1.lineSlop;
		index = s1.index;
		return *this;
	}

};
typedef struct
{
	double xMin;
	double xMax;

	double zMin;
	double zMax;

} ObjectROI;

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
	void update(void);
	void imgProcess(void);
	void FillHoles(cv::Mat& src);
	//mode 0 ����������� 1 ����˳�����
	void FindFenseCorner(int fenseType,int mode);
	int FindPillarCenter (void);
	void FindBinaryThresh(void);
	void FindLineCross(cv::Mat& src, int mode);
	void FindLineEnd(void);
	cv::Point GetCrossPoint(cv::Point pt1, cv::Point pt2, cv::Point pt3, cv::Point pt4);
	cv::Point SetSeedPoint(cv::Mat& src);
	int ClimbingMountainStageJudge(void);
	bool MatchLine(vector<cv::Vec4i>& src, vector<cv::Vec4i>& dst, float angleThresh, float minDistThresh, float maxDistThresh);
	void FindHoughLineCross(void);
	void FindVerticalHoughLine(cv::Mat& src, int flag = 0);
	void FindLineCrossCenter(cv::Mat& src, int flag = 0);
	void FindHorizonalHoughLine(cv::Mat& src,int flag = 0);
	float GetDepth(cv::Point2f& pt, cv::Point3f& pt1);
	float GetyawAngle(const cv::Point2f& pt1, const cv::Point2f& pt2, int fenseType);
	cv::Point3f GetIrCorrdinate(cv::Point2f pt);
	double getThreshVal_Otsu_8u_mask(const cv::Mat src, const cv::Mat& mask);
	void threshold_with_mask(cv::Mat& src, cv::Mat& dst, cv::Mat& mask, int type);
	void mergeLine(vector<cv::Vec4i>& src, float angle = 5, float dist = 5);
	void drawHoughLines(vector<cv::Vec4i>& src, cv::Mat& draw);

private:
	//-- For color-aligned point cloud
	std::tuple<uint8_t, uint8_t, uint8_t> getColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	//pPointCloud pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color);

	//-- For point cloud without color
	void pointsToPointCloud(const rs2::points& points);
	pPointCloud pointsToPCLPointCloud(const rs2::points& points);
public:
	std::mutex mutex1;
	std::mutex mutex2;
	std::mutex mutex3;
	std::mutex mutex4;
	condition_variable cond;
	vector<cv::Vec4i> filterLine;
	vector<float> groundCoeff;
	queue<cv::Mat> srcImageQueue;
	queue<vector<float>> groundCoffQueue;
	queue<Eigen::Matrix3f> RotatedMatrix;
	cv::Point2f frontTarget = cv::Point2f(0, 0);
	cv::Point2f besideTarget = cv::Point2f(0, 0);
	cv::Point3f frontTargetIn3D = cv::Point3f(0, 0, 0);
	cv::Point3f besideTargetIn3D = cv::Point3f(0, 0, 0);

	cv::Point2f linePt1;
	cv::Point2f linePt2;
	cv::Point2f linePoint1;
	cv::Point2f linePoint2;
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

	cv::Point searchBeginPoint;
	cv::Point seedPoint;
	vector<cv::Point2f> linePoints;
	bool targetFoundFlag = false;
	bool farLineFlag = false;
	bool initFlag = true;
	bool pointCloudUpdateFlag = false;
	bool xkFlag = false;
	bool ifUpdate=false;
	bool lineFoundFlag;
	bool addDune=false;
	float lineSlop;
	float intercept;
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
	unsigned int status = BEFORE_GRASSLAND_STAGE_1;
	float addAngle;
	float roteAngle;

	ObjectROI               duneROI;
	ObjectROI               fenseROI;
	ObjectROI               groundROI;

	pPointCloud		 groundCloud;
	pPointCloud		 cloudByRS2;
	pPointCloud		 duneCloud;
	pPointCloud		 fenseCloud;

	cv::Mat grayImage;
private:

	uint16_t* data;

	rs2_intrinsics color_intrin;
	rs2_intrinsics depth_intrin;
	rs2_extrinsics depth2color_extrin;
	rs2_extrinsics color2depth_extrin;
	rs2::pointcloud  rs2Cloud;
	rs2::points      rs2Points;

	rs2::pipeline    pipe;
	rs2::config      cfg;

	rs2::frameset    frameSet;
	rs2::frameset    alignedFrameSet;

	rs2::align       align;

	

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

	
	cv::Mat LABImage;
	cv::Mat HSVImage;
	vector<cv::Mat> channels;
	cv::Mat srcImage;
	cv::Mat maskImage;
	cv::Mat dstImage;
	cv::Mat dst2Image;
	cv::Mat firstPillarMask;

	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	// pcl::visualization::CloudViewer viewer;
};

#endif
