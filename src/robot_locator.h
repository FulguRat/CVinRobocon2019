#define _CRT_SECURE_NO_WARNINGS

#ifndef ROBOT_LOCATOR_H_
#define ROBOT_LOCATOR_H_

#define STARTUP_INITIAL            0
#define BEFORE_DUNE_STAGE_1        1
#define BEFORE_DUNE_STAGE_2        2
#define BEFORE_DUNE_STAGE_3        3
#define PASSING_DUNE               4
#define BEFORE_GRASSLAND_STAGE_1   5
#define BEFORE_GRASSLAND_STAGE_2   6
#define PASSING_GRASSLAND          7

#define PI                         3.1415926
#define STD_ROI {-0.6f, 0.6f, 0.0f, 2.5f}

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Dense>
#include <cmath>
#include "act_d435.h"

using namespace Eigen;

//-- ROI of an object
typedef struct
{
	double xMin;
	double xMax;

	double zMin;
	double zMax;

} ObjectROI;

//-- Algorithm implementation for robot locating
class RobotLocator
{
public:
	RobotLocator();
	RobotLocator(const RobotLocator&) = delete;
	RobotLocator& operator=(const RobotLocator&) = delete;
	~RobotLocator();

	void init(ActD435& d435);

	pPointCloud updateCloud(void);

	void preProcess(void);

	void extractPlaneWithinROI(pPointCloud cloud, ObjectROI roi,
		pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coefficients);

	pcl::ModelCoefficients::Ptr extractGroundCoeff(pPointCloud cloud);

	pPointCloud rotatePointCloudToHorizontal(pPointCloud cloud);

	pPointCloud removeHorizontalPlane(pPointCloud cloud, bool onlyGround = false);

	pPointCloud extractVerticalCloud(pPointCloud cloud);

	ObjectROI updateObjectROI(pPointCloud cloud, pcl::PointIndices::Ptr indices,
		double xMinus, double xPlus, double zMinus, double zPlus);

	void locateBeforeDuneStage1(void);
	void locateBeforeDuneStage2(void);
	void locateBeforeDuneStage3(void);

	void locatePassingDune(void);

	void locateBeforeGrasslandStage1(void);
	void locateBeforeGrasslandStage2(void);

	bool isStoped(void);

	inline pPointCloud getSrcCloud(void) { return srcCloud; }
	inline pPointCloud getFilteredCloud(void) { return filteredCloud; }

public:
	unsigned int status;
	unsigned int nextStatusCounter;

private:
	ActD435*        thisD435;

	pPointCloud		srcCloud;
	pPointCloud     filteredCloud;
	pPointCloud     verticalCloud;
	pPointCloud     dstCloud;

	pcl::ModelCoefficients::Ptr groundCoeff;
	pcl::ModelCoefficients::Ptr groundCoeffRotated;

	pcl::PointIndices::Ptr  indicesROI;
	ObjectROI               leftFenseROI;
	ObjectROI               duneROI;
	ObjectROI               frontFenseROI;

	float leftFenseDist;
	float duneDist;

	pcl::visualization::PCLVisualizer::Ptr dstViewer;
};

#endif
