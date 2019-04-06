#define _CRT_SECURE_NO_WARNINGS

#ifndef ROBOT_LOCATOR_H_
#define ROBOT_LOCATOR_H_

//left is zero, right is one 

#define STD_ROI {-0.6f, 0.6f, 0.0f, 2.5f}

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
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

typedef struct
{
	float x;
	float y;
}robotLoc;
//-- Algorithm implementation for robot locating
class RobotLocator
{
public:
	RobotLocator();
	RobotLocator(const RobotLocator&) = delete;
	RobotLocator& operator=(const RobotLocator&) = delete;
	~RobotLocator();

	void init(ActD435& d435);

	void updateCloud(void);
	void updateSrcCloud(void);
	void cloudPointPreprocess(void);
	void preProcess(void);

	void extractPlaneWithinROI(pPointCloud cloud, ObjectROI roi,
		pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coefficients);

	bool extractGroundCoeff(pPointCloud cloud);

	pPointCloud rotatePointCloudToHorizontal(pPointCloud cloud);
	pPointCloud removeHorizontalPlane(pPointCloud cloud, bool onlyGround = false);

	pPointCloud extractVerticalCloud(pPointCloud cloud);

	ObjectROI updateObjectROI(pPointCloud cloud, pcl::PointIndices::Ptr indices,
		double xMinus, double xPlus, double zMinus, double zPlus, bool ifx, bool ifz, ObjectROI beforeobjROI);

	double calculateDistance(pcl::ModelCoefficients::Ptr groundcoefficients, pcl::ModelCoefficients::Ptr planecoefficients);

	void locateBeforeDuneStage1(void);
	void locateBeforeDuneStage2(void);
	void locateBeforeDuneStage3(void);

	void locatePassingDune(void);

    void locateBeforeGrasslandStage1(void);
	void locateBeforeGrasslandStage2(void);
	void locatePassingGrasslandStage1(void);
	void locatePassingGrasslandStage2(void);
	void locateUnderMountain(void);
	void locateClimbingMountain(void);
	void locateReachingMountain(void);
	bool isStoped(void);

	inline pPointCloud getSrcCloud(void) { return srcCloud; }
	inline pPointCloud getFilteredCloud(void) { return filteredCloud; }

public:
    unsigned int status;
    unsigned int nextStatusCounter;
	unsigned int roiChangeCounter;
	double diatancemeasurement;
	bool segmentStatus = true;

	
private:
	ActD435*        thisD435;
	robotLoc        robotLocNow;
	robotLoc        robotLocLast;
	pPointCloud		srcCloud;
	pPointCloud     filteredCloud;
	pPointCloud     verticalCloud;
	pPointCloud     dstCloud;
	pPointCloud		groundCloud;
	pPointCloud		forGroundCloud;

	pcl::ModelCoefficients::Ptr groundCoeff;
	pcl::ModelCoefficients::Ptr groundCoeffRotated;

    pcl::PointIndices::Ptr  indicesROI;
	ObjectROI               leftFenseROImin;
	ObjectROI               leftFenseROImax;
    ObjectROI               duneROI;
    ObjectROI               frontFenseROI;
	ObjectROI				grasslandFenseROI;
	ObjectROI				groundROI;

	float besideFenseDist;
    float duneDist;
	float frontFenseDist;
	float firstRopeDist;
	float secondRopeDist;
	float grassFenseDist;
	float peakDist;
	float besidebarrierDist;
	float frontbarrierDist;


	float angle;


	pcl::visualization::PCLVisualizer::Ptr dstViewer;
};

#endif
