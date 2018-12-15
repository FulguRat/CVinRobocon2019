#define _CRT_SECURE_NO_WARNINGS

#ifndef ROBOT_LOCATOR_H_
#define ROBOT_LOCATOR_H_

#define STARTUP_INITIAL     0
#define BEFORE_DUNE         1
#define BEFORE_GRASSLAND    2

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>
#include <Eigen/Dense>
#include "act_d435.h"

using namespace Eigen;

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

    pPointCloud removeHorizontalPlanes(pPointCloud cloud);

    pcl::ModelCoefficients::Ptr extractGroundCoeff(pPointCloud cloud);

    void locateBeforeDune(void);

    Vector3i getCloudAverageColor(pPointCloud cloud, pcl::PointIndices::Ptr indices);

    bool isStoped(void);

    inline pPointCloud getSrcCloud(void) { return srcCloud; }
    inline pPointCloud getFilteredCloud(void) { return filteredCloud; }

public:
    unsigned int status;

private:
    ActD435*        thisD435;

    pPointCloud		srcCloud;
	pPointCloud     filteredCloud;
    pPointCloud     verticalCloud;
	pPointCloud     dstCloud;

    pcl::ModelCoefficients::Ptr groundCoefficients;

    pcl::visualization::CloudViewer srcViewer;

    pcl::PointIndices::Ptr indicesROI;
};

#endif
