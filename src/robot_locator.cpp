#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
                               backgroundCloud(new pointCloud),
                               filteredCloud(new pointCloud),
                               tmpCloud(new pointCloud),
                               dstCloud(new pointCloud),
					           viewer("Cloud Viewer")
{

}

RobotLocator::~RobotLocator()
{

}

pPointCloud RobotLocator::setInputCloud(pPointCloud cloud)
{
    //-- copy the pointer to srcCloud
    srcCloud = cloud;
    return srcCloud;
}

void RobotLocator::preProcess()
{
    //-- Pass Through Filter
    pcl::PassThrough<pointType> pass;
    // x
    pass.setInputCloud(srcCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.6, 0.6);
	pass.filter(*filteredCloud);
    // y
    pass.setInputCloud(filteredCloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0.0, 0.6);
	pass.filter(*filteredCloud);
    // z
	pass.setInputCloud(filteredCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 3.0);
	pass.filter(*filteredCloud);

    //-- Down Sampling
    pcl::VoxelGrid<pointType> passVG;
    passVG.setInputCloud(filteredCloud);
    passVG.setLeafSize(0.01f, 0.01f, 0.01f);
    passVG.filter(*filteredCloud);

    //-- Remove Outliers
    pcl::StatisticalOutlierRemoval<pointType> passSOR;
    passSOR.setInputCloud(filteredCloud);
    passSOR.setMeanK(50);
    passSOR.setStddevMulThresh(0.1);
    passSOR.filter(*filteredCloud);

    copyPointCloud(*filteredCloud, *backgroundCloud);
}

void RobotLocator::locateBeforeDune()
{
    //-- Create the segmentation object
    pcl::SACSegmentation<pointType> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    //-- If there are too many outliers
    seg.setOptimizeCoefficients(true);
    //-- Mandatory conditions
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    //-- Create the filtering object
    pcl::ExtractIndices<pointType> extract;
    dstCloud->clear();

    int i = 0; // Test var
    int srcPointNum = (int)filteredCloud->points.size();
    
    //-- Until most of the original cloud have been segmented
    while (filteredCloud->points.size() > 0.05 * srcPointNum && i <= 1)
    {
        //-- Segment the largest planar component from the remaining cloud
        seg.setInputCloud(filteredCloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        //-- Extract the inliers
        extract.setInputCloud(filteredCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*tmpCloud);
        copyPointCloud(*tmpCloud, *dstCloud);

        //-- Copy the rest part to filteredCloud
        extract.setNegative(true);
        extract.filter(*tmpCloud);
        filteredCloud.swap(tmpCloud);

        i++;
    }

    //-- Change the color of the extracted part for debuging
    for (int i = 0; i < dstCloud->points.size(); i++)
    {
        dstCloud->points[i].r = 255;
        dstCloud->points[i].g = 0;
        dstCloud->points[i].b = 0;
    }
    *backgroundCloud += *dstCloud;

	viewer.showCloud(backgroundCloud);
}

bool RobotLocator::isStoped(void)
{
    return viewer.wasStopped();
}