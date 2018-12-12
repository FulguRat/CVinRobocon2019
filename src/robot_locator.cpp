#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
                               backgroundCloud(new pointCloud),
                               filteredCloud(new pointCloud),
                               verticalCloud(new pointCloud),
                               tmpCloud(new pointCloud),
                               dstCloud(new pointCloud),
                               indices(new pcl::PointIndices),
					           srcViewer("Src Viewer")
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
    // z
	pass.setInputCloud(filteredCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 4.0);
	pass.filter(*filteredCloud);

    //-- Down Sampling
    pcl::VoxelGrid<pointType> passVG;
    passVG.setInputCloud(filteredCloud);
    passVG.setLeafSize(0.02f, 0.02f, 0.02f);
    passVG.filter(*filteredCloud);

    //-- Remove Outliers
    pcl::StatisticalOutlierRemoval<pointType> passSOR;
    passSOR.setInputCloud(filteredCloud);
    passSOR.setMeanK(50);
    passSOR.setStddevMulThresh(0.1);
    passSOR.filter(*filteredCloud);

    // x indice
    pass.setInputCloud(filteredCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.6, -0.4);
	pass.filter(indices->indices);

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
    seg.setIndices(indices);

    //-- Create the filtering object
    pcl::ExtractIndices<pointType> extract;
    dstCloud->clear();

    int i = 0; // Test var
    int srcPointNum = (int)filteredCloud->points.size();
    
    //-- Until most of the original cloud have been segmented
    while (filteredCloud->points.size() > 0.05 * srcPointNum && i <= 0)
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
}

pPointCloud RobotLocator::removeHorizontalPlanes(pPointCloud cloud)
{
    verticalCloud->points.clear();
    pointType tmpPoint;

    //-- Plane model segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

    //-- Vector of plane normal and every point on the plane
    Vector3d vecNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	Vector3d vecPoint(0, 0, 0);

    //-- Plane normal estimating
	pcl::NormalEstimation<pointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pointType>::Ptr tree(new pcl::search::KdTree<pointType>());
	ne.setSearchMethod(tree);
	
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.03);
	ne.compute(*normal);

    //-- Compare point normal and plane normal, remove every point on a horizontal plane
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		vecPoint[0] = normal->points[i].normal_x;
		vecPoint[1] = normal->points[i].normal_y;
		vecPoint[2] = normal->points[i].normal_z;

		double angleCosine = (vecNormal[0] * vecPoint[0] + vecNormal[1] * vecPoint[1] + vecNormal[2] * vecPoint[2]) / 
			sqrt(pow(vecNormal[0], 2) + pow(vecNormal[1], 2) + pow(vecNormal[2], 2)) * 
            sqrt(pow(vecPoint[0], 2) + pow(vecPoint[1], 2) + pow(vecPoint[2], 2));

		if (abs(angleCosine) > 0.80f)
		{
			cloud->points[i].r = 66;
			cloud->points[i].g = 133;
			cloud->points[i].b = 244;
		}
        else
        {
            tmpPoint = cloud->points[i];
            tmpPoint.r = 66;
			tmpPoint.g = 133;
			tmpPoint.b = 244;
            verticalCloud->points.push_back(tmpPoint);
        }
	}

    srcViewer.showCloud(verticalCloud);

    return verticalCloud;
}

bool RobotLocator::isStoped(void)
{
    return srcViewer.wasStopped();
}