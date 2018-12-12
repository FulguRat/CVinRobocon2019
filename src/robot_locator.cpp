#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
                               filteredCloud(new pointCloud),
                               verticalCloud(new pointCloud),
                               tmpCloud(new pointCloud),
                               dstCloud(new pointCloud),
                               indicesROI(new pcl::PointIndices),
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
    //-- Pass through filter
    pcl::PassThrough<pointType> pass;
    
    pass.setInputCloud(srcCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.6, 0.6);
	pass.filter(*filteredCloud);
    
	pass.setInputCloud(filteredCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 4.0);
	pass.filter(*filteredCloud);

    //-- Down sampling
    pcl::VoxelGrid<pointType> passVG;
    passVG.setInputCloud(filteredCloud);
    passVG.setLeafSize(0.02f, 0.02f, 0.02f);
    passVG.filter(*filteredCloud);

    //-- Remove outliers
    pcl::StatisticalOutlierRemoval<pointType> passSOR;
    passSOR.setInputCloud(filteredCloud);
    passSOR.setMeanK(50);
    passSOR.setStddevMulThresh(0.1);
    passSOR.filter(*filteredCloud);
}

void RobotLocator::locateBeforeDune()
{
    verticalCloud = removeHorizontalPlanes(filteredCloud); 

    //-- ROI indice
    pcl::PassThrough<pointType> pass;
    pass.setInputCloud(verticalCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.6, -0.2);
	pass.filter(indicesROI->indices);

    //-- Perform the plane segmentation with specific indices
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    pcl::SACSegmentation<pointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setIndices(indicesROI);

    seg.setInputCloud(verticalCloud);
    seg.segment(*inliers, *coefficients);

    //-- Extract the inliers
    pcl::ExtractIndices<pointType> extract;
    pcl::PointIndices::Ptr tmpIndices(new pcl::PointIndices);
    dstCloud->clear();

    extract.setInputCloud(verticalCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(tmpIndices->indices);
    
    //-- Change the color of the extracted part for debuging
    for (int i = 0; i < tmpIndices->indices.size(); i++)
    {
        verticalCloud->points[tmpIndices->indices[i]].r = 234;
        verticalCloud->points[tmpIndices->indices[i]].g = 67;
        verticalCloud->points[tmpIndices->indices[i]].b = 53;
    }

    // //-- Copy the rest part to tmpCloud
    // extract.setNegative(true);
    // extract.filter(*tmpCloud);

    srcViewer.showCloud(verticalCloud);
}

pPointCloud RobotLocator::removeHorizontalPlanes(pPointCloud cloud)
{
    verticalCloud->clear();
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

    //-- Remove Outliers
    pcl::StatisticalOutlierRemoval<pointType> passSOR;
    passSOR.setInputCloud(verticalCloud);
    passSOR.setMeanK(50);
    passSOR.setStddevMulThresh(0.1);
    passSOR.filter(*verticalCloud);

    return verticalCloud;
}

bool RobotLocator::isStoped(void)
{
    return srcViewer.wasStopped();
}