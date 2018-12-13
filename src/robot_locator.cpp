#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
                               filteredCloud(new pointCloud),
                               verticalCloud(new pointCloud),
                               tmpCloud(new pointCloud),
                               dstCloud(new pointCloud),
                               indicesROI(new pcl::PointIndices),
                               groundCoefficients(new pcl::ModelCoefficients),
					           srcViewer("Src Viewer")
{
    
}

RobotLocator::~RobotLocator()
{

}

void RobotLocator::init(ActD435& d435)
{
    cout << "Initializing locator..." << endl;

    //-- Set input device
    thisD435 = &d435;
    
    //-- Drop several frames for stable point cloud
    for (int i = 0; i < 3; i++)
    {
        thisD435->update();
    }

    //-- Initialize ground coefficients
    cout << "Initializing ground coefficients..." << endl;

    groundCoefficients->values.clear();
    groundCoefficients->values.push_back(0.0f);
    groundCoefficients->values.push_back(0.0f);
    groundCoefficients->values.push_back(0.0f);
    groundCoefficients->values.push_back(0.0f);

    const int cycleNum = 10;
    for (int i = 0; i < cycleNum; i++)
    {
        srcCloud = thisD435->update();

        pcl::PassThrough<pointType> pass; 
        pass.setInputCloud(srcCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 4.0);
        pass.filter(*srcCloud);

        pcl::VoxelGrid<pointType> passVG;
        passVG.setInputCloud(srcCloud);
        passVG.setLeafSize(0.02f, 0.02f, 0.02f);
        passVG.filter(*srcCloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pointType> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(srcCloud);
        seg.segment(*inliers, *coefficients);

        groundCoefficients->values[0] += coefficients->values[0];
        groundCoefficients->values[1] += coefficients->values[1];
        groundCoefficients->values[2] += coefficients->values[2];
        groundCoefficients->values[3] += coefficients->values[3];

        cout << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << endl;
    }

    groundCoefficients->values[0] /= cycleNum;
    groundCoefficients->values[1] /= cycleNum;
    groundCoefficients->values[2] /= cycleNum;
    groundCoefficients->values[3] /= cycleNum;

    cout << "Ground coefficients: " << groundCoefficients->values[0] << " " 
                                      << groundCoefficients->values[1] << " "
                                      << groundCoefficients->values[2] << " " 
                                      << groundCoefficients->values[3] << endl;

    cout << "Done initialization." << endl;
}

pPointCloud RobotLocator::updateCloud(void)
{
    //-- copy the pointer to srcCloud
    srcCloud = thisD435->update();
    return srcCloud;
}

void RobotLocator::preProcess(void)
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

pcl::ModelCoefficients::Ptr RobotLocator::extractGroundCoeff(pPointCloud cloud)
{
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

    //-- If plane coefficients changed a little, refresh it. else not
    Vector3d vecNormalLast(groundCoefficients->values[0], groundCoefficients->values[1], groundCoefficients->values[2]);
	Vector3d vecNormalThis(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    double angleCosine = abs(vecNormalLast.dot(vecNormalThis) / (vecNormalLast.norm() * vecNormalThis.norm()));

    double originDistanceLast = groundCoefficients->values[3] / vecNormalLast.norm();
    double originDistanceThis = coefficients->values[3] / vecNormalThis.norm();
    double distDifference = abs(originDistanceThis - originDistanceLast);

    if (angleCosine > 0.8f && distDifference < 0.04f)
    {
        groundCoefficients = coefficients;
    }

    return groundCoefficients;
}

pPointCloud RobotLocator::removeHorizontalPlanes(pPointCloud cloud)
{
    verticalCloud->clear();
    pointType tmpPoint;

    //-- Extract ground coefficients with anti-interference function
    extractGroundCoeff(cloud);

    //-- Vector of plane normal and every point on the plane
    Vector3d vecNormal(groundCoefficients->values[0], groundCoefficients->values[1], groundCoefficients->values[2]);
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

		double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));

		if (angleCosine > 0.80f)
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

void RobotLocator::locateBeforeDune(void)
{
    verticalCloud = removeHorizontalPlanes(filteredCloud); 

    // srcViewer.showCloud(verticalCloud);

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

    //-- Extract indices for the rest part
    extract.setNegative(true);
    extract.filter(indicesROI->indices);

    //-- Perform the plane segmentation for the rest part
    seg.setDistanceThreshold(0.01);
    seg.setIndices(indicesROI);
    seg.setInputCloud(verticalCloud);
    seg.segment(*inliers, *coefficients);

    //-- Extract the inliers
    extract.setInputCloud(verticalCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(tmpIndices->indices);
    
    //-- Change the color of the extracted part for debuging
    for (int i = 0; i < tmpIndices->indices.size(); i++)
    {
        verticalCloud->points[tmpIndices->indices[i]].r = 52;
        verticalCloud->points[tmpIndices->indices[i]].g = 168;
        verticalCloud->points[tmpIndices->indices[i]].b = 83;
    }

    srcViewer.showCloud(verticalCloud);
}

bool RobotLocator::isStoped(void)
{
    return srcViewer.wasStopped();
}