#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
                               filteredCloud(new pointCloud),
                               verticalCloud(new pointCloud),
                               dstCloud(new pointCloud),
                               indicesROI(new pcl::PointIndices),
                               groundCoefficients(new pcl::ModelCoefficients),
					           srcViewer("Src Viewer")
{
    leftFenseROI = { -0.6/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 2.5/*zMax*/ };
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
        pass.setFilterLimits(0.0, 3.0);
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
	pass.setFilterLimits(-1.0, 1.0);
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

pPointCloud RobotLocator::rotatePointCloudToHorizontal(pPointCloud cloud)
{
    //-- Define the rotate angle about x-axis
    double angleAlpha = atan(-groundCoefficients->values[2] / groundCoefficients->values[1]);
    
    //-- Define the rotate transform
    Eigen::Affine3f rotateToXZPlane = Eigen::Affine3f::Identity();
    rotateToXZPlane.rotate(Eigen::AngleAxisf(angleAlpha, Eigen::Vector3f::UnitX()));

    //-- Apply transform
    pcl::transformPointCloud(*cloud, *cloud, rotateToXZPlane);

    //-- Update groundCoefficients
    Vector3d vecNormal(groundCoefficients->values[0], groundCoefficients->values[1], groundCoefficients->values[2]);

    groundCoefficients->values[0] = groundCoefficients->values[0];
    groundCoefficients->values[1] = vecNormal.norm(); /* This value is equivalent to 1 */
    groundCoefficients->values[2] = 0.0f;
    groundCoefficients->values[3] = groundCoefficients->values[3];

    // cout << "Ground coefficients: " << groundCoefficients->values[0] << " " 
    //                                 << groundCoefficients->values[1] << " "
    //                                 << groundCoefficients->values[2] << " " 
    //                                 << groundCoefficients->values[3] << endl;

    return cloud;
}

pPointCloud RobotLocator::removeHorizontalPlanes(pPointCloud cloud)
{
    verticalCloud->clear();
    dstCloud->clear();
    pointType tmpPoint;

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

		if (angleCosine < 0.90f)
		{
            verticalCloud->points.push_back(cloud->points[i]);
		}
	}

    //-- Remove Outliers
    pcl::StatisticalOutlierRemoval<pointType> passSOR;
    passSOR.setInputCloud(verticalCloud);
    passSOR.setMeanK(50);
    passSOR.setStddevMulThresh(0.1);
    passSOR.filter(*verticalCloud);

    //-- Copy points from verticalCloud to dstCloud
    for (size_t i = 0; i < verticalCloud->points.size(); i++)
    {
        tmpPoint = verticalCloud->points[i];
        tmpPoint.r = 66;
        tmpPoint.g = 133;
        tmpPoint.b = 244;
        dstCloud->points.push_back(tmpPoint);
    }

    return verticalCloud;
}

pPointCloud RobotLocator::extractVerticalCloud(pPointCloud cloud)
{
    //-- Extract ground coefficients with anti-interference function
    extractGroundCoeff(cloud);

    //-- Rotate the point cloud to horizontal
    rotatePointCloudToHorizontal(cloud);

    //-- Remove all horizontal planes
    removeHorizontalPlanes(cloud);

    return verticalCloud;
}

pcl::PointIndices::Ptr RobotLocator::getPlaneIndicesWithinROI(pPointCloud cloud, ObjectROI roi)
{   
    pcl::PointIndices::Ptr resultIndices(new pcl::PointIndices);

    //-- Get point cloud indices inside given ROI
    pcl::PassThrough<pointType> pass;
    pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(leftFenseROI.xMin, leftFenseROI.xMax);
	pass.filter(indicesROI->indices);

    pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(leftFenseROI.zMin, leftFenseROI.zMax);
    pass.setIndices(indicesROI);
	pass.filter(indicesROI->indices);

    //-- Plane model segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
    seg.setIndices(indicesROI);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

    //-- TODO: If tracking failed
    if (coefficients->values.size() == 0) { return resultIndices; }

    //-- Vector of plane normal and every point on the plane
    Vector3d vecNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	Vector3d vecPoint(0, 0, 0);

    //-- Plane normal estimating
	pcl::NormalEstimation<pointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pointType>::Ptr tree(new pcl::search::KdTree<pointType>());
	ne.setSearchMethod(tree);
	
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.04);
	ne.compute(*normal);

    //-- Compare point normal and position, extract indices of points meeting the criteria
	for (size_t i = 0; i < inliers->indices.size(); i++)
	{
		vecPoint[0] = normal->points[inliers->indices[i]].normal_x;
		vecPoint[1] = normal->points[inliers->indices[i]].normal_y;
		vecPoint[2] = normal->points[inliers->indices[i]].normal_z;

		double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));
        double distanceToPlane = abs(coefficients->values[0] * cloud->points[inliers->indices[i]].x + 
                                     coefficients->values[1] * cloud->points[inliers->indices[i]].y + 
                                     coefficients->values[2] * cloud->points[inliers->indices[i]].z + 
                                     coefficients->values[3]) / vecNormal.norm();

		if (angleCosine > 0.80f && distanceToPlane < 0.1f)
		{
            resultIndices->indices.push_back(inliers->indices[i]);
		}
	}

    return resultIndices;
}

ObjectROI RobotLocator::updateObjectROI(pPointCloud cloud, pcl::PointIndices::Ptr indices)
{
    ObjectROI objROI;
    Eigen::Vector4f minVector, maxVector;

    pcl::getMinMax3D(*cloud, *indices, minVector, maxVector);

    objROI.xMin = minVector[0] - 0.3f;
    objROI.xMax = maxVector[0] + 0.3f;
    
    objROI.zMin = minVector[2] - 0.3f;
    objROI.zMax = maxVector[2] + 0.3f;

    return objROI;
}

void RobotLocator::locateBeforeDune(void)
{
    extractVerticalCloud(filteredCloud); 

    // srcViewer.showCloud(verticalCloud);

    //-- Perform the plane segmentation with specific indices
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    inliers = getPlaneIndicesWithinROI(verticalCloud, leftFenseROI);
    leftFenseROI = updateObjectROI(verticalCloud, inliers);

    cout << "xMin_  " << leftFenseROI.xMin << "  xMax_  " << leftFenseROI.xMax << 
            "  zMin_  " << leftFenseROI.zMin << "  zMax_  " << leftFenseROI.zMax << endl;
    
    //-- Change the color of the extracted part for debuging
    for (int i = 0; i < inliers->indices.size(); i++)
    {
        dstCloud->points[inliers->indices[i]].r = 234;
        dstCloud->points[inliers->indices[i]].g = 67;
        dstCloud->points[inliers->indices[i]].b = 53;
    }

    //-- Extract indices for the rest part
    pcl::ExtractIndices<pointType> extract;

    extract.setInputCloud(verticalCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(indicesROI->indices);

    pcl::SACSegmentation<pointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
    seg.setIndices(indicesROI);

    seg.setInputCloud(verticalCloud);
	seg.segment(*inliers, *coefficients);

    // double lineA = coefficients->values[0] - groundCoefficients->values[0];
    // double lineB = coefficients->values[1] - groundCoefficients->values[1];
    // double lineC = coefficients->values[2] - groundCoefficients->values[2];

    // lineA /= lineA;
    // lineB /= lineA;
    // lineC /= lineA;

    // cout << "Line coefficients: " << lineA << " " << lineB << " " << lineC << endl;
    
    //-- Change the color of the extracted part for debuging
    for (int i = 0; i < inliers->indices.size(); i++)
    {
        dstCloud->points[inliers->indices[i]].r = 52;
        dstCloud->points[inliers->indices[i]].g = 168;
        dstCloud->points[inliers->indices[i]].b = 83;
    }

    srcViewer.showCloud(dstCloud);
}

bool RobotLocator::isStoped(void)
{
    return srcViewer.wasStopped();
}