#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
filteredCloud(new pointCloud),
verticalCloud(new pointCloud),
dstCloud(new pointCloud),
groundCloud(new pointCloud),
forGroundCloud(new pointCloud),
indicesROI(new pcl::PointIndices),
groundCoeff(new pcl::ModelCoefficients),
groundCoeffRotated(new pcl::ModelCoefficients),
dstViewer(new pcl::visualization::PCLVisualizer("Advanced Viewer"))
{
	leftFenseROImax = { -0.9/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
	leftFenseROImin = { -0.9/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 1.0/*zMax*/ };
	duneROI = { -0.7/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
	// frontFenseROI = { -1.3/*xMin*/,  0.3/*xMax*/, 1.2/*zMin*/, 2.1/*zMax*/ };
	frontFenseROI = { -0.7/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 1.7/*zMax*/ };
	grasslandFenseROI = { -0.6/*xMin*/,  0.4/*xMax*/, 0.0/*zMin*/, 3.0/*zMax*/ };

	dstViewer->setBackgroundColor(0.259, 0.522, 0.957);
    dstViewer->addPointCloud<pointType>(dstCloud, "Destination Cloud");
    dstViewer->addCoordinateSystem(0.2, "view point");
    dstViewer->initCameraParameters();
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

	groundCoeff->values.clear();
	groundCoeff->values.push_back(0.0f);
	groundCoeff->values.push_back(0.0f);
	groundCoeff->values.push_back(0.0f);
	groundCoeff->values.push_back(0.0f);

	groundCoeffRotated->values.clear();
	groundCoeffRotated->values.push_back(0.0f);
	groundCoeffRotated->values.push_back(0.0f);
	groundCoeffRotated->values.push_back(0.0f);
	groundCoeffRotated->values.push_back(0.0f);

	const int cycleNum = 10;
	for (int i = 0; i < cycleNum; i++)
	{
		sourceThrust = thisD435->update();
		mb_cuda::thrust_to_pcl(sourceThrust, srcCloud);

		pcl::PassThrough<pointType> pass;
		pass.setInputCloud(srcCloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0f, 1.5f);
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

		groundCoeff->values[0] += coefficients->values[0];
		groundCoeff->values[1] += coefficients->values[1];
		groundCoeff->values[2] += coefficients->values[2];
		groundCoeff->values[3] += coefficients->values[3];

		cout << "Model coefficients: " << coefficients->values[0] << " "
			<< coefficients->values[1] << " "
			<< coefficients->values[2] << " "
			<< coefficients->values[3] << endl;
	}

	groundCoeff->values[0] /= cycleNum;
	groundCoeff->values[1] /= cycleNum;
	groundCoeff->values[2] /= cycleNum;
	groundCoeff->values[3] /= cycleNum;

	cout << "Ground coefficients: " << groundCoeff->values[0] << " "
		<< groundCoeff->values[1] << " "
		<< groundCoeff->values[2] << " "
		<< groundCoeff->values[3] << endl;

	cout << "Done initialization." << endl;
}

mb_cuda::thrustCloudT RobotLocator::updateCloud(void)
{
	//-- copy the pointer to srcCloud
	sourceThrust = thisD435->update();
	return sourceThrust;
}

void RobotLocator::preProcess(void)
{
	//-- Pass through filter
	thrust::device_vector<mb_cuda::PointXYZRGB> device_cloud;
	mb_cuda::host_to_device(sourceThrust, device_cloud);

	thrust::device_vector<mb_cuda::PointXYZRGB> d_filtered_cloud;

	if (status == PASSING_DUNE)
		groundROI = { -1.5,-0.5,0.5,1.0 };
	else
	{
		groundROI = { -0.5,0.5,0,1.0 };
	}

	pcl::PassThrough<pointType> pass;
	if (status <= 4)
	{
		mb_cuda::pass_through_filter(device_cloud, d_filtered_cloud, 'z', 0.0f, 2.0f);

		mb_cuda::pass_through_filter(d_filtered_cloud, d_filtered_cloud, 'x', -1.0f, 1.0f);
	}
	else if (status == CLIMBING_MOUNTAIN)
	{
		mb_cuda::pass_through_filter(device_cloud, d_filtered_cloud, 'y', 0.0f, 1.5f);

		mb_cuda::pass_through_filter(d_filtered_cloud, d_filtered_cloud, 'x', -2.0f, -0.2f);

		mb_cuda::pass_through_filter(d_filtered_cloud, d_filtered_cloud, 'z', 0.0f, 2.0f);

	}
	else
	{

		mb_cuda::pass_through_filter(device_cloud, d_filtered_cloud, 'y', 0.0f, 1.5f);

		mb_cuda::pass_through_filter(d_filtered_cloud, d_filtered_cloud, 'x', -1.0f, -1.0f);

		mb_cuda::pass_through_filter(d_filtered_cloud, d_filtered_cloud, 'z', 0.0f, 2.0f);

	}

	//-- Down sampling
	float leafSize = 0.02;
	mb_cuda::removeNansOrIfs(d_filtered_cloud, d_filtered_cloud);
	mb_cuda::voxel_grid_filter(d_filtered_cloud, d_filtered_cloud, leafSize);

	//--To filteredCloud
	thrust::host_vector<mb_cuda::PointXYZRGB> hostCloud;
	mb_cuda::device_to_host(d_filtered_cloud, hostCloud);
	mb_cuda::thrust_to_pcl(hostCloud, filteredCloud);

	//for ground point
	thrust::device_vector<mb_cuda::PointXYZRGB> forThrustGroundCloud;

	mb_cuda::pass_through_filter(d_filtered_cloud, forThrustGroundCloud, 'z', groundROI.zMin, groundROI.zMax);

	mb_cuda::pass_through_filter(forThrustGroundCloud, forThrustGroundCloud, 'x', groundROI.xMin, groundROI.xMax);

	mb_cuda::device_to_host(forThrustGroundCloud, hostCloud);
	mb_cuda::thrust_to_pcl(hostCloud, forGroundCloud);

	//-- Remove outliers
	//start = chrono::steady_clock::now();
	pcl::StatisticalOutlierRemoval<pointType> passSOR;
	passSOR.setInputCloud(filteredCloud);
	passSOR.setMeanK(10);
	passSOR.setStddevMulThresh(0.1);
	passSOR.filter(*filteredCloud);
	// cout << double(totalTime.count()) / 1000.0f <<" "<<  double(totalTime1.count()) / 1000.0f <<" " <<double(totalTime2.count()) / 1000.0f <<" "<< endl;
}

bool RobotLocator::extractGroundCoeff(pPointCloud cloud)
{
	dstCloud->clear();
	groundCloud->clear();
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

	if (inliers->indices.size() == 0)
	{
		segmentStatus = false;
		return segmentStatus;
	}

	//-- If plane coefficients changed a little, refresh it. else not
	Vector3d vecNormalLast(groundCoeff->values[0], groundCoeff->values[1], groundCoeff->values[2]);
	Vector3d vecNormalThis(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	double angleCosine = abs(vecNormalLast.dot(vecNormalThis) / (vecNormalLast.norm() * vecNormalThis.norm()));

	double originDistanceLast = groundCoeff->values[3] / vecNormalLast.norm();
	double originDistanceThis = coefficients->values[3] / vecNormalThis.norm();
	double distDifference = abs(originDistanceThis - originDistanceLast);
	if (status <= 4)
	{
		if (angleCosine > 0.8f && distDifference < 0.04f)
		{
			groundCoeff = coefficients;			
		}
	}
	else
	{
		thisD435->groundCoeff[0] = coefficients->values[0];
		thisD435->groundCoeff[1] = coefficients->values[1];
		thisD435->groundCoeff[2] = coefficients->values[2];
		thisD435->groundCoeff[3] = coefficients->values[3];
	}
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		cloud->points[inliers->indices[i]].r = 251;
		cloud->points[inliers->indices[i]].g = 188;
		cloud->points[inliers->indices[i]].b = 5;
	}
	return segmentStatus;
}

pPointCloud RobotLocator::rotatePointCloudToHorizontal(pPointCloud cloud)
{
	Eigen::Vector3f vecNormal(groundCoeff->values[0], groundCoeff->values[1], groundCoeff->values[2]);
	//-- Define the rotate angle about x-axis
	//double angleAlpha = atan(-groundCoeff->values[2] / groundCoeff->values[1]);
	double angle = acos(-groundCoeff->values[1] * groundCoeff->values[3] / (vecNormal.norm() * fabs(groundCoeff->values[3])));
	//cout << "angle: " << angle * 180 / CV_PI;
	Eigen::Vector3f Axis = (-groundCoeff->values[3] / fabs(groundCoeff->values[3])) * vecNormal.cross(Eigen::Vector3f(0, -1, 0));
	Axis.normalize();

	//-- Define the rotate transform
	Eigen::Affine3f rotateToXZPlane = Eigen::Affine3f::Identity();
	Eigen::AngleAxisf v1(angle, -Axis);
	thisD435->RotatedMatrix = v1.toRotationMatrix();

	//rotateToXZPlane.rotate(Eigen::AngleAxisf(angleAlpha, Eigen::Vector3f::UnitX()));
	rotateToXZPlane.rotate(v1);
	
	//-- Apply transform
	pcl::transformPointCloud(*cloud, *cloud, rotateToXZPlane);
	//-- Update rotated ground coefficients
	if (status > 4)
	{
		Eigen::AngleAxisf t_V(angle, -Axis);

		angle = -25.0f / 180.0f*3.14;

		Eigen::Affine3f roteY = Eigen::Affine3f::Identity();

		roteY.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));

		//-- Apply transform
		pcl::transformPointCloud(*cloud, *cloud, roteY);
	}

	//groundCoeffRotated->values[0] = groundCoeff->values[0];
	//groundCoeffRotated->values[1] = -vecNormal.norm() * (groundCoeff->values[3] / abs(groundCoeff->values[3]));
	//groundCoeffRotated->values[2] = 0.0f;
	//groundCoeffRotated->values[3] = groundCoeff->values[3];
	Eigen::Vector3f groundCoeffRotatedVec = rotateToXZPlane * vecNormal;

	groundCoeffRotated->values[0] = groundCoeffRotatedVec[0];
	groundCoeffRotated->values[1] = groundCoeffRotatedVec[1];
	groundCoeffRotated->values[2] = groundCoeffRotatedVec[2];
	groundCoeffRotated->values[3] = fabs(groundCoeff->values[3]) / vecNormal.norm();

	//thisD435->groundCoeff[0] = groundCoeffRotatedVec[0];
	//thisD435->groundCoeff[1] = groundCoeffRotatedVec[1];
	//thisD435->groundCoeff[2] = groundCoeffRotatedVec[2];
	//thisD435->groundCoeff[3] = groundCoeffRotated->values[3];
	if(status>4)
	 cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " " 
	                                 << groundCoeffRotated->values[1] << " "
	                                 << groundCoeffRotated->values[2] << " " 
	                                 << groundCoeffRotated->values[3] << endl;

	return cloud;
}

pPointCloud RobotLocator::removeHorizontalPlane(pPointCloud cloud, bool onlyGround)
{
	verticalCloud->clear();
	dstCloud->clear();
	pointType tmpPoint;

	//-- Vector of plane normal and every point on the plane
	Vector3d vecNormal(groundCoeffRotated->values[0], groundCoeffRotated->values[1], groundCoeffRotated->values[2]);
	Vector3d vecPoint(0, 0, 0);

	// cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " " 
	//                                 << groundCoeffRotated->values[1] << " "
	//                                 << groundCoeffRotated->values[2] << " " 
	//                                 << groundCoeffRotated->values[3] << endl;

	//-- Plane normal estimating
	pcl::NormalEstimationOMP<pointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pointType>::Ptr tree(new pcl::search::KdTree<pointType>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.03); /* setKSearch function can be try */
	ne.compute(*normal);

	//-- Compare point normal and plane normal, remove every point on a horizontal plane

	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		vecPoint[0] = normal->points[i].normal_x;
		vecPoint[1] = normal->points[i].normal_y;
		vecPoint[2] = normal->points[i].normal_z;

		if (onlyGround == false)
		{
			double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));

			if (angleCosine < 0.90)
			{
				verticalCloud->points.push_back(cloud->points[i]);
			}
		}
		else
		{
			double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));
			double distanceToPlane = abs(groundCoeffRotated->values[0] * cloud->points[i].x +
				groundCoeffRotated->values[1] * cloud->points[i].y +
				groundCoeffRotated->values[2] * cloud->points[i].z +
				groundCoeffRotated->values[3]) / vecNormal.norm();

			if (angleCosine < 0.90 || distanceToPlane > 0.05)
			{
				verticalCloud->points.push_back(cloud->points[i]);
			}
		}
	}


	//-- Remove Outliers

	pcl::StatisticalOutlierRemoval<pointType> passSOR;
	passSOR.setInputCloud(verticalCloud);
	passSOR.setMeanK(30);
	passSOR.setStddevMulThresh(0.05);
	passSOR.filter(*verticalCloud);



	//-- Copy points from verticalCloud to dstCloud
	for (size_t i = 0; i < verticalCloud->points.size(); i++)
	{
		tmpPoint = verticalCloud->points[i];
		tmpPoint.r = 0;
		tmpPoint.g = 0;
		tmpPoint.b = 0;
		dstCloud->points.push_back(tmpPoint);
	}

	return verticalCloud;
}

pPointCloud RobotLocator::extractVerticalCloud(pPointCloud cloud)
{
	extractGroundCoeff(cloud);
	if (segmentStatus)
	{
		return 0;
	}
	//-- Rotate the point cloud to horizontal
	rotatePointCloudToHorizontal(cloud);

	//-- Remove all horizontal planes

	removeHorizontalPlane(cloud);
	return verticalCloud;

}

void RobotLocator::extractPlaneWithinROI(pPointCloud cloud, ObjectROI roi,
	pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coefficients)
{
	//-- Get point cloud indices inside given ROI
	pcl::PassThrough<pointType> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(roi.xMin, roi.xMax);
	pass.filter(indicesROI->indices);

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(roi.zMin, roi.zMax);
	pass.setIndices(indicesROI);
	pass.filter(indicesROI->indices);

	//-- Plane model segmentation
	pcl::SACSegmentation<pointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setIndices(indicesROI);

	seg.setInputCloud(cloud);
	seg.segment(*indices, *coefficients);

	if (coefficients->values.size() == 0)
	{
		segmentStatus = false;
		return;

	}

	//-- Vector of plane normal and every point on the plane
	Vector3d vecNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	Vector3d vecPoint(0, 0, 0);

	//-- Plane normal estimating
	pcl::NormalEstimationOMP<pointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pointType>::Ptr tree(new pcl::search::KdTree<pointType>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.04);
	ne.compute(*normal);

	indices->indices.clear();

	//-- Compare point normal and position, extract indices of points meeting the criteria
	for (size_t i = 0; i < indicesROI->indices.size(); i++)
	{
		vecPoint[0] = normal->points[indicesROI->indices[i]].normal_x;
		vecPoint[1] = normal->points[indicesROI->indices[i]].normal_y;
		vecPoint[2] = normal->points[indicesROI->indices[i]].normal_z;

		double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));
		double distanceToPlane = abs(coefficients->values[0] * cloud->points[indicesROI->indices[i]].x +
			coefficients->values[1] * cloud->points[indicesROI->indices[i]].y +
			coefficients->values[2] * cloud->points[indicesROI->indices[i]].z +
			coefficients->values[3]) / vecNormal.norm();

		if (angleCosine > 0.8 && distanceToPlane < 0.1)
		{
			indices->indices.push_back(indicesROI->indices[i]);
		}
	}
}

ObjectROI RobotLocator::updateObjectROI(pPointCloud cloud, pcl::PointIndices::Ptr indices,
	double xMinus, double xPlus, double zMinus, double zPlus, bool ifx, bool ifz, ObjectROI beforeobjROI)
{
	ObjectROI objROI;
	objROI = beforeobjROI;
	Eigen::Vector4f minVector, maxVector;

	pcl::getMinMax3D(*cloud, *indices, minVector, maxVector);
	if (ifx)
	{
		objROI.xMin = minVector[0] - xMinus;
		objROI.xMax = maxVector[0] + xPlus;
	}
	if (ifz)
	{
		objROI.zMin = minVector[2] - zMinus;
		objROI.zMax = maxVector[2] + zPlus;
	}

	return objROI;
}
double RobotLocator::calculateDistance(pcl::ModelCoefficients::Ptr groundcoefficients, pcl::ModelCoefficients::Ptr planecoefficients)
{
	Vector3d groundNormal = Vector3d(groundcoefficients->values[0], groundcoefficients->values[1], groundcoefficients->values[2]);
	Vector3d planeNormal = Vector3d(planecoefficients->values[0], planecoefficients->values[1], planecoefficients->values[3]);

	double cameraHeight = abs(groundCoeffRotated->values[3]) / groundNormal.norm();

	double Distance = abs(planecoefficients->values[1] * (cameraHeight - 0.05) + planecoefficients->values[3]) / planeNormal.norm();

	return Distance;

}

void RobotLocator::locateBeforeDuneStage1(void)
{
#ifdef DEBUG
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime3 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime4 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif

	start = chrono::steady_clock::now();
	extractVerticalCloud(filteredCloud);
	if (!segmentStatus)
	{
		return;
	}
	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	//totalTime3= chrono::duration_cast<chrono::microseconds>(stop - start);
   //-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	start = chrono::steady_clock::now();
	extractPlaneWithinROI(verticalCloud, leftFenseROImax, inliers, coefficients);
	if (!segmentStatus)
	{
		return;
	}
	stop = chrono::steady_clock::now();
	totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
	leftFenseROImax = updateObjectROI(verticalCloud, inliers, 0.3, 0.3, 0.1, 0.1, true, true, leftFenseROImax);

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*verticalCloud, *inliers, minVector, maxVector);

	extractPlaneWithinROI(verticalCloud, leftFenseROImin, inliers, coefficients);
	if (!segmentStatus)
	{
		return;
	}

	double xDistance = calculateDistance(groundCoeffRotated, coefficients);


	duneROI.xMin = leftFenseROImin.xMax - 0.3;
	duneROI.xMax = leftFenseROImin.xMax + 0.9;
	duneROI.zMin = leftFenseROImin.zMin + 0.3;
	duneROI.zMax = leftFenseROImin.zMax + 0.9;

	if (maxVector[2] < 1.50f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }
	diatancemeasurement = xDistance;

#ifdef DEBUG
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 234;
		dstCloud->points[inliers->indices[i]].g = 67;
		dstCloud->points[inliers->indices[i]].b = 53;

	}
	cout << " step1 date3 " << maxVector[2] << " " << xDistance << " " << endl;
	dstViewer->updatePointCloud(groundCloud, "ground Cloud");
	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
	cout << " locateBeforeDuneStage1 2 " << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << " " << endl;
#endif


}

void RobotLocator::locateBeforeDuneStage2(void)
{
	extractVerticalCloud(filteredCloud);
	if (!segmentStatus)
	{
		return;
	}
	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	extractPlaneWithinROI(verticalCloud, leftFenseROImin, inliers, coefficients);
	if (!segmentStatus)
	{
		return;
	}
	leftFenseROImin = updateObjectROI(verticalCloud, inliers, 0.2, 0.2, 0.0, 0.0, true, false, leftFenseROImin);

	//-- The formula of dune is ax + by + cz + d = 0, which z = 0.0 and y = cameraHeight - 0.05
	double xDistance = calculateDistance(groundCoeffRotated, coefficients);

	Vector3d vecXAxis(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
#ifdef DEBUG
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 234;
		dstCloud->points[inliers->indices[i]].g = 67;
		dstCloud->points[inliers->indices[i]].b = 53;
	}
#endif

	extractPlaneWithinROI(verticalCloud, leftFenseROImax, inliers, coefficients);

	if (!segmentStatus)
	{
		return;
	}

	Vector3d vecNormal = Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);


	double angleCosine = abs(vecNormal.dot(vecXAxis) / (vecNormal.norm() * vecXAxis.norm()));

	if (angleCosine < 0.90)
	{
		//-- Extract indices for the rest part
		pcl::ExtractIndices<pointType> extract;

		extract.setInputCloud(verticalCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(inliers->indices);


		//-- Get point cloud indices inside given ROI

		pcl::PassThrough<pointType> pass;
		pass.setInputCloud(verticalCloud);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(leftFenseROImax.xMin, leftFenseROImax.xMax);
		pass.setIndices(inliers);
		pass.filter(inliers->indices);


		pass.setInputCloud(verticalCloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(leftFenseROImax.zMin, leftFenseROImax.zMax);
		pass.setIndices(inliers);
		pass.filter(inliers->indices);



		//-- Plane model segmentation

		pcl::SACSegmentation<pointType> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);
		seg.setIndices(inliers);


		seg.setInputCloud(verticalCloud);
		seg.segment(*inliers, *coefficients);

	}
	if (!segmentStatus)
	{
		return;
	}

	leftFenseROImax = updateObjectROI(verticalCloud, inliers, 0.3, 0.3, 0.3, 0.3, true, true, leftFenseROImax);

	if (leftFenseROImax.xMin < -1.2)
		leftFenseROImax.xMin = -1.2;

	if (leftFenseROImax.xMin > 0)
		leftFenseROImax.xMin = 0;



	// cout << "xMin_  " << leftFenseROI.xMin << "  xMax_  " << leftFenseROI.xMax << 
	//         "  zMin_  " << leftFenseROI.zMin << "  zMax_  " << leftFenseROI.zMax << endl;

	//-- Change the color of the extracted part for debuging

	duneROI.xMin = leftFenseROImax.xMax - 0.3;
	duneROI.xMax = leftFenseROImax.xMax + 0.9;
	duneROI.zMin = leftFenseROImax.zMax - 0.3;
	duneROI.zMax = leftFenseROImax.zMax + 0.9;


	extractPlaneWithinROI(verticalCloud, duneROI, inliers, coefficients);
	if (!segmentStatus)
	{
		return;
	}

	double duneDistance = calculateDistance(groundCoeffRotated, coefficients);

	if (duneDistance < 1.2f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }

	diatancemeasurement = duneDistance;

	//-- Change the color of the extracted part for debuging
#ifdef DEBUG
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}

	cout << "duneDistance  " << duneDistance << " " << "leftX distance  " << xDistance << endl;
	dstViewer->updatePointCloud(groundCloud, "ground Cloud");
	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif


}

void RobotLocator::locateBeforeDuneStage3(void)
{


	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;

	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	start = chrono::steady_clock::now();
	extractVerticalCloud(filteredCloud);
	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);

	if (!segmentStatus)
	{
		return;
	}
	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);


	extractPlaneWithinROI(verticalCloud, duneROI, inliers, coefficients);

	if (!segmentStatus)
	{
		return;
	}

	duneROI = updateObjectROI(verticalCloud, inliers, 0.0, 0.0, 0.3, 0.3, false, true, duneROI);

	double duneDistance = calculateDistance(groundCoeffRotated, coefficients);
	diatancemeasurement = duneDistance;

	if (duneDistance < 0.25f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }


#ifdef DEBUG
	//-- Change the color of the extracted part for debuging
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}

	//	anglemeasurement = plus_minus * acos(angleCosine) / PI * 180 - 45;
	cout << "Dune distance  " << duneDistance << " z_anxis "  << endl;

	//	cout << " dune coefficients: " << coefficients->values[0] << " "
	//		<< coefficients->values[1] << " "
	//		<< coefficients->values[2] << " "
	//		<< coefficients->values[3];
	//	cout << " " << cameraHeight << endl;
	cout << " locateBeforeDuneStage1 3 " << double(totalTime.count()) / 1000.0f << " " << endl;
	dstViewer->updatePointCloud(groundCloud, "ground Cloud");
	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif

}

void RobotLocator::locatePassingDune(void)
{
	extractVerticalCloud(filteredCloud);

	//-- Get point cloud indices inside given ROI
	pcl::PassThrough<pointType> pass;
	pass.setInputCloud(verticalCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(frontFenseROI.xMin, 0.0);
	pass.filter(indicesROI->indices);

	pass.setInputCloud(verticalCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(frontFenseROI.zMin, frontFenseROI.zMax);
	pass.setIndices(indicesROI);
	pass.filter(indicesROI->indices);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pointType>::Ptr tree(new pcl::search::KdTree<pointType>);
	tree->setInputCloud(verticalCloud);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::PointIndices::Ptr largestIndice(new pcl::PointIndices);

	//-- Perform euclidean cluster extraction
	pcl::EuclideanClusterExtraction<pointType> ec;
	ec.setClusterTolerance(0.1);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(verticalCloud);
	ec.setIndices(indicesROI);
	ec.extract(clusterIndices);

	for (int i = 0; i < clusterIndices.size(); i++)
	{
		if (clusterIndices[i].indices.size() > largestIndice->indices.size())
		{
			*largestIndice = clusterIndices[i];
		}
	}
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	pcl::SACSegmentation<pointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setIndices(largestIndice);

	seg.setInputCloud(verticalCloud);
	seg.segment(*inliers, *coefficients);

	frontFenseROI = updateObjectROI(verticalCloud, inliers, 0.7, 0.0, 0.2, 0.2, true, true, frontFenseROI);


	double frountdDistance = calculateDistance(groundCoeffRotated, coefficients);

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*verticalCloud, *inliers, minVector, maxVector);

	//-- Calculate the vertical distance to front fense
	double fenseDistance = minVector[2];

	if (fenseDistance < 0.8f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }

#ifdef DEBUG
	  //-- Change the color of the extracted part for debuging
	  for (int i = 0; i < inliers->indices.size(); i++)
	  {
		  dstCloud->points[inliers->indices[i]].r = 251;
		  dstCloud->points[inliers->indices[i]].g = 188;
		  dstCloud->points[inliers->indices[i]].b = 5;
	  }

	  cout << "front fense distance  " << fenseDistance << " second " << frountdDistance << endl;

	  dstViewer->updatePointCloud(groundCloud, "ground Cloud");
	  dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	  dstViewer->spinOnce(1);
#endif

}

void RobotLocator::locateBeforeGrasslandStage1(void)
{
	extractGroundCoeff(filteredCloud);
	rotatePointCloudToHorizontal(filteredCloud);
	thisD435->imgProcess();

	thisD435->FindFenseCorner(HORIZONAL_FENSE, mode);
	frontFenseDist = thisD435->GetDepth(thisD435->fenseCorner, thisD435->fenseCornerIn3D);

	besideFenseDist = mode == LEFT_MODE ? fenseCorner2fenseDist - thisD435->nowXpos : fenseCorner2fenseDist + thisD435->nowXpos;

	if (besideFenseDist < 1550) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++;}
	dstViewer->updatePointCloud(filteredCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#ifdef DEBUG
	cout << "status: " << status << "\n" << endl;
	cout << "front fense distance  " << frontFenseDist << "  besideFenseDist " << besideFenseDist << endl;
#endif
}
void RobotLocator::locateBeforeGrasslandStage2(void)
{
	extractGroundCoeff(filteredCloud);
	rotatePointCloudToHorizontal(filteredCloud);
	thisD435->imgProcess();

	if (thisD435->pillarStatus)
	{
		secondRopeDist = thisD435->GetDepth(thisD435->center2, thisD435->center2In3D);

		if ((1 - 2 * mode) * thisD435->nowXpos + 400 < 0)
		{
			besideFenseDist = mode == LEFT_MODE ? (pillarRadius - thisD435->nowXpos) : (pillarRadius + thisD435->nowXpos);
		}
		else
		{
			besideFenseDist = mode == LEFT_MODE ? (fenseCorner2fenseDist - pillarRadius - thisD435->nowXpos) : (fenseCorner2fenseDist - pillarRadius + thisD435->nowXpos);
		}
	}
	if (secondRopeDist < 700 && thisD435->pillarStatus != 1) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status ++; thisD435->status ++; colorFrameRoi = mode == LEFT_MODE ? rightRoi : leftRoi;}
#ifdef DEBUG
	cout << "secondRopeDist: " << secondRopeDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << "status: " << status << "\n" << endl;
	dstViewer->updatePointCloud(filteredCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif // DEBUG
}
void RobotLocator::locatePassingGrasslandStage1(void)
{
	extractGroundCoeff(filteredCloud);
	rotatePointCloudToHorizontal(filteredCloud);
	thisD435->imgProcess();

	thisD435->FindLineCrossCenter(10.0, 1.0, 40.0);

	secondRopeDist = thisD435->GetDepth(thisD435->lineCross, thisD435->lineCrossIn3D) - lineCross2RopeDist;
	besideFenseDist = mode == LEFT_MODE ? line2BesidefenseDist - thisD435->nowXpos : line2BesidefenseDist + thisD435->nowXpos;
	frontFenseDist = thisD435->GetDepth(thisD435->fenseCorner, thisD435->fenseCornerIn3D) + lineCross2FrontfenseDist;

	if (frontFenseDist < 1200) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }
	
	if (nextStatusCounter >= 2) { status+=2; thisD435->status+=2; }

#ifdef DEBUG
	cout << "secondRopeDist: " << secondRopeDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << "status: " << status << "\n" << endl;
	dstViewer->updatePointCloud(filteredCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locatePassingGrasslandStage2(void)
{
	extractGroundCoeff(filteredCloud);
	rotatePointCloudToHorizontal(filteredCloud);
	thisD435->imgProcess();

	thisD435->FindLineEnd();

	frontFenseDist = thisD435->GetDepth(thisD435->lineEnd, thisD435->lineEndIn3D);
	besideFenseDist = mode == LEFT_MODE ? line2BesidefenseDist - thisD435->nowXpos : line2BesidefenseDist + thisD435->nowXpos;

	if (frontFenseDist < 1000) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }
#ifdef DEBUG
	cout << "secondRopeDist: " << secondRopeDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << "status: " << status << "\n" << endl;
	dstViewer->updatePointCloud(filteredCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locateUnderMountain(void)
{
	extractGroundCoeff(filteredCloud);
	rotatePointCloudToHorizontal(filteredCloud);
	thisD435->imgProcess();

	thisD435->FindLineEnd();

	frontFenseDist = thisD435->GetDepth(thisD435->lineEnd, thisD435->lineEndIn3D);
	besideFenseDist = mode == LEFT_MODE ? lineEnd2BesidefenseDist - thisD435->nowXpos : lineEnd2BesidefenseDist + thisD435->nowXpos;
	
	if (besideFenseDist > 850 && frontFenseDist < 800)
		colorFrameRoi = mode == LEFT_MODE ? leftRoi : rightRoi;
	//ͣ��
	if (0) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }
#ifdef DEBUG
	cout << "frontFenseDist: " << frontFenseDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << "status: " << status << "\n" << endl;
	dstViewer->updatePointCloud(filteredCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locateClimbingMountain(void)
{
	extractGroundCoeff(filteredCloud);
	rotatePointCloudToHorizontal(filteredCloud);
	
	thisD435->imgProcess();
	int stageJudge = thisD435->ClimbingMountainStageJudge();

	thisD435->FindHoughLineCross();
	frontFenseDist = thisD435->GetDepth(thisD435->lineCross, thisD435->lineCrossIn3D);
	
	switch (stageJudge)
	{
		case 2:
		{
			if (thisD435->filterLine.size() == 2 && (thisD435->nowXpos * (2 * mode - 1)) < 0)
				peakDist = mode == LEFT_MODE ? thisD435->nowXpos : -thisD435->nowXpos;
		}
		break;

		case 3:
		{
			if (thisD435->filterLine.size() == 2)
				peakDist = mode == LEFT_MODE ? thisD435->nowXpos : -thisD435->nowXpos;
		}		
		break;
	}

	if (stageJudge == 3 && thisD435->filterLine.size() != 2) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }

#ifdef DEBUG
	dstViewer->updatePointCloud(filteredCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
	cout << "frontFenseDist:" << frontFenseDist << " peakDist:" << peakDist << endl;

	cout << "climbStage: "<< stageJudge << " status: " << status << "\n" << endl;
#endif
}
void RobotLocator::locateReachingMountain(void)
{
	extractGroundCoeff(filteredCloud);
	rotatePointCloudToHorizontal(filteredCloud);
	thisD435->imgProcess();

	thisD435->FindHoughLineCross();
	frontFenseDist = thisD435->GetDepth(thisD435->lineCross, thisD435->lineCrossIn3D);

	if (thisD435->filterLine.size() == 2 && (thisD435->nowXpos * (2 * mode - 1)) < 0)
		peakDist = mode == LEFT_MODE ? thisD435->nowXpos : -thisD435->nowXpos;

	if (0) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }
#ifdef DEBUG
	cout << "PeakDist: " << peakDist << endl;
	dstViewer->updatePointCloud(filteredCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif // DEBUG
}
bool RobotLocator::isStoped(void)
{
	return dstViewer->wasStopped();
}
