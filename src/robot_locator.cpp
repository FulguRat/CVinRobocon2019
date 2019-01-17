#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
filteredCloud(new pointCloud),
verticalCloud(new pointCloud),
dstCloud(new pointCloud),
indicesROI(new pcl::PointIndices),
groundCoeff(new pcl::ModelCoefficients),
groundCoeffRotated(new pcl::ModelCoefficients),
dstViewer(new pcl::visualization::PCLVisualizer("Advanced Viewer"))
{
    leftFenseROI  = { -0.6/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 2.5/*zMax*/ };
    duneROI       = { -0.3/*xMin*/,  0.3/*xMax*/, 0.0/*zMin*/, 2.5/*zMax*/ };
    // frontFenseROI = { -1.3/*xMin*/,  0.3/*xMax*/, 1.2/*zMin*/, 2.1/*zMax*/ };
    frontFenseROI = { -0.3/*xMin*/,  0.3/*xMax*/, 0.0/*zMin*/, 1.5/*zMax*/ };
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
		srcCloud = thisD435->update();

		pcl::PassThrough<pointType> pass;
		pass.setInputCloud(srcCloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0f, 3.0f);
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
	pass.setFilterLimits(-1.0f, 1.0f);
	pass.filter(*filteredCloud);

	pass.setInputCloud(filteredCloud);
	pass.setFilterFieldName("z");
	if(status == BEFORE_GRASSLAND_STAGE_2 || status == PASSING_GRASSLAND_STAGE_1)
		pass.setFilterLimits(0.0f, 2.0f);
	else
		pass.setFilterLimits(0.0f, 4.0f);
	pass.filter(*filteredCloud);



	//-- Down sampling

	pcl::VoxelGrid<pointType> passVG;
	passVG.setInputCloud(filteredCloud);
	passVG.setLeafSize(0.02f, 0.02f, 0.02f);
	passVG.filter(*filteredCloud);



	//-- Remove outliers
	//start = chrono::steady_clock::now();
	pcl::StatisticalOutlierRemoval<pointType> passSOR;
	passSOR.setInputCloud(filteredCloud);
	passSOR.setMeanK(10);
	passSOR.setStddevMulThresh(0.1);
	passSOR.filter(*filteredCloud);



	// cout << double(totalTime.count()) / 1000.0f <<" "<<  double(totalTime1.count()) / 1000.0f <<" " <<double(totalTime2.count()) / 1000.0f <<" "<< endl;
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
	Vector3d vecNormalLast(groundCoeff->values[0], groundCoeff->values[1], groundCoeff->values[2]);
	Vector3d vecNormalThis(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	double angleCosine = abs(vecNormalLast.dot(vecNormalThis) / (vecNormalLast.norm() * vecNormalThis.norm()));

	double originDistanceLast = groundCoeff->values[3] / vecNormalLast.norm();
	double originDistanceThis = coefficients->values[3] / vecNormalThis.norm();
	double distDifference = abs(originDistanceThis - originDistanceLast);

	if (angleCosine > 0.8f && distDifference < 0.04f)
	{
		groundCoeff = coefficients;
	}

	return groundCoeff;
}

pPointCloud RobotLocator::rotatePointCloudToHorizontal(pPointCloud cloud)
{
	//-- Define the rotate angle about x-axis
	double angleAlpha = atan(-groundCoeff->values[2] / groundCoeff->values[1]);

	//-- Define the rotate transform
	Eigen::Affine3f rotateToXZPlane = Eigen::Affine3f::Identity();
	rotateToXZPlane.rotate(Eigen::AngleAxisf(angleAlpha, Eigen::Vector3f::UnitX()));

	//-- Apply transform
	pcl::transformPointCloud(*cloud, *cloud, rotateToXZPlane);

	//-- Update rotated ground coefficients
	Vector3d vecNormal(groundCoeff->values[0], groundCoeff->values[1], groundCoeff->values[2]);

	groundCoeffRotated->values[0] = groundCoeff->values[0];
	groundCoeffRotated->values[1] = -vecNormal.norm() * (groundCoeff->values[3] / abs(groundCoeff->values[3]));
	groundCoeffRotated->values[2] = 0.0f;
	groundCoeffRotated->values[3] = groundCoeff->values[3];

	// cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " " 
	//                                 << groundCoeffRotated->values[1] << " "
	//                                 << groundCoeffRotated->values[2] << " " 
	//                                 << groundCoeffRotated->values[3] << endl;

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
	passSOR.setMeanK(20);
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
		//-- TODO: If tracking failed

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

		if (angleCosine > 0.80 && distanceToPlane < 0.10)
		{
			indices->indices.push_back(indicesROI->indices[i]);
		}
	}
}

ObjectROI RobotLocator::updateObjectROI(pPointCloud cloud, pcl::PointIndices::Ptr indices,
	double xMinus, double xPlus, double zMinus, double zPlus)
{
	ObjectROI objROI;
	Eigen::Vector4f minVector, maxVector;

	pcl::getMinMax3D(*cloud, *indices, minVector, maxVector);

	objROI.xMin = minVector[0] - xMinus;
	objROI.xMax = maxVector[0] + xPlus;

	objROI.zMin = minVector[2] - zMinus;
	objROI.zMax = maxVector[2] + zPlus;

	return objROI;
}

void RobotLocator::locateBeforeDuneStage1(void)
{
	//chrono::steady_clock::time_point start;
	//chrono::steady_clock::time_point stop;
	//auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	//auto totalTime1= chrono::duration_cast<chrono::microseconds>(stop - start);
	//auto totalTime2= chrono::duration_cast<chrono::microseconds>(stop - start);
	//auto totalTime3= chrono::duration_cast<chrono::microseconds>(stop - start);
	//auto totalTime5= chrono::duration_cast<chrono::microseconds>(stop - start);

   // start = chrono::steady_clock::now();
	extractVerticalCloud(filteredCloud);
	//stop = chrono::steady_clock::now();
	 //totalTime3= chrono::duration_cast<chrono::microseconds>(stop - start);
	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	//start = chrono::steady_clock::now();
	extractPlaneWithinROI(verticalCloud, leftFenseROI, inliers, coefficients);
	leftFenseROI = updateObjectROI(verticalCloud, inliers, 0.1, 0.1, 0.3, 0.3);

	Vector3d normalleft(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

	Vector3d vecNormal = Vector3d(groundCoeffRotated->values[0], groundCoeffRotated->values[1], groundCoeffRotated->values[2]);
	double cameraHeight = abs(groundCoeffRotated->values[3]) / vecNormal.norm();

	double xDistance = (coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / coefficients->values[0];

	Vector2d normalleft2d(coefficients->values[0], coefficients->values[2]);
	Vector2d vecXAxis(1, 0);

	double angleCosine = normalleft2d.dot(vecXAxis) / (normalleft2d.norm()*vecXAxis.norm());

	int plus_minus;
	if (coefficients->values[0] * coefficients->values[2] > 0)
		plus_minus = -1;
	else plus_minus = 1;


	//stop = chrono::steady_clock::now();
	//totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);

	// cout << "xMin_  " << leftFenseROI.xMin << "  xMax_  " << leftFenseROI.xMax << 
	//         "  zMin_  " << leftFenseROI.zMin << "  zMax_  " << leftFenseROI.zMax << endl;

	//-- Change the color of the extracted part for debuging
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 234;
		dstCloud->points[inliers->indices[i]].g = 67;
		dstCloud->points[inliers->indices[i]].b = 53;
	}

	duneROI.xMin = leftFenseROI.xMax - 0.2;
	duneROI.xMax = leftFenseROI.xMax + 0.9;
	duneROI.zMin = leftFenseROI.zMin + 0.3;
	duneROI.zMax = leftFenseROI.zMax + 0.9;

	//-- Get point cloud indices inside given ROI
	//start = chrono::steady_clock::now();
	pcl::PassThrough<pointType> pass;
	pass.setInputCloud(verticalCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(duneROI.xMin, duneROI.xMax);
	pass.filter(inliers->indices);

	pass.setInputCloud(verticalCloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(duneROI.zMin, duneROI.zMax);
	pass.setIndices(inliers);
	pass.filter(inliers->indices);

	//stop = chrono::steady_clock::now();
	//totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);

	// -- Change the color of the extracted part for debuging
	// start = chrono::steady_clock::now();
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*verticalCloud, *inliers, minVector, maxVector);

	// stop = chrono::steady_clock::now();
	//totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);

	if (minVector[2] < 1.80f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }

	cout << minVector[2] << " " << xDistance << " " << plus_minus * acos(angleCosine) / CV_PI * 180 << endl;

	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);

	//cout << double(totalTime.count()) / 1000.0f <<" "<<  double(totalTime1.count()) / 1000.0f <<" " <<double(totalTime2.count()) / 1000.0f <<" "<<double(totalTime3.count()) / 1000.0f << endl;
}

void RobotLocator::locateBeforeDuneStage2(void)
{

	extractVerticalCloud(filteredCloud);

	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);


	extractPlaneWithinROI(verticalCloud, leftFenseROI, inliers, coefficients);



	Vector3d vecNormal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	Vector3d vecXAxis(1.0, 0.0, 0.0);

	double angleCosine = abs(vecNormal.dot(vecXAxis) / (vecNormal.norm() * vecXAxis.norm()));

	if (angleCosine < 0.9)
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
		pass.setFilterLimits(leftFenseROI.xMin, leftFenseROI.xMax);
		pass.setIndices(inliers);
		pass.filter(inliers->indices);




		pass.setInputCloud(verticalCloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(leftFenseROI.zMin, leftFenseROI.zMax);
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

	leftFenseROI = updateObjectROI(verticalCloud, inliers, 0.1, 0.1, 0.3, 0.3);

	//-- Calculate the angle to x-axis
	Vector2d vecNormaleft(coefficients->values[0], coefficients->values[2]);
	Vector2d vecXAxis2d(1.0, 0.0);
	angleCosine = vecNormaleft.dot(vecXAxis2d) / (vecNormaleft.norm()*vecXAxis2d.norm());
	int plus_minus;
	if (coefficients->values[0] * coefficients->values[2] > 0)
		plus_minus = -1;
	else plus_minus = 1;
	//-- Calculate the distance to leftsense along the x-axis
	vecNormal = Vector3d(groundCoeffRotated->values[0], groundCoeffRotated->values[1], groundCoeffRotated->values[2]);
	double cameraHeight = abs(groundCoeffRotated->values[3]) / vecNormal.norm();

	//-- The formula of dune is ax + by + cz + d = 0, which z = 0.0 and y = cameraHeight - 0.05
	double xDistance = (coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / coefficients->values[0];

	// cout << "xMin_  " << leftFenseROI.xMin << "  xMax_  " << leftFenseROI.xMax << 
	//         "  zMin_  " << leftFenseROI.zMin << "  zMax_  " << leftFenseROI.zMax << endl;

	//-- Change the color of the extracted part for debuging
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 234;
		dstCloud->points[inliers->indices[i]].g = 67;
		dstCloud->points[inliers->indices[i]].b = 53;
	}

	duneROI.xMin = leftFenseROI.xMax - 0.3;
	duneROI.xMax = leftFenseROI.xMax + 0.9;
	duneROI.zMin = leftFenseROI.zMax - 0.3;
	duneROI.zMax = leftFenseROI.zMax + 0.9;


	extractPlaneWithinROI(verticalCloud, duneROI, inliers, coefficients);
	//-- Change the color of the extracted part for debuging
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}



	//-- The formula of dune is ax + by + cz + d = 0, which x = 0.0 and y = cameraHeight - 0.05
	double zDistance = -(coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / coefficients->values[2];

	double duneDistance = (coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / vecNormal.norm();

	if (zDistance < 1.40f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }

	cout << "duneDistance  " << duneDistance << "Z distance  " << zDistance << " " << "leftX distance  " << xDistance << " " << "angle  " << plus_minus * acos(angleCosine) / CV_PI * 180 << endl;

	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
	//cout <<"Z distance  " << zDistance<<" "<<angleCosine<<" "<< double(totalTime.count()) / 1000.0f <<" "<<  double(totalTime1.count()) / 1000.0f <<" " <<double(totalTime2.count()) / 1000.0f <<" "<<double(totalTime3.count()) / 1000.0f<<" "<<double(totalTime4.count()) / 1000.0f 
	//<<" "<< double(totalTimeall.count()) / 1000.0f<< endl;
}

void RobotLocator::locateBeforeDuneStage3(void)
{
	extractVerticalCloud(filteredCloud);

	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	extractPlaneWithinROI(verticalCloud, duneROI, inliers, coefficients);
	duneROI = updateObjectROI(verticalCloud, inliers, 0.1, 0.1, 0.3, 0.3);

	//-- Change the color of the extracted part for debuging
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}

	//-- Calculate the vertical distance to dune
	Vector3d vecNormal(groundCoeffRotated->values[0], groundCoeffRotated->values[1], groundCoeffRotated->values[2]);
	double cameraHeight = abs(groundCoeffRotated->values[3]) / vecNormal.norm();

	//-- The formula of dune is ax + by + cz + d = 0, which y = cameraHeight - 0.05
	vecNormal = Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	double duneDistance = (coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / vecNormal.norm();

	Vector2d normalleft2d(coefficients->values[0], coefficients->values[2]);
	Vector2d vecXAxis(1, 0);

	double angleCosine = abs(normalleft2d.dot(vecXAxis) / (normalleft2d.norm()*vecXAxis.norm()));

	int plus_minus;
	if (coefficients->values[0] * coefficients->values[2] > 0)
		plus_minus = -1;
	else plus_minus = 1;
	// if (duneDistance < 0.5f) { nextStatusCounter++; }
	// else { nextStatusCounter = 0; }

	// if (nextStatusCounter >= 3) { status++; }

	cout << "Dune distance  " << duneDistance << " z_anxis " << plus_minus * acos(angleCosine) / CV_PI * 180 - 45 << " anxis " << angleCosine << endl;

	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
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

	frontFenseROI = updateObjectROI(verticalCloud, largestIndice, 0.3, 0.0, 0.1, 0.1);

	//-- Change the color of the extracted part for debuging
	//for (int i = 0; i < largestIndice->indices.size(); i++)
	//{
	//    dstCloud->points[largestIndice->indices[i]].r = 251;
	//    dstCloud->points[largestIndice->indices[i]].g = 188;
	 //   dstCloud->points[largestIndice->indices[i]].b = 5;
   // }

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*verticalCloud, *largestIndice, minVector, maxVector);

	//-- Calculate the vertical distance to front fense
	double fenseDistance = minVector[2];

	// // if (fenseDistance < 0.5f) { nextStatusCounter++; }
	// // else { nextStatusCounter = 0; }

	// // if (nextStatusCounter >= 3) { status++; }

	cout << "front fense distance  " << fenseDistance << endl;

	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
}

void RobotLocator::locateBeforeGrasslandStage1(void)
{
    extractVerticalCloud(filteredCloud); 

    //-- Perform the plane segmentation with specific indices
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    extractPlaneWithinROI(verticalCloud, frontFenseROI, inliers, coefficients);
    frontFenseROI = updateObjectROI(verticalCloud, inliers, 0.1, 0.1, 0.3, 0.3);
    
    //-- Change the color of the extracted part for debuging
    for (int i = 0; i < inliers->indices.size(); i++)
    {
        dstCloud->points[inliers->indices[i]].r = 251;
        dstCloud->points[inliers->indices[i]].g = 188;
        dstCloud->points[inliers->indices[i]].b = 5;
    }
	

	frontFenseROI = updateObjectROI(verticalCloud, inliers, 0.3, 0.0, 0.1, 0.1);

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*verticalCloud, *inliers, minVector, maxVector);

    //-- Calculate the vertical distance to front fense
	thisD435->lastXpos = fenseCornerX;
	thisD435->lastXpos = frontFenseDist;

    frontFenseDist = maxVector[2];
    fenseCornerX = minVector[0];

	thisD435->nowXpos = fenseCornerX;
	thisD435->nowZpos = frontFenseDist;

	angle = thisD435->GetAngle();

     /*if (fenseCornerX > 0.0f) { nextStatusCounter++; }
     else { nextStatusCounter = 0; }

     if (nextStatusCounter >= 3) { status++; }*/
    
    cout << "front fense distance  " << frontFenseDist << "  x_" << fenseCornerX << endl;

    dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
    dstViewer->spinOnce(1);
}

void RobotLocator::locateBeforeGrasslandStage2(void)
{
	extractGroundCoeff(filteredCloud);
	thisD435->imgProcess();
	thisD435->FindPillarCenter();
	firstRopeDist = thisD435->GetDepth(thisD435->center1, groundCoeff->values[0], groundCoeff->values[1], groundCoeff->values[2], 1000 * groundCoeff->values[3]);
	angle = thisD435->GetAngle();
	cout << "firstRopeDist: " << firstRopeDist << endl;
	//model change
	if ((thisD435->center1.y > 440 || thisD435->center1.x > 580) && status == BEFORE_GRASSLAND_STAGE_2)
	{
		status = PASSING_GRASSLAND_STAGE_1;
		thisD435->status = PASSING_GRASSLAND_STAGE_1;
	}

}

void RobotLocator::locatePassingGrasslandStage1(void)
{
	extractGroundCoeff(filteredCloud);
	thisD435->imgProcess();
	thisD435->FindPillarCenter();
	secondRopeDist = thisD435->GetDepth(thisD435->center2, groundCoeff->values[0], groundCoeff->values[1], groundCoeff->values[2], 1000 * groundCoeff->values[3]);
	angle = thisD435->GetAngle();
	cout << "secondRopeDist: " << secondRopeDist << endl;
	//model change
	if (status == PASSING_GRASSLAND_STAGE_1 && (thisD435->center2.y > 440 || thisD435->center2.x > 580))
	{
		status = PASSING_GRASSLAND_STAGE_2;
		thisD435->status = PASSING_GRASSLAND_STAGE_2;
	}

}

void RobotLocator::locatePassingGrasslandStage2(void)
{
	extractVerticalCloud(filteredCloud);

	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	extractPlaneWithinROI(verticalCloud, grasslandFenseROI, inliers, coefficients);
	grasslandFenseROI = updateObjectROI(verticalCloud, inliers, 0.1, 0.1, 0.3, 0.3);

	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}

	//-- The formula of dune is ax + by + cz + d = 0, which y = cameraHeight - 0.05
	Vector3d vecNormal = Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

	Eigen::Vector4f minVector, maxVector;

	pcl::getMinMax3D(*verticalCloud, *inliers, minVector, maxVector);

	thisD435->lastXpos = fenseCornerX;
	thisD435->lastXpos = grassFenseDist;

	grassFenseDist = maxVector[2];
	fenseCornerX = minVector[0];

	thisD435->nowXpos = fenseCornerX;
	thisD435->nowZpos = grassFenseDist;

	angle = thisD435->GetAngle();

	// if (frontFenseDist < 0.3f) { nextStatusCounter++; }
	// else { nextStatusCounter = 0; }

	// if (nextStatusCounter >= 3) { status++; }

	cout << "grassFenseDist  " << grassFenseDist << endl;

	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);

}

bool RobotLocator::isStoped(void)
{
	return dstViewer->wasStopped();
}