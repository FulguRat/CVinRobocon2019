#include "robot_locator.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
filteredCloud(new pointCloud),
verticalCloud(new pointCloud),
dstCloud(new pointCloud),
groundcloud(new pointCloud),
outgroundcloud(new pointCloud),
forgroundcloud(new pointCloud),
indicesROI(new pcl::PointIndices),
groundCoeff(new pcl::ModelCoefficients),
groundCoeffRotated(new pcl::ModelCoefficients),
dstViewer(new pcl::visualization::PCLVisualizer("Advanced Viewer"))
{
	leftFenseROI = { -0.7/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 2.5/*zMax*/ };
	duneROI = { -0.7/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 2.5/*zMax*/ };
	// frontFenseROI = { -1.3/*xMin*/,  0.3/*xMax*/, 1.2/*zMin*/, 2.1/*zMax*/ };
	frontFenseROI = { -0.5/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 1.5/*zMax*/ };


	dstViewer->setBackgroundColor(0.259, 0.522, 0.957);
	dstViewer->addPointCloud<pointType>(dstCloud, "Destination Cloud");
	dstViewer->addPointCloud<pointType>(groundcloud, "ground Cloud");
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
	newFrame = thisD435->color;
	return srcCloud;
}

bool RobotLocator::orbmatch(void)
{
	bool acept = true;
	std::vector<cv::KeyPoint> newkeypoints, lastkeypoints;

	cv::Mat newdescriptors, lastdescriptors;

	cv::Ptr<cv::ORB> orb = cv::ORB::create();

	orb->detect(lastFrame, lastkeypoints);
	orb->detect(newFrame, newkeypoints);

	orb->compute(lastFrame, lastkeypoints, lastdescriptors);
	orb->compute(newFrame, newkeypoints, newdescriptors);


	

	//matcher.match(lastdescriptors, newdescriptors, matches);

	//double min_dist = 0, max_dist = 0;//∂®“Âæ‡¿Î

	//for (int i = 0; i < lastdescriptors.cols; ++i)//±È¿˙
	//{
	//	double dist = matches[i].distance;
	//	if (dist < min_dist) min_dist = dist;
	//	if (dist > max_dist) max_dist = dist;

	//}
	//std::vector<cv::DMatch> good_matches;

	//for (int j = 0; j < lastdescriptors.cols; ++j)
	//{
	//	if (matches[j].distance <= max(2 * min_dist, 30.0))
	//		good_matches.push_back(matches[j]);
	//}

	cv::BFMatcher matcher(cv::NORM_HAMMING);
	vector<vector<cv::DMatch>> getmatches;

	cv::DMatch bestMatch, betterMatch;

	vector<cv::DMatch>bestMatches;

	matcher.knnMatch(lastdescriptors,newdescriptors, getmatches, 2);

	int n = 0;	cv::Point p1, p2;	
	for (int i = 0; i < (int)getmatches.size(); i++)
	{ 
		bestMatch = getmatches[i][0];
		betterMatch = getmatches[i][1];
		p1 = lastkeypoints[bestMatch.trainIdx].pt;
		p2 = newkeypoints[bestMatch.queryIdx].pt;
		double distance = sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));	

		float distanceRatio = bestMatch.distance / betterMatch.distance;  	

		if (distanceRatio < 0.8 && distance < 30) 
		{	
			bestMatches.push_back(bestMatch); 						
		}
	}
	
	double distance=0;

	for (size_t i = 0; i < bestMatches.size(); i++)
	{
		distance += bestMatches[i].distance;
	}

	if (distance/bestMatches.size() >20)
	{
		acept = false;
	}
	lastFrame = newFrame;

	return acept;

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
	pass.setFilterLimits(0.0f, 2.5f);
	pass.filter(*filteredCloud);



	//-- Down sampling

	pcl::VoxelGrid<pointType> passVG;
	passVG.setInputCloud(filteredCloud);
	passVG.setLeafSize(0.02f, 0.02f, 0.02f);
	passVG.filter(*filteredCloud);

	//-- Get ground pcl

	pass.setInputCloud(filteredCloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.5f, 0.5f);
	pass.filter(*forgroundcloud);

	pass.setInputCloud(forgroundcloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0f, 1.0f);
	pass.filter(*forgroundcloud);

	passVG.setInputCloud(forgroundcloud);
	passVG.setLeafSize(0.02f, 0.02f, 0.02f);
	passVG.filter(*forgroundcloud);

	//-- Remove outliers
	//start = chrono::steady_clock::now();
	pcl::StatisticalOutlierRemoval<pointType> passSOR;
	passSOR.setInputCloud(filteredCloud);
	passSOR.setMeanK(20);
	passSOR.setStddevMulThresh(0.1);
	passSOR.filter(*filteredCloud);



	// cout << double(totalTime.count()) / 1000.0f <<" "<<  double(totalTime1.count()) / 1000.0f <<" " <<double(totalTime2.count()) / 1000.0f <<" "<< endl;
}

pcl::ModelCoefficients::Ptr RobotLocator::extractGroundCoeff(pPointCloud cloud)
{
	dstCloud->clear();
	groundcloud->clear();
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

	for (size_t i = 0; i < inliers->indices.size(); ++i)
	{
		tmpPoint = cloud->points[inliers->indices[i]];
		tmpPoint.r = 255;
		tmpPoint.g = 255;
		tmpPoint.b = 0;
		groundcloud->points.push_back(tmpPoint);
	}
	groundpointnumber = inliers->indices.size();

	return groundCoeff;
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
	//rotateToXZPlane.rotate(Eigen::AngleAxisf(angleAlpha, Eigen::Vector3f::UnitX()));
	rotateToXZPlane.rotate(Eigen::AngleAxisf(angle, -Axis));

	//-- Apply transform
	pcl::transformPointCloud(*cloud, *cloud, rotateToXZPlane);

	//-- Define the rote angle about y-axis

	angle = -25.0f/180.0f*PI;

	Eigen::Affine3f roteY= Eigen::Affine3f::Identity();

	roteY.rotate(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY()));
	Eigen::Affine3f rotated = roteY * rotateToXZPlane;

	//-- Apply transform
	pcl::transformPointCloud(*cloud, *cloud, roteY); 


	//-- Update rotated ground coefficients
	Eigen::Vector3f groundCoeffRotatedVec = rotateToXZPlane * vecNormal;

	groundCoeffRotated->values[0] = groundCoeffRotatedVec[0];
	groundCoeffRotated->values[1] = groundCoeffRotatedVec[1];
	groundCoeffRotated->values[2] = groundCoeffRotatedVec[2];
	groundCoeffRotated->values[3] = fabs(groundCoeff->values[3]) / vecNormal.norm();


	cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " "
		<< groundCoeffRotated->values[1] << " "
		<< groundCoeffRotated->values[2] << " "
		<< groundCoeffRotated->values[3];

	return cloud;
}

pPointCloud RobotLocator::removeHorizontalPlane(pPointCloud cloud, bool onlyGround)
{
	verticalCloud->clear();
	outgroundcloud->clear();

	pointType tmpPoint;

	//-- Vector of plane normal and every point on the plane
	Vector3d vecNormal(groundCoeffRotated->values[0], groundCoeffRotated->values[1], groundCoeffRotated->values[2]);
	Vector3d vecPoint(0, 0, 0);
	Vector3d vecPointall(0, 0, 0);

	// cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " " 
	//                                 << groundCoeffRotated->values[1] << " "
	//                                 << groundCoeffRotated->values[2] << " " 
	//                                 << groundCoeffRotated->values[3] << endl;

	//-- Plane normal estimating
	pcl::NormalEstimationOMP<pointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pointType>::Ptr tree(new pcl::search::KdTree<pointType>());
	//pcl::search::Octree<pointType>::Ptr tree(new pcl::search::Octree<pointType>());
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

			if (angleCosine < 0.80 || distanceToPlane > 0.05)
			{
				verticalCloud->points.push_back(cloud->points[i]);
			}
			else if (angleCosine > 0.80 && distanceToPlane > 0.05)
			{
				outgroundcloud->points.push_back(cloud->points[i]);
			}
		}
	}

	if (outgroundcloud->points.size() > groundpointnumber)
	{
		for (size_t i = 0; i < outgroundcloud->points.size(); i++)
		{
			vecPointall[0] += normal->points[i].normal_x;
			vecPointall[1] += normal->points[i].normal_y;
			vecPointall[2] += normal->points[i].normal_z;

		}
		groundCoeffRotated->values[0] = vecPointall[0] / outgroundcloud->points.size();
		groundCoeffRotated->values[1] = vecPointall[1] / outgroundcloud->points.size();
		groundCoeffRotated->values[2] = vecPointall[2] / outgroundcloud->points.size();
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
	extractGroundCoeff(forgroundcloud);

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

		if (angleCosine > 0.90 && distanceToPlane < 0.05)
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
	leftFenseROI = updateObjectROI(verticalCloud, inliers, 0.7, 0.7, 0.5, 0.5);

	Vector3d normalleft(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

	Vector3d vecNormal = Vector3d(groundCoeffRotated->values[0], groundCoeffRotated->values[1], groundCoeffRotated->values[2]);
	double cameraHeight = abs(groundCoeffRotated->values[3]) / vecNormal.norm();

	double xDistance = abs(coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / vecNormal.norm();

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

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*verticalCloud, *inliers, minVector, maxVector);
	//-- Get point cloud indices inside given ROI
	//start = chrono::steady_clock::now();
	//pcl::PassThrough<pointType> pass;
	//pass.setInputCloud(verticalCloud);
	//pass.setFilterFieldName("x");
	//pass.setFilterLimits(duneROI.xMin, duneROI.xMax);
	//pass.filter(inliers->indices);

	//pass.setInputCloud(verticalCloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(duneROI.zMin, duneROI.zMax);
	//pass.setIndices(inliers);
	//pass.filter(inliers->indices);

	////stop = chrono::steady_clock::now();
	////totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);

	//// -- Change the color of the extracted part for debuging
	//// start = chrono::steady_clock::now();
	//for (int i = 0; i < inliers->indices.size(); i++)
	//{
	//	dstCloud->points[inliers->indices[i]].r = 251;
	//	dstCloud->points[inliers->indices[i]].g = 188;
	//	dstCloud->points[inliers->indices[i]].b = 5;
	//}



	// stop = chrono::steady_clock::now();


	if (maxVector[2] < 1.80f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }
	diatancemeasurement = xDistance;

	cout << maxVector[2] << " " << xDistance << " " << plus_minus * acos(angleCosine) / PI * 180 << endl;

	dstViewer->updatePointCloud(groundcloud, "ground Cloud");
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

	leftFenseROI = updateObjectROI(verticalCloud, inliers, 0.7, 0.7, 0.5, 0.5);

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
	double xDistance = abs(coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / vecNormal.norm();

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


	vecNormal = Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	//-- The formula of dune is ax + by + cz + d = 0, which x = 0.0 and y = cameraHeight - 0.05
	double zDistance = -(coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / coefficients->values[2];

	double duneDistance = abs(coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / vecNormal.norm();

	if (zDistance < 1.40f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }

	diatancemeasurement = duneDistance;
	anglemeasurement = plus_minus * acos(angleCosine) / PI * 180;
	//totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
	cout << " dune coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3];

	cout << " " << cameraHeight << endl;
	//cout << "duneDistance  " << duneDistance << "Z distance  " << zDistance << " " << "leftX distance  " << xDistance << " " << "angle  " << plus_minus * acos(angleCosine) / PI * 180 << endl;
	dstViewer->updatePointCloud(groundcloud, "ground Cloud");
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

	if (filteredCloud->points.size() == 0)
	{
		cout << "'error";
		
	}

	extractPlaneWithinROI(verticalCloud, duneROI, inliers, coefficients);
	duneROI = updateObjectROI(verticalCloud, inliers, 0.7, 0.7, 0.5, 0.5);

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
	double duneDistance = abs(coefficients->values[1] * (cameraHeight - 0.05) + coefficients->values[3]) / vecNormal.norm();

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
	diatancemeasurement = duneDistance;                                                                                                                                                                                                                                                                                                                              
	anglemeasurement = plus_minus * acos(angleCosine) / PI * 180 - 45;
	//cout << "Dune distance  " << duneDistance << " z_anxis " << plus_minus * acos(angleCosine) / PI * 180 - 45 <<  endl;

	cout << " dune coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3];
	cout << " " << cameraHeight << endl;
	dstViewer->updatePointCloud(groundcloud, "ground Cloud");
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

	frontFenseROI = updateObjectROI(verticalCloud, largestIndice, 0.7, 0.0, 0.2, 0.2);

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
	double fenseDistance = maxVector[2];

	// // if (fenseDistance < 0.5f) { nextStatusCounter++; }
	// // else { nextStatusCounter = 0; }

	// // if (nextStatusCounter >= 3) { status++; }

	cout << "front fense distance  " << fenseDistance << endl;

	dstViewer->updatePointCloud(groundcloud, "Destination Cloud");
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
	//for (int i = 0; i < inliers->indices.size(); i++)
	//{
	 //   dstCloud->points[inliers->indices[i]].r = 251;
	 //   dstCloud->points[inliers->indices[i]].g = 188;
	 //   dstCloud->points[inliers->indices[i]].b = 5;
   // }

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*verticalCloud, *inliers, minVector, maxVector);

	//-- Calculate the vertical distance to front fense
	double fenseDistance = minVector[2];
	double fenseCornerX = minVector[0];

	// if (fenseCornerX > 0.0f) { nextStatusCounter++; }
	// else { nextStatusCounter = 0; }

	// if (nextStatusCounter >= 3) { status++; }

	cout << "front fense distance  " << fenseDistance << "  x_" << fenseCornerX << endl;

	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
}

void RobotLocator::locateBeforeGrasslandStage2(void)
{
	extractVerticalCloud(filteredCloud);



	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
}

bool RobotLocator::isStoped(void)
{
	return dstViewer->wasStopped();
}

