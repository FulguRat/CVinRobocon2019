#include "robot_locator.h"
#include "getStatus.h"
mutex removeMutex;
RobotLocator::RobotLocator() : srcCloud(new pointCloud),
filteredCloud(new pointCloud),
verticalCloud(new pointCloud),
dstCloud(new pointCloud),
forGroundCloud(new pointCloud),
getFenseVerticalCloud(new pointCloud),
getDuneVerticalCloud(new pointCloud),
fenseCloud(new pointCloud),
duneCloud(new pointCloud),
duneShowCloud(new pointCloud),
fenseShowCloud(new pointCloud),
indicesROI(new pcl::PointIndices),
groundCoeff(new pcl::ModelCoefficients),
groundCoeffRotated(new pcl::ModelCoefficients),
dstViewer(new pcl::visualization::PCLVisualizer("Advanced Viewer"))
{
	
	dstViewer->setBackgroundColor(0.259, 0.522, 0.957);
    	dstViewer->addPointCloud<pointType>(forGroundCloud, "ground Cloud");
    	dstViewer->addPointCloud<pointType>(fenseShowCloud, "fense Cloud");
    	dstViewer->addPointCloud<pointType>(duneShowCloud, "dune Cloud");
    	dstViewer->addCoordinateSystem(0.2, "view point");
    	dstViewer->initCameraParameters();
}

RobotLocator::~RobotLocator()
{
 
}
void RobotLocator::updatemodeROI(void)
{
	if (mode == LEFT_MODE)
	{
		leftFenseROImax = { -1.0/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		leftFenseROImin = { -1.0/*xMin*/, -0.1/*xMax*/, 0.0/*zMin*/, 1.5/*zMax*/ };
		duneROI = { -0.7/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		// frontFenseROI = { -1.3/*xMin*/,  0.3/*xMax*/, 1.2/*zMin*/, 2.1/*zMax*/ };
		frontFenseROI = { -0.7/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 1.7/*zMax*/ };
		grasslandFenseROI = { -0.6/*xMin*/,  0.4/*xMax*/, 0.0/*zMin*/, 3.0/*zMax*/ };
		rihgtOrLeft = -1.0f;
		lastAngle = -25.0f;
	}
	else
	{
		leftFenseROImax = { 0.2/*xMin*/, 1.0/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		leftFenseROImin = { 0.1/*xMin*/, 1.0/*xMax*/, 0.0/*zMin*/, 1.5/*zMax*/ };
		duneROI = { -0.1/*xMin*/,  0.7/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		// frontFenseROI = { -1.3/*xMin*/,  0.3/*xMax*/, 1.2/*zMin*/, 2.1/*zMax*/ };
		frontFenseROI = { -0.1/*xMin*/,  0.7/*xMax*/, 0.0/*zMin*/, 1.7/*zMax*/ };
		grasslandFenseROI = { -0.4/*xMin*/,  0.6/*xMax*/, 0.0/*zMin*/, 3.0/*zMax*/ };
		rihgtOrLeft = 1.0f;
		lastAngle = 25.0f;
	}
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
		thisD435->update();
		//mb_cuda::thrust_to_pcl(thisD435->sourceThrust, srcCloud);
		cout<<"ground cloud size"<<thisD435->groundCloud->points.size()<<endl;
		pcl::PassThrough<pointType> pass;
		pass.setInputCloud(thisD435->groundCloud);
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
	thisD435->initFlag = false;
}

void RobotLocator::updateCloud(void)
{
	//-- copy the pointer to srcCloud
	thisD435->duneROI = duneROI;
	thisD435->fenseROI = leftFenseROImax;
	thisD435->roteAngle = lastAngle / 180 * 3.14;
	cout << "rotate an" << thisD435->roteAngle / 3.14 * 180 << endl;
	thisD435->update();

}
void RobotLocator::cloudUpdate(void)
{
	while (true)
	{
		if (status > 4)
			return;

		updateCloud();

	}
}
void RobotLocator::cloudPointPreprocess(void)
{
	while (true)
	{
		if(status > BEFORE_DUNE_STAGE_3 && status != BONE_RECOGNITION && status != WAIT_STATUS)
		{
			updateCloud();
			auto start = chrono::steady_clock::now();
			//subsampling
			pcl::ApproximateVoxelGrid<pointType> passVG;
			passVG.setInputCloud(thisD435->cloudByRS2);
			passVG.setLeafSize(0.025f, 0.025f, 0.025f);
			passVG.filter(*forGroundCloud);		
		
			extractGroundCoeff(forGroundCloud);
			rotatePointCloudToHorizontal(forGroundCloud);
			auto stop = chrono::steady_clock::now();
			auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
			cout << "thread2 Time: : " << double(totalTime.count()) / 1000.0f << endl;
		}
		

	}

}
void voxelGrid(pPointCloud sorceCloud, pPointCloud cloud, float leafSize)
{
	pcl::ApproximateVoxelGrid<pointType> voxel;
	voxel.setInputCloud(sorceCloud);
	voxel.setLeafSize(leafSize, leafSize, leafSize);
	voxel.filter(*cloud);
}
void RobotLocator::preProcess(void)
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;

	//-- Pass through filter
	float leafSize = 0.025;
	start = chrono::steady_clock::now();
cout<<"error"<<endl;
	if (status < 4)
	{

		//pcl::ApproximateVoxelGrid<pointType> voxel;
		//voxel.setInputCloud(thisD435->groundCloud);
		//voxel.setLeafSize(leafSize, leafSize, leafSize);
		//voxel.filter(*forGroundCloud);
		while (!thisD435->ifUpdate)
		{}
		thisD435->mutex4.lock();
		thisD435->ifUpdate = false;


		if (status == 1)
		{
			thread groundVoxel(voxelGrid, thisD435->groundCloud, forGroundCloud, 0.025);
			thread fenseVoxel(voxelGrid, thisD435->fenseCloud, fenseCloud, 0.025);
			fenseVoxel.join();
			groundVoxel.join();
		}
		else if (status == 2)
		{
			thread groundVoxel(voxelGrid, thisD435->groundCloud, forGroundCloud, 0.025);
			thread fenseVoxel(voxelGrid, thisD435->fenseCloud, fenseCloud, 0.025);
			thread duneVoxel(voxelGrid, thisD435->duneCloud, duneCloud, 0.025);
			duneVoxel.join();
			groundVoxel.join();
			fenseVoxel.join();
		}
		else if (status == 3)
		{
			thread groundVoxel(voxelGrid, thisD435->groundCloud, forGroundCloud, 0.025);
			thread duneVoxel(voxelGrid, thisD435->duneCloud, duneCloud, 0.025);
			duneVoxel.join();
			groundVoxel.join();
		}
		thisD435->mutex4.unlock();
	}


	//-- Remove outliers
	//start = chrono::steady_clock::now();
	//pcl::StatisticalOutlierRemoval<pointType> passSOR;
	//passSOR.setInputCloud(filteredCloud);
	//passSOR.setMeanK(10);
	//passSOR.setStddevMulThresh(0.1);
	//passSOR.filter(*filteredCloud);
	// cout << double(totalTime.count()) / 1000.0f <<" "<<  double(totalTime1.count()) / 1000.0f <<" " <<double(totalTime2.count()) / 1000.0f <<" "<< endl;
	cout << "forground" << forGroundCloud->points.size() << endl;
	stop = chrono::steady_clock::now();
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	cout << "transform data: " << double(totalTime.count()) / 1000.0f << endl;

}

bool RobotLocator::extractGroundCoeff(pPointCloud cloud)
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
	if (status < PASSING_DUNE)
	{
		if (angleCosine > 0.8f && distDifference < 0.04f)
		{
			groundCoeff = coefficients;			
		}
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.push(thisD435->groundCoeff);
		thisD435->mutex3.unlock();
	}
	else
	{
		if (angleCosine > 0.8f)
		{
			thisD435->groundCoeff[0] = coefficients->values[0];
			thisD435->groundCoeff[1] = coefficients->values[1];
			thisD435->groundCoeff[2] = coefficients->values[2];
			thisD435->groundCoeff[3] = coefficients->values[3];
			if(status == BEFORE_GRASSLAND_STAGE_1)
			{
				if(thisD435->groundCoeff[3] < 0.51)
					thisD435->groundCoeff[3] += 0.1;
			}
			thisD435->mutex3.lock();
			thisD435->groundCoffQueue.push(thisD435->groundCoeff);
			thisD435->mutex3.unlock();
		
		}
		else
		{
			thisD435->mutex3.lock();
			thisD435->groundCoffQueue.push(thisD435->groundCoeff);
			thisD435->mutex3.unlock();
		}
		
	}
#ifdef DEBUG
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		cloud->points[inliers->indices[i]].r = 251;
		cloud->points[inliers->indices[i]].g = 255;
		cloud->points[inliers->indices[i]].b = 5;
	}
#endif
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

	if (status >= PASSING_DUNE)
	{
		//-- Define the rotate transform
		Eigen::Affine3f rotateToXZPlane = Eigen::Affine3f::Identity();
		Eigen::AngleAxisf v1(angle, -Axis);
		Eigen::Matrix3f RotatedMatrix = v1.toRotationMatrix();
		while(thisD435->RotatedMatrix.size() != 0)
		{
		}
		thisD435->mutex3.lock();
		thisD435->RotatedMatrix.push(RotatedMatrix);
		thisD435->mutex3.unlock();
#ifndef DEBUG
		rotateToXZPlane.rotate(v1);
		pcl::transformPointCloud(*cloud, *cloud, rotateToXZPlane);

		Eigen::Vector3f groundCoeffRotatedVec = rotateToXZPlane * vecNormal;

		groundCoeffRotated->values[0] = groundCoeffRotatedVec[0];
		groundCoeffRotated->values[1] = groundCoeffRotatedVec[1];
		groundCoeffRotated->values[2] = groundCoeffRotatedVec[2];
		groundCoeffRotated->values[3] = fabs(groundCoeff->values[3]) / vecNormal.norm();
#endif
	}
	else 
	{		
		Eigen::Affine3f rotateToXZPlane = Eigen::Affine3f::Identity();
		Eigen::AngleAxisf v1(angle, -Axis);
		Eigen::Matrix3f RotatedMatrix = v1.toRotationMatrix();
		cout<<"error1 1"<<endl;
		while (thisD435->RotatedMatrix.size() != 0)
		{
		}
		cout<<"error1 2"<<endl;
		thisD435->mutex3.lock();
		thisD435->RotatedMatrix.push(RotatedMatrix);
		thisD435->mutex3.unlock();
		cout<<"error1 3"<<endl;
		//Eigen::AngleAxisf t_V(angle, -Axis);

		float angle = lastAngle / 180 * 3.14;
		Eigen::AngleAxisf t_V1(angle, Eigen::Vector3f::UnitY()), t_V2;

		Eigen::Matrix3f t_R, t_R1, t_R2;

		t_R = v1.toRotationMatrix();
		t_R1 = t_V1.toRotationMatrix();
		t_R2 = t_R1 * t_R;

		t_V2.fromRotationMatrix(t_R2);

		rotateToXZPlane = Eigen::Affine3f::Identity();
		rotateToXZPlane.rotate(t_V2);

		if (status == 1)
		{
			pcl::transformPointCloud(*fenseCloud, *fenseCloud, rotateToXZPlane);
		}
		else if (status == 2)
		{
			pcl::transformPointCloud(*fenseCloud, *fenseCloud, rotateToXZPlane);
			pcl::transformPointCloud(*duneCloud, *duneCloud, rotateToXZPlane);
		}
		else if (status == 3)
		{
			pcl::transformPointCloud(*duneCloud, *duneCloud, rotateToXZPlane);
		}
		else
		{
			pcl::transformPointCloud(*cloud, *cloud, rotateToXZPlane);
		}

		Eigen::Vector3f groundCoeffRotatedVec = rotateToXZPlane * vecNormal;

		groundCoeffRotated->values[0] = groundCoeffRotatedVec[0];
		groundCoeffRotated->values[1] = groundCoeffRotatedVec[1];
		groundCoeffRotated->values[2] = groundCoeffRotatedVec[2];
		groundCoeffRotated->values[3] = fabs(groundCoeff->values[3]) / vecNormal.norm();
	}



#ifdef DEBUG
	if (status > 4)
		cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " "
		<< groundCoeffRotated->values[1] << " "
		<< groundCoeffRotated->values[2] << " "
		<< groundCoeffRotated->values[3] << endl;
#endif // DEBUG 


	return cloud;
}

pPointCloud removeHorizontalPlane(pPointCloud cloud, pPointCloud outCloud, pPointCloud showCloud, bool onlyGround, pcl::ModelCoefficients::Ptr groundCoeffRotated)
{
	outCloud->clear();

	//-- Vector of plane normal and every point on the plane
	cout<<"removemutex.lock"<<endl;
	removeMutex.lock();
	Vector3d vecNormal(groundCoeffRotated->values[0], groundCoeffRotated->values[1], groundCoeffRotated->values[2]);
	removeMutex.unlock();
	cout<<"removemutex.unlock"<<endl;

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
				outCloud->points.push_back(cloud->points[i]);
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
				outCloud->points.push_back(cloud->points[i]);
			}
		}
	}


	//-- Remove Outliers

	pcl::StatisticalOutlierRemoval<pointType> passSOR;
	passSOR.setInputCloud(outCloud);
	passSOR.setMeanK(30);
	passSOR.setStddevMulThresh(0.1);
	passSOR.filter(*outCloud);

	//-- Copy points from verticalCloud to dstCloud
#ifdef DEBUG

	pointType tmpPoint;
	showCloud->clear();

	for (size_t i = 0; i < outCloud->points.size(); ++i)
	{
		tmpPoint = outCloud->points[i];
		tmpPoint.r = 0;
		tmpPoint.g = 0;
		tmpPoint.b = 0;
		showCloud->points.push_back(tmpPoint);
	}

#endif
	return outCloud;
}

void RobotLocator::extractVerticalCloud(void)
{
#ifdef DEBUG
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
	start = chrono::steady_clock::now();
#endif

	extractGroundCoeff(forGroundCloud);
	if (!segmentStatus)
	{
		return;
	}
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);

	cout<<" iferror1 "<<endl;
	//-- Rotate the point cloud to horizontal
	start = chrono::steady_clock::now();
#endif
	//-- Rotate the point cloud to horizontal
	rotatePointCloudToHorizontal(fenseCloud);

#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);

	cout<<" iferror2 "<<endl;
	//-- Rotate the point cloud to horizontal
	start = chrono::steady_clock::now();
#endif
	//-- Remove all horizontal planes

	if (status == 1)
	{
		removeHorizontalPlane(fenseCloud, getFenseVerticalCloud, fenseShowCloud, false, groundCoeffRotated);
	}
	else if (status == 2)
	{
		thread removeFense(removeHorizontalPlane, fenseCloud, getFenseVerticalCloud, fenseShowCloud, false, groundCoeffRotated);
		thread removeDune(removeHorizontalPlane, duneCloud, getDuneVerticalCloud, duneShowCloud, false, groundCoeffRotated);
		removeDune.join();
		removeFense.join();
	}
	else
	{
		removeHorizontalPlane(duneCloud, getDuneVerticalCloud, duneShowCloud, false, groundCoeffRotated);
	}

#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
	cout << "extractVerticalCloud 3 " << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << " " << double(totalTime2.count()) / 1000.0f << endl;
#endif
	return;

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

		if (angleCosine > 0.9 && distanceToPlane < 0.75)
		{
			indices->indices.push_back(indicesROI->indices[i]);
		}
	}
	if (indices->indices.size() == 0)
	{
		segmentStatus = false;
		return;
	}

}
void RobotLocator::extractPlaneWithinROI(pPointCloud cloud, ObjectROI roi,
	pcl::PointIndices::Ptr indices, pcl::PointIndices::Ptr cloudIndices, pcl::ModelCoefficients::Ptr coefficients)
{
	pcl::PointIndices::Ptr filterInliers(new pcl::PointIndices);
	//-- Get point cloud indices inside given ROI
	pcl::PassThrough<pointType> pass;
	pass.setIndices(cloudIndices);
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(roi.xMin, roi.xMax);
	pass.filter(filterInliers->indices);

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(roi.zMin, roi.zMax);
	pass.setIndices(filterInliers);
	pass.filter(filterInliers->indices);

	//-- Plane model segmentation
	pcl::SACSegmentation<pointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setIndices(filterInliers);

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
	for (size_t i = 0; i < filterInliers->indices.size(); i++)
	{
		vecPoint[0] = normal->points[filterInliers->indices[i]].normal_x;
		vecPoint[1] = normal->points[filterInliers->indices[i]].normal_y;
		vecPoint[2] = normal->points[filterInliers->indices[i]].normal_z;

		double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));
		double distanceToPlane = abs(coefficients->values[0] * cloud->points[filterInliers->indices[i]].x +
			coefficients->values[1] * cloud->points[filterInliers->indices[i]].y +
			coefficients->values[2] * cloud->points[filterInliers->indices[i]].z +
			coefficients->values[3]) / vecNormal.norm();

		if (angleCosine > 0.9 && distanceToPlane < 0.07)
		{
			indices->indices.push_back(filterInliers->indices[i]);
		}
	}
	if (indices->indices.size() == 0)
	{
		segmentStatus = false;
		return;
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
	Vector3d planeNormal = Vector3d(planecoefficients->values[0], planecoefficients->values[1], planecoefficients->values[2]);

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
	start = chrono::steady_clock::now(); 
#endif
	dbStatus = 0;
	thisD435->imgProcess();
	preProcess();
	extractVerticalCloud();
	if (!segmentStatus)
	{
		std::cout << "error:: step1 can't find vertical cloud" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}
	//cout << "fencloud size " << getFenseVerticalCloud->points.size() << endl;
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif // DEBUG 

	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
#ifdef DEBUG
	start = chrono::steady_clock::now();
#endif // DEBUG
	cout << "fencverticalcloud size " << getFenseVerticalCloud->points.size() << endl;

	extractPlaneWithinROI(getFenseVerticalCloud, leftFenseROImax, inliers, coefficients);
	if (!segmentStatus)
	{
		std::cout << "error:: step1 can't find maxfense" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif
	leftFenseROImax = updateObjectROI(getFenseVerticalCloud, inliers, 0.2, 0.2, 0.2, 0.2, true, false, leftFenseROImax);

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*getFenseVerticalCloud, *inliers, minVector, maxVector);

	for (int i = 0; i < inliers->indices.size(); i++)
	{
		fenseShowCloud->points[inliers->indices[i]].r = 234;
		fenseShowCloud->points[inliers->indices[i]].g = 67;
		fenseShowCloud->points[inliers->indices[i]].b = 53;

	}
#ifdef DEBUG
	start = chrono::steady_clock::now();
#endif // DEBUG

	indicesROI = inliers;
	extractPlaneWithinROI(getFenseVerticalCloud, leftFenseROImin, inliers, indicesROI, coefficients);
	if (!segmentStatus)
	{
		std::cout << "error:: step1 can't find minfense" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}
	leftFenseROImin = updateObjectROI(getFenseVerticalCloud, inliers, 0.2, 0.2, 0.2, 0.2, true, false, leftFenseROImin);

#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);

#endif

	double xDistance = calculateDistance(groundCoeffRotated, coefficients);

	leftFenseROImax.zMax = 1.7;
	if (!mode)
	{
		if (leftFenseROImax.xMin < -1.2)
			leftFenseROImax.xMin = -1.2;

		if (leftFenseROImax.xMax > 0)
			leftFenseROImax.xMax = 0;

		duneROI.xMin = leftFenseROImax.xMax - 0.2;
		duneROI.xMax = leftFenseROImax.xMax + 1.2;
		duneROI.zMin = maxVector[2] - 0.2;
		duneROI.zMax = maxVector[2] + 0.9;

		if (duneROI.xMin < -1.0)
			duneROI.xMin = -1.0;

		if (duneROI.xMax > 0.5)
			duneROI.xMax = 0.5;


	}
	else
	{
		if (leftFenseROImax.xMin < 0)
			leftFenseROImax.xMin = 0;

		if (leftFenseROImax.xMax > 1.2)
			leftFenseROImax.xMax = 1.2;

		duneROI.xMin = leftFenseROImax.xMin - 1.2;
		duneROI.xMax = leftFenseROImax.xMin + 0.2;
		duneROI.zMin = maxVector[2] - 0.2;
		duneROI.zMax = maxVector[2] + 0.9;
		if (duneROI.xMin < -0.5)
			duneROI.xMin = -0.5;

		if (duneROI.xMax > 1.0)
			duneROI.xMax = 1.0;

	}
	vecXAxis2d = Vector2d(-rihgtOrLeft, 0);
	normalleft2d = Vector2d(coefficients->values[0], coefficients->values[2]);
	angleCosine = abs(normalleft2d.dot(vecXAxis2d) / (normalleft2d.norm() * vecXAxis2d.norm()));
	if (coefficients->values[0] * coefficients->values[2] > 0)
		plus_minus = 1.0f;
	else plus_minus = -1.0f;
	addAngle = plus_minus * acos(angleCosine) / PI * 180;
	lastAngle = (lastAngle + addAngle);
	addAngle=0.0f;
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->cameraYawAngle = lastAngle * PI / 180;
	
	frontFenseDist = thisD435->GetDepth(thisD435->fenseCorner, thisD435->fenseCornerIn3D);

	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	double duneDistance = calculateDistance(groundCoeffRotated, coefficients);
	if(thisD435->targetFoundFlag)
		frontDist = frontFenseDist;
	else
		frontDist = 0;
	if (maxVector[2] < 1.40f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }
	if(nextStatusCounter==1)
	thisD435->addDune=true;
	if (nextStatusCounter >= 2) { status++; thisD435->status++;}
	lateralDist = xDistance * 1000;

	thisD435->duneROI = duneROI;
	thisD435->fenseROI = leftFenseROImax;
	thisD435->roteAngle = lastAngle/180*3.14;

#ifdef DEBUG
	cout << "frontFenseDist  " << frontFenseDist << endl;
	cout << " step1 date3 " << maxVector[2] << " " << xDistance << " " << endl;
	dstViewer->updatePointCloud(forGroundCloud, "ground Cloud");
	dstViewer->updatePointCloud(fenseShowCloud, "fense Cloud");
	dstViewer->spinOnce(1);
	cout << " locateBeforeDuneStage1 2 " << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << " " << double(totalTime2.count()) / 1000.0f << endl;
#endif


}

void RobotLocator::locateBeforeDuneStage2(void)
{
#ifdef DEBUG
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime3 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime4 = chrono::duration_cast<chrono::microseconds>(stop - start);
	start = chrono::steady_clock::now();
#endif
	dbStatus = 1;
	cout << "roteangle" << lastAngle << endl;
	Eigen::Vector4f minVector, maxVector;
	thisD435->imgProcess();
	preProcess();
	extractVerticalCloud();
	if (!segmentStatus)
	{
		std::cout << "error:: step2 can't find  vertical cloud" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}
	pcl::getMinMax3D(*getFenseVerticalCloud, minVector, maxVector);
	cout << maxVector[0] << " " << maxVector[2] << " " << endl;
	cout << minVector[0] << " " << minVector[2] << " " << endl;
	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	cout << "fensecloud " << getFenseVerticalCloud->points.size() << endl;
	extractPlaneWithinROI(getFenseVerticalCloud, leftFenseROImax, inliers, coefficients);
	if (!segmentStatus)
	{
		std::cout << "error:: step2 can't find maxfense" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}
	cout << inliers->indices.size() << endl;
	vecXAxis2d = Vector2d(-rihgtOrLeft, 0);
	normalleft2d = Vector2d(coefficients->values[0], coefficients->values[2]);
	angleCosine = abs(normalleft2d.dot(vecXAxis2d) / (normalleft2d.norm() * vecXAxis2d.norm()));


	while (angleCosine < 0.95)
	{
//		cout << "angleCosine<0.8 " << angleCosine << endl;
//		pcl::getMinMax3D(*getFenseVerticalCloud, *inliers, minVector, maxVector);
//		cout << maxVector[0] << " " << maxVector[2] << " " << endl;
//		cout << minVector[0] << " " << minVector[2] << " " << endl;

//		pcl::ExtractIndices<pointType> extract;

		//		extract.setInputCloud(getFenseVerticalCloud);
		//		extract.setIndices(inliers);
		//		extract.setNegative(true);
		//		extract.filter(inliers->indices);
//		cout << inliers->indices.size() << endl;
		//-- Get point cloud indices inside given ROI
		pcl::getMinMax3D(*getFenseVerticalCloud, *inliers, minVector, maxVector);
//		cout << maxVector[0] << " " << maxVector[2] << " " << endl;
//		cout << minVector[0] << " " << minVector[2] << " " << endl;

		if (mode == LEFT_MODE)
		{
			leftFenseROImax.xMin = minVector[0] - 0.3;
			leftFenseROImax.xMax = minVector[0] + 0.3;
			leftFenseROImax.zMin = maxVector[2] - 1.4;
			leftFenseROImax.zMax = maxVector[2] - 0.2;
		}
		else
		{
			leftFenseROImax.xMin = maxVector[0] - 0.3;
			leftFenseROImax.xMax = maxVector[0] + 0.3;
			leftFenseROImax.zMin = maxVector[2] - 1.4;
			leftFenseROImax.zMax = maxVector[2] - 0.2;
		}
		cout << "leftFenseROImax " << leftFenseROImax.xMin << " " << leftFenseROImax.xMax << " " << leftFenseROImax.zMin << " " << leftFenseROImax.zMax << endl;
		//		pcl::PassThrough<pointType> pass;
		//		pass.setInputCloud(getFenseVerticalCloud);
		//		pass.setFilterFieldName("z");
		//		pass.setFilterLimits(leftFenseROImax.zMin, leftFenseROImax.zMax);
		//		pass.setIndices(inliers);
		//		pass.filter(inliers->indices);
		//		cout<<inliers->indices.size()<<endl;
		//		pass.setInputCloud(getFenseVerticalCloud);
		//		pass.setFilterFieldName("x");
		//		pass.setFilterLimits(leftFenseROImax.xMin, leftFenseROImax.xMax);
		//		pass.setIndices(inliers);
		//		pass.filter(inliers->indices);
		//		cout<<inliers->indices.size()<<endl;	

				//-- Plane model segmentation

		//		pcl::SACSegmentation<pointType> seg;
		//		seg.setOptimizeCoefficients(true);
		//		seg.setModelType(pcl::SACMODEL_PLANE);
		//		seg.setMethodType(pcl::SAC_RANSAC);
		//		seg.setDistanceThreshold(0.01);
		//		seg.setIndices(inliers);

		//		seg.setInputCloud(getFenseVerticalCloud);
		//		seg.segment(*inliers, *coefficients);
		extractPlaneWithinROI(getFenseVerticalCloud, leftFenseROImax, inliers, coefficients);
		cout << inliers->indices.size() << endl;
		pcl::getMinMax3D(*getFenseVerticalCloud, *inliers, minVector, maxVector);
		if (inliers->indices.size() == 0)
		{
			std::cout << "error:: step2 can't find maxfense again" << endl;
			thisD435->mutex3.lock();
			thisD435->groundCoffQueue.pop();
			thisD435->RotatedMatrix.pop();
			thisD435->mutex3.unlock();
			return;
		}
		normalleft2d = Vector2d(coefficients->values[0], coefficients->values[2]);
		angleCosine = abs(normalleft2d.dot(vecXAxis2d) / (normalleft2d.norm() * vecXAxis2d.norm()));
	}
	cout << maxVector[0] << " " << maxVector[2] << " " << endl;
	cout << minVector[0] << " " << minVector[2] << " " << endl;
	cout << inliers->indices.size() << endl;
	pcl::getMinMax3D(*getFenseVerticalCloud, *inliers, minVector, maxVector);
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		fenseShowCloud->points[inliers->indices[i]].r = 234;
		fenseShowCloud->points[inliers->indices[i]].g = 67;
		fenseShowCloud->points[inliers->indices[i]].b = 53;
	}

	if (coefficients->values[0] * coefficients->values[2] > 0)
		plus_minus = 1.0f;
	else plus_minus = -1.0f;

	addAngle = plus_minus * acos(angleCosine) / PI * 180;
	lastAngle = (lastAngle + addAngle);
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->cameraYawAngle = lastAngle * PI / 180;
	
	frontFenseDist = thisD435->GetDepth(thisD435->fenseCorner, thisD435->fenseCornerIn3D);

	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	if(thisD435->targetFoundFlag)
		frontDist = frontFenseDist;
	else
		frontDist = 0;
	addAngle = 0.0f;
	//lastAngle=25;

	if (mode == LEFT_MODE)
	{
		duneROI.xMin = leftFenseROImax.xMax - 0.2;
		duneROI.xMax = leftFenseROImax.xMax + 1.2;
		duneROI.zMin = maxVector[2] - 0.2;
		duneROI.zMax = maxVector[2] + 0.9;
	}
	else
	{
		duneROI.xMin = leftFenseROImax.xMin - 1.2;
		duneROI.xMax = leftFenseROImax.xMin + 0.2;
		duneROI.zMin = maxVector[2] - 0.2;
		duneROI.zMax = maxVector[2] + 0.9;
	}
	cout << maxVector[0] << " " << maxVector[2] << " " << endl;
	cout << minVector[0] << " " << minVector[2] << " " << endl;
	cout << "duneROI1  " << duneROI.xMin << " " << duneROI.xMax << " " << duneROI.zMin << " " << duneROI.zMax << endl;
	leftFenseROImax = updateObjectROI(getFenseVerticalCloud, inliers, 0.1, 0.1, 0.2, 0.2, true, true, leftFenseROImax);

	leftFenseROImax.zMin = 0;

	leftFenseROImin = updateObjectROI(getFenseVerticalCloud, inliers, 0.3, 0.3, 0.1, 0.1, true, false, leftFenseROImin);

	//-- The formula of dune is ax + by + cz + d = 0, which z = 0.0 and y = cameraHeight - 0.05
	double xDistance = calculateDistance(groundCoeffRotated, coefficients);

	if (mode == LEFT_MODE)
	{
		if (leftFenseROImax.xMin < -1.2)
			leftFenseROImax.xMin = -1.2;

		if (leftFenseROImax.xMax > -0.25)
			leftFenseROImax.xMax = -0.25;

		if (duneROI.xMin < -1.0)
			duneROI.xMin = -1.0;

		if (duneROI.xMax > 0.7)
			duneROI.xMax = 0.7;

	}
	else
	{
		if (leftFenseROImax.xMin < 0.25)
			leftFenseROImax.xMin = 0.25;

		if (leftFenseROImax.xMax > 1.0)
			leftFenseROImax.xMax = 1.0;

		if (duneROI.xMin < -0.7)
			duneROI.xMin = -0.7;

		if (duneROI.xMax > 1.0)
			duneROI.xMax = 1.0;

	}

	pcl::PointIndices::Ptr duneInliers(new pcl::PointIndices);
	extractPlaneWithinROI(getDuneVerticalCloud, duneROI, duneInliers, coefficients);
	if (!segmentStatus)
	{
		std::cout << "error:: step2 can't find dune" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}
	double duneDistance = calculateDistance(groundCoeffRotated, coefficients);


	if (frontFenseDist < 700 && frontFenseDist > 500) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; thisD435->status++;}

	diatancemeasurement = duneDistance;

	lateralDist = xDistance * 1000;

	thisD435->duneROI = duneROI;
	thisD435->fenseROI = leftFenseROImax;
	thisD435->roteAngle = lastAngle/180*3.14;
	//-- Change the color of the extracted part for debuging
#ifdef DEBUG
	for (int i = 0; i < duneInliers->indices.size(); i++)
	{
		duneShowCloud->points[duneInliers->indices[i]].r = 251;
		duneShowCloud->points[duneInliers->indices[i]].g = 188;
		duneShowCloud->points[duneInliers->indices[i]].b = 5;
	}
	cout << "duneROI" << duneROI.xMin << " " << duneROI.xMax << " " << duneROI.zMin << " " << duneROI.zMax << " " << endl;
	cout << "frontFenseDist  " << frontFenseDist << " " << "leftX distance  " << xDistance << endl;
	dstViewer->updatePointCloud(forGroundCloud, "ground Cloud");
	dstViewer->updatePointCloud(fenseShowCloud, "fense Cloud");
	dstViewer->updatePointCloud(duneShowCloud, "dune Cloud");
	dstViewer->spinOnce(1);
	cout << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << " " << double(totalTime2.count()) / 1000.0f << " " << double(totalTime3.count()) / 1000.0f << endl;

#endif
}

void RobotLocator::locateBeforeDuneStage3(void)
{
#ifdef DEBUG
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
	auto totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
	start = chrono::steady_clock::now();
#endif
	dbStatus = 1;
	thisD435->imgProcess();
	preProcess();
	extractVerticalCloud();
	if (!segmentStatus)
	{
		std::cout << "error:: step3 can't find vertical cloud" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif
	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

#ifdef DEBUG
	start = chrono::steady_clock::now();
#endif // DEBUG

	extractPlaneWithinROI(getDuneVerticalCloud, duneROI, inliers, coefficients);
	if (!segmentStatus)
	{
		std::cout << "error:: step3 can't find dune" << endl;
		thisD435->mutex3.lock();
		thisD435->groundCoffQueue.pop();
		thisD435->RotatedMatrix.pop();
		thisD435->mutex3.unlock();
		return;
	}

#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif
	duneROI = updateObjectROI(getDuneVerticalCloud, inliers, 0.3, 0.3, 0.2, 0.3, true, true, duneROI);

	if (!mode)
	{
		if (duneROI.xMin < -1.0)
			duneROI.xMin = -1.0;

		if (duneROI.xMax > 0.1)
			duneROI.xMax = 0.1;
	}
	else
	{
		if (duneROI.xMin < -0.1)
			duneROI.xMin = -0.1;

		if (duneROI.xMax > 1.0)
			duneROI.xMax = 1.0;
	}

	vecXAxis2d = Vector2d(-rihgtOrLeft, 0);
	normalleft2d = Vector2d(coefficients->values[0], coefficients->values[2]);
	angleCosine = abs(normalleft2d.dot(vecXAxis2d) / (normalleft2d.norm() * vecXAxis2d.norm()));
	if (coefficients->values[0] * coefficients->values[2] > 0)
		plus_minus = 1.0f;
	else plus_minus = -1.0f;
	addAngle = plus_minus * acos(angleCosine) / PI * 180-rihgtOrLeft*45;
	lastAngle = (lastAngle + addAngle);
	addAngle = 0.0f;
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->cameraYawAngle = (lastAngle - rihgtOrLeft*45) * PI / 180;
	cout << "cameraYawAngle: " << thisD435->cameraYawAngle / PI * 180 << endl;
	frontFenseDist = thisD435->GetDepth(thisD435->besideTarget, thisD435->besideTargetIn3D);

	besideFenseDist = duneLine2FenseDist + (mode == LEFT_MODE ?thisD435->nowXpos : -thisD435->nowXpos);

	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	
	if(thisD435->targetFoundFlag)
		lateralDist = besideFenseDist;
	else
		lateralDist = 0;
	
	
	double duneDistance = calculateDistance(groundCoeffRotated, coefficients);
	frontDist = duneDistance * 1000;
	thisD435->duneROI = duneROI;
	//thisD435->fenseROI = leftFenseROImax;
	thisD435->roteAngle = lastAngle/180*3.14;

	if (duneDistance < 0.4f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 1)
	{
		status++;
		thisD435->status++;
		thisD435->xkFlag = true;
	}
#ifdef DEBUG
	//-- Change the color of the extracted part for debuging
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		duneShowCloud->points[inliers->indices[i]].r = 251;
		duneShowCloud->points[inliers->indices[i]].g = 188;
		duneShowCloud->points[inliers->indices[i]].b = 5;
	}

	//	anglemeasurement = plus_minus * acos(angleCosine) / PI * 180 - 45;
	cout << "lateralDist: " << lateralDist << "Dune distance  " << duneDistance << " z_anxis " << lastAngle <<endl;

	//	cout << " dune coefficients: " << coefficients->values[0] << " "
	//		<< coefficients->values[1] << " "
	//		<< coefficients->values[2] << " "
	//		<< coefficients->values[3];
	//	cout << " " << cameraHeight << endl;
	cout << " locateBeforeDuneStage1 3 " << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << endl;
	dstViewer->updatePointCloud(forGroundCloud, "ground Cloud");
	dstViewer->updatePointCloud(duneShowCloud, "dune Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locatePassingDune(void)
{
	thisD435->imgProcess();
	
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1,thisD435->linePt2,HORIZONAL_FENSE);

	frontFenseDist = thisD435->GetDepth(thisD435->fenseCorner, thisD435->fenseCornerIn3D);

	besideFenseDist = (mode == LEFT_MODE ?thisD435->nowXpos : -thisD435->nowXpos);

	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	
	if(mode == LEFT_MODE)
		frontDist = frontFenseDist - line2LeftDuneDist;
	else
		frontDist = frontFenseDist - line2RightDuneDist;
	lateralDist = besideFenseDist;
	if(!thisD435->targetFoundFlag)	
	{
		frontDist = 0;
		lateralDist = 0;
	}

	cout << "linept1: " << thisD435->linePt1.x << " " << thisD435->linePt1.y << "linept2: " << thisD435->linePt2.x << " " << thisD435->linePt2.y << endl;
	dbStatus = 1;
	if (frontFenseDist < 900 && frontFenseDist > 700) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status = status;}
	
#ifdef DEBUG
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
	cout << "status: " << status << "\n" << endl;
	cout << "Dune distance  " << frontDist << "  besideFenseDist " << besideFenseDist << endl;
	cout << thisD435->srcImageQueue.size() << endl;
	cout << thisD435->RotatedMatrix.size() << endl;
#endif
}

void RobotLocator::locateBeforeGrasslandStage1(void)
{
	thisD435->imgProcess();

	thisD435->FindFenseCorner(HORIZONAL_FENSE, mode);
	
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1,thisD435->linePt2,HORIZONAL_FENSE);

	frontFenseDist = thisD435->GetDepth(thisD435->fenseCorner, thisD435->fenseCornerIn3D);

	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	besideFenseDist = mode == LEFT_MODE ? fenseCorner2fenseDist - thisD435->nowXpos : fenseCorner2fenseDist + thisD435->nowXpos;
	if(thisD435->lineFoundFlag)
	{
		lateralDist = besideFenseDist;
		frontDist = frontFenseDist;
	}
	else
	{
		lateralDist = 0;
		frontDist = 0;
	}
	cout << "linept1: " << thisD435->linePt1.x << " " << thisD435->linePt1.y << "linept2: " << thisD435->linePt2.x << " " << thisD435->linePt2.y << endl;
	dbStatus = 2;
	if (besideFenseDist < 1600 && besideFenseDist > 1400 && frontFenseDist < 1000 && frontFenseDist > 500) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status = status;}
	
#ifdef DEBUG
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
	cout << "status: " << status << "\n" << endl;
	cout << "front fense distance  " << frontFenseDist << "  besideFenseDist " << besideFenseDist << endl;
	cout << thisD435->srcImageQueue.size() << endl;
	cout << thisD435->RotatedMatrix.size() << endl;
#endif
}
void RobotLocator::locateBeforeGrasslandStage2(void)
{
	thisD435->imgProcess();

	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1,thisD435->linePt2,VERTICAL_FENSE);
	secondRopeDist = thisD435->GetDepth(thisD435->center2, thisD435->center2In3D);
	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	if ((1 - 2 * mode) * thisD435->nowXpos + 400 < 0)
	{
		besideFenseDist = mode == LEFT_MODE ? (pillarRadius - thisD435->nowXpos) : (pillarRadius + thisD435->nowXpos);
	}
	else
	{
		besideFenseDist = mode == LEFT_MODE ? (fenseCorner2fenseDist - pillarRadius - thisD435->nowXpos) : (fenseCorner2fenseDist - pillarRadius + thisD435->nowXpos);
	}

	lateralDist = besideFenseDist;
	frontDist = secondRopeDist;

	dbStatus = 3;
	if (besideFenseDist > 550 && besideFenseDist < 850 && secondRopeDist > 500 && secondRopeDist < 2000) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status ++; thisD435->status ++;}
#ifdef DEBUG
	cout << "status: " << status << "\n" << endl;
	cout << "secondRopeDist: " << secondRopeDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << thisD435->srcImageQueue.size() << endl;
	cout << thisD435->RotatedMatrix.size() << endl;
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif // DEBUG
}
void RobotLocator::locatePassingGrasslandStage1(void)
{
	thisD435->imgProcess();

	if (secondRopeDist > 800)
		thisD435->FindVerticalHoughLine(thisD435->grayImage);
	else
		thisD435->FindVerticalHoughLine(thisD435->grayImage, 1);

	while (thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1, thisD435->linePt2, VERTICAL_FENSE);
	secondRopeDist = thisD435->GetDepth(thisD435->center2, thisD435->center2In3D);
	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();

	besideFenseDist = mode == LEFT_MODE ? (pillarRadius - thisD435->nowXpos) : (pillarRadius + thisD435->nowXpos);

	lateralDist = besideFenseDist;
	frontDist = secondRopeDist;

	dbStatus = 3;

	if (secondRopeDist < 500 && secondRopeDist > 300) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }
	
	if (nextStatusCounter >= 2) { status+=2; thisD435->status+=2; }

#ifdef DEBUG
	cout << "secondRopeDist: " << secondRopeDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << "status: " << status << "\n" << endl;
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locatePassingGrasslandStage2(void)
{
	thisD435->imgProcess();
	
	while (thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1, thisD435->linePt2, VERTICAL_FENSE);
	frontFenseDist = thisD435->GetDepth(thisD435->lineCross, thisD435->lineCrossIn3D);
	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();

	besideFenseDist = mode == LEFT_MODE ? -thisD435->nowXpos : thisD435->nowXpos;
	secondRopeDist = frontFenseDist - lineCross2RopeDist;

	lateralDist = besideFenseDist;
	frontDist = secondRopeDist;

	dbStatus = 3;

	if (besideFenseDist > 650 && besideFenseDist < 850) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }
#ifdef DEBUG
	cout << "secondRopeDist: " << secondRopeDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << "status: " << status << "\n" << endl;
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locateUnderMountain(void)
{
	thisD435->imgProcess();

	thisD435->FindLineEnd();
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1,thisD435->linePt2,VERTICAL_FENSE);
	frontFenseDist = thisD435->GetDepth(thisD435->lineEnd, thisD435->lineEndIn3D);
	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	besideFenseDist = mode == LEFT_MODE ? lineEnd2BesidefenseDist - thisD435->nowXpos : lineEnd2BesidefenseDist + thisD435->nowXpos;

	if (frontFenseDist > lineEnd2secondRopeDist - 310)
	{
		frontFenseDist -= lineEnd2secondRopeDist;
		dbStatus = 3;
	}
	else
	{
		dbStatus = 4;
		besideFenseDist = 1440 - besideFenseDist;
	}
	//ͣ��
	lateralDist = besideFenseDist;
	frontDist = frontFenseDist;
	if (0) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }
#ifdef DEBUG
	cout << "frontFenseDist: " << frontFenseDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << "status: " << status << "\n" << endl;
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locateClimbingMountain(void)
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;

	thisD435->imgProcess();

	int stageJudge = thisD435->ClimbingMountainStageJudge();
	
	thisD435->FindHoughLineCross();
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1,thisD435->linePt2,HORIZONAL_FENSE);
	frontFenseDist = thisD435->GetDepth(thisD435->lineCross, thisD435->lineCrossIn3D);
	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	dbStatus = 5;
	switch (stageJudge)
	{
		case 2:
		{
			if (thisD435->filterLine.size() == 2 && (thisD435->nowXpos * (2 * mode - 1)) < 0)
			{
				dbStatus = 5;
				peakDist = mode == LEFT_MODE ? thisD435->nowXpos : -thisD435->nowXpos;
			}
			else
				peakDist = 0;
		}
		break;

		case 3:
		{
			if (thisD435->filterLine.size() == 2)
			{
				peakDist = mode == LEFT_MODE ? thisD435->nowXpos : -thisD435->nowXpos;
				dbStatus = 6;
			}			
			else
				peakDist = 0;
		}		
		break;
	}
	lateralDist = peakDist;
	frontDist = frontFenseDist;
	if (stageJudge == 3 && thisD435->filterLine.size() != 2) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }

#ifndef DEBUG

	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
	cout << "frontFenseDist:" << frontFenseDist << " peakDist:" << peakDist << endl;

	cout << "climbStage: "<< stageJudge << " status: " << status << "\n" << endl;
#endif

}
void RobotLocator::locateReachingMountain(void)
{
	thisD435->imgProcess();

	thisD435->FindHoughLineCross();
	while(thisD435->RotatedMatrix.size() == 0)
	{
	}
	thisD435->mutex3.lock();
	cout << "got coeff" << endl;
	thisD435->GetyawAngle(thisD435->linePt1,thisD435->linePt2,HORIZONAL_FENSE);
	frontFenseDist = thisD435->GetDepth(thisD435->lineCross, thisD435->lineCrossIn3D);
	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	if (thisD435->filterLine.size() == 2 && (thisD435->nowXpos * (2 * mode - 1)) < 0)
		peakDist = mode == LEFT_MODE ? thisD435->nowXpos : -thisD435->nowXpos;
	else
		peakDist = 0;
	lateralDist = peakDist;
	frontDist = frontFenseDist;

	dbStatus = 7;
	if (0) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status++; }
#ifdef DEBUG
	cout << "PeakDist: " << peakDist << endl;
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif // DEBUG
}
bool RobotLocator::isStoped(void)
{
	return dstViewer->wasStopped();
}
