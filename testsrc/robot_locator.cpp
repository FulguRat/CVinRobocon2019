#include "robot_locator.h"
#include "getStatus.h"

RobotLocator::RobotLocator() : srcCloud(new pointCloud),
filteredCloud(new pointCloud),
verticalCloud(new pointCloud),
dstCloud(new pointCloud),
forGroundCloud(new pointCloud),
afterRoteCloud(new pointCloud),
getVerticalCloud(new pointCloud),
finalVerticalCloud(new pointCloud),
indicesROI(new pcl::PointIndices),
groundCoeff(new pcl::ModelCoefficients),
groundCoeffRotated(new pcl::ModelCoefficients),
groundCoeffFinal(new pcl::ModelCoefficients),
pointNormal(new pcl::PointCloud<pcl::Normal>),
dstViewer(new pcl::visualization::PCLVisualizer("Advanced Viewer"))
{
	if (!mode)
	{
		leftFenseROImax = { -0.9/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		leftFenseROImin = { -0.9/*xMin*/, -0.2/*xMax*/, 0.0/*zMin*/, 1.0/*zMax*/ };
		duneROI = { -0.7/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		// frontFenseROI = { -1.3/*xMin*/,  0.3/*xMax*/, 1.2/*zMin*/, 2.1/*zMax*/ };
		frontFenseROI = { -0.7/*xMin*/,  0.1/*xMax*/, 0.0/*zMin*/, 1.7/*zMax*/ };
		grasslandFenseROI = { -0.6/*xMin*/,  0.4/*xMax*/, 0.0/*zMin*/, 3.0/*zMax*/ };
	}
	else
	{
		leftFenseROImax = { 0.2/*xMin*/, 0.9/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		leftFenseROImin = { 0.2/*xMin*/, 0.9/*xMax*/, 0.0/*zMin*/, 1.0/*zMax*/ };
		duneROI = { -0.1/*xMin*/,  0.7/*xMax*/, 0.0/*zMin*/, 2.0/*zMax*/ };
		// frontFenseROI = { -1.3/*xMin*/,  0.3/*xMax*/, 1.2/*zMin*/, 2.1/*zMax*/ };
		frontFenseROI = { -0.1/*xMin*/,  0.7/*xMax*/, 0.0/*zMin*/, 1.7/*zMax*/ };
		grasslandFenseROI = { -0.4/*xMin*/,  0.6/*xMax*/, 0.0/*zMin*/, 3.0/*zMax*/ };
	}


	dstViewer->setBackgroundColor(0.259, 0.522, 0.957);
    dstViewer->addPointCloud<pointType>(forGroundCloud, "ground Cloud");
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
		thisD435->update();
		mb_cuda::thrust_to_pcl(thisD435->sourceThrust, srcCloud);

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
	thisD435->initFlag = false;
}

void RobotLocator::updateCloud(void)
{
	//-- copy the pointer to srcCloud
	thisD435->update();
}
void RobotLocator::updateSrcCloud(void)
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	int count = 0; 
	while (true)
	{
		//if(status != BONE_RECOGNITION)
		{
			count ++;
			start = chrono::steady_clock::now();
			updateCloud();
	#ifdef DEBUG
			stop = chrono::steady_clock::now();
			auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);

			cout << "thread1 Time: " << double(totalTime.count()) / 1000.0f << endl;
			cout << "thread1 Cnt: " << count << endl;

	#endif // DEBUG
		}



	}
	
}
void RobotLocator::cloudPointPreprocess(void)
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	int count = 0; 
	while (true)
	{
			//start = chrono::steady_clock::now();
			exGroSegmentStatus = true;
			count ++;
			
			preProcess();	


			extractGroundCoeff(forGroundCloud);
start = chrono::steady_clock::now();

			if (!exGroSegmentStatus)
				continue;
			rotatePointCloudToHorizontal(filteredCloud);
			stop = chrono::steady_clock::now();
			auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);

			cout << "thread2 Time: : " << double(totalTime.count()) / 1000.0f << endl;
	#ifdef DEBUG
			//stop = chrono::steady_clock::now();
			//auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);

			//cout << "thread2 Time: : " << double(totalTime.count()) / 1000.0f << endl;
			cout << "thread2 Cnt: " << count << endl;
			cout << thisD435->RotatedMatrix.size() << " " << thisD435->groundCoffQueue.size() << endl;

	#endif // DEBUG	
	}

}
void RobotLocator::getVerticalNormal(void)
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	while (/*status<5*/true)
	{
		cout<<"thread3"<<endl;
		start = chrono::steady_clock::now();
		removeHorizontalPlane(afterRoteCloud);
#ifdef DEBUG
		stop = chrono::steady_clock::now();
		auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
		coutMutex.lock();
		cout << "thread3 Time: : " << double(totalTime.count()) / 1000.0f << endl;
		coutMutex.unlock();
#endif // DEBUG
	}
	return ;
}
void RobotLocator::preProcess(void)
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;

	
	//-- Pass through filter
	thrust::device_vector<mb_cuda::PointXYZRGB> device_cloud;
	while(!thisD435->pointCloudUpdateFlag)
	{
	}
	thisD435->mutex2.lock();
	mb_cuda::host_to_device(thisD435->sourceThrust, device_cloud);
	thisD435->pointCloudUpdateFlag = false;
	thisD435->mutex2.unlock();
	thrust::device_vector<mb_cuda::PointXYZRGB> d_filtered_cloud;

	groundROI = { -0.5,0.5,0,1.0 };
	
	
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
		mb_cuda::pass_through_filter(d_filtered_cloud, d_filtered_cloud, 'z', 0.0f, 1.5f);

		mb_cuda::pass_through_filter(d_filtered_cloud, d_filtered_cloud, 'x', 0.0f, 1.0f);
		
		mb_cuda::pass_through_filter(device_cloud, d_filtered_cloud, 'y', 0.0f, 1.5f);


				

		

	}
	//-- Down sampling
	start = chrono::steady_clock::now();
	float leafSize = 0.025;
	mb_cuda::removeNansOrIfs(d_filtered_cloud, d_filtered_cloud);
	cout<<"cuda false1"<<endl;
	mb_cuda::voxel_grid_filter(d_filtered_cloud, d_filtered_cloud, leafSize);
	cout<<"cuda false2"<<endl;
		stop = chrono::steady_clock::now();
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	cout << "transform data: " << double(totalTime.count()) / 1000.0f << endl;
	//--To filteredCloud
	thrust::host_vector<mb_cuda::PointXYZRGB> hostCloud;

	//for ground point
	thrust::device_vector<mb_cuda::PointXYZRGB> forThrustGroundCloud;

	if(status <= 4)
	{
		mb_cuda::device_to_host(d_filtered_cloud, hostCloud);

		mb_cuda::thrust_to_pcl(hostCloud, filteredCloud);
	

		mb_cuda::pass_through_filter(d_filtered_cloud, forThrustGroundCloud, 'z', groundROI.zMin, groundROI.zMax);

		mb_cuda::pass_through_filter(forThrustGroundCloud, forThrustGroundCloud, 'x', groundROI.xMin, groundROI.xMax);

		mb_cuda::device_to_host(forThrustGroundCloud, hostCloud);

		mutex4.lock();
		mb_cuda::thrust_to_pcl(hostCloud, forGroundCloud);
		mutex4.unlock();
		//-- Remove outliers
		//start = chrono::steady_clock::now();
		pcl::StatisticalOutlierRemoval<pointType> passSOR;

		passSOR.setInputCloud(filteredCloud);
		passSOR.setMeanK(10);
		passSOR.setStddevMulThresh(0.1);
	
		passSOR.filter(*filteredCloud);

		// cout << double(totalTime.count()) / 1000.0f <<" "<<  double(totalTime1.count()) / 1000.0f <<" " <<double(totalTime2.count()) / 1000.0f <<" "<< endl;
	}
	else
	{
		mb_cuda::device_to_host(d_filtered_cloud, hostCloud);

		mb_cuda::thrust_to_pcl(hostCloud, forGroundCloud);

	}

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
		cout<<"cant get ground"<<endl;
		exGroSegmentStatus = false;
		return exGroSegmentStatus;
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
		if(status == BEFORE_GRASSLAND_STAGE_1)
		{
			if(thisD435->groundCoeff[3] < 0.50)
				thisD435->groundCoeff[3] += 0.1;
		}

		thisD435->groundCoffQueue.push(thisD435->groundCoeff);

	}
#ifdef DEBUG

	for (int i = 0; i < inliers->indices.size(); i++)
	{
		cloud->points[inliers->indices[i]].r = 251;
		cloud->points[inliers->indices[i]].g = 255;
		cloud->points[inliers->indices[i]].b = 5;
	}

#endif
	return exGroSegmentStatus;
}

void RobotLocator::rotatePointCloudToHorizontal(pPointCloud cloud)
{

	Eigen::Vector3f vecNormal(groundCoeff->values[0], groundCoeff->values[1], groundCoeff->values[2]);
	//-- Define the rotate angle about x-axis
	//double angleAlpha = atan(-groundCoeff->values[2] / groundCoeff->values[1]);
	double angle = acos(-groundCoeff->values[1] * groundCoeff->values[3] / (vecNormal.norm() * fabs(groundCoeff->values[3])));
	//cout << "angle: " << angle * 180 / CV_PI;
	Eigen::Vector3f Axis = (-groundCoeff->values[3] / fabs(groundCoeff->values[3])) * vecNormal.cross(Eigen::Vector3f(0, -1, 0));
	Axis.normalize();

	if (status>4)
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
		rotateToXZPlane.rotate(v1); \

		pcl::transformPointCloud(*forGroundCloud, *forGroundCloud, rotateToXZPlane);

		Eigen::Vector3f groundCoeffRotatedVec = rotateToXZPlane * vecNormal;

		groundCoeffRotated->values[0] = groundCoeffRotatedVec[0];
		groundCoeffRotated->values[1] = groundCoeffRotatedVec[1];
		groundCoeffRotated->values[2] = groundCoeffRotatedVec[2];
		groundCoeffRotated->values[3] = fabs(groundCoeff->values[3]) / vecNormal.norm();
#endif
	}
	else 
	{		

		Eigen::AngleAxisf t_V(angle, -Axis);

		angle = -25.0f / 180.0f*3.14;

		Eigen::AngleAxisf t_V1(angle, Eigen::Vector3f::UnitY()), t_V2;

		Eigen::Matrix3f t_R, t_R1, t_R2;

		t_R = t_V.toRotationMatrix();
		t_R1 = t_V1.toRotationMatrix();
		t_R2 = t_R1 * t_R;

		t_V2.fromRotationMatrix(t_R2);

		Eigen::Affine3f rotateToXZPlane = Eigen::Affine3f::Identity();
		rotateToXZPlane.rotate(t_V2);

		mutex1.lock();
		pcl::transformPointCloud(*cloud, *afterRoteCloud, rotateToXZPlane);
		mutex1.unlock();

		Eigen::Vector3f groundCoeffRotatedVec = rotateToXZPlane * vecNormal;

		groundCoeffRotated->values[0] = groundCoeffRotatedVec[0];
		groundCoeffRotated->values[1] = groundCoeffRotatedVec[1];
		groundCoeffRotated->values[2] = groundCoeffRotatedVec[2];
		groundCoeffRotated->values[3] = fabs(groundCoeff->values[3]) / vecNormal.norm();
	}

	while(ifRotated)
	{}

	mutex2.lock();
	groundCoeffFinal = groundCoeffRotated;
	ifRotated=true;
	mutex2.unlock();
	
#ifndef DEBUG
	if (status > 4)
		cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " "
		<< groundCoeffRotated->values[1] << " "
		<< groundCoeffRotated->values[2] << " "
		<< groundCoeffRotated->values[3] << endl;
#endif // DEBUG 


}

pPointCloud RobotLocator::removeHorizontalPlane(pPointCloud cloud, bool onlyGround)
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	start = chrono::steady_clock::now(); 
	while (!ifRotated)
	{
	}
	stop=chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	coutMutex.lock();
	cout<<"thread3 wait time "<<double(totalTime.count()) / 1000.0f<<endl;
	coutMutex.unlock();

	verticalCloud->clear();

	pointType tmpPoint;

	mutex1.lock();
	ifRotated=false;
	pPointCloud afterRotatedPoint = cloud;
	mutex1.unlock();

	//-- Vector of plane normal and every point on the plane
	mutex2.lock();
	Vector3d vecNormal(groundCoeffFinal->values[0], groundCoeffFinal->values[1], groundCoeffFinal->values[2]);
	mutex2.unlock();
	Vector3d vecPoint(0, 0, 0);

	// cout << "Ground coefficients: " << groundCoeffRotated->values[0] << " " 
	//                                 << groundCoeffRotated->values[1] << " "
	//                                 << groundCoeffRotated->values[2] << " " 
	//                                 << groundCoeffRotated->values[3] << endl;

	//-- Plane normal estimating
	pcl::NormalEstimationOMP<pointType, pcl::Normal> ne;
	ne.setInputCloud(afterRotatedPoint);

	pcl::search::KdTree<pointType>::Ptr tree(new pcl::search::KdTree<pointType>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.03); /* setKSearch function can be try */
	ne.compute(*normal);

	//-- Compare point normal and plane normal, remove every point on a horizontal plane

	for (size_t i = 0; i < afterRotatedPoint->points.size(); i++)
	{
		vecPoint[0] = normal->points[i].normal_x;
		vecPoint[1] = normal->points[i].normal_y;
		vecPoint[2] = normal->points[i].normal_z;

		if (onlyGround == false)
		{
			double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));

			if (angleCosine < 0.90)
			{
				verticalCloud->points.push_back(afterRotatedPoint->points[i]);
			}
		}
		else
		{
			double angleCosine = abs(vecNormal.dot(vecPoint) / (vecNormal.norm() * vecPoint.norm()));
			double distanceToPlane = abs(groundCoeffRotated->values[0] * afterRotatedPoint->points[i].x +
				groundCoeffRotated->values[1] * afterRotatedPoint->points[i].y +
				groundCoeffRotated->values[2] * afterRotatedPoint->points[i].z +
				groundCoeffRotated->values[3]) / vecNormal.norm();

			if (angleCosine < 0.80 || distanceToPlane > 0.05)
			{
				verticalCloud->points.push_back(afterRotatedPoint->points[i]);
			}
		}
	}


	//-- Remove Outliers

	pcl::StatisticalOutlierRemoval<pointType> passSOR;
	passSOR.setInputCloud(verticalCloud);
	passSOR.setMeanK(30);
	passSOR.setStddevMulThresh(0.1);
	passSOR.filter(*verticalCloud);

	while(ifGetVerticalCloud)
	{}
	mutex3.lock();
	getVerticalCloud = verticalCloud;
	ifGetVerticalCloud = true;
	mutex3.unlock();


#ifdef DEBUG
	//-- Copy points from verticalCloud to dstCloud
	mutex5.lock();
	dstCloud->clear();
	for (size_t i = 0; i < verticalCloud->points.size(); i++)
	{
		tmpPoint = verticalCloud->points[i];
		tmpPoint.r = 0;
		tmpPoint.g = 0;
		tmpPoint.b = 0;
		dstCloud->points.push_back(tmpPoint);
	}
	mutex5.unlock();

#endif // DEBUG


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
		exRoiSegmentStatus = false;
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

	//extractVerticalCloud(filteredCloud);
	//if (!segmentStatus)
	//{
	//	return;
	//}
	while (!ifGetVerticalCloud)
	{
	
	}
	#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif // DEBUG
	mutex3.lock();	
	finalVerticalCloud = getVerticalCloud;	
	ifGetVerticalCloud=false;
	mutex3.unlock();


   //-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
#ifdef DEBUG
	start = chrono::steady_clock::now();
#endif // DEBUG

	extractPlaneWithinROI(finalVerticalCloud, leftFenseROImax, inliers, coefficients);
//cout<<verticalCloud->points.size()<<endl;
//cout<<"false"<<endl;
	if (!exRoiSegmentStatus)
	{
		return;
	}
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif
	leftFenseROImax = updateObjectROI(finalVerticalCloud, inliers, 0.3, 0.3, 0.1, 0.1, true, true, leftFenseROImax);

	Eigen::Vector4f minVector, maxVector;
	pcl::getMinMax3D(*finalVerticalCloud, *inliers, minVector, maxVector);
#ifdef DEBUG
	start = chrono::steady_clock::now();
#endif // DEBUG
	extractPlaneWithinROI(finalVerticalCloud, leftFenseROImin, inliers, coefficients);
	if (!exRoiSegmentStatus)
	{
		return;
	}
	leftFenseROImin = updateObjectROI(finalVerticalCloud, inliers, 0.3, 0.3, 0.1, 0.1, true, false, leftFenseROImin);

#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif
	mutex2.lock();
	double xDistance = calculateDistance(groundCoeffFinal, coefficients);
	mutex2.unlock();

	duneROI.xMin = leftFenseROImin.xMax - 0.3;
	duneROI.xMax = leftFenseROImin.xMax + 0.9;
	duneROI.zMin = leftFenseROImin.zMin + 0.3;
	duneROI.zMax = leftFenseROImin.zMax + 0.9;

	if (maxVector[2] < 1.50f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	//if (nextStatusCounter >= 3) { status++; }
	lateralDist = xDistance;
	frontDist = 0.0f;
	dbStatus = 0;

#ifdef DEBUG
	mutex5.lock();
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 234;
		dstCloud->points[inliers->indices[i]].g = 67;
		dstCloud->points[inliers->indices[i]].b = 53;

	}
	mutex5.unlock();

	mutex4.lock();
	dstViewer->updatePointCloud(forGroundCloud, "ground Cloud");
	mutex4.unlock();
	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
	coutMutex.lock();
	cout << " step1 date3 " << maxVector[2] << " " << xDistance << " " << endl;
	cout << " locateBeforeDuneStage1 2 " << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << " " <<  double(totalTime2.count()) / 1000.0f<<endl;
	coutMutex.unlock();
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
	//extractVerticalCloud(filteredCloud);
	//if (!exRoiSegmentStatus)
	//{
	//	return;
	//}

	mutex3.lock();
	while (!ifGetVerticalCloud)
	{
	}
	finalVerticalCloud = getVerticalCloud;
	ifGetVerticalCloud = false;
	mutex3.unlock();

#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	start = chrono::steady_clock::now();
#endif // DEBUG
	//-- Perform the plane segmentation with specific indices
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

	extractPlaneWithinROI(verticalCloud, leftFenseROImin, inliers, coefficients);
	if (!exRoiSegmentStatus)
	{
		return;
	}
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif // DEBUG


	leftFenseROImin = updateObjectROI(verticalCloud, inliers, 0.2, 0.2, 0.0, 0.0, true, false, leftFenseROImin);

	//-- The formula of dune is ax + by + cz + d = 0, which z = 0.0 and y = cameraHeight - 0.05
	mutex2.lock();
	double xDistance = calculateDistance(groundCoeffFinal, coefficients);
	mutex2.unlock();

	Vector3d vecXAxis(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
#ifdef DEBUG
	mutex5.lock();
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 234;
		dstCloud->points[inliers->indices[i]].g = 67;
		dstCloud->points[inliers->indices[i]].b = 53;
	}
	mutex5.unlock();
	start = chrono::steady_clock::now();
#endif

	extractPlaneWithinROI(verticalCloud, leftFenseROImax, inliers, coefficients);
	if (!exRoiSegmentStatus)
	{
		return;
	}
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime2 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif // DEBUG

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
	if (!exRoiSegmentStatus)
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
	if (!mode)
	{
		duneROI.xMin = leftFenseROImax.xMax - 0.3;
		duneROI.xMax = leftFenseROImax.xMax + 0.9;
		duneROI.zMin = leftFenseROImax.zMax - 0.3;
		duneROI.zMax = leftFenseROImax.zMax + 0.9;
	}
	else
	{
		duneROI.xMin = leftFenseROImax.xMax - 0.9;
		duneROI.xMax = leftFenseROImax.xMax + 0.3;
		duneROI.zMin = leftFenseROImax.zMax - 0.3;
		duneROI.zMax = leftFenseROImax.zMax + 0.9;
	}

#ifdef DEBUG
	start = chrono::steady_clock::now();
#endif // DEBUG
	extractPlaneWithinROI(verticalCloud, duneROI, inliers, coefficients);
	if (!exRoiSegmentStatus)
	{
		return;
	}
#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime3 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif // DEBUG


	mutex2.lock();
	xDistance = calculateDistance(groundCoeffFinal, coefficients);
	mutex2.unlock();

	if (duneDistance < 1.2f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status++; }

	diatancemeasurement = duneDistance;

	lateralDist = xDistance;
	frontDist = duneDistance;
	dbStatus = 1;
	//-- Change the color of the extracted part for debuging
#ifdef DEBUG
	mutex5.lock();
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}
	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	mutex5.unlock();

	mutex4.lock();
	dstViewer->updatePointCloud(forGroundCloud, "ground Cloud");
	mutex4.unlock();
	dstViewer->spinOnce(1);
	coutMutex.lock();
	cout << "duneDistance  " << duneDistance << " " << "leftX distance  " << xDistance << endl;
	cout << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << " " << double(totalTime2.count()) / 1000.0f << " " << double(totalTime3.count()) / 1000.0f << endl;
	coutMutex.unlock();

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
	//extractVerticalCloud(filteredCloud);
	//if (!exRoiSegmentStatus)
	//{
	//	return;
	//}
	mutex3.lock();
	while (!ifGetVerticalCloud)
	{
	}
	finalVerticalCloud = getVerticalCloud;
	ifGetVerticalCloud = false;
	mutex3.unlock();
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

	extractPlaneWithinROI(verticalCloud, duneROI, inliers, coefficients);
	if (!exRoiSegmentStatus)
	{
		return;
	}

#ifdef DEBUG
	stop = chrono::steady_clock::now();
	totalTime1 = chrono::duration_cast<chrono::microseconds>(stop - start);
#endif
	duneROI = updateObjectROI(verticalCloud, inliers, 0.0, 0.0, 0.3, 0.3, false, true, duneROI);

	mutex2.lock();
	double xDistance = calculateDistance(groundCoeffFinal, coefficients);
	mutex2.unlock();
	diatancemeasurement = duneDistance;

	lateralDist = 0;
	frontDist = duneDistance;

	if (duneDistance < 0.3f) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 3) { status+=2; }

	dbStatus = 1;
#ifdef DEBUG
	//-- Change the color of the extracted part for debuging
	mutex5.lock();
	for (int i = 0; i < inliers->indices.size(); i++)
	{
		dstCloud->points[inliers->indices[i]].r = 251;
		dstCloud->points[inliers->indices[i]].g = 188;
		dstCloud->points[inliers->indices[i]].b = 5;
	}
	dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	mutex5.unlock();

	mutex4.lock();
	dstViewer->updatePointCloud(forGroundCloud, "ground Cloud");
	mutex4.unlock();

	dstViewer->spinOnce(1);

	cout << "Dune distance  " << duneDistance << " z_anxis " << endl;
	cout << " locateBeforeDuneStage1 3 " << double(totalTime.count()) / 1000.0f << " " << double(totalTime1.count()) / 1000.0f << endl;
#endif

}

void RobotLocator::locatePassingDune(void)
{
	//extractVerticalCloud(filteredCloud);

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

	if (!exRoiSegmentStatus)
	{
		return;
	}
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

	if (!exRoiSegmentStatus)
	{
		return;
	}
	if (!mode)
	{
		frontFenseROI = updateObjectROI(verticalCloud, inliers, 0.7, 0.0, 0.2, 0.2, true, true, frontFenseROI);
	}
	else
	{
		frontFenseROI = updateObjectROI(verticalCloud, inliers, 0.0, 0.7, 0.2, 0.2, true, true, frontFenseROI);
	}



	mutex2.lock();
	double xDistance = calculateDistance(groundCoeffFinal, coefficients);
	mutex2.unlock();

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

	  cout << "front fense distance  " << fenseDistance  << endl;

	  dstViewer->updatePointCloud(forGroundCloud, "ground Cloud");
	  dstViewer->updatePointCloud(dstCloud, "Destination Cloud");
	  dstViewer->spinOnce(1);
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

	lateralDist = besideFenseDist;
	frontDist = frontFenseDist + carWidth;

	dbStatus = 2;
	if (besideFenseDist < 1600) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	if (nextStatusCounter >= 2) { status++; thisD435->status = status;}
	
#ifdef DEBUG
	//dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	//dstViewer->spinOnce(1);
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
	if (secondRopeDist < 700 && thisD435->pillarStatus != 1) { nextStatusCounter++; }
	else { nextStatusCounter = 0; }

	//if (nextStatusCounter >= 2) { status ++; thisD435->status ++; colorFrameRoi = mode == LEFT_MODE ? rightRoi : leftRoi;}
#ifdef DEBUG
	cout << "status: " << status << "\n" << endl;
	cout << "secondRopeDist: " << secondRopeDist << " besideFenseDist: " << besideFenseDist << endl;
	cout << thisD435->srcImageQueue.size() << endl;
	cout << thisD435->RotatedMatrix.size() << endl;
	//dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	//dstViewer->spinOnce(1);
#endif // DEBUG
}
void RobotLocator::locatePassingGrasslandStage1(void)
{
	extractGroundCoeff(forGroundCloud);
	rotatePointCloudToHorizontal(forGroundCloud);
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
	dstViewer->updatePointCloud(forGroundCloud, "Destination Cloud");
	dstViewer->spinOnce(1);
#endif
}

void RobotLocator::locatePassingGrasslandStage2(void)
{

	extractGroundCoeff(forGroundCloud);
	rotatePointCloudToHorizontal(forGroundCloud);
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
	thisD435->GetyawAngle(thisD435->linePt1,thisD435->linePt2,HORIZONAL_FENSE);
	frontFenseDist = thisD435->GetDepth(thisD435->lineEnd, thisD435->lineEndIn3D) + carWidth;
	thisD435->groundCoffQueue.pop();
	thisD435->RotatedMatrix.pop();
	thisD435->mutex3.unlock();
	besideFenseDist = mode == LEFT_MODE ? lineEnd2BesidefenseDist - thisD435->nowXpos : lineEnd2BesidefenseDist + thisD435->nowXpos;

	if (frontFenseDist > lineEnd2secondRopeDist)
	{
		frontFenseDist -= lineEnd2secondRopeDist;
		dbStatus = 3;
	}
	else
	{
		dbStatus = 4;
		besideFenseDist = 1440 - besideFenseDist;
	}
	if (besideFenseDist > 850 && frontFenseDist < 800)
		colorFrameRoi = mode == LEFT_MODE ? leftRoi : rightRoi;
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
	frontDist = frontFenseDist + carWidth;
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
	frontDist = frontFenseDist + carWidth;

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