#include "act_d435.h"

ActD435::ActD435() : align(RS2_STREAM_COLOR),
cloudByRS2(new pointCloud)/*,
viewer("Temp Viewer")*/
{
	
}

ActD435::~ActD435()
{

}

void ActD435::init(void)
{
	//Kalman Init
	KF.init(2, 1);
	measurement = cv::Mat::zeros(1, 1, CV_32F);

	KF.transitionMatrix = (cv::Mat_<float>(2, 2) << 1, 1, 0, 1);  //转移矩阵A
	setIdentity(KF.measurementMatrix);                                             //测量矩阵H
	setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));                            //系统噪声方差矩阵Q
	setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1));                        //测量噪声方差矩阵R
	setIdentity(KF.errorCovPost, cv::Scalar::all(1));
	randn(KF.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));

	argsLeft = CAMERA_ARGS_LEFT;
	argsRight = CAMERA_ARGS_RIGHT;

	rotationMatrix = ROTATION_MATRIX;
	translationMatrix = TRANSLATION_MATRIX;

	intrinsicMatrixLeft = (cv::Mat_<float>(3, 3) << argsLeft.fx, argsLeft.skew, argsLeft.cx, \
		0.0f, argsLeft.fy, argsLeft.cy, \
		0.0f, 0.0f, 1.0f);
	intrinsicMatrixRight = (cv::Mat_<float>(3, 3) << argsRight.fx, argsRight.skew, argsRight.cx, \
		0.0f, argsRight.fy, argsRight.cy, \
		0.0f, 0.0f, 1.0f);
	groundCoeff.push_back(0);
	groundCoeff.push_back(0);
	groundCoeff.push_back(0);
	groundCoeff.push_back(0);
	//sensor sen;
	//sen.set_option(RS2_OPTION_EXPOSURE, 25);

	//-- Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

	//cfg.enable_device_from_file("1.bag");
	//-- Instruct pipeline to start streaming with the requested configuration
	pipe.start(cfg);

	//-- Wait for frames from the camera to settle
	for (int i = 0; i < 5; i++)
	{
		//Drop several frames for auto-exposure
		frameSet = pipe.wait_for_frames();
	}
}

pPointCloud ActD435::update(void)
{
	static int time = 0;

	//chrono::steady_clock::time_point start = chrono::steady_clock::now();

	//-- Wait for the next set of frames from the camera
	frameSet = pipe.wait_for_frames();

	// chrono::steady_clock::time_point stop = chrono::steady_clock::now();
	// cout << "retrieve time:" << double(totalTime.count()) / 1000.0f << "\t processing time:";
	//start = chrono::steady_clock::now();

	//-- Get processed aligned frame
	//alignedFrameSet = align.process(frameSet);

	//-- Get both color and aligned depth frames
	//rs2::video_frame colorFrame = alignedFrameSet.first(RS2_STREAM_COLOR);
	//rs2::depth_frame alignedDepthFrame = alignedFrameSet.get_depth_frame();

	//-- For not align
	rs2::video_frame colorFrame = frameSet.get_color_frame();
	rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();

	srcImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
	//tColor(srcImage, srcImage, CV_RGB2BGR);

	//-- Map Color texture to each point
	//rs2Cloud.map_to(colorFrame);

	//-- Generate the pointcloud and texture mappings
	rs2Points = rs2Cloud.calculate(alignedDepthFrame);
	cloudByRS2 = pointsToPointCloud(rs2Points);

	//stop = chrono::steady_clock::now();
	//totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	// cout << double(totalTime.count()) / 1000.0f << endl;

	return cloudByRS2;
}
void ActD435::imgProcess()
{
	
	//srcImage = cv::imread("18.jpg");
	cv::split(srcImage, channels);
	channelR = channels[2].clone();
	//dstImage = srcImage.clone();

	//imshow("BGR", srcImage);
	//imshow("ChannelR", channelR);

	cv::cvtColor(srcImage, srcImage, CV_BGR2HSV);
	cv::split(srcImage, channels);
	channelH = channels[0].clone();
	channelS = channels[1].clone();

	//imshow("HSV", srcImage);
	//imshow("ChannelH", channelH);
	//imshow("ChannelS", channelS);

	//inRange(channelR, minRTkb.slider, maxRTkb.slider, channels[0]);
	//inRange(channelH, minHTkb.slider, maxHTkb.slider, channels[1]);
	//inRange(channelS, minSTkb.slider, maxSTkb.slider, channels[2]);

	cv::inRange(channelR, 89, 255, channels[0]);
	cv::inRange(channelH, 6, 38, channels[1]);
	cv::inRange(channelS, 101, 255, channels[2]);

	//imshow("Channel 0", channels[0]);
	//imshow("Channel 1", channels[1]);
	//imshow("Channel 2", channels[2]);

	cv::bitwise_and(channels[0], channels[1], maskImage);
	cv::bitwise_and(maskImage, channels[2], maskImage);
	//dst2Image = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
	//bitwise_and(dstImage, dstImage, dst2Image, maskImage);

	//imshow("dst2", dst2Image);
	cv::imshow("mask", maskImage);
	cv::waitKey(1);
}

void ActD435::FindPillarCenter(void)
{
	cv::Mat dst = maskImage.clone();

	contours.clear();
	hierarchy.clear();

	if (status == PASSING_GRASSLAND_STAGE_1)
	{
		cv::Rect remove(400, 0, 239, 479);
		dst(remove).setTo(0);
	}

	findContours(dst, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	int idx = 0;


	for (size_t i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) > contourArea(contours[idx]))
			idx = i;
	}

	switch (status)
	{
	case BEFORE_GRASSLAND_STAGE_2:
	{
		if (contours.size() < 1)
			break;
		cv::Rect rect = boundingRect(contours[idx]);
		float x = rect.x + rect.width / 2;
		float y = rect.y + rect.height;
		if (rect.width < 100)
		{
			center1 = cv::Point2f(x, y);
		}
		else
		{
			int whitePixNum = 0;
			int index = 0;
			int i = 0;
			for (i = rect.x + rect.width - 20; i > rect.x; i--)
			{
				whitePixNum = 0;
				for (int j = rect.y + rect.height * 4 / 5; j < y; j++)
				{
					if (maskImage.at<uchar>(j, i) > 0)
						whitePixNum++;
				}
				if (whitePixNum <= 1)
				{
					index = i;
					break;
				}
			}
			cout << index << "\t" << rect.x << endl;
			if(index == 0)
				center1 = cv::Point2f(x, y);
			else
				center1 = cv::Point2f((index + rect.x + rect.width) / 2, y);
		}
		circle(dst, center1, 5, 255, -1);

		cout << "x: " << center1.x << "y: " << center1.y << endl;
	}
	break;
	case PASSING_GRASSLAND_STAGE_1:
	{
		if (contours.size() < 1)
			break;
		cv::Rect rect = boundingRect(contours[idx]);
		float x = rect.x + rect.width / 2;
		float y = rect.y + rect.height;

		center2 = cv::Point2f(x, y);

		circle(dst, center2, 5, 255, -1);
		cout << "x: " << center2.x << "y: " << center2.y << endl;
	}
	break;
	}
	imshow("dst", dst);
	cv::waitKey(1);

}

void ActD435::FindFenseCorner(int fenseType)
{
	cv::split(srcImage, channels);
	channelR = channels[2].clone();
	
	cv::inRange(channelR, 100, 255, channels[2]);

	maskImage = channels[2].clone();

	vector<cv::Point2f> line;
	int begin = maskImage.rows - 1;
	int end = 0;
	int cols = 0;
	int countFlag = 0;
	int blackPixNum = 0;
	uchar thresh = 10;

	//按列从最后一行逆序遍历
	if(fenseType == HORZISONAL_FENSE)
		cols = maskImage.cols - 1;
	//按列从最后一行顺序遍历
	else 
		cols = 0;

	do
	{
		blackPixNum = 0;
		int tempbegin = 0;

		if (fenseType == HORZISONAL_FENSE)
			--cols;
		else
			++cols;
		for (int rows = begin; rows > end; --rows)
		{
			if (maskImage.at<uchar>(rows, cols) == 0 && countFlag == 0)
			{
				tempbegin = rows;
				countFlag = 1;
			}
			if (countFlag)
			{
				//begin count
				if (maskImage.at<uchar>(rows, cols) == 0)
					blackPixNum++;
				else
				{
					//remove white noise in the fense
					if (   maskImage.at<uchar>(rows - 1, cols) > 0
						&& maskImage.at<uchar>(rows, cols - 1) > 0
						&& maskImage.at<uchar>(rows + 1, cols) > 0
						&& maskImage.at<uchar>(rows, cols + 1) > 0
						)
						blackPixNum = 0;
					else
						blackPixNum++;
				}
					
				if (blackPixNum > thresh)
				{					
					line.push_back(cv::Point(cols, tempbegin));
					begin = tempbegin + 10;
					end = rows - 15;
					countFlag = 0;
					break;
				}

			}
		}
		if (begin > 479)
			begin = 479;
		if (end < 0)
			end = 0;
	} while (blackPixNum > thresh);

	if (line.size() > 20)
	{
		fenseCorner = line[line.size() - 10];
	}
	
	if (fenseType == HORZISONAL_FENSE)
		GetyawAngle(line[10], line[line.size() - 10], 0);
	else
		GetyawAngle(line[10], line[line.size() - 10], CV_PI / 2);

	cout << "robotYawAngle: " << robotYawAngle << endl;
	circle(maskImage, fenseCorner, 10, 255, -1);

	line.clear();
	cv::imshow("dst", maskImage);
	cvWaitKey(1);
	cout << "x: " << fenseCorner.x << " y:" << fenseCorner.y << endl;
}

void ActD435::FindLineEnd(void)
{
	cv::split(srcImage, channels);
	channelB = channels[0].clone();
	channelR = channels[2].clone();

	cv::inRange(channelB, 0, 110, channels[0]);
	cv::inRange(channelR, 0, 131, channels[2]);

	cv::imshow("channels 0", channels[0]);
	cvWaitKey(1);

	vector<cv::Point2f> line;

	if (MODEL == LEFT_MODEL)
		maskImage = channels[0].clone();
	else
		maskImage = channels[2].clone();


	int begin = 0;
	int end = 0;
	int rows = maskImage.rows - 1;

	if (MODEL == LEFT_MODEL)
	{
		begin = maskImage.cols - 1;
		end = 0;
		
		int countFlag = 0;
		int blackPixNum = 0;
		int blackPixNumLast = 0;
		uchar thresh = 6;

		do
		{
			int tempbegin = 0;
			blackPixNumLast = blackPixNum;
			blackPixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols > end - 10; --cols)
			{
				if (data[cols] == 0 && countFlag == 0)
				{
					tempbegin = cols;
					countFlag = 1;
				}
				if (countFlag)
				{
					//begin count
					if (data[cols] == 0)
						blackPixNum++;
					//finish count
					else
					{
						countFlag = 0;
						//update
						if (blackPixNum < thresh)
						{
							blackPixNum = 0;
							continue;
						}

						if (blackPixNum > thresh)
						{
							if (blackPixNum - blackPixNumLast > 3 || blackPixNum > 100)
							{
								rows -= 20;
								break;
							}
							if (tempbegin > 550 || end > tempbegin)
							{
								continue;
							}
							line.push_back(cv::Point(cols, tempbegin));
							begin = tempbegin + 10;
							end = cols - 10;
							break;
						}
					}
				}
			}
			if (begin > 639)
				begin = 639;
			if (end < 0)
				end = 0;
		} while (blackPixNum > thresh && rows > 0);
	}
	else
	{
		begin = 1;
		end = maskImage.cols / 2;

		int countFlag = 0;
		int blackPixNum = 0;
		int blackPixNumLast = 0;	
		uchar thresh = 5;
	
		do
		{
			int tempbegin = 0;
			blackPixNumLast = blackPixNum;
			blackPixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols < end + 30; ++cols)
			{
				if (data[cols] == 0 && countFlag == 0)
				{
					tempbegin = cols;
					countFlag = 1;
				}
				if (countFlag)
				{
					//begin count
					if (data[cols] == 0)
						blackPixNum++;
					//finish count
					else
					{
						countFlag = 0;
						//update
						if (blackPixNum < thresh)
						{
							blackPixNum = 0;
							continue;
						}

						if (blackPixNum > thresh)
						{
							if (blackPixNum - blackPixNumLast > 5 || blackPixNum > 200)
							{
								rows -= 20;
								break;
							}
							if (tempbegin < 100 || tempbegin > end)
							{
								continue;
							}
							begin = tempbegin;
							end = cols - 1;
							break;
						}
					}
				}
			}
		} while (blackPixNum > thresh && rows > 0);
		
	}

	if (line.size() > 20)
	{
		lineEnd = line[line.size() - 1];
		GetyawAngle(line[10], line[line.size() - 10], CV_PI / 2);
	}

	cout << "robotYawAngle: " << robotYawAngle << endl;

	cv::circle(maskImage, cv::Point((begin - 10) / 2 + end / 2, rows + 1), 3, 0, -1);

	
	cout << "x: " << lineEnd.x << " y:" << lineEnd.y << endl;

	imshow("dst", maskImage);

	cvWaitKey(1);
}

cv::Point3f ActD435::GetIrCorrdinate(cv::Point2f& pt)
{
	float a = groundCoeff[0];
	float b = groundCoeff[1];
	float c = groundCoeff[2];
	float d = groundCoeff[3] * 1000;
	angleAlpha = asin(fabs(c) / sqrt(a * a + b * b + c * c));

	cout << "angleAlpha: " << angleAlpha * 180 / CV_PI << endl;
	cv::Mat RGBpixed;
	RGBpixed = (cv::Mat_<float>(3, 1) << pt.x, pt.y, 1);

	float Zc = 0, Xir = 0, Yir = 0, Zir = 0, m = 0, n = 0, t = 0;
	cv::Mat C = (rotationMatrix.t()) * (translationMatrix);

	cv::Mat K = rotationMatrix.t() * intrinsicMatrixLeft.inv() * (RGBpixed);

	m = K.at<float>(0, 0);
	n = K.at<float>(1, 0);
	t = K.at<float>(2, 0);

	Zc = (-d + (a * C.at<float>(0, 0) + b * C.at<float>(1, 0) + c * C.at<float>(2, 0))) / ((a * m + b * n + c * t));

	Xir = m * Zc - C.at<float>(0, 0);
	Yir = n * Zc - C.at<float>(1, 0);
	Zir = t * Zc - C.at<float>(2, 0);

	cout << "groundCoeff: " << a << "\t" << b << "\t" << c << "\t" << d << endl;
	cout << "IrCoorinate: " << Xir << "\t" << Yir << "\t" << Zir << endl;
	cout << "Zc: " << Zc << "\tZir: " << Zir << endl;

	return cv::Point3f(Xir, Yir, Zir);
}

float ActD435::GetDepth(cv::Point2f& pt,cv::Point3f& pt1)
{
	float realDistance = 0;
	float distance = 0;
	float angle = 0;

	pt1 = GetIrCorrdinate(pt);
	distance = pt1.z * cos(angleAlpha) - pt1.y * sin(angleAlpha);
	angle = atan(pt1.x / distance);
	realDistance = (sqrt(distance * distance + pt1.x * pt1.x)) * cos(cameraYawAngel + robotYawAngle - angle);
	nowXpos = (sqrt(distance * distance + pt1.x * pt1.x)) * sin(cameraYawAngel + robotYawAngle - angle);

	cout << "distance: " << distance << endl;
	cout << "angle: " << angle * 180 / CV_PI << endl;
	return realDistance;
}

float ActD435::GetyawAngle(cv::Point2f& pt1, cv::Point2f& pt2, float angle)
{
	cv::Point3f point1In3D;
	cv::Point3f point2In3D;
	point1In3D = GetIrCorrdinate(pt1);
	point2In3D = GetIrCorrdinate(pt2);

	double rotatedAngle = 0;
	float x = point1In3D.x - point2In3D.x;
	float y = point1In3D.y - point2In3D.y;
	float z = point1In3D.z - point2In3D.z;

	Eigen::Vector3f Vec(x,y,z);
	if(z > 0)
		rotatedAngle = -acos(x / Vec.norm());
	else
		rotatedAngle = acos(x / Vec.norm());
	cout << "rotatedAngle: " << rotatedAngle * 180 / CV_PI;
	
	robotYawAngle = rotatedAngle - cameraYawAngel - angle;

	return robotYawAngle;
}

//======================================================
// getColorTexture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values.
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<uint8_t, uint8_t, uint8_t> ActD435::getColorTexture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
	//-- Get Width and Height coordinates of texture
	int width = texture.get_width();  // Frame width in pixels
	int height = texture.get_height(); // Frame height in pixels

	//-- Normals to Texture Coordinates conversion
	int xValue = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int yValue = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = xValue * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = yValue * texture.get_stride_in_bytes(); // Get line width in bytes
	int textIndex = (bytes + strides);

	const auto newTexture = reinterpret_cast<const uint8_t*>(texture.get_data());

	//-- RGB components to save in tuple
	int newText1 = newTexture[textIndex];
	int newText2 = newTexture[textIndex + 1];
	int newText3 = newTexture[textIndex + 2];

	return std::tuple<uint8_t, uint8_t, uint8_t>(newText1, newText2, newText3);
}

//===================================================
// pointsToPointCloud
// - Function is utilized to fill a point cloud
// object with depth and RGB data from a single
// frame captured using the Realsense.
//===================================================
pPointCloud ActD435::pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color)
{
	// Object Declaration (Point Cloud)
	pPointCloud cloud(new pointCloud);

	// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

	//================================
	// PCL Cloud Object Configuration
	//================================
	// Convert data captured from Realsense camera to Point Cloud
	auto sp = points.get_profile().as<rs2::video_stream_profile>();

	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto textureCoord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	for (int i = 0; i < points.size(); i++)
	{
		//===================================
		// Mapping Depth Coordinates
		// - Depth data stored as XYZ values
		//===================================
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;

		// Obtain color texture for specific point
		RGB_Color = getColorTexture(color, textureCoord[i]);

		// Mapping Color (BGR due to Camera Model)
		//cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
	   // cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
		//cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

	}

	return cloud; // PCL RGB Point Cloud generated
}

//===================================================
// pointsToPointCloud
// - For point cloud without color information
//===================================================
pPointCloud ActD435::pointsToPointCloud(const rs2::points& points)
{
	pPointCloud cloud(new pointCloud);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto ptr = points.get_vertices();

	for (auto& p : cloud->points)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		p.z = ptr->z;
		ptr++;
	}

	return cloud;
}