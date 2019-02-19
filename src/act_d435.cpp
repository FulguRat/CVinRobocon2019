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
	imshow("src", srcImage);
	cvWaitKey(1);
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
	switch (status)
	{
	case BEFORE_GRASSLAND_STAGE_1:
	{
		cv::split(srcImage, channels);
		channelR = channels[2].clone();

		cv::inRange(channelR, 100, 255, channels[2]);
		maskImage = channels[2].clone();
	}
	break;

	case BEFORE_GRASSLAND_STAGE_2:
	{

	}
	case PASSING_GRASSLAND_STAGE_1:
	{
		cv::split(srcImage, channels);
		channelR = channels[2].clone();

		cv::cvtColor(srcImage, srcImage, CV_BGR2HSV);
		cv::split(srcImage, channels);
		channelH = channels[0].clone();
		channelS = channels[1].clone();

		cv::inRange(channelR, 89, 255, channels[0]);
		cv::inRange(channelH, 10, 38, channels[1]);
		cv::inRange(channelS, 101, 255, channels[2]);

		cv::bitwise_and(channels[0], channels[1], maskImage);
		cv::bitwise_and(maskImage, channels[2], maskImage);

		dstImage = maskImage.clone();

		cv::split(srcImage, channels);
		channelR = channels[2].clone();

		cv::inRange(channelR, 100, 255, channels[2]);
		maskImage = channels[2].clone();
	}
	break;

	case PASSING_GRASSLAND_STAGE_2:
	{
		cv::split(srcImage, channels);
		channelB = channels[0].clone();
		channelR = channels[2].clone();

		cv::inRange(channelB, 0, 140, channels[0]);
		cv::inRange(channelR, 0, 131, channels[2]);

		if (MODEL == LEFT_MODEL)
			maskImage = channels[0].clone();
		else
			maskImage = channels[2].clone();
	}
	break;

	case CLIMBING_MOUNTAIN:
	{
		cv::split(srcImage, channels);
		channelB = channels[0].clone();

		inRange(channelB, 0, 110, channels[0]);
		cv::Mat beforeFill = channels[0].clone();
		maskImage = channels[0].clone();
		SetSeedPoint();
		floodFill(channels[0], seedPoint, 255);
		
		bitwise_xor(beforeFill, channels[0], maskImage);
		cout << "seedPoint: " << seedPoint.x << " " << seedPoint.y << endl;
		cv::imshow("floorFill", maskImage);
	}
		break;
	default:
		break;
	}
}

void ActD435::FindPillarCenter(void)
{
	cv::Mat dst = dstImage.clone();

	contours.clear();
	hierarchy.clear();

	if (status == PASSING_GRASSLAND_STAGE_1)
	{
		if (MODEL == LEFT_MODEL)
		{
			cv::Rect mask(400, 0, 239, 479);
			dst(mask).setTo(0);
		}
		else
		{
			cv::Rect mask(0, 0, 239, 479);
			dst(mask).setTo(0);
		}
		if (center2.y != 0)
		{
			cv::Rect mask(0, static_cast<int>(center2.y) + 80, 200, 400 - static_cast<int>(center2.y));
			dst(mask).setTo(0);
		}
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
			if(index == 0)
				center1 = cv::Point2f(x, y);
			else
				center1 = cv::Point2f((index + rect.x + rect.width) / 2, y);
		}
		cv::circle(dst, center1, 5, 255, -1);

		//cout << "x: " << center1.x << "y: " << center1.y << endl;
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

		cv::circle(dst, center2, 5, 255, -1);
		//cout << "x: " << center2.x << "y: " << center2.y << endl;
	}
	break;
	}
	imshow("dst2", dst);
	cv::waitKey(1);

}

void ActD435::FindFenseCorner(int fenseType, int mode)
{
	vector<cv::Point2f> line;
	int begin = maskImage.rows - 1;
	int end = 0;
	int cols = 0;
	int countFlag = 0;
	int blackPixNum = 0;
	uchar thresh = 10;

	//按列从最后一行逆序遍历
	if(mode == 0)
		cols = maskImage.cols - 1;
	//按列从最后一行顺序遍历
	else 
		cols = 0;

	do
	{
		blackPixNum = 0;
		int tempbegin = 0;

		if (mode == 0)
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

	if (line.size() > 30)
	{	
		if (fenseType == HORIZONAL_FENSE)
		{
			GetyawAngle(line[line.size() - 20], line[5], HORIZONAL_FENSE);
			if(robotYawAngle < 0)
				fenseCorner = line[line.size() - 5];
			else
				fenseCorner = line[line.size() - 5 - static_cast<int>(robotYawAngle * 180 / CV_PI)];
		}		
		else
		{
			GetyawAngle(line[5], line[line.size() - 10], VERTICAL_FENSE);
			fenseCorner = line[line.size() - 15];
		}	
	}

	cout << "robotYawAngle: " << robotYawAngle * 180 / CV_PI << endl;
	cv::circle(maskImage, fenseCorner, 10, 255, -1);

	line.clear();
	cv::imshow("fenseCorner", maskImage);
	cvWaitKey(1);
	cout << "x: " << fenseCorner.x << " y:" << fenseCorner.y << endl;
}
void ActD435::FindLineCross(void)
{
	vector<cv::Point2f> line;
	int begin = 0;
	int end = 0;
	int rows = maskImage.rows - 1;

	if (lineCross.y < 200)
		rows = maskImage.rows - 100;

	if (MODEL == LEFT_MODEL)
	{
		begin = maskImage.cols - 1;
		end = 0;
		int countFlag = 0;
		int blackPixNum = 0;
		int blackPixNumLast = 0;
		uchar thresh = 10;
		do
		{
			int tempbegin = 0;
			blackPixNumLast = blackPixNum;
			blackPixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols > end; --cols)
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
							continue;
						}

						if (blackPixNum > 60)
						{
							rows -= 20;
							blackPixNumLast = 0;
							break;
						}

						if (blackPixNumLast > 0 && (blackPixNum - blackPixNumLast) >= 10)
						{
							break;
						}
						else
						{
							if (blackPixNum > thresh)
							{

								if (tempbegin < 150 || end > tempbegin)
								{
									continue;
								}
								line.push_back(cv::Point(tempbegin, rows));
								begin = tempbegin + 10;
								end = cols - 10;
								break;
							}
						}
					}
				}
			}
			if (begin > 639)
				begin = 639;
			if (end < 0)
				end = 0;
		} while (blackPixNum > thresh && rows > 0 && (blackPixNumLast == 0 || (blackPixNum - blackPixNumLast) < 10 || blackPixNum > 60));
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

	if (line.size() > 30)
	{
		lineCross = line[line.size() - 2];
		GetyawAngle(line[5], line[line.size() - 20], VERTICAL_FENSE);
	}
	else
	{
		cout << "Do not find lineCross." << endl;
		cout << line[line.size() - 1].y << endl;
	}

	cout << "robotYawAngle: " << robotYawAngle * 180 / CV_PI << endl;

	cv::circle(maskImage, lineCross, 5, 255, -1);
	line.clear();

	cout << "x: " << lineCross.x << " y: " << lineCross.y << endl;

	imshow("lineCross", maskImage);

	cvWaitKey(1);
}
void ActD435::FindLineEnd(void)
{
	vector<cv::Point2f> line;
	int begin = 0;
	int end = 0;
	int rows = maskImage.rows - 1;
	uchar thresh = 0;
	if (MODEL == LEFT_MODEL)
	{
		begin = maskImage.cols - 1;
		rows = maskImage.rows - 1;
		end = 0;
		int countFlag = 0;
		int blackPixNum = 0;
		int blackPixNumLast = 0;

		thresh = 8;
		do
		{
			int tempbegin = 0;
			blackPixNumLast = blackPixNum;
			blackPixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols > end; --cols)
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
							if (blackPixNum > 100)
							{
								rows -= 20;
								break;
							}
							line.push_back(cv::Point(tempbegin, rows));
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
		GetyawAngle(line[5], line[line.size() - 10], VERTICAL_FENSE);
		cv::circle(maskImage, lineEnd, 5, 255, -1);
		cout << "x: " << lineEnd.x << " y:" << lineEnd.y << endl;		
	}
	else
	{
		cout << "line not found." << endl;
	}

	cout << "robotYawAngle: " << robotYawAngle * 180 / CV_PI << endl;
	
	imshow("lineEnd", maskImage);

	cvWaitKey(1);
}
cv::Point ActD435::GetCrossPoint(cv::Point pt1, cv::Point pt2, cv::Point pt3, cv::Point pt4)
{
	cv::Point crossPoint;
	if (pt1.x != pt2.x && pt3.x != pt4.x)
	{
		float k1 = static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x);
		float k2 = static_cast<float>(pt4.y - pt3.y) / (pt4.x - pt3.x);

		float b1 = pt1.y - k1 * pt1.x;
		float b2 = pt3.y - k2 * pt3.x;

		crossPoint.x = (b2 - b1) / (k1 - k2);
		crossPoint.y = k1 * crossPoint.x + b1;
	}
	else if (pt1.x == pt2.x)
	{
		float k2 = static_cast<float>(pt4.y - pt3.y) / (pt4.x - pt3.x);
		float b2 = pt3.y - k2 * pt3.x;

		crossPoint.x = pt1.x;
		crossPoint.y = k2 * crossPoint.x + b2;
	}
	else
	{
		float k1 = static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x);
		float b1 = pt3.y - k1 * pt3.x;

		crossPoint.x = pt3.x;
		crossPoint.y = k1 * crossPoint.x + b1;
	}

	return crossPoint;
}
void ActD435::ClimbingMountain(void)
{
	Canny(maskImage, maskImage, 80, 80 * 2);

	vector<cv::Vec4i> line;
	vector<cv::Vec4i> filterLine;

	cv::HoughLinesP(maskImage, line, 1, CV_PI / 180, 5, 0, 30);

	houghLine houghLine;

	for (size_t i = 0; i < line.size() - 1; i++)
	{
		float angle = 0;
		float distance = 0;
		cv::Point pt1, pt2;
		pt1 = cv::Point(line[i][0], line[i][1]);
		pt2 = cv::Point(line[i][2], line[i][3]);

		distance = sqrt((pt2.y - pt1.y) * (pt2.y - pt1.y) + (pt2.x - pt1.x) * (pt2.x - pt1.x));
		if (pt1.x != pt2.x)
		{
			angle = atan(fabs(static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x))) * 180 / CV_PI;
		}
		else
		{
			angle = 90;
		}

		if (i == 0)
		{
			houghLine.lineAngle[0] = angle;
			houghLine.distance[0] = distance;
			houghLine.index[0] = 0;
		}
		else
		{
			if (fabs(angle - houghLine.lineAngle[0]) > 5)
			{
				if (distance > houghLine.distance[1])
				{
					houghLine.lineAngle[1] = angle;
					houghLine.distance[1] = distance;
					houghLine.index[1] = i;
				}
			}
			else
			{
				if (distance > houghLine.distance[0])
				{
					houghLine.lineAngle[0] = angle;
					houghLine.distance[0] = distance;
					houghLine.index[0] = i;
				}
			}
		}
	}

	cv::Mat lines = cv::Mat::zeros(maskImage.size(), CV_8UC1);
	//find one line
	if (houghLine.index[1] == 0)
	{
		cv::Point pt1, pt2;

		pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
		pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);

		fenseCorner = pt1;
		GetyawAngle((cv::Point2f)pt1, (cv::Point2f)pt2, HORIZONAL_FENSE);
	}
	//find two lines 
	else
	{
		//horzisonal line's index is zero
		if (houghLine.lineAngle[0] <= houghLine.lineAngle[1])
		{
			filterLine.push_back(line[houghLine.index[0]]);
			filterLine.push_back(line[houghLine.index[1]]);
		}
		else
		{
			filterLine.push_back(line[houghLine.index[1]]);
			filterLine.push_back(line[houghLine.index[0]]);
		}
		//draw lines and find crossPoint
		cv::Point pt1, pt2, pt3, pt4;

		pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
		pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);
		pt3 = cv::Point(filterLine[1][0], filterLine[1][1]);
		pt4 = cv::Point(filterLine[1][2], filterLine[1][3]);

		fenseCorner = GetCrossPoint(pt1, pt2, pt3, pt4);
		GetyawAngle((cv::Point2f)pt1, (cv::Point2f)pt2, HORIZONAL_FENSE);
	}
	imshow("lines", lines);
	cvWaitKey(1);
	cv::circle(maskImage, fenseCorner, 8, 255);
	cv::imshow("maskImage", maskImage);
	cvWaitKey(1);
}

cv::Point ActD435::SetSeedPoint(void)
{
	climbingMountainStage = ClimbingMountainStageJudge();
		
	if (MODEL == LEFT_MODEL)
	{
		switch (climbingMountainStage)
		{
		case 1:
		{
			seedPoint = cv::Point(maskImage.cols - 2, maskImage.rows - 1);
			break;
		}	
		
		case 2:
		{
			seedPoint = cv::Point(maskImage.cols / 2, maskImage.rows - 1);
			break;
		}
			
		case 3:
		{
			seedPoint = cv::Point(2, maskImage.rows - 1);
			break;
		}
			
		}
	}
	else
	{
		switch (climbingMountainStage)
		{
		case 1:
			seedPoint = cv::Point(2, maskImage.rows - 1);
			break;
		case 2:
			seedPoint = cv::Point(maskImage.cols / 2, maskImage.rows - 1);
			break;
		case 3:
			seedPoint = cv::Point(maskImage.cols - 2, maskImage.rows - 1);
		}
	}
	return seedPoint;
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
	cv::Mat C = -(rotationMatrix) * (translationMatrix);

	cv::Mat K = rotationMatrix * intrinsicMatrixLeft.inv() * (RGBpixed);

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
	float depth = 0;
	float angle = 0;

	pt1 = GetIrCorrdinate(pt);
	depth = pt1.z * cos(angleAlpha) - pt1.y * sin(angleAlpha);
	angle = atan(pt1.x / depth);
	realDistance = (sqrt(depth * depth + pt1.x * pt1.x)) * cos(-cameraYawAngle + robotYawAngle + angle);
	nowXpos = (sqrt(depth * depth + pt1.x * pt1.x)) * sin(-cameraYawAngle + robotYawAngle + angle);

	cout << "depth: " << depth << endl;
	cout << "angle: " << angle * 180 / CV_PI << endl;
	return realDistance;
}

float ActD435::GetyawAngle(cv::Point2f& pt1, cv::Point2f& pt2, int fenseType)
{
	cv::Point3f point1In3D;
	cv::Point3f point2In3D;
	point1In3D = GetIrCorrdinate(pt1);
	point2In3D = GetIrCorrdinate(pt2);

	double rotatedAngle = 0;
	float x = point2In3D.x - point1In3D.x;
	float y = point2In3D.y - point1In3D.y;
	float z = point2In3D.z - point1In3D.z;
	
	if ((fenseType == HORIZONAL_FENSE && x < 0) || (fenseType == VERTICAL_FENSE && z < 0))
	{
		x = -x;
		y = -y;
		z = -z;
	}

	Eigen::Vector3f Vec(x,y,z);
	if(fenseType == HORIZONAL_FENSE)
		rotatedAngle = acos((x * sin(cameraYawAngle) - y * sin(angleAlpha) * cos(cameraYawAngle) + z * cos(angleAlpha) * cos(cameraYawAngle)) / Vec.norm());
	else
		rotatedAngle = acos(-(x * cos(cameraYawAngle) + y * sin(angleAlpha) * sin(cameraYawAngle) - z * cos(angleAlpha) * sin(cameraYawAngle)) / Vec.norm());
	cout << "rotatedAngle: " << rotatedAngle * 180 / CV_PI << endl;
	
	robotYawAngle = -rotatedAngle + CV_PI / 2;

	return robotYawAngle;
}

int ActD435::ClimbingMountainStageJudge(void)
{
	int firstStage = 1;
	int secondStage = 2;
	int thirdStage = 3;

	int begin = 0;
	int end = maskImage.cols - 1;

	uchar* data = maskImage.ptr<uchar>(maskImage.rows - 1);

	int cols = 1;
			
	if (data[cols] == 0
		|| data[cols + 1] == 0
		|| data[cols - 1] == 0
		|| maskImage.at<uchar>(maskImage.rows - 2, cols) == 0
		|| maskImage.at<uchar>(maskImage.rows - 2, cols - 1) == 0
		|| maskImage.at<uchar>(maskImage.rows - 2, cols + 1) == 0
		)
		begin = 1;
	else
		begin = 2;
	
	cols = maskImage.cols - 2;
	if (data[cols] == 255
		|| data[cols + 1] == 0
		|| data[cols - 1] == 0
		|| maskImage.at<uchar>(maskImage.rows - 2, cols) == 0
		|| maskImage.at<uchar>(maskImage.rows - 2, cols - 1) == 0
		|| maskImage.at<uchar>(maskImage.rows - 2, cols + 1) == 0
		)
		end = maskImage.cols - 2;
	else
		end = maskImage.cols - 3;
		
	if (MODEL == LEFT_MODEL)
	{
		if (begin > 1)
		{
			return firstStage;
		}
		else
		{
			if (end == maskImage.cols - 2)
			{
				return secondStage;
			}
			else
			{
				return thirdStage;
			}
		}
	}
	else
	{
		if (end < maskImage.cols - 2)
		{
			return firstStage;
		}
		else
		{
			if (begin > 1)
			{
				return thirdStage;
			}
			else
			{
				return secondStage;
			}
		}
	}
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