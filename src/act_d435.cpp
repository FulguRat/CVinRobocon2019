#include "act_d435.h"

roiFlag colorFrameRoi = midRoi;
int mode = LEFT_MODE;
//int mode = RIGHT_MODE;
float frontDist;
float lateralDist;
int dbStatus = 0;
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
	//-- Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

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

	cout << "step1 ok" << endl;
	//-- Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cout << "step2 ok" << endl;
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

mb_cuda::thrustCloudT ActD435::update(void)
{
	static int time = 0;

	//chrono::steady_clock::time_point start = chrono::steady_clock::now();

	//-- Wait for the next set of frames from the camera
	frameSet = pipe.wait_for_frames();

	//-- Get processed aligned frame
	//alignedFrameSet = align.process(frameSet);

	//-- Get both color and aligned depth frames
	//rs2::video_frame colorFrame = alignedFrameSet.first(RS2_STREAM_COLOR);
	//rs2::depth_frame alignedDepthFrame = alignedFrameSet.get_depth_frame();

	//-- For not align
	rs2::video_frame colorFrame = frameSet.get_color_frame();
	rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();
	
	srcImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

	//cv::Mat temp = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

	/*switch (colorFrameRoi)
	{
		case midRoi:
		{
			temp(cv::Rect(104, 0, 640, 480)).copyTo(srcImage);
		}
		break;
		case leftRoi:
		{
			temp(cv::Rect(0, 0, 640, 480)).copyTo(srcImage);
			intrinsicMatrixLeft = (cv::Mat_<float>(3, 3) << argsLeft.fx, argsLeft.skew, argsLeft.cx + 104, \
				0.0f, argsLeft.fy, argsLeft.cy, \
				0.0f, 0.0f, 1.0f);
			intrinsicMatrixRight = (cv::Mat_<float>(3, 3) << argsRight.fx, argsRight.skew, argsRight.cx + 104, \
				0.0f, argsRight.fy, argsRight.cy, \
				0.0f, 0.0f, 1.0f);
		}
		break;
		case rightRoi:
		{
			temp(cv::Rect(208, 0, 640, 480)).copyTo(srcImage);
			intrinsicMatrixLeft = (cv::Mat_<float>(3, 3) << argsLeft.fx, argsLeft.skew, argsLeft.cx - 104, \
				0.0f, argsLeft.fy, argsLeft.cy, \
				0.0f, 0.0f, 1.0f);
			intrinsicMatrixRight = (cv::Mat_<float>(3, 3) << argsRight.fx, argsRight.skew, argsRight.cx - 104, \
				0.0f, argsRight.fy, argsRight.cy, \
				0.0f, 0.0f, 1.0f);
		}
		break;
		default:
			break;
	}*/
#ifdef DEBUG
	imshow("src", srcImage);
	cvWaitKey(1);
#endif // DEBUG

	//-- Map Color texture to each point
	//rs2Cloud.map_to(colorFrame);

	//-- Generate the pointcloud and texture mappings
	rs2Points = rs2Cloud.calculate(alignedDepthFrame);
	thrustcloud = pointsToPointCloud(rs2Points);

	// cout << double(totalTime.count()) / 1000.0f << endl;

	return thrustcloud;
}
void ActD435::imgProcess()
{
	switch (status)
	{
		case BEFORE_GRASSLAND_STAGE_1:
		{
			cv::split(srcImage, channels);
			channelR = channels[2].clone();

			cv::inRange(channelR, 0, 100, channelR);
			FillHoles(channelR);
			maskImage = channelR.clone();		
		}
		break;

		case BEFORE_GRASSLAND_STAGE_2:
		{
			cv::cvtColor(srcImage, grayImage, CV_BGR2GRAY);
			cv::cvtColor(srcImage, srcImage, CV_BGR2Lab);
			cv::split(srcImage, channels);

			inRange(channels[1], 0, 149, channelA);
			inRange(channels[2], 157, 255, channelB);

			cv::bitwise_and(channelA, channelB, maskImage);
#ifdef DEBUG
			cv::imshow("pillar", maskImage);
#endif // DEBUG
			//GetyawAngle
			pillarStatus = FindPillarCenter();
			if(pillarStatus)
			{
				cv::threshold(grayImage, grayImage, 100, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
				FillHoles(grayImage);
				cv::Mat beforeFill = grayImage.clone();
				SetSeedPoint();
				floodFill(grayImage, seedPoint, 255);
				bitwise_xor(beforeFill, grayImage, maskImage);
				FindHorizonalHoughLine(maskImage);
			}
		}
		break;

		case UNDER_MOUNTAIN:
		{
			cv::cvtColor(srcImage, grayImage, CV_BGR2GRAY);
			cv::threshold(grayImage, grayImage, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
			FillHoles(grayImage);
			maskImage = grayImage.clone();
		}
		break;
		case CLIMBING_MOUNTAIN:
		{
			cv::split(srcImage, channels);
			inRange(channels[1], 105, 255, channelG);
			FillHoles(channelG);
			cv::Mat beforeFill = channelG.clone();
			cv::imshow("before fill", beforeFill);
			maskImage = channelG.clone();
			SetSeedPoint();
			floodFill(channelG, seedPoint, 0);

			bitwise_xor(beforeFill, channelG, maskImage);
		}
		break;
		case REACH_MOUNTAIN:
		{
			cv::cvtColor(srcImage, srcImage, CV_BGR2Lab);
			cv::split(srcImage, channels);
			if (status == LEFT_MODE)
			{
				channelA = channels[1].clone();
				cv::threshold(channelA, channelA, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
				FillHoles(channelA);
				maskImage = channelA.clone();
			}
			else
			{
				channelB = channels[2].clone();
				cv::threshold(channelB, channelB, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
				FillHoles(channelB);
				maskImage = channelB.clone();
			}			
		}
		break;
		default:
			break;
	}
#ifdef DEBUG
	cv::imshow("binary", maskImage);
#endif // DEBUG
}
void ActD435::FindBinaryThresh(void)
{

}

void ActD435::FillHoles(cv::Mat& src)
{
	cv::Size m_Size = src.size();
	cv::Mat temImage = cv::Mat::zeros(m_Size.height + 2, m_Size.width + 2, src.type());//��չͼ��  

	src.copyTo(temImage(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)));
	floodFill(temImage, cv::Point(0, 0), cv::Scalar(255));
	cv::Mat cutImg;//�ü���չ��ͼ��  
	temImage(cv::Range(1, m_Size.height + 1), cv::Range(1, m_Size.width + 1)).copyTo(cutImg);

	bitwise_not(cutImg, cutImg);
	bitwise_or(src, cutImg, src);
}
int ActD435::FindPillarCenter(void)
{
	vector<vector<cv::Point>>contours;
	vector<vector<cv::Point>>filterContours;
	vector<cv::Vec4i>hierarchy;

	cv::findContours(maskImage,contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	for (size_t i = 0; i < contours.size(); ++i)
	{
		if (contourArea(contours[i]) > 120)
		{
			cv::Rect rect = cv::boundingRect(contours[i]);
			if(rect.height > 30 && rect.y < maskImage.rows / 2)
				filterContours.push_back(contours[i]);
		}
	}

	if(filterContours.size() >= 2)
	{
		sort(filterContours.begin(), filterContours.end(), [](const std::vector<cv::Point> &s1,
			const std::vector<cv::Point> &s2) {
			double a1 = cv::contourArea(s1);
			double a2 = cv::contourArea(s2);
			return a1 > a2;
		});

		cv::Rect rect1, rect2;
		rect1 = cv::boundingRect(filterContours[0]);
		rect2 = cv::boundingRect(filterContours[1]);

		//cout << rect1.x << " " << rect1.y; 
		//cout << rect2.x << " " << rect2.y;
		if (rect1.y < rect2.y)
		{
			pillarLeftUpPt = cv::Point2f(rect1.x, rect1.y);
			pillarHeight = rect1.height;
			center2.x = rect1.x + rect1.width / 2;
			center2.y = rect1.y + rect1.height;
			if (rect1.x <= 10)
				return 2;
		}
		else
		{
			pillarLeftUpPt = cv::Point2f(rect2.x, rect2.y);
			pillarHeight = rect2.height;
			center2.x = rect2.x + rect2.width / 2;
			center2.y = rect2.y + rect2.height;
			if (rect2.x <= 10)
				return 2;
		}
	}
	else if(filterContours.size() == 1)
	{
		cv::Rect rect;
		rect = cv::boundingRect(filterContours[0]);
		pillarLeftUpPt = cv::Point2f(rect.x, rect.y);
		pillarHeight = rect.height;
		center2.x = rect.x + rect.width / 2;
		center2.y = rect.y + rect.height;
		if (rect.x <= 10)
			return 2;
	}
	else
	{
#ifdef DEBUG
		cout << "No Pillar Found." << endl;
#endif // DEBUG
		return 0;
	}
#ifdef DEBUG
	cv::circle(maskImage, center2, 5, 255, -1);
	cout << "contoursize: " << filterContours.size() << endl;
	cout << "center2: " << center2.x << " " << center2.y << endl;
	cv::imshow("maskImage", maskImage);
	cvWaitKey(1);
#endif
	return 1;
}

void ActD435::FindFenseCorner(int fenseType, int mode)
{
	vector<cv::Point2f> line;
	int begin = maskImage.rows - 2;
	int end = 0;
	int cols = 0;
	int countFlag = 0;
	int whitePixNum = 0;
	uchar thresh = 10;

	//���д����һ���������
	if(mode == 0)
		cols = maskImage.cols - 2;
	//���д����һ��˳�����
	else 
		cols = 0;

	do
	{
		whitePixNum = 0;
		int tempbegin = 0;

		if (mode == 0)
			--cols;
		else
			++cols;
		for (int rows = begin; rows > end; --rows)
		{
			if (maskImage.at<uchar>(rows, cols) == 255 && countFlag == 0)
			{
				tempbegin = rows;
				countFlag = 1;
			}
			if (countFlag)
			{
				//begin count
				if (maskImage.at<uchar>(rows, cols) == 255)
					whitePixNum++;
				else
				{
					//remove black noise in the fense
					if (   maskImage.at<uchar>(rows - 1, cols) == 0
						&& maskImage.at<uchar>(rows, cols - 1) == 0
						&& maskImage.at<uchar>(rows + 1, cols) == 0
						&& maskImage.at<uchar>(rows, cols + 1) == 0
						)
						whitePixNum = 0;
					else
						whitePixNum++;
				}
					
				if (whitePixNum > thresh)
				{
					if(tempbegin < 478)
						line.push_back(cv::Point(cols, tempbegin));
					begin = tempbegin + 15;
					end = rows - 15;
					countFlag = 0;
					break;
				}
			}
		}
		if (begin > 478)
			begin = 478;
		if (end < 0)
			end = 0;
	} while (whitePixNum > thresh/* && cols > 1*/);

	if (line.size() > 30)
	{	
		if (fenseType == HORIZONAL_FENSE)
		{
			GetyawAngle(line[line.size() - 20], line[5], HORIZONAL_FENSE);
			if(cameraYawAngle < -25.0f * CV_PI / 180.0f )
				fenseCorner = line[line.size() - 5];
			else
				fenseCorner = line[line.size() - 30 + 50 * mode - static_cast<int>(cameraYawAngle * 180 / CV_PI)];
		}		
		else
		{
			GetyawAngle(line[5], line[line.size() - 10], VERTICAL_FENSE);
			fenseCorner = line[line.size() - 15];
		}	
	}
	else
	{
		cout << "fenseCorner not found" << endl;
	}
	line.clear();
	cv::circle(maskImage, fenseCorner, 10, 255, -1);
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

	if (mode == LEFT_MODE)
	{
		begin = maskImage.cols - 1;
		end = 0;
		int countFlag = 0;
		int whitePixNum = 0;
		int whitePixNumLast = 0;
		uchar thresh = 10;
		do
		{
			int tempbegin = 0;
			whitePixNumLast = whitePixNum;
			whitePixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols > end; --cols)
			{
				if (data[cols] == 255 && countFlag == 0)
				{
					tempbegin = cols;
					countFlag = 1;
				}
				if (countFlag)
				{
					//begin count
					if (data[cols] == 255)
						whitePixNum++;
					//finish count
					else
					{
						countFlag = 0;
						//update
						if (whitePixNum < thresh)
						{
							continue;
						}

						if (whitePixNum > 60)
						{
							rows -= 20;
							whitePixNumLast = 0;
							break;
						}

						if (whitePixNumLast > 0 && (whitePixNum - whitePixNumLast) >= 10)
						{
							break;
						}
						else
						{
							if (whitePixNum > thresh)
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
		} while (whitePixNum > thresh && rows > 0 && (whitePixNumLast == 0 || (whitePixNum - whitePixNumLast) < 10 || whitePixNum > 60));
	}
	else
	{
		begin = 1;
		end = maskImage.cols / 2;

		int countFlag = 0;
		int whitePixNum = 0;
		int whitePixNumLast = 0;
		uchar thresh = 5;

		do
		{
			int tempbegin = 0;
			whitePixNumLast = whitePixNum;
			whitePixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols < end + 30; ++cols)
			{
				if (data[cols] == 255 && countFlag == 0)
				{
					tempbegin = cols;
					countFlag = 1;
				}
				if (countFlag)
				{
					//begin count
					if (data[cols] == 255)
						whitePixNum++;
					//finish count
					else
					{
						countFlag = 0;
						//update
						if (whitePixNum < thresh)
						{
							whitePixNum = 0;
							continue;
						}

						if (whitePixNum > thresh)
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
		} while (whitePixNum > thresh && rows > 0);

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

	cv::circle(maskImage, lineCross, 5, 255, -1);
	line.clear();

	cout << "x: " << lineCross.x << " y: " << lineCross.y << endl;

	imshow("lineCross", maskImage);

	cvWaitKey(1);
}
void ActD435::FindLineEnd(void)
{
	vector<cv::Point2f> line;
	int notFoundCnt = 0;
	int begin = 0;
	int end = 0;
	int rows = maskImage.rows - 1;
	uchar thresh = 0;
	if (mode == LEFT_MODE)
	{
		begin = maskImage.cols - 1;
		end = 0;
		int countFlag = 0;
		int whitePixNum = 0;
		int whitePixNumLast = 0;

		thresh = 6;
		do
		{
			int tempbegin = 0;
			whitePixNumLast = whitePixNum;
			whitePixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols > end; --cols)
			{
				if (data[cols] == 255 && countFlag == 0)
				{
					tempbegin = cols;	
					countFlag = 1;
				}
				if (countFlag)
				{
					//begin count
					if (data[cols] == 255)
						whitePixNum++;
					//finish count
					else
					{
						countFlag = 0;
						//update
						if (whitePixNum < thresh)
						{
							whitePixNum = 0;
							continue;
						}

						if (whitePixNum > thresh)
						{
							if (whitePixNum > 50 || (whitePixNum - whitePixNumLast >= 10 && whitePixNumLast != 0))
							{
								rows -= 20;
								whitePixNum = whitePixNumLast;
								break;
							}
							notFoundCnt = 0;				
							line.push_back(cv::Point(tempbegin, rows));
							begin = tempbegin + 20;
							end = cols - 20;
							break;
						}
					}
				}
				if (cols <= end + 1 && whitePixNum == 0)
				{
					notFoundCnt++;
				}
			}
			if (begin > 639)
				begin = 639;
			if (end < 0)
				end = 0;
		} while (notFoundCnt < 45 && rows > 1);
	}
	else
	{
		begin = 0;
		end = maskImage.cols - 1;;
		int countFlag = 0;
		int whitePixNum = 0;
		int whitePixNumLast = 0;

		thresh = 6;
		do
		{
			int tempbegin = 0;
			whitePixNumLast = whitePixNum;
			whitePixNum = 0;
			uchar* data = maskImage.ptr<uchar>(rows--);
			for (int cols = begin; cols < end; ++cols)
			{
				if (data[cols] == 255 && countFlag == 0)
				{
					tempbegin = cols;
					countFlag = 1;
				}
				if (countFlag)
				{
					//begin count
					if (data[cols] == 255)
						whitePixNum++;
					//finish count
					else
					{
						countFlag = 0;
						//update
						if (whitePixNum < thresh)
						{
							whitePixNum = 0;
							continue;
						}

						if (whitePixNum > thresh)
						{
							notFoundCnt = 0;
							if (whitePixNum > 50 || (whitePixNum - whitePixNumLast >= 10 && whitePixNumLast != 0))
							{
								rows -= 20;
								whitePixNum = whitePixNumLast;
								break;
							}								
							line.push_back(cv::Point(tempbegin, rows));
							begin = tempbegin - 20;
							end = cols + 20;
							break;
						}
					}
				}
				if (cols >= end - 1 && whitePixNum <= 1)
				{
					notFoundCnt++;
				}
			}
			if (end > 639)
				end = 639;
			if (begin < 0)
				begin = 0;
		} while (notFoundCnt < 45 && rows > 1);
	}

	
	if (line.size() > 30)
	{
		lineEnd = line[line.size() - 3];
		GetyawAngle(line[10], line[line.size() - 10], VERTICAL_FENSE);
#ifdef DEBUG
		cv::circle(maskImage, lineEnd, 5, 255, -1);
		cv::line(maskImage, line[10], line[line.size() - 10], 255, 5);
		cout << "x: " << lineEnd.x << " y:" << lineEnd.y << endl;
#endif // DEBUG
	}
#ifdef DEBUG
	else
	{
		cout << "line not found." << endl;
	}
#endif // DEBUG
#ifdef DEBUG
	cout << "break condition: " << notFoundCnt << endl;
	cout << "cameraYawAngle: " << cameraYawAngle * 180 / CV_PI << endl;
	imshow("lineEnd", maskImage);
	cvWaitKey(1);
#endif // DEBUG
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
		cout << pt1.x << " " << pt1.y << " " << pt2.x << " " << pt2.y << pt3.x << " " << pt3.y << " " << pt4.x << " " << pt4.y << endl;
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

cv::Point ActD435::SetSeedPoint(void)
{
	int rows = 100;
	switch (status)
	{
		case BEFORE_GRASSLAND_STAGE_2:
		{
			if(center2.x < 320)
				rows = center2.y - 2 * pillarHeight / 3;
		}
		break;
		case CLIMBING_MOUNTAIN:
		{
			rows = maskImage.rows - 50;
			break;
		}
		default:
			break;
	}
	if (status == BEFORE_GRASSLAND_STAGE_2)
	{
		if (center2.x > 320)
		{
			uchar* data = maskImage.ptr<uchar>(pillarLeftUpPt.y);
			for (int i = pillarLeftUpPt.x - 2; i > 2; --i)
			{
				if (data[i] == 0 && data[i - 1] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i - 2] == 0)
				{
					seedPoint = cv::Point(i, pillarLeftUpPt.y);
					break;
				}
			}
		}
		else
		{
			uchar* data = maskImage.ptr<uchar>(rows);
			for (int i = center2.x + 4; i < maskImage.cols; i++)
			{
				if (data[i] == 0 && data[i - 1] == 0 && data[i + 1] == 0 && data[i + 2] == 0 && data[i - 2] == 0 && data[i + 3] == 0 && data[i - 3] == 0)
				{
					seedPoint = cv::Point(i, rows);
					break;
				}
			}
		}
	}
	else
	{
		if (mode == LEFT_MODE)
		{
			uchar* data = maskImage.ptr<uchar>(rows);
			for (int i = maskImage.cols - 4; i > 3; i--)
			{
				if (data[i] == 255 && data[i - 1] == 255 && data[i + 1] == 255 && data[i + 2] == 255 && data[i - 2] == 255 && data[i + 3] == 255 && data[i - 3] == 255)
				{
					seedPoint = cv::Point(i, rows);
					return seedPoint;
				}
			}
		}
		else
		{
			uchar* data = maskImage.ptr<uchar>(rows);
			for (int i = 3; i < maskImage.cols - 4; i++)
			{
				if (data[i] == 255 && data[i - 1] == 255 && data[i + 1] == 255 && data[i + 2] == 255 && data[i - 2] == 255 && data[i + 3] == 255 && data[i - 3] == 255)
				{
					seedPoint = cv::Point(i, rows);
					return seedPoint;
				}
			}
		}
		cout << "fail to find seedPoint ." << endl;
	}

	return seedPoint;
}
void ActD435::FindHoughLineCross(void)
{
	Canny(maskImage, maskImage, 80, 80 * 2);
#ifdef DEBUG
	cv::imshow("Canny", maskImage);
#endif // DEBUG
	vector<cv::Vec4i> line;
	filterLine.resize(0);
	cv::HoughLinesP(maskImage, line, 1, CV_PI / 180, houghlineThresh, 10, 30);

	HoughLine firstHoughLine;
	HoughLine secondHoughLine;
	HoughLine comparedHoughLine;
	for (size_t i = 0; i < line.size(); i++)
	{
		cv::Point2f pt1, pt2;
		pt1 = cv::Point2f(line[i][0], line[i][1]);
		pt2 = cv::Point2f(line[i][2], line[i][3]);

		comparedHoughLine.distance = sqrt((pt2.y - pt1.y) * (pt2.y - pt1.y) + (pt2.x - pt1.x) * (pt2.x - pt1.x));
		comparedHoughLine.index = i;
		if (pt1.x != pt2.x)
		{
			comparedHoughLine.lineSlop = static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x);
			comparedHoughLine.lineAngle = atan(fabs(comparedHoughLine.lineSlop)) * 180 / CV_PI;
		}
		else
		{
			comparedHoughLine.lineSlop = 0;
			comparedHoughLine.lineAngle = 90;
		}
		if (firstHoughLine.distance == 0)
		{
			firstHoughLine = comparedHoughLine;
		}
		else
		{
			//find the longest horizonal line and vertical line
			if (fabs(comparedHoughLine.lineAngle - firstHoughLine.lineAngle) > 10)
			{
				if (comparedHoughLine.distance > secondHoughLine.distance)
				{
					secondHoughLine = comparedHoughLine;
				}
			}
			else
			{
				if (comparedHoughLine.distance > firstHoughLine.distance)
				{
					firstHoughLine = comparedHoughLine;
				}
			}
		}
	}

	cv::Mat lines = cv::Mat::zeros(maskImage.size(), CV_8UC1);
	if (line.size() > 0)
	{
		//find one line
		if (firstHoughLine.distance != 0 && secondHoughLine.distance == 0)
		{
			filterLine.push_back(line[firstHoughLine.index]);
			cv::Point pt1, pt2;

			pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
			pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);

			lineCross = pt1;
			GetyawAngle((cv::Point2f)pt1, (cv::Point2f)pt2, HORIZONAL_FENSE);
#ifdef DEBUG
			cv::line(lines, pt1, pt2, 255, 5);
#endif // DEBUG		
		}
		//find two lines 
		else
		{
			filterLine.push_back(line[firstHoughLine.index]);
			filterLine.push_back(line[secondHoughLine.index]);
			//draw lines and find crossPoint
			cv::Point pt1, pt2, pt3, pt4;

			pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
			pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);
			pt3 = cv::Point(filterLine[1][0], filterLine[1][1]);
			pt4 = cv::Point(filterLine[1][2], filterLine[1][3]);

			lineCross = GetCrossPoint(pt1, pt2, pt3, pt4);

			if (firstHoughLine.lineAngle != 0 && secondHoughLine.lineAngle != 0)
			{
				if ((firstHoughLine.lineSlop < 0 && mode == LEFT_MODE)
					|| (firstHoughLine.lineSlop > 0 && mode == RIGHT_MODE))
				{
					GetyawAngle((cv::Point2f)pt1, (cv::Point2f)pt2, HORIZONAL_FENSE);
#ifdef DEBUG
					cv::line(lines, pt1, pt2, 255, 5);
					cv::line(lines, pt3, pt4, 255, 1);
#endif // DEBUG
				}				
				else
				{
					GetyawAngle((cv::Point2f)pt3, (cv::Point2f)pt4, HORIZONAL_FENSE);
#ifdef DEBUG
					cv::line(lines, pt1, pt2, 255, 1);
					cv::line(lines, pt3, pt4, 255, 5);
#endif // DEBUG
				}			
			}
			else if (firstHoughLine.lineAngle == 90)
			{
				GetyawAngle((cv::Point2f)pt3, (cv::Point2f)pt4, HORIZONAL_FENSE);
#ifdef DEBUG
				cv::line(lines, pt1, pt2, 255, 1);
				cv::line(lines, pt3, pt4, 255, 5);
#endif // DEBUG
			}
			else
			{
				GetyawAngle((cv::Point2f)pt1, (cv::Point2f)pt2, HORIZONAL_FENSE);
#ifdef DEBUG
				cv::line(lines, pt1, pt2, 255, 5);
				cv::line(lines, pt3, pt4, 255, 1);
#endif // DEBUG
			}
		}
#ifdef DEBUG
		cv::circle(maskImage, lineCross, 8, 255);
		cv::imshow("lines", lines);
		cv::imshow("maskImage", maskImage);
		cvWaitKey(1);
#endif // DEBUG
	}
	else
	{
#ifdef DEBUG
		cout << "cannot find line." << endl;
#endif // DEBUG
	}
	
}
void ActD435::FindHorizonalHoughLine(cv::Mat& src)
{
	Canny(src, src, 80, 80 * 2);

	vector<cv::Vec4i> line;
	vector<cv::Vec4i> filterLine;

	cv::HoughLinesP(src, line, 1, CV_PI / 180, 60, 10, 30);

	HoughLine horizonalHoughLine;;
	HoughLine comparedHoughLine;

	for (size_t i = 0; i < line.size(); i++)
	{
		cv::Point2f pt1, pt2;
		pt1 = cv::Point2f(line[i][0], line[i][1]);
		pt2 = cv::Point2f(line[i][2], line[i][3]);

		comparedHoughLine.distance = sqrt((pt2.y - pt1.y) * (pt2.y - pt1.y) + (pt2.x - pt1.x) * (pt2.x - pt1.x));
		comparedHoughLine.index = i;
		if (pt1.x != pt2.x)
		{
			comparedHoughLine.lineSlop = static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x);
			comparedHoughLine.lineAngle = atan(fabs(comparedHoughLine.lineSlop)) * 180 / CV_PI;
		}
		else
		{
			comparedHoughLine.lineSlop = 0;
			comparedHoughLine.lineAngle = 90;
		}
		if (comparedHoughLine.lineAngle > 35)
		{
			continue;
		}
		if (comparedHoughLine.distance > horizonalHoughLine.distance)
		{
			horizonalHoughLine = comparedHoughLine;
		}

	}
	if(horizonalHoughLine.distance != 0)
		filterLine.push_back(line[horizonalHoughLine.index]);

	cv::Mat lines = cv::Mat::zeros(src.size(), CV_8UC1);
	if (filterLine.size() > 0)
	{
		cv::Point pt1, pt2;

		pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
		pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);

		GetyawAngle((cv::Point2f)pt1, (cv::Point2f)pt2, HORIZONAL_FENSE);
#ifdef DEBUG
		cv::line(lines, pt1, pt2, 255, 5);
		cv::imshow("lines", lines);
		cvWaitKey(1);
#endif // DEBUG
	}
	else
	{
#ifdef DEBUG
		cout << "cannot find line." << endl;
#endif // DEBUG
	}
}

bool ActD435::MatchLine(vector<cv::Vec4i>& src, vector<cv::Vec4i>& dst, float angleThresh, float minDistThresh, float maxDistThresh)
{
	int matchLineCnt = 0;
	float matchLineLength[100] = { 0 };
	vector<cv::Vec4i> filterLine;
	vector<cv::Vec4i> matchLine;
	for (size_t i = 0; i < src.size(); ++i)
	{
		float angle = 0;
		float lineSlop = 0;
		float distance = 0;
		float interception = 0;
		cv::Point pt1, pt2;
		pt1 = cv::Point(src[i][0], src[i][1]);
		pt2 = cv::Point(src[i][2], src[i][3]);
		if (dst.size() != 0)
		{
			cv::Point pt3, pt4, pt5, pt6;
			pt3 = cv::Point(dst[0][0], dst[0][1]);
			pt4 = cv::Point(dst[0][2], dst[0][3]);
			pt5 = cv::Point(dst[1][0], dst[1][1]);
			pt6 = cv::Point(dst[1][2], dst[1][3]);
			if ((pt1.y > pt2.y ? pt1.y : pt2.y) - max(max(pt3.y, pt4.y), max(pt5.y, pt6.y)) > -100
				|| (pt1.x > pt2.x ? pt1.x : pt2.x) - min(min(pt3.x, pt4.x), min(pt5.x, pt6.x)) < -150)
			{
				src.erase(src.begin() + i);
				i--;
				continue;
			}
		}
		if (pt1.x != pt2.x)
		{
			lineSlop = static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x);
			angle = atan(fabs(static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x))) * 180 / CV_PI;
			interception = pt1.y - lineSlop * pt1.x;
		}
		else
		{
			angle = 90;
			interception = 0;
		}
		distance = sqrt((pt2.y - pt1.y) * (pt2.y - pt1.y) + (pt2.x - pt1.x) * (pt2.x - pt1.x));


		for (size_t j = i + 1; j < src.size(); ++j)
		{
			cv::Point pt3, pt4;
			pt3 = cv::Point(src[j][0], src[j][1]);
			pt4 = cv::Point(src[j][2], src[j][3]);
			float lineSlopComp = 0;
			float angleComp = 0;
			float interceptionComp = 0;
			float distanceComp = 0;
			if (pt3.x != pt4.x)
			{
				lineSlopComp = static_cast<float>(pt4.y - pt3.y) / (pt4.x - pt3.x);
				angleComp = atan(fabs(lineSlopComp)) * 180 / CV_PI;
				interceptionComp = pt3.y - lineSlop * pt3.x;
			}
			else
			{
				angleComp = 90;
				interceptionComp = 0;
			}
			distanceComp = sqrt((pt2.y - pt1.y) * (pt2.y - pt1.y) + (pt2.x - pt1.x) * (pt2.x - pt1.x));
			if (fabs(angleComp - angle) < angleThresh)
			{
				if (interceptionComp != 0 && interception != 0)
				{
					if (fabs(interceptionComp - interception) / sqrt(1 + lineSlopComp * lineSlopComp) < minDistThresh)
					{
						if (distanceComp > distance)
						{
							src.erase(src.begin() + i);
							i--;
							break;
						}
						else
						{
							src.erase(src.begin() + j);
							j--;
							continue;
						}
					}
					else if (fabs(interceptionComp - interception) / sqrt(1 + lineSlopComp * lineSlopComp) > maxDistThresh)
					{
						continue;
					}
					else
					{
						matchLine.push_back(src[i]);
						matchLine.push_back(src[j]);
						matchLineLength[matchLineCnt++] = distance / 2 + distanceComp / 2;
					}
				}
				else
				{
					if (fabs(pt1.x + pt2.x - pt3.x - pt4.x) / 2 < minDistThresh)
					{
						if (distanceComp > distance)
						{
							src.erase(src.begin() + i);
							i--;
							break;
						}
						else
						{
							src.erase(src.begin() + j);
							j--;
							continue;
						}
					}
					else if (fabs(pt1.x + pt2.x - pt3.x - pt4.x) / 2 > maxDistThresh)
					{
						continue;
					}
					else
					{
						matchLine.push_back(src[i]);
						matchLine.push_back(src[j]);
						matchLineLength[matchLineCnt++] = distance / 2 + distanceComp / 2;
					}
				}
			}
			else
			{
				continue;
			}
		}
	}
	if (matchLineCnt > 0)
	{
		int index = 0;
		float maxLength = 0;
		for (int i = 0; i < matchLineCnt; ++i)
		{
			if (matchLineLength[i] > maxLength)
			{
				index = i;
				maxLength = matchLineLength[i];
			}
			if (i == 29)
				break;
		}
		dst.push_back(matchLine[2 * index]);
		dst.push_back(matchLine[2 * index + 1]);
		return true;
	}
	return false;
}

void ActD435::FindLineCrossCenter(float angleThresh, float minDistThresh, float maxDistThresh)
{
	Canny(maskImage, maskImage, 80, 80 * 2);

	vector<cv::Vec4i> line;
	vector<cv::Vec4i> filterLine;
	vector<cv::Vec4i> VerticalfilterLine;
	vector<cv::Vec4i> HorizonalfilterLine;
	bool veticalFilterFlag = 0;
	bool horizonalFilterFlag = 0;

	cv::HoughLinesP(maskImage, line, 1, CV_PI / 180, 60, 10, 30);
	cv::Mat lines = cv::Mat::zeros(maskImage.size(), CV_8UC1);

	//�ֹ��˳��ഹֱ�߶�
	for (size_t i = 0; i < line.size(); ++i)
	{
		float angle = 0;
		cv::Point pt1, pt2;
		pt1 = cv::Point(line[i][0], line[i][1]);
		pt2 = cv::Point(line[i][2], line[i][3]);

		if (pt1.x != pt2.x)
		{
			angle = atan(fabs(static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x))) * 180 / CV_PI;
		}
		else
		{
			angle = 90;
		}

		if (angle < 45)
			continue;
		if (pt1.x > pt2.x ? pt1.x : pt2.x < maskImage.cols / 2)
			continue;
		VerticalfilterLine.push_back(line[i]);
		cv::line(lines, pt1, pt2, 255, 3);
	}
	
	cout << VerticalfilterLine.size() << endl;

	veticalFilterFlag = MatchLine(VerticalfilterLine, filterLine, angleThresh, minDistThresh, maxDistThresh);

	for (size_t i = 0; i < line.size(); ++i)
	{
		float angle = 0;
		cv::Point pt1, pt2;
		pt1 = cv::Point(line[i][0], line[i][1]);
		pt2 = cv::Point(line[i][2], line[i][3]);

		if (pt1.x != pt2.x)
		{
			angle = atan(fabs(static_cast<float>(pt2.y - pt1.y) / (pt2.x - pt1.x))) * 180 / CV_PI;
		}
		else
		{
			angle = 90;
		}

		if (angle > 45)
			continue;
		HorizonalfilterLine.push_back(line[i]);
		cv::line(lines, pt1, pt2, 255, 3);
	}

	horizonalFilterFlag = MatchLine(HorizonalfilterLine, filterLine, angleThresh, minDistThresh, maxDistThresh);
	cv::Mat filterlines = cv::Mat::zeros(maskImage.size(), CV_8UC1);
	for (size_t i = 0; i < filterLine.size(); ++i)
	{
		cv::Point pt1, pt2;
		pt1 = cv::Point(filterLine[i][0], filterLine[i][1]);
		pt2 = cv::Point(filterLine[i][2], filterLine[i][3]);
		cv::line(filterlines, pt1, pt2, 255, 3);
	}

	cv::Point pt1, pt2, pt3, pt4;
	if (horizonalFilterFlag && veticalFilterFlag)
	{
		pt1 = GetCrossPoint(cv::Point(filterLine[0][0], filterLine[0][1]), cv::Point(filterLine[0][2], filterLine[0][3])
			, cv::Point(filterLine[2][0], filterLine[2][1]), cv::Point(filterLine[2][2], filterLine[2][3]));
		pt2 = GetCrossPoint(cv::Point(filterLine[0][0], filterLine[0][1]), cv::Point(filterLine[0][2], filterLine[0][3])
			, cv::Point(filterLine[3][0], filterLine[3][1]), cv::Point(filterLine[3][2], filterLine[3][3]));
		pt3 = GetCrossPoint(cv::Point(filterLine[1][0], filterLine[1][1]), cv::Point(filterLine[1][2], filterLine[1][3])
			, cv::Point(filterLine[2][0], filterLine[2][1]), cv::Point(filterLine[2][2], filterLine[2][3]));
		pt4 = GetCrossPoint(cv::Point(filterLine[1][0], filterLine[1][1]), cv::Point(filterLine[1][2], filterLine[1][3])
			, cv::Point(filterLine[3][0], filterLine[3][1]), cv::Point(filterLine[3][2], filterLine[3][3]));
		GetyawAngle(cv::Point2f(filterLine[0][0], filterLine[0][1]), cv::Point2f(filterLine[0][2], filterLine[0][3]), VERTICAL_FENSE);
	}
	else if(horizonalFilterFlag)
	{ 
		GetyawAngle(cv::Point2f(filterLine[0][0], filterLine[0][1]), cv::Point2f(filterLine[0][2], filterLine[0][3]), HORIZONAL_FENSE);
	}
	else
	{
		GetyawAngle(cv::Point2f(filterLine[0][0], filterLine[0][1]), cv::Point2f(filterLine[0][2], filterLine[0][3]), VERTICAL_FENSE);
	}
	lineCross = cv::Point2f((pt1.x + pt2.x + pt3.x + pt4.x) / 4, (pt1.y + pt2.y + pt3.y + pt4.y) / 4);
#ifdef DEBUG
	imshow("before match", lines);
	imshow("after match", filterlines);
	cout << "flag: " << horizonalFilterFlag << " " << veticalFilterFlag << endl;
	cv::circle(maskImage, lineCross, 5, 255, -1);
	cv::imshow("lincross", maskImage);
#endif // DEBUG
}

cv::Point3f ActD435::GetIrCorrdinate(cv::Point2f pt)
{
	double a = groundCoeff[0];
	double b = groundCoeff[1];
	double c = groundCoeff[2];
	double d = groundCoeff[3] * 1000;

	cv::Mat_<float> RGBpixed;
	RGBpixed = (cv::Mat_<float>(3, 1) << pt.x, pt.y, 1);

	cout << "pt: " << pt.x << " " << pt.y << endl;
	float Zc = 0, Xir = 0, Yir = 0, Zir = 0, m = 0, n = 0, t = 0, C1 = 0, C2 = 0, C3 = 0;
	cv::Mat_<float> C = rotationMatrix.inv() * (translationMatrix);
	cv::Mat_<float> K = rotationMatrix.inv() * intrinsicMatrixLeft.inv() * (RGBpixed);

	m = K.at<float>(0, 0);
	n = K.at<float>(1, 0);
	t = K.at<float>(2, 0);

	C1 = C.at<float>(0, 0);
	C2 = C.at<float>(1, 0);
	C3 = C.at<float>(2, 0);

	Zc = (-d + (a * C1 + b * C2 + c * C3)) / ((a * m + b * n + c * t));

	Xir = m * Zc - C1;
	Yir = n * Zc - C2;
	Zir = t * Zc - C3;
	Eigen::Vector3f point3D(Xir, Yir, Zir);
	point3D = RotatedMatrix * point3D;

	Xir = point3D[0];
	Yir = point3D[1];
	Zir = point3D[2];
#ifdef DEBUG
	cout << "m: " << m << " n: " << n << " t: " << t << endl;
	cout << "C1: " << C1 << " C2: " << C2 << " C3: " << C3 << endl;
	cout << "angleAlpha: " << angleAlpha * 180 / CV_PI << endl;
	cout << "groundCoeff: " << a << "\t" << b << "\t" << c << "\t" << d << endl;
	cout << "IrCoorinate: " << Xir << "\t" << Yir << "\t" << Zir << endl;
	cout << "Zc: " << Zc << "\tZir: " << Zir << endl;
#endif // DEBUG

	return cv::Point3f(Xir, Yir, Zir);
}

float ActD435::GetDepth(cv::Point2f& pt,cv::Point3f& pt1)
{
	float realDistance = 0;
	float depth = 0;
	float angle = 0;

	pt1 = GetIrCorrdinate(pt);
	if(pt1.z != 0)
		angle = atan(pt1.x / pt1.z);
	if (status == BEFORE_GRASSLAND_STAGE_2)
	{
		realDistance = (sqrt(pt1.z * pt1.z + pt1.x * pt1.x) + pillarRadius) * cos(cameraYawAngle + angle);
		nowXpos = (sqrt(pt1.z * pt1.z + pt1.x * pt1.x) + pillarRadius) * sin(cameraYawAngle + angle);
	}
	else
	{
		realDistance = (sqrt(pt1.z * pt1.z + pt1.x * pt1.x)) * cos(cameraYawAngle + angle);
		nowXpos = (sqrt(pt1.z * pt1.z + pt1.x * pt1.x)) * sin(cameraYawAngle + angle);
	}

#ifdef DEBUG
	cout << "distance: " << sqrt(pt1.z * pt1.z + pt1.x * pt1.x) << endl;
	cout << "nowXpos: " << nowXpos << endl;
	cout << "depth: " << pt1.z << endl;
	cout << "angle: " << angle * 180 / CV_PI << endl;
#endif
	return realDistance;
}

float ActD435::GetyawAngle(const cv::Point2f& pt1,const cv::Point2f& pt2, int fenseType)
{
	cv::Point3f point1In3D;
	cv::Point3f point2In3D;
	point1In3D = GetIrCorrdinate(pt1);
	point2In3D = GetIrCorrdinate(pt2);

	cout << "Pt1: " << point1In3D.x << " " << point1In3D.y << " " << point1In3D.z << endl;
	cout << "Pt2: " << point2In3D.x << " " << point2In3D.y << " " << point2In3D.z << endl;
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
	if (fenseType == HORIZONAL_FENSE)
		rotatedAngle = acos(z / Vec.norm());
	else
		rotatedAngle = acos(-x / Vec.norm());
	cameraYawAngle = -rotatedAngle + CV_PI / 2;
#ifdef DEBUG
	cout << "cameraYawAngle: " << cameraYawAngle * 180 / CV_PI << endl;
#endif // DEBUG
	return cameraYawAngle;
}

int flag = 0;
int ActD435::ClimbingMountainStageJudge()
{
	int whitePixNum = 0;
	uchar* data = maskImage.ptr(maskImage.rows - 50);

	for(int i = maskImage.cols - 1; i > 0; --i)
	{
		if (data[i] == 255)
			whitePixNum++;
	}
	if (whitePixNum > 600)
		flag = 1;

	if(whitePixNum > 600)
		return 2;	
	else
	{
		if (flag)
			return 3;
		else
			return 1;
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
//pPointCloud ActD435::pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color)
//{
//	// Object Declaration (Point Cloud)
//	pPointCloud cloud(new pointCloud);
//
//	// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
//	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;
//
//	//================================
//	// PCL Cloud Object Configuration
//	//================================
//	// Convert data captured from Realsense camera to Point Cloud
//	auto sp = points.get_profile().as<rs2::video_stream_profile>();
//
//	cloud->width = static_cast<uint32_t>(sp.width());
//	cloud->height = static_cast<uint32_t>(sp.height());
//	cloud->is_dense = false;
//	cloud->points.resize(points.size());
//
//	auto textureCoord = points.get_texture_coordinates();
//	auto Vertex = points.get_vertices();
//
//	// Iterating through all points and setting XYZ coordinates
//	// and RGB values
//	for (int i = 0; i < points.size(); i++)
//	{
//		//===================================
//		// Mapping Depth Coordinates
//		// - Depth data stored as XYZ values
//		//===================================
//		cloud->points[i].x = Vertex[i].x;
//		cloud->points[i].y = Vertex[i].y;
//		cloud->points[i].z = Vertex[i].z;
//
//		// Obtain color texture for specific point
//		RGB_Color = getColorTexture(color, textureCoord[i]);
//
//		// Mapping Color (BGR due to Camera Model)
//		//cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
//	   // cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
//		//cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>
//
//	}
//
//	return cloud; // PCL RGB Point Cloud generated
//}

//===================================================
// pointsToPointCloud
// - For point cloud without color information
//===================================================
mb_cuda::thrustCloudT ActD435::pointsToPointCloud(const rs2::points& points)
{
	mb_cuda::thrustCloudT thrust_cloud;
	thrust_cloud.resize(points.size());


	auto ptr = points.get_vertices();
	for (auto& p : thrust_cloud)
	{
		p.x = ptr->x;
		p.y = ptr->y;
		if(status<4)
			p.z = ptr->z*1.02;
		else
			p.z = ptr->z;

		
		ptr++;
	}


	return thrust_cloud;
}
