#include "act_d435.h"

roiFlag colorFrameRoi = midRoi;
//int mode = LEFT_MODE;
int mode = RIGHT_MODE;
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
	cout << "step3 ok" << endl;
	//-- Wait for frames from the camera to settle
	for (int i = 0; i < 5; i++)
	{
		//Drop several frames for auto-exposure
		frameSet = pipe.wait_for_frames();
	}
}

void ActD435::update(void)
{
		chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	static int time = 0;

	//chrono::steady_clock::time_point start = chrono::steady_clock::now();

	//-- Wait for the next set of frames from the camera
	while(RotatedMatrix.size() != 0)
	{
	}
	frameSet = pipe.wait_for_frames();


	//-- Get processed aligned frame
	//alignedFrameSet = align.process(frameSet);

	//-- Get both color and aligned depth frames
	//rs2::video_frame colorFrame = alignedFrameSet.first(RS2_STREAM_COLOR);
	//rs2::depth_frame alignedDepthFrame = alignedFrameSet.get_depth_frame();

	//-- For not align
	rs2::video_frame colorFrame = frameSet.get_color_frame();
	rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();
	if (time == 0)
	{
		rs2::stream_profile dprofile = alignedDepthFrame.get_profile();
		rs2::stream_profile cprofile = colorFrame.get_profile();
		
		rs2::video_stream_profile cvsprofile(cprofile);
		color_intrin = cvsprofile.get_intrinsics();

		rs2::video_stream_profile dvsprofile(dprofile);
		depth_intrin = dvsprofile.get_intrinsics();

		depth2color_extrin = dprofile.get_extrinsics_to(cprofile);
		color2depth_extrin = cprofile.get_extrinsics_to(dprofile);
		cout << "Get intrinics" << endl;
		time++;
	}
	srcImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
	data = (uint16_t*)alignedDepthFrame.get_data();
	if(!initFlag && xkFlag)
	{
		mutex1.lock();
		srcImageQueue.push(srcImage);
		mutex1.unlock();
	}

	//-- Map Color texture to each point
	//rs2Cloud.map_to(colorFrame);

	//-- Generate the pointcloud and texture mappings
	rs2Points = rs2Cloud.calculate(alignedDepthFrame);
	if(!xkFlag)
		sourceThrust = pointsToPointCloud(rs2Points);
	else
		cloudByRS2 = pointsToPCLPointCloud(rs2Points);
	stop = chrono::steady_clock::now();
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
cout << "mode in actd435: " << mode << endl;
	//cout << "update Time: : " << double(totalTime.count()) / 1000.0f << endl;
	// cout << double(totalTime.count()) / 1000.0f << endl;
}
void ActD435::imgProcess()
{
	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;
	cv::Mat tempSrc;
	while(srcImageQueue.size() == 0)
	{
	}
	mutex1.lock();	
	tempSrc = srcImageQueue.front();
	srcImageQueue.pop();		
	mutex1.unlock();	
	cv::imshow("src", tempSrc);
	cvWaitKey(1);
	switch (status)
	{
		case PASSING_DUNE:
		{
			cv::cvtColor(srcImage, LABImage, CV_BGR2Lab);
			cv::cvtColor(srcImage, HSVImage, CV_BGR2HSV);
			cv::cvtColor(srcImage, grayImage, CV_BGR2GRAY);
			cv::split(LABImage, channels);
			cv::threshold(channels[2], channelB, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);

			FindHorizonalHoughLine(channelB);

			cv::split(HSVImage, channels);
			cv::threshold(channels[1], channelS, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
			cv::threshold(grayImage, grayImage, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
			FillHoles(channelS);
			cv::imshow("channelS", channelS);

			cv::bitwise_or(channelS, grayImage, grayImage);

			cv::imshow("grayImage", grayImage);
			maskImage = grayImage.clone();
			cvWaitKey(1);
		}
		break;
		case BEFORE_GRASSLAND_STAGE_1:
		{
			cv::split(srcImage, channels);

			cv::threshold(channels[2], channelR, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
			cv::imshow("channelR", channelR);
			maskImage = channelR.clone();
			FillHoles(maskImage);
			FindHorizonalHoughLine(maskImage);
		}
		break;

		case BEFORE_GRASSLAND_STAGE_2:
		{
			cv::cvtColor(srcImage, HSVImage, CV_BGR2HSV);
			cv::cvtColor(srcImage, LABImage, CV_BGR2Lab);
			cv::split(LABImage, channels);

			inRange(channels[1], 0, 149, channelA);
			inRange(channels[2], 157, 255, channelB);

			//桩
			cv::bitwise_and(channelA, channelB, maskImage);
			FillHoles(maskImage);
#ifdef DEBUG
			cv::imshow("pillar", maskImage);
#endif // DEBUG
			//GetyawAngle
			pillarStatus = FindPillarCenter();

			if (mode == RIGHT_MODE)
			{
				cv::threshold(channels[2], channelB, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
				cv::imshow("channelB", channelB);
				grayImage = channelB.clone();
			}
			else
			{
				cv::Mat mask;
				cv::bitwise_not(channelB, mask);
				cv::imshow("mask", mask);
				threshold_with_mask(channels[2], grayImage, mask, CV_THRESH_BINARY_INV);
			}
			cv::imshow("gray", grayImage);
			FindVerticalHoughLine(grayImage);
		}
		break;

		case PASSING_GRASSLAND_STAGE_1:
		{
			cv::cvtColor(srcImage, LABImage, CV_BGR2Lab);
			cv::split(LABImage, channels);

			inRange(channels[1], 0, 149, channelA);
			inRange(channels[2], 157, 255, channelB);

			cv::bitwise_and(channelA, channelB, maskImage);
#ifdef DEBUG
			cv::imshow("pillar", maskImage);
#endif // DEBUG
			//GetyawAngle
			pillarStatus = FindPillarCenter();

			cv::GaussianBlur(srcImage, srcImage, cv::Size(3, 3), 0);
			cv::split(srcImage, channels);
			cv::threshold(channels[2], channelR, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
			grayImage = channelR.clone();
			cv::imshow("gray", grayImage);
		}
		break;

		case PASSING_GRASSLAND_STAGE_2:
		{
			cv::split(srcImage, channels);
			//cv::cvtColor(srcImage, grayImage, CV_BGR2GRAY);
			cv::threshold(channels[1], grayImage, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
			if (mode == LEFT_MODE)
			{
				cv::threshold(channels[2], channels[2], 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
				channelR = channels[2].clone();
				imshow("beforeFill", channelR);
				FillHoles(channelR);
				SetSeedPoint(channelR);
				cv::floodFill(channelR, seedPoint, 0);
				cv::bitwise_xor(channels[2], channelR, maskImage);
				imshow("result", maskImage);
				FindVerticalHoughLine(maskImage);
				imshow("channelR", channelR);
			}
			else
			{
				cv::threshold(channels[0], channels[0], 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY);
				channelB = channels[0].clone();
				//FillHoles(channelB);
				imshow("beforeFill", channelB);
				SetSeedPoint(channelB);
				cv::floodFill(channelB, seedPoint, 0);
				cv::bitwise_xor(channels[0], channelB, maskImage);
				imshow("result", maskImage);
				FindVerticalHoughLine(maskImage);
				imshow("afterFill", channelB);
			}
			FindLineCrossCenter(grayImage);
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
			cv::split(tempSrc, channels);
			inRange(channels[1], 105, 255, channelG);
			FillHoles(channelG);
			cv::Mat beforeFill = channelG.clone();
			cv::imshow("before fill", beforeFill);
			maskImage = channelG.clone();
			SetSeedPoint(channelG);
			floodFill(channelG, seedPoint, 0);

			bitwise_xor(beforeFill, channelG, maskImage);
		}
		break;
		case REACH_MOUNTAIN:
		{
			cv::cvtColor(tempSrc, tempSrc, CV_BGR2Lab);
			cv::split(tempSrc, channels);
			if (mode == LEFT_MODE)
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
stop = chrono::steady_clock::now();
auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
cout << "Time: : " << double(totalTime.count()) / 1000.0f << endl;
#ifdef DEBUG
//	cv::imshow("binary", maskImage);
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
	line.clear();
	int begin = searchBeginPoint.y;
	int end = 0;
	int cols = searchBeginPoint.x;
	int countFlag = 0;
	int whitePixNum = 0;
	uchar thresh = 20;

	do
	{
		whitePixNum = 0;
		int tempbegin = 0;

		if (mode == LEFT_MODE)
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
					if (maskImage.at<uchar>(rows - 1, cols) == 0
						&& maskImage.at<uchar>(rows, cols - 1) == 0
						&& maskImage.at<uchar>(rows + 1, cols) == 0
						&& maskImage.at<uchar>(rows, cols + 1) == 0
						)
					{
						whitePixNum = 0;
						countFlag = 0;
					}

					else
						whitePixNum++;
				}

				if (whitePixNum > thresh)
				{
					thresh = 6;
					if (tempbegin < 478)
						line.push_back(cv::Point(cols, tempbegin));
					{
						begin = tempbegin + 15;
						end = rows - 15;
					}
					countFlag = 0;
					break;
				}
			}
		}
		if (begin > 478)
			begin = 478;
		if (end < 0)
			end = 0;
	} while (whitePixNum > thresh && cols > 0 && cols < maskImage.cols - 1);

	if (line.size() > 30)
	{
		fenseCorner = line[line.size() - 7];
		fenseCorner.x = fenseCorner.x;
		fenseCorner.y = lineSlop * fenseCorner.x + intercept;
		cout << "lineSlop: " << lineSlop << " intercept: " << intercept << endl;
	}
	else
	{
		cout << "fenseCorner not found" << endl;
	}
	line.clear();
	cv::circle(maskImage, fenseCorner, 10, 255, -1);
	cv::circle(maskImage, searchBeginPoint, 10, 255, -1);
	cv::circle(maskImage, line[line.size() - 7], 10, 255, -1);
	cv::imshow("fenseCorner", maskImage);
	cvWaitKey(1);
	cout << "x: " << fenseCorner.x << " y:" << fenseCorner.y << endl;
cout << "linex: " << line[line.size() - 7].x << " liney:" << line[line.size() - 7].y << endl;
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
	int lastBegin = 0;
	int lastEnd = 0;
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
							if (whitePixNum > 50 || (whitePixNum - whitePixNumLast >= 10 && whitePixNumLast != 0) || cols > lastBegin)
							{
								rows -= 20;
								whitePixNum = whitePixNumLast;
								break;
							}
							if (tempbegin > 590 && line.size() < 10)
							{
								whitePixNum = 0;
								continue;
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
			lastBegin = tempbegin;
			lastEnd = end + 20;
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
							if (whitePixNum > 50 || (whitePixNum - whitePixNumLast >= 10 && whitePixNumLast != 0) || cols < lastBegin)
							{
								rows -= 20;
								whitePixNum = whitePixNumLast;
								break;
							}				
							if (tempbegin < 50 && line.size() < 10)
							{
								whitePixNum = 0;
								continue;
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
			lastBegin = tempbegin;
			lastEnd = end - 20;
		} while (notFoundCnt < 45 && rows > 1);
	}

	
	if (line.size() > 30)
	{
		lineEnd = line[line.size() - 3];

		linePt1 = line[line.size() - 10];
		linePt2 = line[10];
		lineFoundFlag = true;
#ifdef DEBUG
		cv::circle(maskImage, lineEnd, 5, 255, -1);
		cv::line(maskImage, line[10], line[line.size() - 10], 255, 5);
		cout << "x: " << lineEnd.x << " y:" << lineEnd.y << endl;
#endif // DEBUG
	}
#ifdef DEBUG
	else
	{
		lineFoundFlag = false;
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

cv::Point ActD435::SetSeedPoint(cv::Mat& src)
{
	int rows = 100;
	switch (status)
	{
		case PASSING_GRASSLAND_STAGE_2:
		{
			rows = src.rows - 100;
		}
		break;
		case CLIMBING_MOUNTAIN:
		{
			rows = src.rows - 50;
			break;
		}
		default:
			break;
	}
	if (status == PASSING_GRASSLAND_STAGE_2)
	{
		uchar* data = src.ptr<uchar>(rows);
		if (mode == RIGHT_MODE)
		{
			for (int i = 50; i < src.cols; i++)
			{
				if (data[i] == 255 && data[i - 1] == 255 && data[i + 1] == 255 && data[i + 2] == 255 && data[i - 2] == 255 && data[i + 3] == 255 && data[i - 3] == 255)
				{
					seedPoint = cv::Point(i, rows);
					break;
				}
			}
		}
		else
		{
			for (int i = src.cols - 50; i > 0; i--)
			{
				if (data[i] == 255 && data[i - 1] == 255 && data[i + 1] == 255 && data[i + 2] == 255 && data[i - 2] == 255 && data[i + 3] == 255 && data[i - 3] == 255)
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
			uchar* data = src.ptr<uchar>(rows);
			for (int i = src.cols - 4; i > 3; i--)
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
			uchar* data = src.ptr<uchar>(rows);
			for (int i = 3; i < src.cols - 4; i++)
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
			if ((comparedHoughLine.lineAngle == 90 && fabs(comparedHoughLine.lineAngle - firstHoughLine.lineAngle) > 20)
				|| (comparedHoughLine.lineAngle != 90 && comparedHoughLine.lineSlop * firstHoughLine.lineSlop < 0)
				)	
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
			linePt1 = pt1;
			linePt2 = pt2;
			lineFoundFlag = true;
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
					linePt1 = pt1;
					linePt2 = pt2;
					lineFoundFlag = true;
#ifdef DEBUG
					cv::line(lines, pt1, pt2, 255, 5);
					cv::line(lines, pt3, pt4, 255, 1);
#endif // DEBUG
				}				
				else
				{
					linePt1 = pt3;
					linePt2 = pt4;
					lineFoundFlag = true;
#ifdef DEBUG
					cv::line(lines, pt1, pt2, 255, 1);
					cv::line(lines, pt3, pt4, 255, 5);
#endif // DEBUG
				}			
			}
			else if (firstHoughLine.lineAngle == 90)
			{
				linePt1 = pt3;
				linePt2 = pt4;
				lineFoundFlag = true;
#ifdef DEBUG
				cv::line(lines, pt1, pt2, 255, 1);
				cv::line(lines, pt3, pt4, 255, 5);
#endif // DEBUG
			}
			else
			{
				linePt1 = pt1;
				linePt2 = pt2;
				lineFoundFlag = true;
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
		lineFoundFlag = false;
#ifdef DEBUG
		cout << "cannot find line." << endl;
#endif // DEBUG
	}
	
}
void ActD435::FindHorizonalHoughLine(cv::Mat& src, int flag)
{
	cv::Mat canny;
	linePoints.clear();
	Canny(src, canny, 80, 80 * 2);

	vector<cv::Vec4i> line;
	vector<cv::Vec4i> filterLine;

	cv::HoughLinesP(canny, line, 1, CV_PI / 180, 60, 10, 30);

	HoughLine horizonalHoughLine;
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
			comparedHoughLine.intercept = pt1.y - comparedHoughLine.lineSlop * pt1.x;
			comparedHoughLine.lineAngle = atan(fabs(comparedHoughLine.lineSlop)) * 180 / CV_PI;
		}
		else
		{
			comparedHoughLine.lineSlop = 0;
			comparedHoughLine.intercept = pt2.y;
			comparedHoughLine.lineAngle = 90;
		}
		if (status == PASSING_DUNE)
		{
			if (comparedHoughLine.lineAngle > 45.0f
				|| comparedHoughLine.lineSlop * (2 * mode - 1) < 0.0f
				|| (flag == 0 && line[i][1] / 2 + line[i][3] / 2 < 150)
				|| (flag == 1 && (std::max(line[i][1],line[i][3]) > 300 || std::min(line[i][1],line[i][3]) < 50))
				)
			{
				continue;
			}
		}
		if (status == BEFORE_GRASSLAND_STAGE_1)
		{
			if (comparedHoughLine.lineAngle > 50.0f
				|| comparedHoughLine.lineSlop * (2 * mode - 1) > 0.0f
				|| (flag == 0 && std::max(line[i][1], line[i][3]) < src.rows / 2)
				)
			{
				continue;
			}
		}


		if (comparedHoughLine.distance > horizonalHoughLine.distance)
		{
			horizonalHoughLine = comparedHoughLine;
		}

	}
	if (horizonalHoughLine.distance != 0)
		filterLine.push_back(line[horizonalHoughLine.index]);

	cv::Mat lines = cv::Mat::zeros(src.size(), CV_8UC1);
	if (filterLine.size() > 0)
	{
		cv::Point pt1, pt2;
		if (flag == 0)
		{
			if (status == PASSING_DUNE)
			{
				float b = filterLine[0][1] - horizonalHoughLine.lineSlop * filterLine[0][0];
				b += 20;
				pt1 = cv::Point(filterLine[0][0], horizonalHoughLine.lineSlop * filterLine[0][0] + b);
				pt2 = cv::Point(filterLine[0][2], horizonalHoughLine.lineSlop * filterLine[0][2] + b);
			}
			else
			{
				lineSlop = horizonalHoughLine.lineSlop;
				intercept = horizonalHoughLine.intercept;
				pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
				pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);
				searchBeginPoint = cv::Point((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2);
			}
			linePt1 = pt1;
			linePt2 = pt2;
			lineFoundFlag = true;
#ifdef DEBUG

			cv::line(lines, pt1, pt2, 255, 5);
			cv::imshow("lines", lines);
			cvWaitKey(1);
#endif // DEBUG
		}
		else
		{
			pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
			pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);
			linePoints.push_back(pt1);
			linePoints.push_back(pt2);
			linePoints.push_back(cv::Point(1.0 * filterLine[0][0] / 2 + 1.0 * filterLine[0][2] / 2, 1.0 * filterLine[0][1] / 2 + 1.0 * filterLine[0][3] / 2));
			linePoints.push_back(cv::Point(1.0 * filterLine[0][0] / 4 + 3.0 * filterLine[0][2] / 4, 1.0 * filterLine[0][1] / 4 + 3.0 * filterLine[0][3] / 4));
			linePoints.push_back(cv::Point(3.0 * filterLine[0][0] / 4 + 1.0 * filterLine[0][2] / 4, 3.0 * filterLine[0][1] / 4 + 1.0 * filterLine[0][3] / 4));

			int cols = pt1.x / 2 + pt2.x / 2;
			int rows = pt1.y / 2 + pt2.y / 2;
			int whitePixNum = 0;
			for (int i = 0; i < 10; i++)
			{
				if (src.at<uchar>(rows++, cols) == 255)
					whitePixNum++;
			}
			if (whitePixNum > 6)
				farLineFlag = true;
			else
				farLineFlag = false;

#ifdef DEBUG
			cv::line(canny, pt1, pt2, 255, 5);
			cv::circle(canny, linePoints[2], 10, 255, -1);
			cv::circle(canny, linePoints[3], 10, 255, -1);
			cv::circle(canny, linePoints[4], 10, 255, -1);
			cv::imshow("target line", canny);
			cvWaitKey(1);
			cout << "lineSlop: " << horizonalHoughLine.lineSlop << endl;
#endif // DEBUG
		}

	}
	else
	{
		if (flag == 0)
			lineFoundFlag = false;
#ifdef DEBUG
		cout << "cannot find line." << endl;
#endif // DEBUG
	}
}
void ActD435::FindVerticalHoughLine(cv::Mat& src, int flag)
{
	cv::Mat canny;
	cv::Canny(src, canny, 80, 80 * 2);
	cv::imshow("Canny", canny);
	cvWaitKey(1);
	vector<cv::Vec4i> line;
	vector<cv::Vec4i> filterLine;
	if (status == PASSING_GRASSLAND_STAGE_1)
	{
		cv::HoughLinesP(canny, line, 1, CV_PI / 180, 100, 10, 70);
	}
	else
	{
		cv::HoughLinesP(canny, line, 1, CV_PI / 180, 60, 10, 30);
	}

	HoughLine verticalHoughLine;;
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
			comparedHoughLine.intercept = pt1.y - comparedHoughLine.lineSlop * pt1.x;
			comparedHoughLine.lineAngle = atan(fabs(comparedHoughLine.lineSlop)) * 180 / CV_PI;
		}
		else
		{
			continue;
		}
		if (comparedHoughLine.lineSlop * (2 * mode - 1) < 0.0f
			|| comparedHoughLine.lineAngle < 25.0f
			|| (status == BEFORE_GRASSLAND_STAGE_2 && ((mode == RIGHT_MODE && std::max(pt1.x, pt2.x) > 600) || (mode == RIGHT_MODE && std::min(pt1.x, pt2.x) < 100) || std::max(pt1.y, pt2.y) < 200))
			)
		{
			continue;
		}
		if (status == PASSING_GRASSLAND_STAGE_1)
		{
			float distance = abs(comparedHoughLine.lineSlop * center2.x - center2.y + comparedHoughLine.intercept) / sqrt(1 + comparedHoughLine.lineSlop * comparedHoughLine.lineSlop);
			if (distance > 100.0f)
				continue;
		}

		if (comparedHoughLine.distance > verticalHoughLine.distance)
		{
			verticalHoughLine = comparedHoughLine;
		}

	}
	cv::Point pt1, pt2;

	if (verticalHoughLine.distance != 0)
	{
		filterLine.push_back(line[verticalHoughLine.index]);
		if (status == PASSING_GRASSLAND_STAGE_1)
		{
			float b = filterLine[0][1] - verticalHoughLine.lineSlop * filterLine[0][0];
			b -= 20;
			if (flag == 0)
			{
				pt1.y = center2.y - 20;
				pt1 = cv::Point((pt1.y - b) / verticalHoughLine.lineSlop, pt1.y);
				pt2 = cv::Point((pt1.y - b + 100) / verticalHoughLine.lineSlop, pt1.y + 100);
			}
			else
			{
				pt1.y = center2.y;
				pt1 = cv::Point((pt1.y - b) / verticalHoughLine.lineSlop, pt1.y);
				pt2 = cv::Point((pt1.y - b - 200) / verticalHoughLine.lineSlop, pt1.y - 200);
			}
		}
		else if (status == PASSING_GRASSLAND_STAGE_2)
		{
			linePoint1 = cv::Point(filterLine[0][0], filterLine[0][1]);
			linePoint2 = cv::Point(filterLine[0][2], filterLine[0][3]);
			if (linePoint1.y < linePoint2.y)
			{
				pt1 = linePoint1;
				pt2 = cv::Point(1.0 * linePoint1.x / 2 + 1.0 * filterLine[0][2] / 2, 1.0 * linePoint1.y / 2 + 1.0 * filterLine[0][3] / 2);
			}
			else
			{
				pt1 = linePoint2;
				pt2 = cv::Point(1.0 * linePoint2.x / 2 + 1.0 * filterLine[0][0] / 2, 1.0 * linePoint2.y / 2 + 1.0 * filterLine[0][1] / 2);
			}
		}
		else
		{
			pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
			pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);
		}
	}


	cv::Mat lines = cv::Mat::zeros(src.size(), CV_8UC1);
	if (verticalHoughLine.distance != 0)
	{
		linePt1 = pt1;
		linePt2 = pt2;
		lineFoundFlag = true;
#ifdef DEBUG
		cv::line(canny, pt1, pt2, 255, 5);
		cv::imshow("lines", canny);
		cvWaitKey(1);
#endif // DEBUG
	}
	else
	{
		lineFoundFlag = false;
#ifdef DEBUG
		cout << "cannot find line." << endl;
#endif // DEBUG
	}
}

void ActD435::FindLineCrossCenter(cv::Mat& src, int flag)
{
	cv::Mat canny;
	cv::Canny(src, canny, 80, 80 * 2);
	cv::imshow("Canny", canny);
	cvWaitKey(1);
	vector<cv::Vec4i> line;
	vector<cv::Vec4i> filterLine;

	cv::HoughLinesP(canny, line, 1, CV_PI / 180, 60, 10, 30);

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
			comparedHoughLine.intercept = pt1.y - comparedHoughLine.lineSlop * pt1.x;
			comparedHoughLine.lineAngle = atan(fabs(comparedHoughLine.lineSlop)) * 180 / CV_PI;
		}
		else
		{
			comparedHoughLine.lineSlop = 0;
			comparedHoughLine.intercept = pt2.y;
			comparedHoughLine.lineAngle = 90;
		}
		if (comparedHoughLine.lineSlop * (2 * mode - 1) > 0.0f
			|| std::max(pt1.y, pt2.y) < 100
			|| std::min(pt1.y, pt2.y) < 50
			|| std::max(pt1.y, pt2.y) > 300
			|| ((pt1.x / 2 + pt2.x / 2 - linePoint1.x / 2 - linePoint2.x / 2) * (2 * mode - 1) > 0)
			)
		{
			continue;
		}
		comparedHoughLine.distance = sqrt((pt2.y - pt1.y) * (pt2.y - pt1.y) + (pt2.x - pt1.x) * (pt2.x - pt1.x));
		comparedHoughLine.index = i;

		if (comparedHoughLine.distance > horizonalHoughLine.distance)
		{
			horizonalHoughLine = comparedHoughLine;
		}

	}

	if (horizonalHoughLine.distance != 0)
		filterLine.push_back(line[horizonalHoughLine.index]);
	if (filterLine.size() != 0)
	{
		cv::Point pt1 = cv::Point(filterLine[0][0], filterLine[0][1]);
		cv::Point pt2 = cv::Point(filterLine[0][2], filterLine[0][3]);
		int cols = pt1.x / 2 + pt2.x / 2;
		int rows = pt1.y / 2 + pt2.y / 2;
		int whitePixNum = 0;
		for (int i = 0; i < 8; i++)
		{
			if (src.at<uchar>(rows++, cols) == 255)
				whitePixNum++;
		}
		if (whitePixNum > 4)
			farLineFlag = true;
		else
			farLineFlag = false;
#ifdef DEBUG
		cv::line(canny, pt1, pt2, 255, 2, -1);
		cv::line(canny, linePoint1, linePoint2, 255, 2, -1);
#endif
		lineCross = GetCrossPoint(pt1, pt2, linePoint1, linePoint2);
	}

#ifdef DEBUG
	cv::circle(canny, lineCross, 5, 255, -1);
	cv::imshow("lincross", canny);
#endif // DEBUG
}

cv::Point3f ActD435::GetIrCorrdinate(cv::Point2f pt)
{	
	vector<float> groundCoeff = groundCoffQueue.front();
	double a = groundCoeff[0];
	double b = groundCoeff[1];
	double c = groundCoeff[2];
	double d = groundCoeff[3] * 1000;

	float depth_pixel[2] = { 0 };
	float color_pixel[2] = { pt.x, pt.y };
	float point[3] = { 0 };

	rs2_project_color_pixel_to_depth_pixel(depth_pixel, data, 0.001, 0.1, 2.5, &depth_intrin, &color_intrin, &color2depth_extrin, &depth2color_extrin, color_pixel);

	float depth = data[(int)depth_pixel[1] * depth_intrin.width + (int)depth_pixel[0]];

	rs2_deproject_pixel_to_point(point, &depth_intrin, depth_pixel, depth);

	Eigen::Vector3f point3D(point[0], point[1], point[2]);
	point3D = RotatedMatrix.front() * point3D;

	float Xir = 0, Yir = 0, Zir = 0;

	Xir = point3D[0];
	Yir = point3D[1];
	Zir = point3D[2];

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
	if(status == PASSING_DUNE && farLineFlag)
		realDistance -= 50;
	if(status == PASSING_GRASSLAND_STAGE_2 && farLineFlag)
		realDistance -= 30;
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

double ActD435::getThreshVal_Otsu_8u_mask(const cv::Mat src, const cv::Mat& mask)
{
	const int N = 256;
	int M = 0;
	int i, j, h[N] = { 0 };
	for (i = 0; i < src.rows; i++)
	{
		const uchar* psrc = src.ptr(i);
		const uchar* pmask = mask.ptr(i);
		for (j = 0; j < src.cols; j++)
		{
			if (pmask[j])
			{
				h[psrc[j]]++;
				++M;
			}
		}
	}
	double mu = 0, scale = 1. / (M);
	for (i = 0; i < N; i++)
		mu += i * (double)h[i];

	mu *= scale;
	double mu1 = 0, q1 = 0;
	double max_sigma = 0, max_val = 0;
	for (i = 0; i < N; i++)
	{
		double p_i, q2, mu2, sigma;
		p_i = h[i] * scale;
		mu1 *= q1;
		q1 += p_i;
		q2 = 1. - q1;
		if (std::min(q1, q2) < FLT_EPSILON || std::max(q1, q2) > 1. - FLT_EPSILON)
			continue;

		mu1 = (mu1 + i * p_i) / q1;
		mu2 = (mu - q1 * mu1) / q2;
		sigma = q1 * q2 * (mu1 - mu2)*(mu1 - mu2);
		if (sigma > max_sigma)
		{
			max_sigma = sigma;
			max_val = i;
		}
	}

	return max_val;
}

void ActD435::threshold_with_mask(cv::Mat& src, cv::Mat& dst, cv::Mat& mask, int type)
{
	double th = getThreshVal_Otsu_8u_mask(src, mask);
	if (type == CV_THRESH_BINARY)
	{
		threshold(src, dst, th, 255, CV_THRESH_BINARY);
	}
	else
	{
		threshold(src, dst, th, 255, CV_THRESH_BINARY_INV);
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
		p.z = ptr->z*1.015;
	
		ptr++;
	}
	return thrust_cloud;
}
pPointCloud ActD435::pointsToPCLPointCloud(const rs2::points& points)
{
	pPointCloud cloud(new pointCloud);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;

	int count = 0;
	auto ptr = points.get_vertices();

	for (int i = 0;i < points.size();++i)
	{	
		if(ptr->z > 0.0 && ptr->z < 1.5 && ptr->x > -0.5 && ptr->x < 0.5 && ptr->y > 0.1)
		{
			count++;
			pointType point;
			point.x = ptr->x;
			point.y = ptr->y;
			point.z = ptr->z - 0.0042;
			cloud->points.push_back(point);
		}

		ptr++;
	}
	cout << "count: " << count << endl;
	return cloud;
}
