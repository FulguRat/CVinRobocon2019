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
	//-- Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

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
	chrono::steady_clock::time_point start = chrono::steady_clock::now();

	//-- Wait for the next set of frames from the camera
	frameSet = pipe.wait_for_frames();

	chrono::steady_clock::time_point stop = chrono::steady_clock::now();
	auto totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	// cout << "retrieve time:" << double(totalTime.count()) / 1000.0f << "\t processing time:";
	start = chrono::steady_clock::now();

	//-- Get processed aligned frame
	//alignedFrameSet = align.process(frameSet);

	//-- Get both color and aligned depth frames
	//rs2::video_frame colorFrame = alignedFrameSet.first(RS2_STREAM_COLOR);
	//rs2::depth_frame alignedDepthFrame = alignedFrameSet.get_depth_frame();

	//-- For not align
	rs2::video_frame colorFrame = frameSet.get_color_frame();
	rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();

	srcImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);

	//-- Map Color texture to each point
	//rs2Cloud.map_to(colorFrame);

	//-- Generate the pointcloud and texture mappings
	rs2Points = rs2Cloud.calculate(alignedDepthFrame);
	cloudByRS2 = pointsToPointCloud(rs2Points);

	stop = chrono::steady_clock::now();
	totalTime = chrono::duration_cast<chrono::microseconds>(stop - start);
	// cout << double(totalTime.count()) / 1000.0f << endl;

	return cloudByRS2;
}
void ActD435::imgProcess()
{
	//srcImage = cv::imread("18.jpg");
	cv::split(srcImage, channels);
	channelR = channels[2].clone();
	dstImage = srcImage.clone();

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
	dst2Image = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
	bitwise_and(dstImage, dstImage, dst2Image, maskImage);

	//imshow("dst2", dst2Image);
	cv::imshow("mask", maskImage);
	cv::waitKey(1);
}

void ActD435::FindPillarCenter(void)
{
	cv::Mat dst = maskImage.clone();

	contours.clear();
	hierarchy.clear();

	if (center2.y < 280 && status == PASSING_GRASSLAND_STAGE_1)
	{
		cv::Rect remove(320, 300, 320, 180);
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
		if (center1.y > 350)
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
		circle(dst, center1, 2, 255, -1);
		contours.erase(contours.begin() + idx);
		idx = 0;
		if (contours.size() < 1)
			return;
		for (size_t i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i]) > contourArea(contours[idx]))
				idx = i;
		}
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

		//if ((fabs(x - center2.x) < 30 && fabs(y - center2.y) < 30) || (center2.x < 0.1))
		center2 = cv::Point2f(x, y);
		//else
		{

		}
		circle(dst, center2, 2, 255, -1);
		cout << "x: " << center2.x << "y: " << center2.y << endl;
	}
	break;
	}
	imshow("dst", dst);
	cv::waitKey(1);

}

void ActD435::FindLines(void)
{
	cv::Mat lineImg(cv::Size(640, 480), CV_8UC1);

	vector<cv::Vec2f> lines;

	Canny(channels[0], lineImg, 80, 80 * 2);

	cv::Mat houghline = lineImg.clone();

	int maxy = 0;
	int index = 0;

	HoughLines(lineImg, lines, 1.0, CV_PI / 180, 150, 0, 0);

	for (size_t i = 0; i < lines.size(); i++)
	{
		float rpho = lines[i][0];
		float theta = lines[i][1];

		//cout << theta << endl;
		if ((theta > 47 * CV_PI / 90) || (theta < 43 * CV_PI / 90))
		{
			lines.erase(lines.begin() + i);
			continue;
		}

		double a = cos(theta);
		double b = sin(theta);

		double x0 = a * rpho;
		double y0 = b * rpho;

		//find plane lines
		{
			if (y0 > maxy)
			{
				index = i;
				maxy = y0;
			}
		}
	}

	if (lines.size() < 1)
		return;

	float rpho = lines[index][0];
	float theta = lines[index][1];

	if ((theta > 47 * CV_PI / 90) || (theta < 43 * CV_PI / 90))
	{
		return;
	}

	cout << "theta: " << theta << endl;

	double a = cos(theta);
	double b = sin(theta);

	double x0 = a * rpho;
	double y0 = b * rpho;

	cv::Point pt1, pt2;
	pt1.x = cvRound(x0 - 1000 * b);
	pt1.y = cvRound(y0 + 1000 * a);
	pt2.x = cvRound(x0 + 1000 * b);
	pt2.y = cvRound(y0 - 1000 * a);
	line(houghline, pt1, pt2, 255, 2, CV_AA);

	imshow("houghline", houghline);

	cvWaitKey(1);
}

float ActD435::GetDepth(cv::Point2f& pt, float a, float b, float c, float d)
{
	double angleAlpha = (atan(c / b));

	cout << "angle: " << angleAlpha << endl;
	cv::Mat RGBpixed;
	RGBpixed = (cv::Mat_<float>(3, 1) << pt.x, pt.y, 1);

	float Zc = 0, Xir = 0, Yir = 0, Zir = 0, m = 0, n = 0, t = 0;
	cv::Mat C = (rotationMatrix.inv()) * (translationMatrix);
	//cout << C << endl;
	cv::Mat K = rotationMatrix.inv() * intrinsicMatrixLeft.inv() * (RGBpixed);
	//cout << K << endl;

	m = K.at<float>(0, 0);
	n = K.at<float>(1, 0);
	t = K.at<float>(2, 0);

	Zc = (-d + (a * C.at<float>(0, 0) + b * C.at<float>(1, 0) + c * C.at<float>(2, 0))) / ((a * m + b * n + c * t));

	Xir = m * Zc - C.at<float>(0, 0);
	Yir = n * Zc - C.at<float>(1, 0);
	Zir = t * Zc - C.at<float>(2, 0);

	if (status == BEFORE_GRASSLAND_STAGE_2)
	{
		lastXpos = center1In3D.x;
		lastZpos = center1In3D.z;
		center1In3D = cv::Point3f(Xir, Yir, Zir);
		nowXpos = center1In3D.x;
		nowZpos = center1In3D.z;
	}
		
	if (status == PASSING_GRASSLAND_STAGE_1)
	{
		lastXpos = center2In3D.x;
		lastZpos = center2In3D.z;
		center2In3D = cv::Point3f(Xir, Yir, Zir);
		nowXpos = center2In3D.x;
		nowZpos = center2In3D.z;
	}	
	cout << "groundCoeff: " << a << "\t" << b << "\t" << c << "\t" << d << endl;
	cout << "IrCoorinate: " << Xir << "\t" << Yir << "\t" << Zir << endl;
	cout << "Zc: " << Zc << "\tZir: " << Zir << endl;

	float realDistance = Zir * cos(angleAlpha) - Yir * sin(angleAlpha) + 120;

	return realDistance;
}

float ActD435::GetAngle(void)
{
	if (lastZpos == 0)
		return angle;
	angle = atan((lastZpos - nowZpos) / (lastXpos - nowXpos)) * 180 / CV_PI ;
	return angle; 
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
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    //-- Normals to Texture Coordinates conversion
    int xValue = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
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

    cloud->width  = static_cast<uint32_t>(sp.width() );
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
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

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