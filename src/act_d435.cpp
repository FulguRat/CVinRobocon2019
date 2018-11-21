#include "act_d435.h"

CalcTime calcTime;

ActD435::ActD435() : align(RS2_STREAM_COLOR), 
					 cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGBA>), 
					 viewer("Cloud Viewer")
{

}

ActD435::~ActD435()
{

}

void ActD435::init(void)
{
	//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);

	//Instruct pipeline to start streaming with the requested configuration
	pipe.start(cfg);
}

void ActD435::update(void)
{
	calcTime.Begin();

	// Wait for the next set of frames from the camera
	frameSet = pipe.wait_for_frames();

	//////Get processed aligned frame
	////auto alignedFrameSet = align.process(frameSet);

	////// Trying to get both other and aligned depth frames
	////rs2::video_frame colorFrame = alignedFrameSet.first(RS2_STREAM_COLOR);
	////rs2::depth_frame alignedDepthFrame = alignedFrameSet.get_depth_frame();

	rs2::video_frame colorFrame = frameSet.get_color_frame();
	rs2::depth_frame alignedDepthFrame = frameSet.get_depth_frame();

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;

	// Tell pointcloud object to map to this color frame
	pc.map_to(colorFrame);

	// Generate the pointcloud and texture mappings
	points = pc.calculate(alignedDepthFrame);

	//auto cloudByPoints = pointsToPointCloud(points, colorFrame);

	//pcl::PassThrough<pcl::PointXYZRGBA> pass;
	//pass.setInputCloud(cloudByPoints);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(0.0, 6.0);
	//pass.filter(*cloudFiltered);

	////blocks until the cloud is actually rendered
	//viewer.showCloud(cloudFiltered);

	

	cout << calcTime.End() << endl;

	/*Mat depthImage(480, 640, CV_16UC1, (void*)alignedDepthFrame.get_data());
	imshow("depth", depthImage);

	Mat colorImage(480, 640, CV_8UC3, (void*)colorFrame.get_data());
	imshow("color", colorImage);

	waitKey(1);*/
}

std::tuple<uint8_t, uint8_t, uint8_t> ActD435::getTexcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();

	int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);

	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], \
		texture_data[idx + 1], texture_data[idx + 2]);
}

pPointCloud ActD435::pointsToPointCloud(const rs2::points& points, const rs2::video_frame& color)
{
	pPointCloud cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto texCoords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	for (int i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;

		std::tuple<uint8_t, uint8_t, uint8_t> currentColor;
		currentColor = getTexcolor(color, texCoords[i]);

		cloud->points[i].r = std::get<0>(currentColor);
		cloud->points[i].g = std::get<1>(currentColor);
		cloud->points[i].b = std::get<2>(currentColor);
	}

	return cloud;
}