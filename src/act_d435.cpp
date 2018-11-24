#include "act_d435.h"

ActD435::ActD435() : cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>), 
					 viewer("Cloud Viewer")
{

}

ActD435::~ActD435()
{

}

void ActD435::init(void)
{
	// Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
	// cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);

	//Instruct pipeline to start streaming with the requested configuration
	pipe.start(cfg);
}

void ActD435::update(void)
{
	clock_t start = clock();

	// Wait for the next set of frames from the camera
	frameSet = pipe.wait_for_frames();

	rs2::depth_frame depthFrame = frameSet.get_depth_frame();
	
	// Generate the pointcloud and texture mappings
	points = pc.calculate(depthFrame);

	auto cloudByPoints = pointsToPointCloud(points);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloudByPoints);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 6.0);
	pass.filter(*cloudFiltered);

	//blocks until the cloud is actually rendered
	viewer.showCloud(cloudFiltered);

	/*Mat depthImage(480, 640, CV_16UC1, (void*)depthFrame.get_data());
	imshow("depth", depthImage);

	Mat colorImage(480, 640, CV_8UC3, (void*)colorFrame.get_data());
	imshow("color", colorImage);

	waitKey(1);*/

	float totalTime = float(clock() - start) / float(CLOCKS_PER_SEC) * 1000.0f;
	cout << totalTime << endl;
}

pPointCloud ActD435::pointsToPointCloud(const rs2::points& points)
{
    pPointCloud cloud(new pcl::PointCloud<pcl::PointXYZ>);

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