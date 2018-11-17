// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

using pcl_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr;

std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
	const int w = texture.get_width(), h = texture.get_height();

	int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
	int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);

	int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
	const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
	return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], \
		texture_data[idx + 1], texture_data[idx + 2]);
}


pcl_ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color) 
{
	pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto tex_coords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	for (int i = 0; i < points.size(); ++i)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;

		std::tuple<uint8_t, uint8_t, uint8_t> current_color;
		current_color = get_texcolor(color, tex_coords[i]);

		cloud->points[i].r = std::get<0>(current_color);
		cloud->points[i].g = std::get<1>(current_color);
		cloud->points[i].b = std::get<2>(current_color);
	}

	return cloud;
}


void configViewer(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
}

int main(int argc, char* argv[])
{
	//// Declare depth colorizer for pretty visualization of depth data
	//rs2::colorizer color_map;

	//// Declare pointcloud object, for calculating pointclouds and texture mappings
	//rs2::pointcloud pc;
	//// We want the points object to be persistent so we can display the last cloud when a frame drops
	//rs2::points points;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	//Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;
	
	////Add desired streams to configuration
	//cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	//cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

	//Instruct pipeline to start streaming with the requested configuration
	pipe.start(cfg);

	// Create a rs2::align object.
	// rs2::align allows us to perform alignment of depth frames to others frames
	rs2::align align(RS2_STREAM_COLOR);

	pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//This will only get called once
	viewer.runOnVisualizationThreadOnce(configViewer);

	while (!viewer.wasStopped())
	{
		// Wait for the next set of frames from the camera
		auto frames = pipe.wait_for_frames();

		//Get processed aligned frame
		auto aligned_frames = align.process(frames);

		// Trying to get both other and aligned depth frames
		rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
		rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

		//auto depth = frames.get_depth_frame()/*.apply_filter(color_map)*/;
		//auto color = frames.get_color_frame();

		//If one of them is unavailable, continue iteration
		if (!aligned_depth_frame || !color_frame)
		{
			continue;
		}

		// Declare pointcloud object, for calculating pointclouds and texture mappings
		rs2::pointcloud pc;

		// Tell pointcloud object to map to this color frame
		pc.map_to(color_frame);

		// Generate the pointcloud and texture mappings
		rs2::points points = pc.calculate(aligned_depth_frame);

		//// Tell pointcloud object to map to this color frame
		//pc.map_to(color_frame);

		auto pcl_points = points_to_pcl(points, color_frame);

		
		pcl::PassThrough<pcl::PointXYZRGBA> pass;
		pass.setInputCloud(pcl_points);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, 6.0);
		pass.filter(*cloud_filtered);

		//blocks until the cloud is actually rendered
		viewer.showCloud(cloud_filtered);

		//Mat depthImage(Size(640, 480), CV_16UC1, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
		//depthImage = depthImage.mul(10);

		//// Update the window with new data
		//imshow("depth", depthImage);

		//waitKey(1);
	}

	return EXIT_SUCCESS;
}