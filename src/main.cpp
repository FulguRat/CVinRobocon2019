#include <iostream>
#include <librealsense2/rs.hpp>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/opencv.hpp>
#include "act_d435.h"


using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	ActD435 actD435;

	actD435.init();

	while (true)
	{
		actD435.update();
	}

	return EXIT_SUCCESS;
}