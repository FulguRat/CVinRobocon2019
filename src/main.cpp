#include <iostream>
#include <librealsense2/rs.hpp>
#include "act_d435.h"
#include "robot_locator.h"

using namespace std;

int main(int argc, char* argv[])
{
	ActD435 		actD435;
	RobotLocator 	fajLocator;

	actD435.init();
	fajLocator.status = STARTUP_INITIAL;

	while (!fajLocator.isStoped())
	{
		fajLocator.setInputCloud(actD435.update());
		fajLocator.preProcess();
		
		switch (fajLocator.status)
		{
			case STARTUP_INITIAL:
				fajLocator.status = BEFORE_DUNE;
				break;

			case BEFORE_DUNE:
				fajLocator.removeHorizontalPlanes(fajLocator.getFilteredCloud());
				break;

			default:
				break;
		}
	}

	return EXIT_SUCCESS;
}