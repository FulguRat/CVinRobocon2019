#include <iostream>
#include <librealsense2/rs.hpp>
#include "act_d435.h"
#include "robot_locator.h"

using namespace std;

int main(int argc, char* argv[])
{
	ActD435			fajD435;
	RobotLocator 	fajLocator;

	fajD435.init();
	fajLocator.init(fajD435);
	fajLocator.status = STARTUP_INITIAL;

	while (!fajLocator.isStoped())
	{
		fajLocator.updateCloud();
		fajLocator.preProcess();
		
		switch (fajLocator.status)
		{
			case STARTUP_INITIAL:
				fajLocator.status = BEFORE_DUNE;
				break;

			case BEFORE_DUNE:
				fajLocator.locateBeforeDune();
				break;

			default:
				break;
		}
	}

	return EXIT_SUCCESS;
}