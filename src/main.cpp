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
				fajLocator.status = BEFORE_DUNE_STAGE_3;
				break;

			case BEFORE_DUNE_STAGE_1:
				fajLocator.locateBeforeDuneStage1();
				break;

			case BEFORE_DUNE_STAGE_2:
				fajLocator.locateBeforeDuneStage2();
				break;

			case BEFORE_DUNE_STAGE_3:
				fajLocator.locateBeforeDuneStage3();
				break;

			default:
				break;
		}
	}

	return EXIT_SUCCESS;
}