#include <iostream>
#include <librealsense2/rs.hpp>
#include "act_d435.h"
#include "robot_locator.h"


using namespace std;
using namespace std::chrono;

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
		//steady_clock::time_point t1 = steady_clock::now();
		switch (fajLocator.status)
		{
			case STARTUP_INITIAL:
				fajLocator.status = CLIMBING_MOUNTAIN;
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

			case PASSING_DUNE:
				fajLocator.locatePassingDune();
				break;

			case BEFORE_GRASSLAND_STAGE_1:
				fajLocator.locateBeforeGrasslandStage1();
				break;

			case BEFORE_GRASSLAND_STAGE_2:
				fajLocator.locateBeforeGrasslandStage2();
				break;

			case UNDER_MOUNTAIN:
				fajLocator.locateUnderMountain();
				break;
			case BONE_RECOGNITION:
			{

			}
				break;
			case CLIMBING_MOUNTAIN:
				fajLocator.locateClimbingMountain();
				break;
			case REACH_MOUNTAIN:
				fajLocator.locateReachingMountain();
				break;
			default:
				break;
		}
	}

	return EXIT_SUCCESS;
}