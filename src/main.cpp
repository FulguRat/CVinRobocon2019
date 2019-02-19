#include <iostream>
#include <librealsense2/rs.hpp>
#include "act_d435.h"
#include "robot_locator.h"
#include <chrono>

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

			case PASSING_GRASSLAND_STAGE_1:
				fajLocator.locatePassingGrasslandStage1();
				break;

			case PASSING_GRASSLAND_STAGE_2:
				fajLocator.locatePassingGrasslandStage2();
				break;

			case CLIMBING_MOUNTAIN:
				fajLocator.locateClimbingMountain();
				break;
			default:
				break;
		}
		//steady_clock::time_point t2 = steady_clock::now();
		//duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

		//printf("%f ms\n", time_span.count() * 1000);
	}

	return EXIT_SUCCESS;
}