#include <iostream>
#include <librealsense2/rs.hpp>
#include "act_d435.h"
#include "robot_locator.h"
#include "kalman_filter.h"

using namespace std;

int main(int argc, char* argv[])
{
	ActD435			fajD435;
	RobotLocator 	fajLocator;
	kalman_filter   distancefilter;
	kalman_filter	anglefilter;

	double	distance;
	//int dynamParams, int measureParams, Mat transitionMatrix,double processNoiseCov, double measurementNoiseCov, double errorCovPost
	distancefilter.initKalmanFilter(2, 1, (cv::Mat_<float>(2, 2) << 1, 0.3, 0, 1), (cv::Mat_<float>(1, 2) << 1, 0), 1e-5, 1e-2, 0.1);

	fajD435.init();
	fajLocator.init(fajD435);
	fajLocator.status = BEFORE_DUNE_STAGE_2;;

	while (!fajLocator.isStoped())
	{
		fajLocator.updateCloud();

		if (/*fajLocator.orbmatch()*/true)
		{
			fajLocator.preProcess();

			switch (fajLocator.status)
			{
			case STARTUP_INITIAL:
				fajLocator.status = BEFORE_GRASSLAND_STAGE_2;
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

			default:
				break;
			}
		}

		distance=distancefilter.predictAndCorrect(Mat_<float>(1, 1)<<fajLocator.diatancemeasurement).at<float>(0,0);

		cout << distance << endl;
	}

	return EXIT_SUCCESS;
}