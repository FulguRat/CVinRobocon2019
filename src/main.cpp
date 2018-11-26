#include <iostream>
#include <librealsense2/rs.hpp>
#include "act_d435.h"

using namespace std;

int main(int argc, char* argv[])
{
	ActD435 actD435;

	actD435.init();

	while (true)
	{
		actD435.update();

		if (actD435.isStoped()) { break; }
	}

	return EXIT_SUCCESS;
}