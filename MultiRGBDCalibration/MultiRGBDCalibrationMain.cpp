#include "App/MultiRGBDCalibrationApp.h"

int main(int argc, char* argv[])
{
#if 0
	if (argc < 1)
	{
		printf("Please add path for configuration file\n");
		return 0;
	}
#endif

	MultiRGBDCalibrationApp app;

	//app.loadConfig("E:/Data/MultiRGBDCalibration/example00/config.ini");
	app.loadConfig("E:/Data/MultiRGBDCalibration/realsense00/config.ini");
	app.startMainLoop();

	return 0;
}