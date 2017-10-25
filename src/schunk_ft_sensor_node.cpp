#include <schunk_ft_sensor/schunk_ft.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, ros::this_node::getName());

	SchunkFTSensorInterface interface;
	if(!interface.initialize())
	{
		interface.finalize(); // to make sure that the driver is shut down in case if it was initialized
		return 1;
	}

	ros::spin();

	interface.finalize();

	ros::waitForShutdown();

	return 0;
}
