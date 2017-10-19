/*
 * schunk_ft_sensor.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */

#include <schunk_ft_sensor/schunk_ft.h>

bool SchunkFTSensorInterface::initialize()
{
	if(initParams()
			&& initDriver()
			&& contactSensor()
			&& requestMatrix()
			&& requestBias()
			&& initRos())
	{
		ROS_INFO_STREAM("Sensor was successfully initialized.");
		return true;
	}else{
		return false;
	}
}

bool SchunkFTSensorInterface::finalize()
{
	if(!driver_initialized) return true;

	driver->shutdown();
	driver.reset();
	return true;
}

bool SchunkFTSensorInterface::initParams()
{
	if(!nh.getParam(ros::this_node::getName() + "/can_device", can_device))
			return err("Set \"can_device\" parameter.");

	if(!nh.getParam(ros::this_node::getName() + "/can_node_id", node_id)
			|| node_id < 0 || node_id > 127)
		return err("Set valid \"can_node_id\" parameter (0...127).");

	nh.getParam(ros::this_node::getName() + "/debug", debug);
	nh.getParam(ros::this_node::getName() + "/publish_rate", publish_rate);
	nh.getParam(ros::this_node::getName() + "/silence_limit", silence_limit);

	return true;
}

bool SchunkFTSensorInterface::initDriver()
{
	if(driver_initialized) return true;

	driver = boost::make_shared<can::ThreadedSocketCANInterface> ();
	if (!driver->init(can_device, 0))  // initialize device at can_device, 0 for no loopback.
	{
		return err("Failed to initialize can_device at " + can_device);
	}

	frame_listener = driver->createMsgListener(can::CommInterface::FrameDelegate(this, &SchunkFTSensorInterface::frameCB));
	state_listener = driver->createStateListener(can::StateInterface::StateDelegate(this, &SchunkFTSensorInterface::stateCB));

	ROS_INFO("Successfully connected to %s.", can_device.c_str());

	driver_initialized = true;
	return driver_initialized;
}

bool SchunkFTSensorInterface::contactSensor()
{
	can::Frame f_serial_number_request;
	f_serial_number_request.id = (node_id << 4) + REQ_SERIAL_NUMBER;

	int attempts = 0;

	ros::Rate r(0.333333);

	while(!can_node_contacted && ++attempts < 4 && ros::ok())
	{
		ROS_INFO_STREAM("Attempt to contact can node with id " << node_id << ":  " << attempts);
		driver->send(f_serial_number_request);

		r.sleep();
	}

	if(can_node_contacted)
		ROS_INFO_STREAM("Successfully contacted can node with id " << node_id);
	else
		ROS_ERROR_STREAM("Failed to contact can node with id " << node_id);

	return can_node_contacted;
}

bool SchunkFTSensorInterface::requestMatrix()
{
	return true;
}

bool SchunkFTSensorInterface::requestBias()
{
	return true;
}

bool SchunkFTSensorInterface::initRos()
{
	sensorTopic = nh.advertise<geometry_msgs::Wrench>(ros::this_node::getName() + "/sensor_data", 10);

	f_data_request.id = (node_id << 4) + REQ_SG_DATA;
	f_data_request.dlc = 0;

	dataRequestTimer = nh.createTimer(ros::Duration(1 / publish_rate), &SchunkFTSensorInterface::dataRequestTimerCB, this);
	silenceTimer = nh.createTimer(ros::Duration(silence_limit), &SchunkFTSensorInterface::silenceTimerCB, this);

	return true;
}

bool SchunkFTSensorInterface::err(std::string mes)
{
	ROS_ERROR_STREAM(ros::this_node::getName() << " --> " << mes);
	return false;
}

bool SchunkFTSensorInterface::failure(std::string mes)
{
	dataRequestTimer.stop();
	silenceTimer.stop();

	// TODO handle data acquisition failure here

	return err("DATA ACQUISITION FAILURE: " + mes);
}
