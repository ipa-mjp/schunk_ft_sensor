/*
 * schunk_ft_sensor.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */

#include <schunk_ft_sensor/schunk_ft.h>

bool SchunkFTSensorInterface::initialize()
{
	// ORDER: initParams -> initDriver -> contactSensor -> requestMatrix -> requestBias -> initRos
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

	f_data_request = getMessage(Read_SG_Data);

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
	int attempts = 0;

	ros::Rate r(0.5);

	while(!can_node_contacted && ++attempts < 4 && ros::ok())
	{
		ROS_INFO_STREAM("Attempt to contact CAN node with id " << node_id << ":  " << attempts);
		driver->send(getMessage(Read_FT_Serial_Number));

		r.sleep();
	}

	if(can_node_contacted)
		ROS_INFO_STREAM("Successfully contacted CAN node with id " << node_id);
	else
		ROS_ERROR_STREAM("Failed to contact CAN node with id " << node_id);

	return can_node_contacted;
}

bool SchunkFTSensorInterface::requestMatrix()
{
	unsigned char axis_row;
	ros::Rate r(6);

	for(axis_row = 0; axis_row < 6; axis_row++)
	{
		ROS_INFO_STREAM("Reading calibration matrix row " << (int)axis_row);

		driver->send(getMessage(Read_Matrix, axis_row));
		r.sleep();

		if(!matrix_data_obtained[axis_row])
			return err("Reading calibration matrix failed.");
	}

	ROS_INFO_STREAM("Calibration matrix was successfully read.");

	return true;
}

bool SchunkFTSensorInterface::requestBias()
{
	ROS_INFO("Requesting biasing values...");

	if(bias_obtained) // if bias was previously obtained => request data timer is already running and data is being requested regularly
	{
		bias_obtained = false;
	}
	else // send data request
	{
		driver->send(f_data_request);
	}

	ros::Rate r(publish_rate / 3);
	r.sleep();

	if(bias_obtained)
		ROS_INFO_STREAM("Successfully received biasing values.");
	else
		return err("Failed to receive biasing values.");

	return bias_obtained;
}

bool SchunkFTSensorInterface::initRos()
{
	sensorTopic = nh.advertise<geometry_msgs::Wrench>(ros::this_node::getName() + "/sensor_data", 1);

	dataRequestTimer = nh.createTimer(ros::Duration(1 / publish_rate), &SchunkFTSensorInterface::dataRequestTimerCB, this);
	silenceTimer = nh.createTimer(ros::Duration(silence_limit), &SchunkFTSensorInterface::silenceTimerCB, this);

	return true;
}

bool SchunkFTSensorInterface::err(std::string mes)
{
	ROS_ERROR_STREAM(ros::this_node::getName() << " --> " << mes);
	return false; // always return false
}

bool SchunkFTSensorInterface::failure(std::string mes)
{
	dataRequestTimer.stop();
	silenceTimer.stop();

	// TODO handle data acquisition failure here

	return err("DATA ACQUISITION FAILURE: " + mes);
}

can::Frame SchunkFTSensorInterface::getMessage(SchunkFTSensorInterface::message_types type)
{
	boost::array<unsigned char, 8> data;
	return getMessage(type, data);
}

can::Frame SchunkFTSensorInterface::getMessage(SchunkFTSensorInterface::message_types type, unsigned char b)
{
	boost::array<unsigned char, 8> data;
	data[0] = b;
	return getMessage(type, data);
}

can::Frame SchunkFTSensorInterface::getMessage(SchunkFTSensorInterface::message_types type, boost::array<unsigned char, 8> data)
{
	can::Frame f;
	f.id = (node_id << 4) + (type >> 4);
	f.dlc = type & 0x0000000F;
	f.data = data;
	return f;
}

SchunkFTSensorInterface::message_types SchunkFTSensorInterface::getType(const can::Frame &f)
{
	if(((f.id) & 0x000007F0) >> 4 != node_id)
	{
		return INVALID;
	}
	else
	{
		return message_types(((f.id & 0x0000000F) << 4) + f.dlc);
	}
}
