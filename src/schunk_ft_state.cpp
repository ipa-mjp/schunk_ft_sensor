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
			&& setCalibration()
			&& requestCountsPerUnits()
			&& initRos())
	{

		ROS_INFO_STREAM("Sensor was successfully initialized.");
		runSensor();
		return true;
	}

	return false;
}

bool SchunkFTSensorInterface::finalize()
{
	stopSensor();

	if(!driver_initialized) return true;

	driver->shutdown();
	driver.reset();
	driver_initialized = false;
	return true;
}

bool SchunkFTSensorInterface::initParams()
{
	if(!nh.getParam(ros::this_node::getName() + "/can_device", can_device))
			return err("Set \"can_device\" parameter.");

	if(!nh.getParam(ros::this_node::getName() + "/can_node_id", node_id)
			|| node_id < 0 || node_id > 127)
		return err("Set valid \"can_node_id\" parameter (0...127).");

	int calibr = 0;
	nh.getParam(ros::this_node::getName() + "/calibration", calibr);
	if(calibr > 15 || calibr < 0)
		return err("Set valid \"calibration\" parameter (0...15).");
	calibration = (unsigned char)calibr;

	nh.getParam(ros::this_node::getName() + "/debug", debug);
	nh.getParam(ros::this_node::getName() + "/silence_limit", silence_limit);

	f_data_request = makeFrame(Read_SG_Data);
	for(int i = 0; i < 6; i++) sample_sum[i] = 0;
	sample_cnt = 0;

	sg_data_received = true;

	resetBias();

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

bool SchunkFTSensorInterface::setCalibration()
{
	calibration_message_received = false;
	calibration_successfully_set = false;

	ros::Rate r(6);
	ROS_INFO_STREAM("Setting calibration to " << (int)calibration);

	driver->send(makeFrame(Active_Calibration, calibration));
	r.sleep();

	if(!calibration_message_received)
	{
		ROS_ERROR_STREAM("Failed to receive response from node with id " << node_id);
		return false;
	}
	else if(!calibration_successfully_set)
	{
		ROS_ERROR_STREAM("Failed to set calibration to value " << calibration);
		return false;
	}

	ROS_INFO_STREAM("Calibration was successfully set to " << (int)calibration);

	return requestMatrix();
}


bool SchunkFTSensorInterface::requestFirmwareVersion()
{
	ver.received = false;
	ros::Rate r(4);
	ROS_INFO("Requesting Firmware version.");
	driver->send(makeFrame(Read_Firmware_Version));
	r.sleep();
	if(ver.received)
	{
		ROS_INFO_STREAM("Firmware version was successfully read: " << ver.getVersionStr());
	}
	else
	{
		ROS_WARN_STREAM("Failed to read Firmware version.");
	}
	return true;
}

bool SchunkFTSensorInterface::requestCountsPerUnits()
{
	requestFirmwareVersion();

	counts_per_unit_received = false;
	if(ver.received && ver.standardCpTCpF())
	{
		ROS_WARN_STREAM("With firmware version below 3.7 standard values for Counts per Force and Torque units will be used.");
	}
	else
	{
		ros::Rate r(4);
		ROS_INFO("Requesting counts per Force and Torque.");

		driver->send(makeFrame(Read_Counts_Per_Unit));
		r.sleep();
		if(counts_per_unit_received)
			ROS_INFO_STREAM("Counts per unit were successfully read (CpF: " << (int)CpF << ", CpT: " << (int)CpT << ").");
		else
			ROS_WARN_STREAM("Failed to read counts per unit. Standard values for Counts per Force and Torque units will be used (CpF: " << (int)CpF << ", CpT: " << (int)CpT << ").");
	}

	int i, j;
	for(i = 0; i < 3; i++)
		for(j = 0; j < 6; j++)
			matrix[i][j] /= CpF;

	for(i = 3; i < 6; i++)
		for(j = 0; j < 6; j++)
			matrix[i][j] /= CpT;

	return true;
}

bool SchunkFTSensorInterface::requestMatrix()
{
	for(int i = 0; i < 6; i++) matrix_data_obtained[i] = false;

	unsigned char axis_row;
	ros::Rate r(6);

	for(axis_row = 0; axis_row < 6; axis_row++)
	{
		ROS_INFO_STREAM("Reading calibration matrix row " << (int)axis_row);

		driver->send(makeFrame(Read_Matrix, axis_row));
		r.sleep();

		if(!matrix_data_obtained[axis_row])
			return err("Reading calibration matrix failed.");
	}

	ROS_INFO_STREAM("Calibration matrix was successfully read.");

	return true;
}

bool SchunkFTSensorInterface::initRos()
{
	sensorTopic = nh.advertise<geometry_msgs::Wrench>(ros::this_node::getName() + "/sensor_data", 1);
	return true;
}

void SchunkFTSensorInterface::requestSGDataThread()
{
	while(ros::ok() && sensor_running)
	{
		if(!sg_data_received)
		{
			if(ros::Time::now().toSec() - sg_data_request_timstamp > silence_limit)
			{
				failure("Silence limit exceeded.");
				return;
			}
			else
			{
				continue;
			}
		}
		sg_data_received = false;
		driver->send(f_data_request);
		sg_data_request_timstamp = ros::Time::now().toSec();
	}
}

void SchunkFTSensorInterface::resetBias()
{
	bias_obtained = false;
}

void SchunkFTSensorInterface::runSensor()
{
	if(sensor_running) return;
	sensor_running = true;
	boost::thread t(boost::bind(&SchunkFTSensorInterface::requestSGDataThread, this));
	ROS_INFO_STREAM("Data transfer started.");
}

void SchunkFTSensorInterface::stopSensor()
{
	sensor_running = false;
	ROS_INFO_STREAM("Data transfer stopped.");
}

bool SchunkFTSensorInterface::err(std::string mes)
{
	ROS_ERROR_STREAM(ros::this_node::getName() << " --> " << mes);
	return false; // always return false
}

bool SchunkFTSensorInterface::failure(std::string mes)
{
	// TODO handle failure here

	finalize();
	return err("FAILURE: " + mes);
}

can::Frame SchunkFTSensorInterface::makeFrame(message_types type)
{
	boost::array<unsigned char, 8> data;
	return makeFrame(type, data);
}

can::Frame SchunkFTSensorInterface::makeFrame(message_types type, unsigned char b)
{
	boost::array<unsigned char, 8> data;
	data[0] = b;
	return makeFrame(type, data);
}

can::Frame SchunkFTSensorInterface::makeFrame(message_types type, boost::array<unsigned char, 8> data)
{
	can::Frame f;
	f.id = (node_id << 4) + (type >> 4);
	f.dlc = type & 0x0000000F;
	f.data = data;
	return f;
}

message_types SchunkFTSensorInterface::getType(const can::Frame &f)
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

bool SchunkFTSensorInterface::checkStatus()
{
	//if(!(status & ANY)) return true;
	bool critical = false;

	if(status & Watchdog_Reset){
		failure("Watchdog reset.");
	}
	if(status & DAC_ADC_Too_High){
		failure("DAC/ADC check result too high.");
		critical = true;
	}
	if(status & DAC_ADC_Too_Low){
		failure("DAC/ADC check result too low.");
		critical = true;
	}
	if(status & Artificial_Ground_Out_of_Range){
		failure("Artificial analog ground out of range.");
		critical = true;
	}
	if(status & Power_Supply_Too_High){
		failure("Power supply too high.");
		critical = true;
	}
	if(status & Power_Supply_Too_Low){
		failure("Power supply too low.");
		critical = true;
	}
	if(status & Bad_Active_Calibration){
		failure("Bad active calibration. Select a valid calibration slot.");
		critical = true;
	}
	if(status & EEPROM_Failure){
		failure("EEPROM failure.");
		critical = true;
	}
	if(status & Config_Invalid){
		failure("Configuration Invalid");
	}
	if(status & Sensor_Temp_Too_High){
		failure("Sensor temperature too high.");
		critical = true;
	}
	if(status & Sensor_Temp_Too_Low){
		failure("Sensor temperature too low.");
		critical = true;
	}
	if(status & CANbus_Error){
		failure("CAN bus error.");
	}

	return !critical;
}
