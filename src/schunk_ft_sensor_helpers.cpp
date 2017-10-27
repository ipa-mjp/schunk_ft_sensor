/*
 * schunk_ft_sensor_helpers.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: root
 */

#include <schunk_ft_sensor/schunk_ft.h>

bool SchunkFTSensorInterface::err(std::string mes)
{
	ROS_ERROR_STREAM(ros::this_node::getName() << " --> " << mes);
	return false; // always return false
}

bool SchunkFTSensorInterface::failure(std::string mes)
{
	// TODO handle failure here

	std_msgs::String s;
	s.data = mes;
	failureTopic.publish(s);

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


