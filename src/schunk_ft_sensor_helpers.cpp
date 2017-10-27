/*
 /*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2017 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * ROS stack name: schunk_ft_sensor
 * \note
 * ROS package name: schunk_ft_sensor
 *
 * \author
 * Author: Turan Elchuev email: turan.elchuev@ipa.fraunhofer.de
 *
 * \date Date of creation: October, 2017
 *
 * \brief
 *
 * *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *
 ******************************************************************/

#include <schunk_ft_sensor/schunk_ft_sensor.h>

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


