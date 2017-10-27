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

void SchunkFTSensorInterface::frameCB(const can::Frame &f){

    if(f.is_error){
        std::cout << "E " << std::hex << f.id << std::dec; // TODO make a proper ROS ERROR
        return;
    }

    switch(getType(f))
    {
    case Active_Calibration:
    	checkCalibration(f);
    	break;
    case Firmware_Version:
    	extractFirmwareVersion(f);
    	break;
    case Counts_Per_Unit:
    	extractCountsPerUnit(f);
    	break;
    case SG_Data_Packet_1:
    case SG_Data_Packet_2:
    	extractRawSGData(f);
    	break;
    case Matrix_Packet_1:
    case Matrix_Packet_2:
    case Matrix_Packet_3:
    	extractMatrix(f);
    	break;
    }
}

void SchunkFTSensorInterface::stateCB(const can::State & s)
{
  std::string err;
  driver->translateError(s.internal_error, err);
  if (!s.internal_error)
  {
    ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
  }
  else
  {
    ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
  }
}
