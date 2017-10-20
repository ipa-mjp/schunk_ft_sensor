/*
 * schunk_ft_init.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */
#include <schunk_ft_sensor/schunk_ft.h>

void SchunkFTSensorInterface::frameCB(const can::Frame &f){

    if(f.is_error){
        std::cout << "E " << std::hex << f.id << std::dec; // TODO make a proper ROS ERROR
        return;
    }

    switch(getType(f))
    {
    case Serial_Number:
    	can_node_contacted = true;
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

void SchunkFTSensorInterface::dataRequestTimerCB(const ros::TimerEvent& event)
{
	driver->send(f_data_request);
}

void SchunkFTSensorInterface::silenceTimerCB(const ros::TimerEvent& event)
{
	if(!sg_data_received)
	{
		failure("Silence limit exceeded.");
	}

	sg_data_received = false;
}
