/*
 * schunk_ft_init.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */
#include <schunk_ft_sensor/schunk_ft.h>

void SchunkFTSensorInterface::frameCB(const can::Frame &f){

	if(NODE_ID(f.id) != node_id){
		return;
	}

    if(f.is_error){
        std::cout << "E " << std::hex << f.id << std::dec; // TODO make a proper ROS ERROR
        return;
    }

    unsigned int opcode = OPCODE(f.id);

    switch(opcode)
    {
    case REQ_SERIAL_NUMBER:
    	if(f.dlc > 0) can_node_contacted = true;
    	break;
    case RESP_SG_DATA_1:
    	extractRawSGData(f, false);
    	break;
    case RESP_SG_DATA_2:
    	extractRawSGData(f, true);
    	break;
    default:
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
