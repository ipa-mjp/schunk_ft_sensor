#include "ros/ros.h"
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <string>
#include <geometry_msgs/Wrench.h>

#include <iostream>

void frameCallback(const can::Frame &f);
void stateCallback(const can::State & s);
void sendDataRequest(const ros::TimerEvent& event);
void processSGData(const can::Frame &f, bool second);

#define REQ_SG_DATA			0x0

#define RESP_SG_DATA_1		0x0
#define RESP_SG_DATA_2		0x1

#define OPCODE(mes_id)		((mes_id) & 0x0000000F)
#define NODE_ID(mes_id)		(((mes_id) & 0x000007F0) >> 4)


boost::shared_ptr<can::DriverInterface> receiver;

ros::Timer requestTimer;

ros::Publisher sensorTopic;

double reqFreq = 20;

unsigned int node_id;

short sg[6] = {};

can::Frame f_data_request;

void frameCallback(const can::Frame &f){
    
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
    case RESP_SG_DATA_1:
    	processSGData(f, false);
    	break;
    case RESP_SG_DATA_2:
    	processSGData(f, true);
    	break;
    default:
    	break;
    }
}

void processSGData(const can::Frame &f, bool second)
{
	if(f.dlc == 0) // request frame
	{
		//ROS_INFO("request");
		//ROS_INFO_STREAM(f);
		return;
	}

	if(!second) // first part of SG data (byte indexes sg0 = [2,3], sg2 = [4,5], sg4 = [6,7], big endian)
	{
		sg[0] = ((short)f.data[2] << 8) | f.data[3];
		sg[2] = ((short)f.data[4] << 8) | f.data[5];
		sg[4] = ((short)f.data[6] << 8) | f.data[7];

		//ROS_INFO("first");
		//ROS_INFO_STREAM(f);
		return;
	}
	else // second part of SG data (byte indexes sg1 = [0,1], sg3 = [2,3], sg5 = [4,5], big endian)
	{
		sg[1] = ((short)f.data[0] << 8) | f.data[1];
		sg[3] = ((short)f.data[2] << 8) | f.data[3];
		sg[5] = ((short)f.data[4] << 8) | f.data[5];

		//ROS_INFO("second");
		//ROS_INFO_STREAM(f);
	}

	for(int i = 0; i < 6; i++)
	{
		if(sg[i] == -32768 || sg[i] == 32767) // GAGE IS SATURATED, DATA IS UNUSABLE (refer to the documentation)
		{
			return; // TODO make a proper ROS ERROR
		}
	}

	geometry_msgs::Wrench msg;

	// TODO calculate correct values
	msg.force.x = sg[0];
	msg.force.y = sg[1];
	msg.force.z = sg[2];
	msg.torque.x = sg[3];
	msg.torque.y = sg[4];
	msg.torque.z = sg[5];

	//ROS_INFO_STREAM(msg);

	sensorTopic.publish(msg);
}

void stateCallback(const can::State & s)
{
  std::string err;
  receiver->translateError(s.internal_error, err);
  if (!s.internal_error)
  {
    ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
  }
  else
  {
    ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
  }
}

void sendDataRequest(const ros::TimerEvent& event)
{
	receiver->send(f_data_request);
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "schunk_ft_driver");
    ros::NodeHandle nh;

	node_id = 32;
	int message_base_id = node_id * 16;

	std::string can_device = "can0";

	receiver = boost::make_shared<can::ThreadedSocketCANInterface> ();

	if (!receiver->init(can_device, 0))  // initialize device at can_device, 0 for no loopback.
	{
		ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
		return 1;
	}
	else
	{
		ROS_INFO("Successfully connected to %s.", can_device.c_str());
	}

	can::CommInterface::FrameListener::Ptr frame_printer = receiver->createMsgListener(frameCallback);
	can::StateInterface::StateListener::Ptr state_printer = receiver->createStateListener(stateCallback);

	sensorTopic = nh.advertise<geometry_msgs::Wrench>("sg_data", 10);

	f_data_request.id = message_base_id + REQ_SG_DATA;
	f_data_request.dlc = 0;
	requestTimer = nh.createTimer(ros::Duration(1 / reqFreq), sendDataRequest);

	ros::spin();

	receiver->shutdown();
	receiver.reset();

	ros::waitForShutdown();
    
}
