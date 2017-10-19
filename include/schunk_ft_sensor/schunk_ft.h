/*
 * schunk_ft_sensor.h
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */
#ifndef SCHUNK_FT_SENSOR_INCLUDE_SCHUNK_FT_SENSOR_H_
#define SCHUNK_FT_SENSOR_INCLUDE_SCHUNK_FT_SENSOR_H_

#include "ros/ros.h"
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <geometry_msgs/Wrench.h>

// CAN REQUEST OPCODES
#define REQ_SG_DATA			0x0
#define REQ_SERIAL_NUMBER	0x5

// CAN RESPONCE OPCODES
#define RESP_SG_DATA_1		0x0
#define RESP_SG_DATA_2		0x1

#define OPCODE(mes_id)		((mes_id) & 0x0000000F)
#define NODE_ID(mes_id)		(((mes_id) & 0x000007F0) >> 4)

// MESSAGES

class SchunkFTSensorInterface
{
	protected:

		bool debug = false;

		ros::NodeHandle nh;
		ros::Timer dataRequestTimer, silenceTimer;
		ros::Publisher sensorTopic;

		boost::shared_ptr<can::DriverInterface> driver;

		int node_id = 128;

		short sg[6] = {};

		double publish_rate = 20.0, // Hz
				silence_limit = 3.0; // sec, must be at least 5/publish_rate

		bool driver_initialized = false;

		volatile bool can_node_contacted = false,
				matrix_obtained = false,
				bias_obtained = false,
				sg_data_received = true;

		std::string can_device;

		can::Frame f_data_request;

		can::CommInterface::FrameListener::Ptr frame_listener;
		can::StateInterface::StateListener::Ptr state_listener;

		void frameCB(const can::Frame &f);
		void stateCB(const can::State & s);
		void dataRequestTimerCB(const ros::TimerEvent& event);
		void silenceTimerCB(const ros::TimerEvent& event);
		void extractRawSGData(const can::Frame &f, bool second);
		void biasRawSGData();

		bool initParams();
		bool initDriver();
		bool contactSensor();
		bool requestMatrix();
		bool requestBias();
		bool initRos();

		bool failure(std::string mes);

		bool err(std::string mes);
	public:

		SchunkFTSensorInterface()
		{

		}

		~SchunkFTSensorInterface()
		{
			finalize();
		}

		bool initialize();
		bool finalize();

};

#endif /* SCHUNK_FT_SENSOR_INCLUDE_SCHUNK_FT_SENSOR_H_ */
