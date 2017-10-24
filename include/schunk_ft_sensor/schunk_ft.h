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

class SchunkFTSensorInterface
{
	protected:

		bool debug = false;

		ros::NodeHandle nh;
		ros::Timer dataRequestTimer, silenceTimer;
		ros::Publisher sensorTopic;

		boost::shared_ptr<can::DriverInterface> driver;

		int node_id = 128;

		short	sg[6] = {0},
				bias[6] = {0};

		float matrix[6][6];

		double 	publish_rate = 20.0, // Hz
				silence_limit = 3.0; // sec, must be at least 2/publish_rate

		bool driver_initialized = false;

		volatile bool 	can_node_contacted = false,
						matrix_data_obtained[6] = {false},
						bias_obtained = false,
						sg_data_received = true;

		std::string can_device;

		can::Frame f_data_request;

		can::CommInterface::FrameListener::Ptr frame_listener;
		can::StateInterface::StateListener::Ptr state_listener;

		enum message_types: unsigned int
		{
			/*
			 * 2 - digit hex number
			 * 1st digit - opcode
			 * 2ns digit - number of bytes in data (dlc)
			 */
			INVALID = 0xFF,
			Read_SG_Data = 0x00,
			SG_Data_Packet_1 = 0x08,
			SG_Data_Packet_2 = 0x16,
			Read_Matrix = 0x21,
			Matrix_Packet_1 = 0x28,
			Matrix_Packet_2 = 0x38,
			Matrix_Packet_3 = 0x48,
			Read_FT_Serial_Number = 0x50,
			Serial_Number = 0x58,
			Active_Calibration = 0x61,
			Read_Counts_Per_Unit = 0x70,
			Counts_Per_Unit = 0x78,
			Read_Unit_Codes = 0x80,
			Unit_Codes = 0x82,
			Reset = 0xC0
		};

		SchunkFTSensorInterface::message_types getType(const can::Frame &f);
		can::Frame getMessage(SchunkFTSensorInterface::message_types type);
		can::Frame getMessage(SchunkFTSensorInterface::message_types type, unsigned char b);
		can::Frame getMessage(SchunkFTSensorInterface::message_types type, boost::array<unsigned char, 8> data);

		void frameCB(const can::Frame &f);
		void stateCB(const can::State & s);
		void dataRequestTimerCB(const ros::TimerEvent& event);
		void silenceTimerCB(const ros::TimerEvent& event);
		void extractRawSGData(const can::Frame &f);
		void extractMatrix(const can::Frame &f);
		void biasRawSGData();
		void convertToFT();

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
