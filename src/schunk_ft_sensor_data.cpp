/*
 * schunk_ft_init.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */
#include <schunk_ft_sensor/schunk_ft.h>

void SchunkFTSensorInterface::extractRawSGData(const can::Frame &f, bool second)
{
	if(f.dlc == 0) // request frame
	{
		return;
	}

	if(!second) // first part of SG data (byte indexes sg0 = [2,3], sg2 = [4,5], sg4 = [6,7], big endian, 16 bit signed)
	{
		sg[0] = ((short)f.data[2] << 8) | f.data[3];
		sg[2] = ((short)f.data[4] << 8) | f.data[5];
		sg[4] = ((short)f.data[6] << 8) | f.data[7];

		return;
	}
	else // second part of SG data (byte indexes sg1 = [0,1], sg3 = [2,3], sg5 = [4,5], big endian, 16 bit signed)
	{
		sg[1] = ((short)f.data[0] << 8) | f.data[1];
		sg[3] = ((short)f.data[2] << 8) | f.data[3];
		sg[5] = ((short)f.data[4] << 8) | f.data[5];

		//ROS_INFO("second");
		//ROS_INFO_STREAM(f);
	}

	for(int i = 0; i < 6; i++)
	{
		if(sg[i] == -32768 || sg[i] == 32767) // GAUGE IS SATURATED, DATA IS UNUSABLE (refer to the documentation)
		{
			failure("Invalid data read from sensor. Gauge is saturated. Please refer to the user manual for more information.");
			return;
		}
	}

	if(!bias_obtained)
	{

	}

	sg_data_received = true;

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

void::SchunkFTSensorInterface::biasRawSGData()
{

}



