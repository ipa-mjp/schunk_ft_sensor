/*
 * schunk_ft_init.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */
#include <schunk_ft_sensor/schunk_ft.h>

void SchunkFTSensorInterface::extractRawSGData(const can::Frame &f)
{
	switch(getType(f))
	{
	case SG_Data_Packet_1:
		sg[0] = ((short)f.data[2] << 8) | f.data[3];
		sg[2] = ((short)f.data[4] << 8) | f.data[5];
		sg[4] = ((short)f.data[6] << 8) | f.data[7];

		return;

	case SG_Data_Packet_2:
		sg[1] = ((short)f.data[0] << 8) | f.data[1];
		sg[3] = ((short)f.data[2] << 8) | f.data[3];
		sg[5] = ((short)f.data[4] << 8) | f.data[5];

		break;

	default:
		return;
	}

	// check for data validity
	for(int i = 0; i < 6; i++)
	{
		if(sg[i] == -32768 || sg[i] == 32767) // GAUGE IS SATURATED, DATA IS UNUSABLE (refer to the documentation)
		{
			failure("Invalid data read from sensor. Gauge is saturated. Please refer to the user manual for more information.");
			return;
		}
	}

	sg_data_received = true;

	biasRawSGData();
}

void SchunkFTSensorInterface::extractMatrix(const can::Frame &f)
{
	int current_row;
	for(current_row = 0; current_row < 6; current_row++){
		if(!matrix_data_obtained[current_row]) break;
	}

	float first, second;
	unsigned char bytes[4];

	bytes[0] = f.data[3];
	bytes[1] = f.data[2];
	bytes[2] = f.data[1];
	bytes[3] = f.data[0];

	memcpy(&first, bytes, 4);

	bytes[0] = f.data[7];
	bytes[1] = f.data[6];
	bytes[2] = f.data[5];
	bytes[3] = f.data[4];

	memcpy(&second, bytes, 4);

	//ROS_INFO_STREAM(" ========================== " << first << "  " << second);

	switch(getType(f))
	{
	case Matrix_Packet_1:
		matrix[current_row][0] = first;
		matrix[current_row][1] = second;
		return;
	case Matrix_Packet_2:
		matrix[current_row][2] = first;
		matrix[current_row][3] = second;
		return;
	case Matrix_Packet_3:
		matrix[current_row][4] = first;
		matrix[current_row][5] = second;
		break;
	default:
		return;
	}

	matrix_data_obtained[current_row] = true;
}

void SchunkFTSensorInterface::biasRawSGData()
{
	// if biasing values are not obtained yet => use current data as bias
	if(!bias_obtained)
	{
		bias[0] = sg[0];
		bias[1] = sg[1];
		bias[2] = sg[2];
		bias[3] = sg[3];
		bias[4] = sg[4];
		bias[5] = sg[5];
		bias_obtained = true;
		return;
	}

	// bias
	sg[0] -= bias[0];
	sg[1] -= bias[1];
	sg[2] -= bias[2];
	sg[3] -= bias[3];
	sg[4] -= bias[4];
	sg[5] -= bias[5];

	convertToFT();
}

void SchunkFTSensorInterface::convertToFT()
{
	// TODO calculate correct values
	geometry_msgs::Wrench msg;
	msg.force.x = sg[0];
	msg.force.y = sg[1];
	msg.force.z = sg[2];
	msg.torque.x = sg[3];
	msg.torque.y = sg[4];
	msg.torque.z = sg[5];

	//ROS_INFO_STREAM(msg);

	sensorTopic.publish(msg);
}



