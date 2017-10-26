/*
 * schunk_ft_init.cpp
 *
 *  Created on: Oct 17, 2017
 *      Author: root
 */
#include <schunk_ft_sensor/schunk_ft.h>

void SchunkFTSensorInterface::checkCalibration(const can::Frame &f)
{
	calibration_message_received = true;
	calibration_successfully_set = calibration == f.data[0];
}

void SchunkFTSensorInterface::extractFirmwareVersion(const can::Frame &f)
{
	ver.major = f.data[0];
	ver.minor = f.data[1];
	ver.build = ((unsigned short)f.data[2] << 8) | f.data[3];
	ver.received = true;
}

void SchunkFTSensorInterface::extractCountsPerUnit(const can::Frame &f)
{
	CpF = ((unsigned int)f.data[0] << 24) | ((unsigned int)f.data[1] << 16) | ((unsigned int)f.data[2] << 8) | f.data[3];
	CpT = ((unsigned int)f.data[4] << 24) | ((unsigned int)f.data[5] << 16) | ((unsigned int)f.data[6] << 8) | f.data[7];
	counts_per_unit_received = true;
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

void SchunkFTSensorInterface::extractRawSGData(const can::Frame &f)
{
	switch(getType(f))
	{
	case SG_Data_Packet_1:

		status = ((unsigned short)f.data[0] << 8) | f.data[1];

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

	if(!checkStatus()) return;

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

	short data[6];
	for(int i = 0; i < 6; i++)
		data[i] = sg[i];

	averageRawSGData(data);
}

void SchunkFTSensorInterface::averageRawSGData(short *data)
{
	int r;
	if(sample_cnt < AVG_SMPL_CNT)
	{
		for(r = 0; r < 6; r++)
			sample_sum[r] += data[r];
		sample_cnt++;
		return;
	}

	for(r = 0; r < 6; r++)
	{
		data[r] = sample_sum[r] / sample_cnt;
		sample_sum[r] = 0;
	}
	sample_cnt = 0;

	biasRawSGData(data);
}

void SchunkFTSensorInterface::biasRawSGData(short *data)
{
	int r;
	if(!bias_obtained)
	{
		for(r = 0; r < 6; r++)
			bias[r] = data[r];
		bias_obtained = true;
		return;
	}

	// bias the data
	for(r = 0; r < 6; r++)
		data[r] -= bias[r];

	convertToFT(data);
}

void SchunkFTSensorInterface::convertToFT(short *data)
{
	float result[6] = {0};

	for(int r = 0; r < 6; r++)
		for(int c = 0; c < 6; c++)
			result[r] += matrix[r][c] * data[c];

	geometry_msgs::Wrench msg;
	msg.force.x = result[0];
	msg.force.y = result[1];
	msg.force.z = result[2];
	msg.torque.x = result[3];
	msg.torque.y = result[4];
	msg.torque.z = result[5];

	sensorTopic.publish(msg);
}



