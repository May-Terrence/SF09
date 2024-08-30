

#pragma once

#include "stdint.h"
#include "math.h"

/**
 * 数据基类
 */
class Data
{
public:
	uint64_t time_us;
};

template <typename DataType>
class Data_1D: public Data
{
public:
	Data_1D(void) {}

	Data_1D(DataType _x)
	{	x = _x; }

	void dataInput(DataType input_x, uint64_t _time_us)
	{
		x = input_x;
		time_us = _time_us;
	}

	DataType x;
};



template <typename DataType>
class Data_2D: public Data
{
public:
	Data_2D(void)
	{data_array = (DataType *)&data.x;}

	Data_2D(DataType _x, DataType _y)
	{
		Data_2D();
		data.x = _x;
		data.y = _y;
	}

	void dataInput(DataType input_x, DataType input_y, uint64_t _time_us)
	{
		data.x = input_x;
		data.y = input_y;
		time_us = _time_us;
	}

	float getMagnitude() const {
        return sqrt((float)data.x*data.x + data.y*data.y);
    }

	struct{
		DataType x;
		DataType y;
	}data;

	DataType *data_array;
};

template <typename DataType>
class Data_3D: public Data
{
public:
	Data_3D(void)
	{data_array = (DataType *)&data.x;}

	Data_3D(DataType _x, DataType _y, DataType _z)
	{
		Data_3D();
		data.x = _x;
		data.y = _y;
		data.z = _z;
	}

	void dataInput(DataType input_x, DataType input_y, DataType input_z, uint64_t _time_us)
	{
		data.x = input_x;
		data.y = input_y;
		data.z = input_z;
		time_us = _time_us;
	}

	float getMagnitude() const {
        return sqrt((float)data.x*data.x + data.y*data.y + data.z*data.z);
    }

	struct{
		DataType x;
		DataType y;
		DataType z;
	}data;

	DataType *data_array;
};

