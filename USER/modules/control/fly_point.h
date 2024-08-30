
#pragma once

#include "userlib/data_struct.h"
#include "stddef.h"


class FlyPoint
{
public:
	FlyPoint() {}
	typedef enum __ArriveStatus
	{
		NotArrived = 0,
		Arrived,
	}ArriveStatus;

	ArriveStatus CheckArrive(Data_3D<float> cur_pos_m);

	void GetPosition(Data_3D<float> *position)
	{*position = fly_point_pos_m;}

	void SetPosition(Data_3D<float> *position)
	{fly_point_pos_m = *position;}

	void GetYaw(float *_yaw)
	{*_yaw = fly_yaw;}

	void SetYaw(float *_yaw)
	{fly_yaw = *_yaw;}

	void GetSpeed(float *_speed)
	{*_speed = fly_speed_mps;}

	void SetSpeed(float *_speed)
	{fly_speed_mps = *_speed;}

	void GetStayTime(uint16_t *_staytime)
	{*_staytime = fly_staytime;}

	void SetStayTime(uint16_t *_staytime)
	{fly_staytime = *_staytime;}

	~FlyPoint() {}

	int8_t enable {0};
	bool planned_flag;

private:
	uint16_t		fly_staytime;
	float			fly_yaw;
//	Data_3D<float>	fly_speed_mps;
	float			fly_speed_mps;
	Data_3D<float>	fly_point_pos_m;
};


