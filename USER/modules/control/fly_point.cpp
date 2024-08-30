
#include "fly_point.h"


FlyPoint::ArriveStatus FlyPoint::CheckArrive(Data_3D<float> cur_pos_m)
{
	Data_3D<float> derta_pos;

	derta_pos.data.x = fly_point_pos_m.data.x - cur_pos_m.data.x;
	derta_pos.data.y = fly_point_pos_m.data.y - cur_pos_m.data.y;
	derta_pos.data.z = fly_point_pos_m.data.z - cur_pos_m.data.z;

	if(derta_pos.getMagnitude() < 0.1)
		return Arrived;
	else
		return NotArrived;
}


