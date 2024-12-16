/*
 * control_step.hpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */

#ifndef CONTROL_STEP_HPP_
#define CONTROL_STEP_HPP_


#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "control/control.hpp"
#include "trajectory/trajectory.hpp"
#include "hardware.h"


#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif


#ifdef __cplusplus


class CONTROL_STEP:public CONTROL
{
public:
	CONTROL_STEP(){}
	~CONTROL_STEP(){}

	void Control_Step(void);
	void Control_Step2(void);
	void PID_Para_Update(void);
	void Tranfer_Data_Updata(void);

	uint32_t startTimer;
	uint32_t stopTimer;
	uint32_t  startTimerLast;
	uint32_t  executionTime_us;
	uint32_t cycleTime_us;

	uint32_t TakeOffTimer;
	bool return_to_base;

	bool flag1{false};
	bool flag2{false};


private:
//	RC_command_msg rcCommand;
	gps_msg gps;
	laserFlow_msg laserFlow;
	CLAW_msg claw_msg;

//	float roll ,pitch,yaw;
//	int time;

};


#endif


#endif /* CONTROL_STEP_HPP_ */
