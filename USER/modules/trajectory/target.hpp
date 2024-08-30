/*
 * target.hpp
 *
 *  Created on: 2020年12月2日
 *      Author: 刘成吉
 */

#ifndef MODULES_TRAJECTORY_TARGET_HPP_
#define MODULES_TRAJECTORY_TARGET_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"

#ifdef __cplusplus

#include <Eigen>

using namespace std;
using namespace Eigen;

#define MAX_QUEUE_SIZE 20  //队列中所能容纳的最大目标点数
#define MAX_COMMAND_SIZE 15 //地面站发出的最大目标点数

#define C_WGS84_a 6378137.0        //地球长半轴
#define C_WGS84_b 6356752.314245   //地球短半轴   f=(a-b)/a
#define C_WGS84_f 0.00335281066474748072//地球扁率
#define C_WGS84_e 0.081819790992   //第一偏心率

typedef struct{
	float target[MAX_COMMAND_SIZE][4];    //期望目标点数组，0，1，2为北东地位置，单位m，3为标志位：-1正常，-2不正常
	int	   count;													//尚未被接受进队列的目标点数
}Target_Command;

typedef struct{
	float queue[MAX_QUEUE_SIZE][4];   	    //队列目标点数组
	int   index;                         //指针，指向队列中最后一个有效目标点
	int   cur_targert;				//队列由地面站写入后不变，改变指针位置实现目标点切换。

	double N;
	double LLH[3];
	double ECFF[3];
	double Re2t[3][3];
	double Zero_ECFF[3];
}Target_Queue;


class TARGET
{
private:
	Target_Command target_command;
	Target_Queue   target_queue;


public:
	void Target_Command_Reset(void);       //函数，目标点复位
	void Target_Queue_Reset(void);				 //函数，队列复位
	bool Target_Queue_Push_Target(void);   //目标点压栈
	bool Target_Queue_Pop_Target(float * target_p,float* hover_time);//目标点出栈
	bool Target_Queue_Is_Queue_Empty(void);  //判断队列是否为空

	float        hover_ned[3];

	Target_Buffer  target_buffer;
	gps_msg gps;

//	RC_command_msg rcCommand;
	eskf_msg eskf;
	height_msg height;

//	control_transfer_msg control_data;

};


#endif

#endif /* MODULES_TRAJECTORY_TARGET_HPP_ */
