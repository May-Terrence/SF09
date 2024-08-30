/*
 * target.cpp
 *
 *  Created on: 2020年12月2日
 *      Author: 刘成吉
 */

#include "target.hpp"

//TARGET target;
void TARGET::Target_Command_Reset(void)
{
	xQueuePeek(queueTargetBuffer, &target_buffer, 0);
	for(int i =0 ; i<= MAX_COMMAND_SIZE; ++i)
	{
	 for(int j =0 ; j!=3 ; ++j)
	 {
		target_buffer.TargetPointList[i][j] = 0.0;
		target_buffer.TargetPointList[i][3] = -1;

		target_buffer.hovertime[i] = 0;
	 }

//	 target_command.target[i][j] = 0.0;
//	 target_command.target[i][3] = -1;
	}
	target_buffer.cur_targert = -1;
	target_buffer.receive_flag = 0;
	target_buffer.index = -1;
	target_buffer.Last_index = -1;
	xQueueOverwrite(queueTargetBuffer,&target_buffer);
}

void TARGET::Target_Queue_Reset(void)
{
	xQueuePeek(queueGps,&gps,0);
//	target_queue.index = 0;
	for(int i =0 ; i!=MAX_QUEUE_SIZE; ++i)
	{
		for(int j =0 ; j!=3 ; ++j)
		target_queue.queue[i][j] =0;
		target_queue.queue[i][3] = -1 ;
	}


	target_queue.LLH[0] = gps.lat*D2R;
	target_queue.LLH[1] = gps.lng*D2R;
	target_queue.LLH[2] = gps.alti;

	double LLH_Init[3];
	LLH_Init[0] = gps.Zero_LLH[0]*D2R_D;
	LLH_Init[1] = gps.Zero_LLH[1]*D2R_D;
	LLH_Init[2] = gps.Zero_LLH[2];

	double N_Init = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(LLH_Init[0])));		//地球椭圆体法线长度
	target_queue.Zero_ECFF[0] = (N_Init + gps.Zero_LLH[2]) * cos(LLH_Init[0]) * cos(LLH_Init[1]);
	target_queue.Zero_ECFF[1] = (N_Init + gps.Zero_LLH[2]) * cos(LLH_Init[0]) * sin(LLH_Init[1]);
	target_queue.Zero_ECFF[2] = (N_Init * (1-SQR(C_WGS84_e)) +gps.Zero_LLH[2]) * sin(LLH_Init[0]);

	target_queue.N = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(target_queue.LLH[0])));
	target_queue.ECFF[0]= (target_queue.N + target_queue.LLH[2])
					* cos(target_queue.LLH[0]) * cos(target_queue.LLH[1]);
	target_queue.ECFF[1]= (target_queue.N + target_queue.LLH[2])
					* cos(target_queue.LLH[0]) * sin(target_queue.LLH[1]);
	target_queue.ECFF[2]= (target_queue.N*(1-SQR(C_WGS84_e)) +target_queue.LLH[2])
					* sin(target_queue.LLH[0]);


	target_queue.ECFF[0] -= target_queue.Zero_ECFF[0];
	target_queue.ECFF[1] -= target_queue.Zero_ECFF[1];
	target_queue.ECFF[2] -= target_queue.Zero_ECFF[2];

	double clat=cos(LLH_Init[0]),slat=sin(LLH_Init[0]),clng=cos(LLH_Init[1]),slng=sin(LLH_Init[1]);
	target_queue.Re2t[0][0] = -slat*clng;
	target_queue.Re2t[0][1] = -slat*slng;
	target_queue.Re2t[0][2] =  clat;
	target_queue.Re2t[1][0] = -slng;
	target_queue.Re2t[1][1] =  clng;
	target_queue.Re2t[1][2] =  0.0;
	target_queue.Re2t[2][0] = -clat*clng;
	target_queue.Re2t[2][1] = -clat*slng;
	target_queue.Re2t[2][2] = -slat;
}

bool TARGET::Target_Queue_Push_Target(void)
{
	bool brake_flag = true;

//	xQueuePeek(queueRCCommand, &rcCommand, 0);//用轨迹规划实现一键起降
//	if(rcCommand.OneKeyTakeoff && rcCommand.Frist_Entry_TakeOff == 1)//用轨迹规划实现一键起降
//	{
//		xQueuePeek(queueESKF,&eskf,0);
//		target_queue.queue[0][0] = eskf.Pos[0];
//		target_queue.queue[0][1] = eskf.Pos[1];
//		target_queue.queue[0][2] = eskf.Pos[2]-1;
//		target_queue.index ++ ;
//
//		hover_ned[0] = target_queue.queue[0][0];
//		hover_ned[1] = target_queue.queue[0][1];
//		hover_ned[2] = target_queue.queue[0][2];
//
//		rcCommand.Frist_Entry_TakeOff = 0;
//
//		xQueueOverwrite(queueRCCommand,&rcCommand);
//	}

//	for( int i =0 ; target_command.count >0 && target_queue.index != MAX_QUEUE_SIZE; ++i)
//	for( int i =0 ; target_buffer.count >0 && target_queue.index <= MAX_QUEUE_SIZE; ++i)
//	{
//		for(int j=0; j!=3;++j)
////		target_queue.queue[target_queue.index][j]=target_command.target[i][j];
//		{
//			target_queue.queue[target_queue.index][j] = hover_ned[j] + target_buffer.TargetPointList[i][j];
//		}
//		target_queue.queue[target_queue.index][3] = target_buffer.hovertime[target_queue.index];
//
////		if(target_buffer.NED_or_LLH)
////		{
////			target_queue.LLH[0] =  target_queue.queue[target_queue.index][0]*D2R_D;
////			target_queue.LLH[1] =  target_queue.queue[target_queue.index][1]*D2R_D;
////			target_queue.LLH[2] =  target_queue.queue[target_queue.index][2];
////
////			target_queue.N = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(target_queue.LLH[0])));
////			target_queue.ECFF[0]= (target_queue.N + target_queue.LLH[2])* cos(target_queue.LLH[0]) * cos(target_queue.LLH[1]);
////			target_queue.ECFF[1]= (target_queue.N + target_queue.LLH[2])* cos(target_queue.LLH[0]) * sin(target_queue.LLH[1]);
////			target_queue.ECFF[2]= (target_queue.N*(1-SQR(C_WGS84_e)) +target_queue.LLH[2])* sin(target_queue.LLH[0]);
////
////			target_queue.ECFF[0] -= target_queue.Zero_ECFF[0];
////			target_queue.ECFF[1] -= target_queue.Zero_ECFF[1];
////			target_queue.ECFF[2] -= target_queue.Zero_ECFF[2];
////
////
////			target_queue.queue[target_queue.index][0] = target_queue.Re2t[0][0]*target_queue.ECFF[0]+target_queue.Re2t[0][1]*target_queue.ECFF[1]+target_queue.Re2t[0][2]*target_queue.ECFF[2];
////			target_queue.queue[target_queue.index][1] = target_queue.Re2t[1][0]*target_queue.ECFF[0]+target_queue.Re2t[1][1]*target_queue.ECFF[1]+target_queue.Re2t[1][2]*target_queue.ECFF[2];
////			target_queue.queue[target_queue.index][2] = target_queue.Re2t[2][0]*target_queue.ECFF[0]+target_queue.Re2t[2][1]*target_queue.ECFF[1]+target_queue.Re2t[2][2]*target_queue.ECFF[2];
////		}
//
//		target_buffer.count--;
//
////		target_command.count--;
//		target_queue.index ++ ;
//		if(-2 == target_queue.queue[i][3])		//加进去多少个TARGET 就检查queue前多少个是否正常
//												//若有不正常则初始化队列并刹车
//		{
//			Target_Queue_Reset();
//			Target_Command_Reset();//清空target_buffer
//			brake_flag = false;
//			break;
//		}

//	for(int j=0; j<=4;++j)
//	{
//		target_queue.queue[target_buffer.index][j] = hover_ned[j] + target_buffer.TargetPointList[target_buffer.index][j];
//	}


	target_queue.queue[target_buffer.index][0] = hover_ned[0] + target_buffer.TargetPointList[target_buffer.index][0];
	target_queue.queue[target_buffer.index][1] = hover_ned[1] + target_buffer.TargetPointList[target_buffer.index][1];
	target_queue.queue[target_buffer.index][2] = hover_ned[2] + target_buffer.TargetPointList[target_buffer.index][2];



	target_queue.queue[target_buffer.index][3] = target_buffer.hovertime[target_buffer.index];


	return brake_flag;
}

//***需优化  每次pop的时候都要把队列的值重新刷新
bool TARGET::Target_Queue_Pop_Target(float * target_p,float* hover_time)
{
//	if(target_queue.index <= 0)
//		return false ;
//	//
//	for(int i =0 ; i!=3 ; ++i)
//	*(target_p+i) =  target_queue.queue[0][i] ;//+ trajectory.hover_xyz[i];
//	//trajectory.hover_time =  target_queue.queue[0][3]*30;
////	*hover_time = target_queue.queue[0][3]*60;
//	*hover_time = target_queue.queue[0][3];
//
//	for(int i =0 ; i !=MAX_QUEUE_SIZE-1 ; ++i )//队列向左移一个
//	{
//		for(int j =0 ; j!= 4 ; ++j)
//			target_queue.queue[i][j] =  target_queue.queue[i+1][j];
//	}
//////
//	target_queue.queue[MAX_QUEUE_SIZE-1][3] = -1;
//	target_queue.index--;

	if(target_buffer.cur_targert > target_buffer.index)
		return false ;
	target_buffer.cur_targert++;
	for(int i =0 ; i!=3 ; ++i)
	{
		*(target_p+i) =  target_queue.queue[target_buffer.cur_targert][i];
	}
	*hover_time = target_queue.queue[target_buffer.cur_targert][3]*100;//将地面站发送的s转换为ms，hover_time每1ms减一，


	xQueueOverwrite(queueTargetBuffer,&target_buffer);

	return true;
}

bool TARGET::Target_Queue_Is_Queue_Empty(void){
//  return target_queue.index<=0? true:false;
  if(target_buffer.cur_targert >= target_buffer.index || target_buffer.index == -1)
	  return true;
  else
	  return false;

}

