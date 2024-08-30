/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : userlib.cpp
  * Description        : 
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : 2020年10月11日
  ******************************************************************************
  */
/* USER CODE END Header */

#include "userlib.hpp"

//字符串转矩阵，将"+X+Y+Z"转成 +1 0 +1 1 +1 2
bool Dir_Trans(s8 Dir[6],const char DirChr[6])
{
	for(int i=0;i<6;)
	{
		switch(DirChr[i])
		{
			case '+':Dir[i]= 1;break;
			case '-':Dir[i]=-1;break;
			default:return false;
		}
		i++;
		switch(DirChr[i])
		{
			case 'X':Dir[i]=0;break;
			case 'Y':Dir[i]=1;break;
			case 'Z':Dir[i]=2;break;
			default:return false;
		}
		i++;
	}
	return true;
}

void getTimer_us(uint32_t *timer)
{
	*timer = TIM_COUNT->CNT;
}

//限幅函数
float fConstrain(float Input, float minValue, float maxValue)
{
	if (Input < minValue) return minValue;
	else if (Input > maxValue) return maxValue;
	else return Input;
}

//限幅函数
s16 iConstrain(s16 Input, s16 minValue, s16 maxValue)
{
	if (Input < minValue) return minValue;
	else if (Input > maxValue) return maxValue;
	else return Input;
}

//循环限幅函数
float LoopConstrain(float Input, float minValue, float maxValue)
{
	if(Input>=maxValue)return LoopConstrain(Input-(maxValue-minValue),minValue,maxValue);
	if(Input<minValue)return LoopConstrain(Input+(maxValue-minValue),minValue,maxValue);
	return Input;
}
/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
