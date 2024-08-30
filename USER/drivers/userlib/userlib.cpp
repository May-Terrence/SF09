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


void Tim_Calc(sTIM *timx)
{
	timx->OUT=TIM_COUNT->CNT - timx->CNT;
	timx->CNT=TIM_COUNT->CNT;
}

//用户延时函数，1us计时，最大延时为4294s
void User_Delay(u32 nus)
{
	u32 StrTim = TIM_COUNT->CNT;
	while(TIM_COUNT->CNT-StrTim<=nus);
}

void getTimer_us(uint32_t *timer)
{
	*timer = TIM_COUNT->CNT;
}

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

//滑窗均值滤波器，CMD:0重新初始化，1正常滤波，2使用同个FIL滤波。
void SlideFilt(float *Dat,float *DatRel,u8 num,sCNT *Filt,u8 Cmd)
{
	switch(Cmd)
	{
		case 0:Filt->CNT=1;break;
		case 1:if(Filt->CNT < Filt->CCR)Filt->CNT++;break;
		case 2:break;
	}
	if(Filt->CNT==1)
	{
		for(int i=0;i<num;i++)
		{
			Dat[i]=DatRel[i];
		}
	}
	for(int i=0;i<num;i++)
	{
		Dat[i]=(Dat[i]*(Filt->CNT-1)+DatRel[i])/Filt->CNT;
	}
}


void LineFit(float* Y,u16 N,float *ab) //y=ax+b
{
	float SumXX=0.0f,SumX=0.0f,SumXY=0.0f,SumY=0.0f;
	for(u16 i=0;i<N;i++)
	{
		SumXX += i*i;
		SumX  += i;
		SumXY += Y[i]*i;
		SumY  += Y[i];
	}
	ab[0] = (SumXY*N-SumX*SumY)/(SumXX*N-SumX*SumX);
	ab[1] = (SumXX*SumY-SumX*SumXY)/(SumXX*N-SumX*SumX);
}


//判断符号位
float Sign(float value)
{
	if(value>=0.0f)return 1.0f;
	else return -1.0f;
}



float SQR(float x)
{
	return x*x;
}

float absf(float x)
{
	if(x<0)
		return -x;
	else
		return x;
}

float MAX(float x,float y)
{
	if(x>y)
		return x;
	else
		return y;
}

float MIN(float x,float y)
{
	if(x<y)
		return x;
	else
		return y;
}

//字符分割
u8 StrSeg(const char *str,char chr,char *para[],u8 num)
{
	u8 i;
	char *pStr = (char *)strchr(str,chr);
	if(pStr==NULL)return 0;         //找不到字符chr
	for(i=0;i<num;i++)
	{
		pStr = strchr(pStr,chr);
		if(pStr==NULL)break;
		para[i]=++pStr;
	}
	return i;
}

bool TarHit(sCNT *Hit, float Tol, double err)
{
	if (Hit->CNT>=Hit->CCR)
	{
		return false;
	}
	else
	{
		if (absf(err)<=Tol)
		{
			Hit->CNT++;
		}
		return true;
	}
}
void mod2PI(float *theta)
{
	if(*theta<0.0f)
	{
		while(*theta<0.0f)
		{
			*theta+=2*M_PI;
		}
	}
	else if(*theta>=2*M_PI)
	{
		while(*theta>=2*M_PI)
		{
			*theta-=2*M_PI;
		}
	}
	else
	{
		return;
	}
}
void modPI(float *theta)
{
	if(*theta<-M_PI)
	{
		while(*theta<-M_PI)
		{
			*theta+=2*M_PI;
		}
	}
	else if(*theta>M_PI)
	{
		while(*theta>M_PI)
		{
			*theta-=2*M_PI;
		}
	}
	else
	{
		return;
	}
}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
