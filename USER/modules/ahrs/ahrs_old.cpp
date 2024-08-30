/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ahrs.cpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 11, 2020
  ******************************************************************************
  */
/* USER CODE END Header */

#include "ahrs.hpp"


void AHRS::Ahrs_Init()
{
	Update = false;
	dt = 0.005f;
	osDelay(180);						//等待传感器数据

	/*求状态变量初始值*/
	//获取IMU+MAG数据
	xQueuePeek(queueGyrDat, &gyro, 0);					//从队列中获取陀螺仪数据
	xQueuePeek(queueAccDat, &acc, 0);					//从队列中获取加速度计数据
	xQueuePeek(queueMag, &mag, 0);					//从队列中获取磁力计数据

	MatGyr << gyro.gyro[0],gyro.gyro[1],gyro.gyro[2];	//Unit:rad/s
	MatAcc << acc.acc[0],acc.acc[1],acc.acc[2];			//Unit:m/s2
	MatMag << mag.MagRel[0],mag.MagRel[1],mag.MagRel[2];			//Unit:uT

	//Acc、Mag数据归一化
	MatAcc.normalize();
	MatMag.normalize();

	Euler[2] = atan2f(MatAcc(1),MatAcc(2));
	Euler[1] = -asinf(MatAcc(0));

	float mag_calib[2];
	float sinR,cosR,sinP,cosP;
	sinR = sinf(Euler[2]);
	cosR = cosf(Euler[2]);
	sinP = sinf(Euler[1]);
	cosP = cosf(Euler[1]);

	mag_calib[0]=-MatMag(1)*cosR + MatMag(2)*sinR;//分子
	mag_calib[1]=MatMag(0)*cosP + MatMag(1)*sinP*sinR + MatMag(2)*sinP*cosR;//分母

	if (mag_calib[1] != 0)
	{
		Euler[0] = atan2f(mag_calib[0], mag_calib[1]);
	}
	else
	{
		if (mag_calib[0] < 0)
			Euler[0] = -M_PI/2;
		else
			Euler[0] = M_PI/2;
	}
	Euler << Euler[0],Euler[1],Euler[2];

	//欧拉角转旋转矩阵,再转四元数
	q = AngleAxisf(Euler(0),Vector3f::UnitZ())*AngleAxisf(Euler(1),Vector3f::UnitY())*AngleAxisf(Euler(2),Vector3f::UnitX());

	q.normalize();

	X_p<<q.w(),q.x(),q.y(),q.z(),0.0f,0.0f,0.0f;//EKF状态量为四元数、陀螺仪偏置
	/*求状态变量初始值*/

	/* 初始协方差矩阵初值 */
	P <<0.0001f,0.0f,0.0f,0.0f,0.0f,0.0f,
		0.0f,0.0001f,0.0f,0.0f,0.0f,0.0f,
		0.0f,0.0f,0.0001f,0.0f,0.0f,0.0f,
		0.0f,0.0f,0.0f,0.0001f,0.0f,0.0f,
		0.0f,0.0f,0.0f,0.0f,0.0001f,0.0f,
		0.0f,0.0f,0.0f,0.0f,0.0f,0.0001f,
	/* 初始协方差矩阵初值 */

	/* Q  R 阵初始化 */
	Q<<5.5e-06f,0.0f,0.0f,0.0f,0.0f,0.0f,
		0.0f,5.5e-06f,0.0f,0.0f,0.0f,0.0f,
		0.0f,0.0f,5.5e-04f,0.0f,0.0f,0.0f,
		0.0f,0.0f,0.0f,5.5e-06f,0.0f,0.0f,
		0.0f,0.0f,0.0f,0.0f,5.5e-06f,0.0f,
		0.0f,0.0f,0.0f,0.0f,0.0f,5.5e-04f;
	R<<1.0f,0.0f,0.0f,0.0f,
		0.0f,1.0f,0.0f,0.0f,
		0.0f,0.0f,1.0f,0.0f,
		0.0f,0.0f,0.0f,0.1f;
	/* Q  R 阵初始化 */


}

void AHRS::Ahrs_Update()
{
	getTimer_us(&startTimer);
	/* 获取加速度计 陀螺仪和磁力计数据 ,耗时16~17us*/
	xQueuePeek(queueGyrDat, &gyro, 0);					//从队列中获取陀螺仪数据
	xQueuePeek(queueAccDat, &acc, 0);					//从队列中获取加速度计数据
	xQueuePeek(queueMag, &mag, 0);					//从队列中获取磁力计数据

	MatGyr << gyro.gyro[0],gyro.gyro[1],gyro.gyro[2];	//Unit:rad/s
	MatAcc << acc.acc[0],acc.acc[1],acc.acc[2];			//Unit:m/s2
	MatMag << mag.MagRel[0],mag.MagRel[1],mag.MagRel[2];			//Unit:uT
	/* 获取加速度计 陀螺仪和磁力计数据 ,耗时16~17us */

	/* 由加速度计和磁力计数据计算得到欧拉角,耗时6us */
	//Acc、Mag数据归一化
	MatAcc.normalize();
	MatMag.normalize();

	acc_sum = acc.acc[0]*acc.acc[0] + acc.acc[1]*acc.acc[1] + acc.acc[2]*acc.acc[2] - 9.788f*9.788f;
	if(acc_sum<0)
		acc_sum = -acc_sum;
	acc_R_a = 0.5f;
	acc_R_b = 1.0f;
	R_ele = acc_R_b*exp(acc_R_a*acc_R_x);

	R<< R_ele,0.0f,0.0f,0.0f,
		0.0f,R_ele,0.0f,0.0f,
		0.0f,0.0f,R_ele,0.0f,
		0.0f,0.0f,0.0f,R_ele;


	Euler[2] = atan2f(MatAcc(1),MatAcc(2));		//横滚角
	Euler[1] = -asinf(MatAcc(0));				//俯仰角

	float mag_calib[2];
	float sinR,cosR,sinP,cosP;
	sinR = sinf(Euler[2]);
	cosR = cosf(Euler[2]);
	sinP = sinf(Euler[1]);
	cosP = cosf(Euler[1]);

	mag_calib[0]=-MatMag(1)*cosR + MatMag(2)*sinR;//分子
	mag_calib[1]=MatMag(0)*cosP + MatMag(1)*sinP*sinR + MatMag(2)*sinP*cosR;//分母

	if (mag_calib[1] != 0)
	{
		Euler[0] = atan2f(mag_calib[0], mag_calib[1]);	//航向角
	}
	else
	{
		if (mag_calib[0] < 0)
			Euler[0] = -M_PI/2;
		else
			Euler[0] = M_PI/2;
	}
	/* 由加速度计和磁力计数据计算得到欧拉角,耗时6us */
	/* ESKF Start */
	/* 计算H1  H2;; 耗时23us */
	float yaw_molecule = 2*(X_p(0)*X_p(3)+X_p(1)*X_p(2));
	float yaw_Denominator = 1-2*(X_p(2)*X_p(2)+X_p(3)*X_p(3));
	H1 << -2*X_p(2),2*X_p(3),-2*X_p(0),2*X_p(1),0.0f,0.0f,0.0f,
			2*X_p(1),2*X_p(0),2*X_p(3),2*X_p(2),0.0f,0.0f,0.0f,
			0.0f,-4*X_p(1),-4*X_p(2),0.0f,0.0f,0.0f,0.0f,
			2*X_p(3)*yaw_Denominator/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator),
			2*X_p(2)*yaw_Denominator/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator),
			(2*X_p(1)*yaw_Denominator+4*X_p(2)*yaw_molecule)/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator),
			(2*X_p(0)*yaw_Denominator+4*X_p(3)*yaw_molecule)/(yaw_molecule*yaw_molecule+yaw_Denominator*yaw_Denominator),
			0.0f,0.0f,0.0f;
	H2 << -0.5f*X_p(1),-0.5f*X_p(2),-0.5f*X_p(3),0.0f,0.0f,0.0f,
			0.5f*X_p(0),-0.5f*X_p(3),0.5f*X_p(2),0.0f,0.0f,0.0f,
			0.5f*X_p(3),0.5f*X_p(0),-0.5f*X_p(1),0.0f,0.0f,0.0f,
			-0.5f*X_p(2),0.5f*X_p(1),0.5f*X_p(0),0.0f,0.0f,0.0f,
			0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,
			0.0f,0.0f,0.0f,0.0f,1.0f,0.0f,
			0.0f,0.0f,0.0f,0.0f,0.0f,1.0f;
	H = H1*H2;
	/* 计算H1  H2;; 耗时23us */

	/* 卡尔曼增益K;; 耗时31us */
	K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
	/* 卡尔曼增益K;; 耗时31us */

	/* 更新状态, 耗时26us */
	//预测观测值
	Z_p << 2*(q.x()*q.z()-q.w()*q.y()),
		2*(q.y()*q.z()+q.w()*q.x()),
		1-2*(q.x()*q.x()+q.y()*q.y()),
		atan2f(yaw_molecule,yaw_Denominator);

	//实际测量值
	Z << MatAcc(0),MatAcc(1),MatAcc(2),Euler[0];


	Z_sub_H = Z-Z_p;
	if(Z_sub_H(3)>M_PI)		Z_sub_H(3) -= 2*M_PI;
	if(Z_sub_H(3)<-M_PI)	Z_sub_H(3) += 2*M_PI;
	X_k_error = K*Z_sub_H;

	q = q * AngleAxisf(X_k_error(2),Vector3f::UnitZ())*AngleAxisf(X_k_error(1),Vector3f::UnitY())*AngleAxisf(X_k_error(0),Vector3f::UnitX());

	q.normalize();			//四元数归一化
	rotation = q.toRotationMatrix();	//四元数转旋转矩阵

	for(u8 i=0;i<3;i++)
		for(u8 j=0;j<3;j++)
			ahrsEuler.rotation[i][j] = rotation(i,j);

	X_k(0) = q.w();
	X_k(1) = q.x();
	X_k(2) = q.y();
	X_k(3) = q.z();
	X_k(4) = X_p(4) + X_k_error(3);
	X_k(5) = X_p(5) + X_k_error(4);
	X_k(6) = X_p(6) + X_k_error(5);
	/* 更新状态, 耗时26us */


	ahrsEuler.Ang[0] =  atan2f(2.0f * q.y() * q.z() + 2.0f * q.w() * q.x(), -2.0f * q.x() * q.x() - 2.0f * q.y() * q.y() + 1.0f);
	ahrsEuler.Ang[1] =  asinf(fConstrain(-2.0f * q.x() * q.z() + 2.0f * q.w() * q.y(),-1.0f,1.0f));
	ahrsEuler.Ang[2] =  atan2f(2.0f * q.x() * q.y() + 2.0f * q.w() * q.z(), -2.0f * q.y() * q.y() - 2.0f * q.z() * q.z() + 1.0f);
	ahrsEuler.pqr[0] = MatGyr(0)-X_k(4);
	ahrsEuler.pqr[1] = MatGyr(1)-X_k(5);
	ahrsEuler.pqr[2] = MatGyr(2)-X_k(6);

	/* 更新P,耗时7us */
//	P = P - K*H*P;       //耗时23us
	P = (MatrixXf::Identity(6, 6) - K*H)*P;		//耗时7us
	/* 更新P,耗时7us */

	/* 预测状态变量,耗时19us */

//	AngleAxisf dprollAngle(AngleAxisf((MatGyr(0)-X_k(4))*0.001f,Vector3f::UnitX()));
//	AngleAxisf dppitchAngle(AngleAxisf((MatGyr(1)-X_k(5))*0.001f,Vector3f::UnitY()));
//	AngleAxisf dpyawAngle(AngleAxisf((MatGyr(2)-X_k(6))*0.001f,Vector3f::UnitZ()));

	drotation = AngleAxisf((MatGyr(2)-X_k(6))*dt,Vector3f::UnitZ())*AngleAxisf((MatGyr(1)-X_k(5))*dt,Vector3f::UnitY())*AngleAxisf((MatGyr(0)-X_k(4))*dt,Vector3f::UnitX());

	q =q * drotation;
	q.normalize();		//四元数预测值

	X_p << q.w(),q.x(),q.y(),q.z(),X_k(4),X_k(5),X_k(6);//陀螺仪偏置预测值等于上一次陀螺仪偏置更新值
	/* 预测状态变量,耗时19us */

	/*计算F,耗时14us */
	F << drotation(0,0),drotation(1,0),drotation(2,0),-0.005f,0.0f,0.0f,
			drotation(0,1),drotation(1,1),drotation(2,1),0.0f,-0.005f,0.0f,
			drotation(0,2),drotation(1,2),drotation(2,2),0.0f,0.0f,-0.005f,
			0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,
			0.0f,0.0f,0.0f,0.0f,1.0f,0.0f,
			0.0f,0.0f,0.0f,0.0f,0.0f,1.0f;
	/*计算F,耗时14us */
	/*预测P , 耗时28us*/
	P = F*P*F.transpose() + Q;
	/*预测P , 耗时28us*/
	X_k_error = X_k_error.Zero();

	/* ESKF End */


	xQueueOverwrite(queueAhrsEuler,&ahrsEuler);

	getTimer_us(&stopTimer);

	executionTime_us = stopTimer - startTimer;

}

AHRS ahrs((char *)"AHRS");
extern "C" void ahrs_main(void *argument)
{
	ahrs.Ahrs_Init();
	osDelay(1);
	for(;;)
	{
		osSemaphoreAcquire(semAhrs,0xffffffff);
		ahrs.Ahrs_Update();		//Ahrs任务耗时大概176us
	}
}

/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/
