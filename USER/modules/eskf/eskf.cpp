/*
 * eskf.cpp
 *
 *  Created on: 2020年12月30日
 *      Author: 刘成吉
 */

#include <eskf/eskf.hpp>
ESKF eskfObj;
MAHANYCPF mahanyObj;

void ESKF::init(const MAHANYCPF& mahany)
{
//	getTimer_us(&startTimer);

	//执行此函数前提是gps星数&精度足够
	//初始化
	//G阵
	//H阵
	//Q,R阵
	//初始状态
	//-----------------------------------------离散Q阵-------------------------------------------
	Matrix<float,12,1> Q_diag; //包括速度,姿态,加速度,和角速度偏置对应的协方差矩阵
	//         σav^2I·dt^2(x)       y              z             σgv^2I·dt^2(x)          y               z               σau^2I·dt(x)     y      z          σgu^2I·dt(x)    y      z
	Q_diag<<  1e-4*pow(dt,2), 1e-4*pow(dt,2),   1e-2*pow(dt,2),    1e-6*pow(dt,2),   1e-6*pow(dt,2), 1e-6*pow(dt,2),     1e-7*dt,	  1e-7*dt, 1e-7*dt,   1e-10*dt,  1e-10*dt,  1e-10*dt;
	Q = Q_diag.asDiagonal();

	//-------------------R阵--------------------
	Matrix<float,7,1> R_diag;
	//       σ^2_N     σ^2_E         σ^2_D     σ^2_Vned(N)    E       D	   σ^2_mag_declination
	R_diag<< 1e-4,      1e-4,         1e-4,      1e-3,  	1e-3,	1e-3,	 1e-2;
	R = R_diag.asDiagonal();

//	Matrix<float,12,1> Q_diag; //包括速度,姿态,加速度,和角速度偏置对应的协方差矩阵
//	//         σav^2I·dt^2(x)       y              z             σgv^2I·dt^2(x)          y               z               σau^2I·dt(x)     y      z          σgu^2I·dt(x)    y      z
//	Q_diag<<  1e-4*pow(dt,2), 1e-4*pow(dt,2),   1e-3*pow(dt,2),    1e-6*pow(dt,2),   1e-6*pow(dt,2), 1e-6*pow(dt,2),     1e-7*dt,	  1e-7*dt, 1e-5*dt,   1e-10*dt,  1e-10*dt,  1e-10*dt;
//	Q = Q_diag.asDiagonal();
//
//	//-------------------R阵--------------------
//	Matrix<float,7,1> R_diag;
//	//       σ^2_N     σ^2_E         σ^2_D     σ^2_Vned(N)    E       D	   σ^2_mag_declination
//	R_diag<< 1e-5,      1e-5,         1e-6,      1e-4,  	1e-4,	1e-5,	 1e-2;
//	R = R_diag.asDiagonal();

	//-------------------P0阵---------------------------------------------------
	//σ^2I          北东地                    速度                            姿态      		                   加速度偏置     	       	陀螺仪偏置σ^2I
	Matrix<float,15,1> P_diag;
	P_diag << 1e-2,	1e-2, 1e-2,  1e-2,1e-2,1e-2,  1e-4, 1e-4, 1e-4,   3*1e-6, 3*1e-6, 3*1e-6,   1e-10, 1e-10, 1e-10;
	P = P_diag.asDiagonal();

	//
	H.setZero();
	H.block<6,6>(0, 0) = Matrix<float,6,6>::Identity();
	H(6,8) = 1; //？

	//I.C.
	this->q = mahany.q;
	this->g << 0,0,scalar_g;
	this->Pos << gps.NED[0], gps.NED[1], gps.NED[2];
	this->Vned <<gps.NED_spd[0], gps.NED_spd[1], gps.NED_spd[2];
	this->Ba<< 0,0,0;
	this->Bg<< 0,0,0;


	I3.setIdentity();

	G.setZero();
	G.block<12,12>(3,0).setIdentity(); //取12*12的矩阵，从（3，0）开始
	QQ = G*Q*G.transpose();

	//F阵常数块
	F.setZero();
	F.block<3,3>(0,0) = I3;
	F.block<3,3>(0,3) = I3*dt;

	F.block<3,3>(3,3) = I3;
	F.block<3,3>(6,12)= -I3*dt;

	F.block<3,3>(9,9) = I3;
	F.block<3,3>(12,12) = I3;


	//填充队列
	while(!states_Q.IsFull())
	{
		//初始化状态队列states_Q
		stateTempEle.q    = this->q;
		stateTempEle.Pos  = this->Pos;
		stateTempEle.Vned = this->Vned;
		stateTempEle.Ba	 = this->Ba;
		stateTempEle.Bg	 = this->Bg;
		states_Q.EnQueue(stateTempEle); //入队

		//初始化imu队列imu_Q
		ImuMagTempEle.magm << mag.MagRel[0], mag.MagRel[1], mag.MagRel[2];
		ImuMagTempEle.wm << imu.gyro[0], imu.gyro[1], imu.gyro[2];
		ImuMagTempEle.accm << imu.acc[0], imu.acc[1], imu.acc[2];
		ImuMagTempEle.updtime = imu.timestamp;
		imu_Q.EnQueue(ImuMagTempEle);
	}
	gps_Q.Pos << gps.NED[0], gps.NED[1], gps.NED[2];
	gps_Q.Vned << gps.NED_spd[0], gps.NED_spd[1], gps.NED_spd[2];
	gps_Q.updtime = gps.timestamp;


	FS_matrix << 0.0f, 0.0f, 1.0f,
 	   	   	  	 0.0f, 1.0f, 0.0f,
				-1.0f, 0.0f, 0.0f;
	output.bodyFrame = HELICOPTOR;
	for(uint8_t i=0;i<3;i++){
		output.Pos[i] = 0.0f;
		output.Ned_spd[i] = 0.0f;
	}
	output.q[0] = this->q.x();
	output.q[1] = this->q.y();
	output.q[2] = this->q.z();
	output.q[3] = this->q.w();

	output.P[0] = P(0,0);
	output.P[1] = P(3,3);
	output.P[2] = P(6,6);
	output.P[3] = P(9,9);
	output.P[4] = P(11,11);
	output.P[5] = P(12,12);
	xQueueOverwrite(queueESKF, &output);
//	getTimer_us(&stopTimer);
//	executionTime_us = stopTimer - startTimer;
}

bool ESKF::readSensors()
{
	static uint8_t consecutive_absence_of_gps = 0;
	//读取imu、磁力计信息
	ImuMagTempEle.magm << mag.MagRel[0], mag.MagRel[1], mag.MagRel[2];
	ImuMagTempEle.wm << imu.gyro[0], imu.gyro[1], imu.gyro[2];
	ImuMagTempEle.accm << imu.acc[0], imu.acc[1], imu.acc[2];
	ImuMagTempEle.updtime = imu.timestamp;
	this->imu_Q.DeQueue();//出队
	this->imu_Q.EnQueue(ImuMagTempEle);//入队

	imu_Q.getFront(ImuMagTempEle);			//以imu_Q中的队头数据（最早的数据）为gps对齐数据
	this->accm = ImuMagTempEle.accm;		//用最早的imu数据参与传播运算
	this->wm   = ImuMagTempEle.wm;
	this->magm = ImuMagTempEle.magm;
	//读取gps信息
	if(gps_Q.updtime != gps.timestamp || consecutive_absence_of_gps == 20)//gps有更新，否则不读gps信息
	{
		gps_Q.Pos << gps.NED[0], gps.NED[1], gps.NED[2];
		gps_Q.Vned << gps.NED_spd[0], gps.NED_spd[1], gps.NED_spd[2];
		gps_Q.updtime = gps.timestamp;
		consecutive_absence_of_gps = 0;
		return true;
	}
	consecutive_absence_of_gps++;
	return false;
}
void ESKF::propagate()
{
//	accm:加速度测量值
//	wm	:陀螺仪测量值

	Vector3f vv;
	vv = Vned;

	A = q.toRotationMatrix();
	//估计参量

	accb = accm - Ba; //body系下加速度

	accn = A*accb; //ned系下加速度

	wb = wm - Bg;

	drotation =  AngleAxisf(wb[2]*dt,Vector3f::UnitZ())*AngleAxisf(wb[1]*dt,Vector3f::UnitY())*AngleAxisf(wb[0]*dt,Vector3f::UnitX());
	q = q*drotation;
//	q = q*Expqua(wb*dt/2);
	q.normalize();
	Vned += (accn + g)*dt;
	Pos  += 0.5*(vv+Vned)*dt;

	//系统矩阵

	F.block<3,3>(3,6) = -A*crossM(accb)*dt;
	F.block<3,3>(3,9) = -A*dt;

	F.block<3,3>(6,6) = drotation.transpose();

	P = F*P*F.transpose() + QQ; //协方差矩阵预测
}

void ESKF::update()
{
//	magm：	磁力计测量值
//	H阵
//	H.block<3,3>(6,6) = (Matrix3f() << 0.0f, -2*(-Magfiel.y()*q.w()*q.x() + Magfiel.x()*q.w()*q.y() + Magfiel.x()*q.x()*q.z() + Magfiel.y()*q.y()*q.z()) + Magfiel.z()*(1 - 2*q.w()*q.w() - 2*q.z()*q.z()), 2*(Magfiel.z()*q.w()*q.x() + Magfiel.x()*q.x()*q.y() - Magfiel.x()*q.w()*q.z() + Magfiel.z()*q.y()*q.z()) + Magfiel.y()*(1 - 2*q.x()*q.x() - 2*q.z()*q.z()),
// 	  	  	   	   	   	   	   	   	   2*(-Magfiel.y()*q.w()*q.x() + Magfiel.x()*q.w()*q.y() + Magfiel.x()*q.x()*q.z() + Magfiel.y()*q.y()*q.z()) + Magfiel.z()*(1 - 2*q.x()*q.x() - 2*q.y()*q.y()), 0.0f, -2*(-Magfiel.z()*q.w()*q.y() + Magfiel.y()*q.x()*q.y() + Magfiel.y()*q.w()*q.z() + Magfiel.z()*q.x()*q.z()) + Magfiel.x()*(1 - 2*q.w()*q.w() - 2*q.x()*q.x()),
//									   -2*(Magfiel.z()*q.w()*q.x() + Magfiel.x()*q.x()*q.y() - Magfiel.x()*q.w()*q.z() + Magfiel.z()*q.y()*q.z()) + Magfiel.y()*(1 - 2*q.w()*q.w() - 2*q.y()*q.y()), 2*(-Magfiel.z()*q.w()*q.y() + Magfiel.y()*q.x()*q.y() + Magfiel.y()*q.w()*q.z() + Magfiel.z()*q.x()*q.z()) + Magfiel.x()*(1 - 2*q.y()*q.y() - 2*q.z()*q.z()), 0.0f).finished();
	//卡尔曼增益
	K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
//
	this->Ang = qua2eul(q);
	maglevel = AngleAxisf(this->Ang(1),Vector3f::UnitY())*AngleAxisf(this->Ang(0),Vector3f::UnitX())*magm;
	psim = -atan2(maglevel(1),maglevel(0)) - 3.1167*D2R;	//测量ψ加上磁偏角（3.1167°偏西）
	dpsim = psim - this->Ang(2);
	if(dpsim > M_PI)		dpsim -= 2*M_PI;
	if(dpsim < -M_PI)		dpsim += 2*M_PI;

	z_sub_y << gps_Q.Pos - this->Pos, gps_Q.Vned - this->Vned, dpsim;
	delta_x = K*z_sub_y;
////update states
	Pos	 += delta_x.head<3>();
	Vned += delta_x.segment<3>(3);
	q     = q*Expqua(delta_x.segment<3>(6)/2);
	q.normalize();
	Ba 	 += delta_x.segment<3>(9);
	Bg 	 += delta_x.segment<3>(12);
	for(int i = 0; i < 3; i++){
		Bg[i] = fConstrain(Bg[i], -0.05f, 0.05f);
	}
	Ba[0] = fConstrain(Ba[0], -0.2f, 0.2f);
	Ba[1] = fConstrain(Ba[1], -0.2f, 0.2f);
	Ba[2] = fConstrain(Ba[2], -0.5f, 0.5f);

	P = (Matrix<float,15,15>::Identity() - K*H)*P;
}
void ESKF::refreshStatesQ()
{
	states_Q.getRear(stateTempEle); //找队尾
	imu_Q.getRear(ImuMagTempEle);
	Vector3f vv;
	vv = stateTempEle.Vned;

	stateTempEle.q = stateTempEle.q*Expqua((ImuMagTempEle.wm - stateTempEle.Bg)*dt/2);
	stateTempEle.q.normalize();
	stateTempEle.Vned += (stateTempEle.q.toRotationMatrix()*(ImuMagTempEle.accm-stateTempEle.Ba) + g)*dt;
	stateTempEle.Pos  += 0.5*(vv+stateTempEle.Vned)*dt;

	Vector3f Ang1 = qua2eul(stateTempEle.q);
	for(uint8_t i = 0; i<3; i++)
	{
		output.Attitude1[i]  = Ang1[i];//调试用临时注释
		output.Pos1[i]       = stateTempEle.Pos[i];
		output.Ned_spd1[i]   = stateTempEle.Vned[i];
//		output.Gyro_bias[i] = this->Bg[i];
//		output.Acc_bias[i]	= this->Ba[i];
	}

	states_Q.DeQueue();		//队头出队
	states_Q.EnQueue(stateTempEle); //入队

	states_Q.getFront(stateTempEle);
	pos_err 	   = this->Pos - stateTempEle.Pos;
//	pos_err_inter += pos_err*dt;
	pos_err_inter(0) += (pos_err_inter(0)<3.0 && pos_err_inter(0)>-3.0)?pos_err(0)*dt:0.0;
	pos_err_inter(1) += (pos_err_inter(1)<3.0 && pos_err_inter(1)>-3.0)?pos_err(1)*dt:0.0;
	pos_err_inter(2) += (pos_err_inter(2)<3.0 && pos_err_inter(2)>-3.0)?pos_err(2)*dt:0.0;

	vel_err        = this->Vned - stateTempEle.Vned;
//	vel_err_inter += vel_err*dt;
	vel_err_inter(0) += (vel_err_inter(0)<7.0 && vel_err_inter(0)>-7.0)?vel_err(0)*dt:0.0;
	vel_err_inter(1) += (vel_err_inter(1)<7.0 && vel_err_inter(1)>-7.0)?vel_err(1)*dt:0.0;
	vel_err_inter(2) += (vel_err_inter(2)<7.0 && vel_err_inter(2)>-7.0)?vel_err(2)*dt:0.0;

	ang_err        = Logqua(stateTempEle.q.conjugate()*this->q);
//	ang_err_inter += ang_err*dt;
	ang_err_inter(0) += (ang_err_inter(0)<2.5 && ang_err_inter(0)>-2.5)?ang_err(0)*dt:0.0;
	ang_err_inter(1) += (ang_err_inter(1)<2.5 && ang_err_inter(1)>-2.5)?ang_err(1)*dt:0.0;
	ang_err_inter(2) += (ang_err_inter(2)<2.5 && ang_err_inter(2)>-2.5)?ang_err(2)*dt:0.0;

	/*-------------------PI互补滤波----------------------------------
	 *			    	kp					ki
	 */
	pos_cor = pos_err*0.6 + pos_err_inter*0.09;
	vel_cor = vel_err*0.6 + vel_err_inter*0.09;
	ang_cor = ang_err*0.3 + ang_err_inter*0.04;
//	ang_cor = ang_err*0.8 + ang_err_inter*0.1;

	for(uint8_t i=0; states_Q.getelement(i, stateTempEle)==true; i++)
	{
		stateTempEle.Pos += pos_cor;
		stateTempEle.Vned+= vel_cor;
		stateTempEle.q    = stateTempEle.q*Expqua(ang_cor);
		stateTempEle.q.normalize();
		stateTempEle.Ba   = this->Ba;
		stateTempEle.Bg   = this->Bg;
		states_Q.setelement(i, stateTempEle);
	}
}

void ESKF::outputStates()
{
	this->Ang = qua2eul(q);
	for(uint8_t i = 0; i<3; i++)
	{
		output.Attitude0[i]  = this->Ang[i];//调试用临时注释
		output.Pos0[i]       = this->Pos[i];
		output.Ned_spd0[i]   = this->Vned[i];
//		output.Gyro_bias[i] = this->Bg[i];
//		output.Acc_bias[i]	= this->Ba[i];
	}

	states_Q.getRear(stateTempEle);
	this->Ang = qua2eul(stateTempEle.q);
	A = stateTempEle.q.toRotationMatrix();
	//转NED坐标系坐标
	for(uint8_t i = 0; i<3; i++)
	{
		output.Attitude[i]  = this->Ang[i];//调试用临时注释
		output.Pos[i]       = stateTempEle.Pos[i];
		output.Ned_spd[i]   = stateTempEle.Vned[i];
		output.Gyro_bias[i] = stateTempEle.Bg[i];
		output.Acc_bias[i]	= stateTempEle.Ba[i];

//		output.pos_cor[i] = pos_cor[i];
//		output.vel_cor[i] = vel_cor[i];
//		output.ang_cor[i] = ang_cor[i]*R2D;
	}
	output.P[0] = P(0,0);
	output.P[1] = P(3,3);
	output.P[2] = P(6,6);
	output.P[3] = P(9,9);
	output.P[4] = P(11,11);
	output.P[5] = P(12,12);

	for(u8 i=0;i<3;i++)
		for(u8 j=0;j<3;j++)
			output.rotation[i][j] = A(i,j);

	output.update =true;
	xQueueOverwrite(queueESKF, &output);
}

void ESKF::Eskf_Calc()
{
	startTimerLast = startTimer;
	getTimer_us(&startTimer);
	cycleTime_us = startTimer - startTimerLast;

	bool isGpsAvaliable;
	static bool isMahanyInitialized = false;
	static bool isESKFInitialized = false;

	xQueuePeek(queueGps, &gps, 0);
	xQueuePeek(queueMag, &mag, 0);
	xQueuePeek(queueDownsampleIMU, &imu, 0);
	*(imu.sample_cnt) = 0;

	bool isGpsReady = gps.isReady;//= false;

//	bool isGpsReady =  true;

	if(!isMahanyInitialized)
	{
		isMahanyInitialized = mahanyObj.init(); //获取当地磁场Magfiel
	}
	else if(!isGpsReady)  //star小于等于18或位置精度大于0.1m则不使用ESKF
	{
		mahanyObj.MahanyCPF(); //获取mahany滤波后的Attitude
	}
	else if(!isESKFInitialized)
	{
		init(mahanyObj); //ESKF初始化
		isESKFInitialized = true;
	}
	else
	{
//		switchFrame();
		isGpsAvaliable = readSensors(); //更新GPS和IMU
		propagate(); //预测过程
		if(isGpsAvaliable)
			update(); //更新过程
		refreshStatesQ();
		outputStates();
	}
//	xQueuePeek(queueRCCommand, &rcCommand, 0);
//	xQueuePeek(queueESKF, &output, 0);
//	output.Attitude[0] = 0;
//	output.Attitude[1] = 0;
//	output.Attitude[2] = 0;

//	output.Pos[0] = output.Pos[0] + 0.001;
//	output.Pos[1] = output.Pos[1] + 0.002;
//	output.Pos[2] = output.Pos[1] + 0.005;

//	output.Pos[0] = 1;
//	output.Pos[1] = 1;
//	output.Pos[2] = 1;

//	if(rcCommand.OneKeyTakeoff)
//	{
//		if(output.Pos[2] > -1)
//		{
//			output.Pos[2] = output.Pos[2] - 0.0005f;
//		}
//
//		else
//		{
//			output.Pos[2] = -1;
//		}
//	}
//	xQueueOverwrite(queueESKF, &output);

	getTimer_us(&stopTimer);
	executionTime_us = stopTimer - startTimer;
}

void ESKF::switchFrame()
{
	//运行时间：237us
	if(abs(this->Ang(1) + PI/2) < 20*D2R && output.bodyFrame == HELICOPTOR)
	{
		q = q*FS_matrix;
		q.normalize();
		for(uint8_t i=0; states_Q.getelement(i, stateTempEle)==true; i++)
		{
			stateTempEle.q = stateTempEle.q*FS_matrix;
			stateTempEle.q.normalize();
			float interVal;
			interVal		   = stateTempEle.Ba(0);
			stateTempEle.Ba(0) =-stateTempEle.Ba(2);
			stateTempEle.Ba(2) = interVal;
			interVal           = stateTempEle.Bg(0);
			stateTempEle.Bg(0) =-stateTempEle.Bg(2);
			stateTempEle.Bg(2) = interVal;
			states_Q.setelement(i, stateTempEle);
		}

		for(uint8_t i=0; imu_Q.getelement(i, ImuMagTempEle)==true; i++)
		{
			float interVal;
			interVal              = ImuMagTempEle.accm(0);
			ImuMagTempEle.accm(0) =-ImuMagTempEle.accm(2);
			ImuMagTempEle.accm(2) = interVal;
			interVal			= ImuMagTempEle.wm(0);
			ImuMagTempEle.wm(0) =-ImuMagTempEle.wm(2);
			ImuMagTempEle.wm(2) = interVal;
			interVal			  = ImuMagTempEle.magm(0);
			ImuMagTempEle.magm(0) =-ImuMagTempEle.magm(2);
			ImuMagTempEle.magm(2) = interVal;
			imu_Q.setelement(i, ImuMagTempEle);
		}
		output.bodyFrame = FIXWING;
	}
	else if(this->Ang(1) > 70*D2R && output.bodyFrame == FIXWING)
	{
		q = q*FS_matrix.transpose();
		q.normalize();
		output.bodyFrame = HELICOPTOR;
	}
	if(output.bodyFrame == FIXWING)
	{
		float interVal;
		interVal     = imu.gyro[0];
		imu.gyro[0] =-imu.gyro[2];
		imu.gyro[2] = interVal;
		interVal     = imu.acc[0];
		imu.acc[0]   =-imu.acc[2];
		imu.acc[2]   = interVal;
		interVal     = mag.MagRel[0];
		mag.MagRel[0]=-mag.MagRel[2];
		mag.MagRel[2]= interVal;
	}
}

//ESKF_BARO ESKF_baro;
//
//void ESKF_BARO::init(const MAHANYCPF& mahany)
//{
//	//执行此函数前提是gps星数&精度足够
//	//初始化
//	//G阵
//	//H阵
//	//Q,R阵
//	//初始状态
//	//-----------------------------------------离散Q阵-------------------------------------------
//	Matrix<float,12,1> Q_diag; //包括速度,姿态,加速度,和角速度偏置对应的协方差矩阵
//	//         σav^2I·dt^2(x)       y              z             σgv^2I·dt^2(x)          y               z               σau^2I·dt(x)     y      z          σgu^2I·dt(x)    y      z
//	Q_diag<<  1e-5*pow(dt,2), 1e-5*pow(dt,2),   1e-5*pow(dt,2),    1e-6*pow(dt,2),   1e-6*pow(dt,2), 1e-6*pow(dt,2),     1e-9*dt,	  1e-9*dt, 1e-9*dt,   1e-10*dt,  1e-10*dt,  1e-10*dt;
//	Q = Q_diag.asDiagonal();
//
//	//-------------------R阵--------------------
//	Matrix<float,7,1> R_diag;
//	//       σ^2_N     σ^2_E         σ^2_D     σ^2_Vned(N)    E       D	   σ^2_mag_declination
//	R_diag<< 1e-4,      1e-4,         1e-4,      1e-3,  	1e-3,	1e-3,	 1e-2;
//	R = R_diag.asDiagonal();
//
//	//-------------------P0阵---------------------------------------------------
//	//σ^2I          北东地                    速度                            姿态      		                   加速度偏置     	       	陀螺仪偏置σ^2I
//	Matrix<float,15,1> P_diag;
//	P_diag << 1e-2,	1e-2, 1e-2,  1e-2,1e-2,1e-2,  1e-4, 1e-4, 1e-4,   3*1e-6, 3*1e-6, 3*1e-6,   1e-10, 1e-10, 1e-10;
//	P = P_diag.asDiagonal();
//
//	//
//	H.setZero();
//	H.block<6,6>(0, 0) = Matrix<float,6,6>::Identity();
//	H(6,8) = 1; //？
//
//	//I.C.
//	this->q = mahany.q;
//	this->g << 0,0,scalar_g;
//	this->Pos << gps.NED[0], gps.NED[1], gps.NED[2];
//	this->Vned <<gps.NED_spd[0], gps.NED_spd[1], gps.NED_spd[2];
//	this->Ba<< 0,0,0;
//	this->Bg<< 0,0,0;
//
//
//	I3.setIdentity();
//
//	G.setZero();
//	G.block<12,12>(3,0).setIdentity(); //取12*12的矩阵，从（3，0）开始
//	QQ = G*Q*G.transpose();
//
//	//F阵常数块
//	F.setZero();
//	F.block<3,3>(0,0) = I3;
//	F.block<3,3>(0,3) = I3*dt;
//
//	F.block<3,3>(3,3) = I3;
//	F.block<3,3>(6,12)= -I3*dt;
//
//	F.block<3,3>(9,9) = I3;
//	F.block<3,3>(12,12) = I3;
//
//
//	//填充队列
//	while(!states_Q.IsFull())
//	{
//		//初始化状态队列states_Q
//		stateTempEle.q    = this->q;
//		stateTempEle.Pos  = this->Pos;
//		stateTempEle.Vned = this->Vned;
//		stateTempEle.Ba	 = this->Ba;
//		stateTempEle.Bg	 = this->Bg;
//		states_Q.EnQueue(stateTempEle); //入队
//
//		//初始化imu队列imu_Q
//		ImuMagTempEle.magm << mag.MagRel[0], mag.MagRel[1], mag.MagRel[2];
//		ImuMagTempEle.wm << imu.gyro[0], imu.gyro[1], imu.gyro[2];
//		ImuMagTempEle.accm << imu.acc[0], imu.acc[1], imu.acc[2];
//		ImuMagTempEle.updtime = imu.timestamp;
//		imu_Q.EnQueue(ImuMagTempEle);
//	}
//	gps_Q.Pos << gps.NED[0], gps.NED[1], gps.NED[2];
//	gps_Q.Vned << gps.NED_spd[0], gps.NED_spd[1], gps.NED_spd[2];
//	gps_Q.updtime = gps.timestamp;
//
//
//	FS_matrix << 0.0f, 0.0f, 1.0f,
// 	   	   	  	 0.0f, 1.0f, 0.0f,
//				-1.0f, 0.0f, 0.0f;
//	output.bodyFrame = HELICOPTOR;
//	for(uint8_t i=0;i<3;i++){
//		output.Pos[i] = 0.0f;
//		output.Ned_spd[i] = 0.0f;
//	}
//	xQueueOverwrite(queueESKF, &output);
//}
//
//bool ESKF_BARO::readSensors()
//{
//	static uint8_t consecutive_absence_of_gps = 0;
//	//读取imu、磁力计信息
//	ImuMagTempEle.magm << mag.MagRel[0], mag.MagRel[1], mag.MagRel[2];
//	ImuMagTempEle.wm << imu.gyro[0], imu.gyro[1], imu.gyro[2];
//	ImuMagTempEle.accm << imu.acc[0], imu.acc[1], imu.acc[2];
//	ImuMagTempEle.updtime = imu.timestamp;
//	this->imu_Q.DeQueue();//出队
//	this->imu_Q.EnQueue(ImuMagTempEle);//入队
//
//	imu_Q.getFront(ImuMagTempEle);			//以imu_Q中的队头数据（最早的数据）为gps对齐数据
//	this->accm = ImuMagTempEle.accm;		//用最早的imu数据参与传播运算
//	this->wm   = ImuMagTempEle.wm;
//	this->magm = ImuMagTempEle.magm;
//	//读取gps信息
//	if(gps_Q.updtime != gps.timestamp || consecutive_absence_of_gps == 20)//gps有更新，否则不读gps信息
//	{
//		gps_Q.Pos << gps.NED[0], gps.NED[1], gps.NED[2];
//		gps_Q.Vned << gps.NED_spd[0], gps.NED_spd[1], gps.NED_spd[2];
//		gps_Q.updtime = gps.timestamp;
//		consecutive_absence_of_gps = 0;
//		return true;
//	}
//	consecutive_absence_of_gps++;
//	return false;
//}
//void ESKF_BARO::propagate()
//{
////	accm:加速度测量值
////	wm	:陀螺仪测量值
//
//	Vector3f vv;
//	vv = Vned;
//
//	A = q.toRotationMatrix();
//	//估计参量
//
//	accb = accm - Ba; //body系下加速度
//
//	accn = A*accb; //ned系下加速度
//
//	wb = wm - Bg;
//
//	drotation =  AngleAxisf(wb[2]*dt,Vector3f::UnitZ())*AngleAxisf(wb[1]*dt,Vector3f::UnitY())*AngleAxisf(wb[0]*dt,Vector3f::UnitX());
//	q = q*drotation;
////	q = q*Expqua(wb*dt/2);
//	q.normalize();
//	Vned += (accn + g)*dt;
//	Pos  += 0.5*(vv+Vned)*dt;
//
//	//系统矩阵
//
//	F.block<3,3>(3,6) = -A*crossM(accb)*dt;
//	F.block<3,3>(3,9) = -A*dt;
//
//	F.block<3,3>(6,6) = drotation.transpose();
//
//	P = F*P*F.transpose() + QQ; //协方差矩阵预测
//}
//
//void ESKF_BARO::update()
//{
////	magm：	磁力计测量值
////	H阵
////	H.block<3,3>(6,6) = (Matrix3f() << 0.0f, -2*(-Magfiel.y()*q.w()*q.x() + Magfiel.x()*q.w()*q.y() + Magfiel.x()*q.x()*q.z() + Magfiel.y()*q.y()*q.z()) + Magfiel.z()*(1 - 2*q.w()*q.w() - 2*q.z()*q.z()), 2*(Magfiel.z()*q.w()*q.x() + Magfiel.x()*q.x()*q.y() - Magfiel.x()*q.w()*q.z() + Magfiel.z()*q.y()*q.z()) + Magfiel.y()*(1 - 2*q.x()*q.x() - 2*q.z()*q.z()),
//// 	  	  	   	   	   	   	   	   	   2*(-Magfiel.y()*q.w()*q.x() + Magfiel.x()*q.w()*q.y() + Magfiel.x()*q.x()*q.z() + Magfiel.y()*q.y()*q.z()) + Magfiel.z()*(1 - 2*q.x()*q.x() - 2*q.y()*q.y()), 0.0f, -2*(-Magfiel.z()*q.w()*q.y() + Magfiel.y()*q.x()*q.y() + Magfiel.y()*q.w()*q.z() + Magfiel.z()*q.x()*q.z()) + Magfiel.x()*(1 - 2*q.w()*q.w() - 2*q.x()*q.x()),
////									   -2*(Magfiel.z()*q.w()*q.x() + Magfiel.x()*q.x()*q.y() - Magfiel.x()*q.w()*q.z() + Magfiel.z()*q.y()*q.z()) + Magfiel.y()*(1 - 2*q.w()*q.w() - 2*q.y()*q.y()), 2*(-Magfiel.z()*q.w()*q.y() + Magfiel.y()*q.x()*q.y() + Magfiel.y()*q.w()*q.z() + Magfiel.z()*q.x()*q.z()) + Magfiel.x()*(1 - 2*q.y()*q.y() - 2*q.z()*q.z()), 0.0f).finished();
//	//卡尔曼增益
//	K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
////
//	this->Ang = qua2eul(q);
//	maglevel = AngleAxisf(this->Ang(1),Vector3f::UnitY())*AngleAxisf(this->Ang(0),Vector3f::UnitX())*magm;
//	psim = -atan2(maglevel(1),maglevel(0)) - 3.1167*D2R;	//测量ψ加上磁偏角（3.1167°偏西）
//	dpsim = psim - this->Ang(2);
//	if(dpsim > M_PI)		dpsim -= 2*M_PI;
//	if(dpsim < -M_PI)		dpsim += 2*M_PI;
//
//	z_sub_y << gps_Q.Pos - this->Pos, gps_Q.Vned - this->Vned, dpsim;
//	delta_x = K*z_sub_y;
//////update states
//	Pos	 += delta_x.head<3>();
//	Vned += delta_x.segment<3>(3);
//	q     = q*Expqua(delta_x.segment<3>(6)/2);
//	q.normalize();
//	Ba 	 += delta_x.segment<3>(9);
//	Bg 	 += delta_x.segment<3>(12);
//	for(int i = 0; i < 3; i++){
//		Bg[i] = fConstrain(Bg[i], -0.005f, 0.005f);
//	}
//	Ba[0] = fConstrain(Ba[0], -0.2f, 0.2f);
//	Ba[1] = fConstrain(Ba[1], -0.2f, 0.2f);
//	Ba[2] = fConstrain(Ba[2], -0.5f, 0.5f);
//
//	P = (Matrix<float,15,15>::Identity() - K*H)*P;
//}
//void ESKF_BARO::refreshStatesQ()
//{
//	states_Q.getRear(stateTempEle); //找队尾
//	imu_Q.getRear(ImuMagTempEle);
//	Vector3f vv;
//	vv = stateTempEle.Vned;
//
//	stateTempEle.q = stateTempEle.q*Expqua((ImuMagTempEle.wm - stateTempEle.Bg)*dt/2);
//	stateTempEle.q.normalize();
//	stateTempEle.Vned += (stateTempEle.q.toRotationMatrix()*(ImuMagTempEle.accm-stateTempEle.Ba) + g)*dt;
//	stateTempEle.Pos  += 0.5*(vv+stateTempEle.Vned)*dt;
//
//	states_Q.DeQueue();		//队头出队
//	states_Q.EnQueue(stateTempEle); //入队
//
//	states_Q.getFront(stateTempEle);
//	pos_err 	   = this->Pos - stateTempEle.Pos;
////	pos_err_inter += pos_err*dt;
//	pos_err_inter(0) += (pos_err_inter(0)<3.0 && pos_err_inter(0)>-3.0)?pos_err(0)*dt:0.0;
//	pos_err_inter(1) += (pos_err_inter(1)<3.0 && pos_err_inter(1)>-3.0)?pos_err(1)*dt:0.0;
//	pos_err_inter(2) += (pos_err_inter(2)<3.0 && pos_err_inter(2)>-3.0)?pos_err(2)*dt:0.0;
//
//	vel_err        = this->Vned - stateTempEle.Vned;
////	vel_err_inter += vel_err*dt;
//	vel_err_inter(0) += (vel_err_inter(0)<7.0 && vel_err_inter(0)>-7.0)?vel_err(0)*dt:0.0;
//	vel_err_inter(1) += (vel_err_inter(1)<7.0 && vel_err_inter(1)>-7.0)?vel_err(1)*dt:0.0;
//	vel_err_inter(2) += (vel_err_inter(2)<7.0 && vel_err_inter(2)>-7.0)?vel_err(2)*dt:0.0;
//
//	ang_err        = Logqua(stateTempEle.q.conjugate()*this->q);
////	ang_err_inter += ang_err*dt;
//	ang_err_inter(0) += (ang_err_inter(0)<2.5 && ang_err_inter(0)>-2.5)?ang_err(0)*dt:0.0;
//	ang_err_inter(1) += (ang_err_inter(1)<2.5 && ang_err_inter(1)>-2.5)?ang_err(1)*dt:0.0;
//	ang_err_inter(2) += (ang_err_inter(2)<2.5 && ang_err_inter(2)>-2.5)?ang_err(2)*dt:0.0;
//
//	/*-------------------PI互补滤波----------------------------------
//	 *			    	kp					ki
//	 */
//	pos_cor = pos_err*0.6 + pos_err_inter*0.09;
//	vel_cor = vel_err*0.6 + vel_err_inter*0.09;
//	ang_cor = ang_err*0.3 + ang_err_inter*0.04;
//	for(uint8_t i=0; states_Q.getelement(i, stateTempEle)==true; i++)
//	{
//		stateTempEle.Pos += pos_cor;
//		stateTempEle.Vned+= vel_cor;
//		stateTempEle.q    = stateTempEle.q*Expqua(ang_cor);
//		stateTempEle.q.normalize();
//		stateTempEle.Ba   = this->Ba;
//		stateTempEle.Bg   = this->Bg;
//		states_Q.setelement(i, stateTempEle);
//	}
//}
//void ESKF_BARO::outputStates()
//{
//	states_Q.getRear(stateTempEle);
//	this->Ang = qua2eul(stateTempEle.q);
//	A = stateTempEle.q.toRotationMatrix();
//	//转NED坐标系坐标
//	for(uint8_t i = 0; i<3; i++)
//	{
//		output.Attitude[i]  = this->Ang[i];//调试用临时注释
//		output.Pos[i]       = stateTempEle.Pos[i];
//		output.Ned_spd[i]   = stateTempEle.Vned[i];
//	}
//	for(u8 i=0;i<3;i++)
//		for(u8 j=0;j<3;j++)
//			output.rotation[i][j] = A(i,j);
//
//	xQueueOverwrite(queueESKF_baro, &output);
//}
//
//void ESKF_BARO::Eskf_Calc()
//{
//	startTimerLast = startTimer;
//	getTimer_us(&startTimer);
//	cycleTime_us = startTimer - startTimerLast;
//
//	bool isGpsAvaliable;
//	static bool isMahanyInitialized = false;
//	static bool isESKFInitialized = false;
//
////	xQueuePeek(queueOrdinaryGps,&gps,0);
//	xQueuePeek(queueBaroAlt,&baroAlt,0);
//	xQueuePeek(queueMag, &mag, 0);
//	xQueuePeek(queueDownsampleIMU, &imu, 0);
//	*(imu.sample_cnt) = 0;
//
//	gps.NED[2] = -baroAlt.altitude;
//	gps.NED_spd[2] = -baroAlt.altSlope;
//
//	bool isGpsReady = gps.isReady;//= false;
//
////	bool isGpsReady =  true;
//
//	if(!isMahanyInitialized)
//	{
//		isMahanyInitialized = mahanyObj.init(); //获取当地磁场Magfiel
//	}
//	else if(!isGpsReady)
//	{
//		mahanyObj.MahanyCPF(); //获取mahany滤波后的Attitude
//	}
//	else if(!isESKFInitialized)
//	{
//		init(mahanyObj); //ESKF初始化
//		isESKFInitialized = true;
//	}
//	else
//	{
//		isGpsAvaliable = readSensors(); //更新GPS和IMU
//		propagate(); //预测过程
//		if(isGpsAvaliable)
//			update(); //更新过程
//		refreshStatesQ();
//		outputStates();
//	}
//	getTimer_us(&stopTimer);
//	executionTime_us = stopTimer - startTimer;
//}
extern "C" void eskf_main(void *argument)
{
	osDelay(1000);
	for(;;)
	{
		osSemaphoreAcquire(semEskf,0xffffffff);
		eskfObj.Eskf_Calc();
	}
}
//extern "C" void eskf_baro_main(void *argument)
//{
//	osDelay(2000);
//	for(;;)
//	{
//		osSemaphoreAcquire(semEskf_baro,0xffffffff);
//		ESKF_baro.Eskf_Calc();
//	}
//}
