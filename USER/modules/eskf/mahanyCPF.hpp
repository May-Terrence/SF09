/*
 * mahanyCPF.hpp
 *
 *  Created on: 2020年12月10日
 *      Author: 刘成吉
 */

#ifndef MODULES_ESKF_MAHANYCPF_HPP_
#define MODULES_ESKF_MAHANYCPF_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "MySeqQueue.hpp"


#ifdef __cplusplus
#include <Eigen>
#include <userlib/utility.hpp>
#include <eskf/eskf.hpp>
//#define dt	0.005f

using namespace std;
using namespace Eigen;

class MAHANYCPF
{
private:
	Quaternion<float> q;
	Quaternion<float> frame_switch_quat;
	Matrix3f FS_matrix;
	Vector3f Ang; //roll, pitch, yaw
	Vector3f Magfiel;//当地磁场
	Vector3f last_Magfiel;
	Vector3f g;

	eskf_msg mahany;
	uint32_t startTimer;
	uint32_t stopTimer;
	uint32_t executionTime_us;

	uint8_t P,I;

	PID_msg pid;
	///////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////////////////////////////////////////////////////

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW; //该宏用于告诉编译器为当前类的对象分配对齐内存
	void MahanyCPF();		//Mahany互补滤波，gps不可用时用Mahany互补滤波解算姿态
	bool init();
	Quaternion<float> GetQ(void)
	{return q;}
	friend class ESKF;
	friend class ESKF_BARO;
};
#endif
#endif /* MODULES_ESKF_MAHANYCPF_HPP_ */
