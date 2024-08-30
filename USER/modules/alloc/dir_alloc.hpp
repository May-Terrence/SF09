/*
 * DIR_ALLOC.hpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */

#ifndef DIR_ALLOC_HPP_
#define DIR_ALLOC_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "motor/motor.hpp"
#ifdef __cplusplus
extern "C"
{
#endif

#define DBL_MAX 1.79769313486231571e+16
typedef struct
{
	double u[4];//舵量，单位：弧度
	double umin[4];//最小舵量，单位：弧度
	double umax[4];//最大舵量，单位：弧度
	double umin_res[4];//剩余最小舵量，单位：弧度
	double umax_res[4];//剩余最大舵量，单位：弧度
	double p_limits;//舵机幅值，单位：度
	double theta_min;
	double A[13][20];
	double Ad5[13];
	double Ad10[13];
	double b[13];
	double c[20];
	int basis[13];
	double x[20];
	int L;
	double z;
	double iters;
	int e;
	double temp;
}sDir;

typedef struct
{
	float v[3];//总输入量
	float u[6];//舵面输出
	float B[18];//控制效率矩阵,3X6
	float umin[6];//舵面限幅
	float umax[6];//
	//----------------------------for six vane ducted fan

	short iters;
	float z;
	float indi_fb[3];//优先级1 from indi high prior
	float error_fb[3];//优先级2 from p-control base on error
	//-----------------------------for other allocation method

}sAlloc;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


#include <iostream>
#include <Dense>
using namespace std;
using namespace Eigen;

class DIR_ALLOC
{
public:
	DIR_ALLOC(){}
	~DIR_ALLOC(){}

	void dir_alloc_mch(double v[3], double umin[4], double umax[4], double u[4]);
	void dir_alloc_mch_initialize(void);
	//———————————————————————————————————————by mch
	bool check_limits(double u[4]);
	void two_dir_alloc_mch(double v_T[3], double v_D[3], double u[4]);
	bool check_c(const double c[20], int *e);

	void AllocHex_Init(void);
	void AllocHex_Calc(double output[3], double output1[3], double output2[3]);
	void dir_alloc_six(const float umin[6], const float umax[6], const float v[3],
            const float B[18], float u[6], float *z, short *iters);
	sDir dir;
	sAlloc alloc;
    float B[3][4] = { {-0.5,0.0,0.5,0.0}, {0.0,-0.5,0.0,0.5},{0.25,0.25,0.25,0.25}};
    double p_limits = CS_LIMIT_MAX/RAD_TO_PWM;
private:

	double P_inv[13][13]={{-1,     1,     2,     0,     0,     0,     0,    0,     0,     0,     0,     0,     0},
		{0,    -2,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
		{1,     1,     2,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0},
		{1,    -1,    -2,     1,     0,     0,     0,     0,     0,     0,     0,     0,     0},
		{0,     2,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0,     0},
		{-1,    -1,    -2,     0,     0,     1,     0,     0,     0,     0,     0,     0,     0},
		{0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0,     0},
		{0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0,     0},
		{-1,     1,     2,     0,     0,     0,     0,     0,     1,     0,     0,     0,     0},
		{0,    -2,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0,     0},
		{1,     1,     2,     0,     0,     0,     0,     0,     0,     0,     1,     0,     0},
		{0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1,     0},
	    {0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0,     1}};



};


#endif





#endif /* DIR_ALLOC_HPP_ */
