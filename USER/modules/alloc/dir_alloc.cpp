/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : dir_alloc.cpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 22, 2020
  ******************************************************************************
  */

/*
 * (c) mengchaoheng
 *  Last edited 2019-11
 *    min z=c*x   subj. to  A*x (=、 >=、 <=) b
 *    x
 *  原问题
 *  Performs direct control allocation by solving the LP
 *    max z=a   subj. to  Bu = av
 *    a,u               umin <= u <= umax
 *  If a > 1, set u = u/a.
 *  Note: This function has not been optimized for speed.
 *   Inputs:
 *   -------
 *  B     control effectiveness matrix (k x m)
 *  v     commanded virtual control (k x 1)
 *  umin  lower position limits (m x 1)
 *  umax  upper position limits (m x 1)
 *   Outputs:
 *   -------
 *  u     optimal control (m x 1)
 *  a     scaling factor
 *  整理成
 *    min z=[0 -1]x   subj. to  [B -v]x = 0
 *    x                       [I 0;-I 0]x <= [umax; -umin]
 *    其中 x=[u; a]
 *  对应《凸优化》p139,记为
 *    min z=c*x   subj. to  Aeq*x = beq
 *    x                     G*x <= h
 *  合并
 *    min z=c*x   subj. to  [Aeq; G]*x (=、<=) [beq;h]
 *    x
 *  保证x>=0，变形
 *    min z=[c -c]*X   subj. to  [Aeq -Aeq;G -G]*X (=、<=) [beq;h]
 *     X
 *  其中 X=[x^+; x^-]
 *
 *  B=[-0.5   0       0.5   0;
 *       0  -0.5    0       0.5;
 *      0.25   0.25   0.25   0.25];
 * Arguments    : const double v[3]
 *                const double umin[4]
 *                const double umax[4]
 *                double u[4]
 * Return Type  : void
 */
#include <alloc/dir_alloc.hpp>
#include <alloc/controlAllocation.hpp>

DIR_ALLOC DPLP;
Aircraft<3, 4> df_4(DPLP.B, -DPLP.p_limits, DPLP.p_limits); // 创建一个具有 4 个操纵向量和 3 个广义力矩的飞行器对象
DP_LP_ControlAllocator<3, 4> Allocator(df_4); // 创建一个控制分配器对象，用于具有 4 个操纵向量和 3 个广义力矩的飞行器(转化为线性规划问题，其维数和参数 <3, 4> 有关。)


bool DIR_ALLOC::check_c(const double c[20], int *e)
{
	for(int i=0;i<20;++i)
	{
		if(c[i]<0)
		{
			*e=i;
			return 1;
		}
	}
	return 0;
}

//double Ad5[13]={0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1};
//double Ad10[13]={0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};
//double b[13]={0, 0, 0, 0, 0, 0, 0, 50, 0, 0, 0, 0, 0};
//double A[13][20]={{1.0f,     0.0f,     0.0f,     1.0f,     0.0f,    -1.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     1.0f,     0.0f,    -1.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     1.0f,     1.0f,     0.0f,     0.0f,     0.0f,    -1.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,    -1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f,     0.0f},
//									{0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     0.0f,     1.0f}};

void DIR_ALLOC::dir_alloc_mch(double v[3], double umin[4], double umax[4],double u[4])
{
	memset(dir.Ad5, 0, 13U * sizeof(double));
	memset(dir.Ad10, 0, 13U * sizeof(double));
	dir.Ad5[7]=1;
	dir.Ad5[12]=-1;
	dir.Ad10[7]=-1;
	dir.Ad10[12]=1;
	memset(dir.A, 0, 260U * sizeof(double));
	for(int i=0;i<3;++i)
	{
		dir.A[i][i]=1;
		dir.A[i][i+5]=-1;
	}
	for(int i=0;i<10;++i)
	{
		dir.A[i+3][i+10]=1;
	}
	dir.A[0][3]=1;
	dir.A[1][3]=-1;
	dir.A[2][3]=1;
	dir.A[3][3]=-1;
	dir.A[4][3]=1;
	dir.A[5][3]=-1;
	dir.A[6][3]=1;
	dir.A[7][3]=0;
	dir.A[8][3]=1;
	dir.A[9][3]=-1;
	dir.A[10][3]=1;
	dir.A[11][3]=-1;
	dir.A[12][3]=0;

	dir.A[0][8]=-1;
	dir.A[1][8]=1;
	dir.A[2][8]=-1;
	dir.A[3][8]=1;
	dir.A[4][8]=-1;
	dir.A[5][8]=1;
	dir.A[6][8]=-1;
	dir.A[7][8]=0;
	dir.A[8][8]=-1;
	dir.A[9][8]=1;
	dir.A[10][8]=-1;
	dir.A[11][8]=1;
	dir.A[12][8]=0;
	memset(dir.b, 0, 13U * sizeof(double));
	for(int i=0;i<4;++i)
	{
		dir.b[i+3]=umax[i];
		dir.b[i+8]=-umin[i];
	}
	dir.b[7]=50;
	for(int i=0;i<3;++i)
	{
		dir.Ad5[i]  = -v[i];
		dir.Ad10[i] =  v[i];
	}
	for(int i=0;i<13;++i)
	{
		double temp1=0;
		double temp2=0;
		for(int k=0;k<13;++k)
		{
			temp1 += P_inv[i][k]*dir.Ad5[k];
			temp2 += P_inv[i][k]*dir.Ad10[k];
		}
		dir.A[i][4]=temp1;
		dir.A[i][9]=temp2;
	}
	memset(dir.basis, 0, 13U * sizeof(int));
	for(int i=0;i<13;++i)
	{
		if(i<3)
			dir.basis[i]  = i;
		else
			dir.basis[i]  = i+7;
	}
	memset(dir.c, 0, 20U * sizeof(double));
	dir.c[4]=-1.0f;
	dir.c[9]= 1.0f;
	dir.L=0;
	dir.z=0;
	dir.iters=0;
	dir.e=0;
	//循环计算单纯形表
	while(check_c(dir.c, &(dir.e)))
	{

		dir.theta_min=DBL_MAX;
		for(int i=0;i<13;++i)
		{
			if(dir.A[i][dir.e]>0)
			{
				dir.temp=dir.b[i]/dir.A[i][dir.e];
				if(dir.temp<dir.theta_min)
				{
					dir.theta_min=dir.temp;
					dir.L=i;
				}
			}
		}
		if(dir.theta_min==DBL_MAX)
			break;
		else
		{
			double xishu=dir.A[dir.L][dir.e];
			dir.b[dir.L]=dir.b[dir.L]/xishu;
			for(int i=0;i<20;++i)
			{
				dir.A[dir.L][i]  = dir.A[dir.L][i] / xishu;
			}

			for(int i=0;i<13;++i)
			{
				if(i!=dir.L)
				{
					xishu=dir.A[i][dir.e];
					dir.b[i]=dir.b[i]-xishu*dir.b[dir.L];
					for(int j=0;j<20;++j)
					{
						dir.A[i][j]=dir.A[i][j]-xishu*dir.A[dir.L][j];
					}
				}
			}
			dir.z=dir.z-dir.c[dir.e]*dir.b[dir.L];
			xishu=dir.c[dir.e];
			for(int i=0;i<20;++i)
			{
				dir.c[i]  = dir.c[i] - xishu* dir.A[dir.L][i];
			}
			dir.basis[dir.L]=dir.e;
		}
		dir.iters +=  1;
	}
	memset(dir.x, 0, 20U * sizeof(double));
	for(int i=0;i<13;++i)
	{
		dir.x[dir.basis[i]]  = dir.b[i];
	}
	for(int i=0;i<4;++i)
	{
		u[i]  = dir.x[i]-dir.x[i+5];
		if(dir.z>1)
			u[i]  /=dir.z;
	}
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void DIR_ALLOC::dir_alloc_mch_initialize(void)
{
	dir.p_limits = this->p_limits;//单位：弧度
	for(int i=0;i<4;i++)
	{
		dir.u[i]=0;
		dir.umin[i]= -dir.p_limits;//单位：弧度
		dir.umax[i]= dir.p_limits;
	}
}


//----------------------------------------------------------by mch
bool DIR_ALLOC::check_limits(double u[4])
{
	for(int i=0; i<4; ++i){
		if(abs(u[i] - this->p_limits) <= 1e-3) return false;
	}
	return true;
}

void DIR_ALLOC::two_dir_alloc_mch(double v_T[3], double v_D[3], double u[4])
{
	// v_T	扰动补偿
	// v_D  动态力矩
	// p_limits  舵机最大偏转角度（弧度）
	// u  舵量（弧度）
	double v[3]{0.0};
	double u_v[4]{0.0};
	double u_v_T[4]{0.0};
	double u_v_D[4]{0.0};
	int i;
	int err;
	float rho;
	float vf[3]{0.0};
	float u_vf[4]{0.0};
	float u_v_Tf[4]{0.0};
	float u_v_Df[4]{0.0};
	float v_Tf[3], v_Df[3];

	for(int i=0;i<3;i++)
	{
		v[i] = v_T[i] + v_D[i];		//总舵量
		v_Tf[i] = (float)v_T[i];	//扰动补偿副本
		v_Df[i] = (float)v_D[i];	//动态力矩副本
		vf[i] = v_Tf[i] + v_Df[i];	//总舵量副本

	}
	dir_alloc_mch(v, dir.umin, dir.umax, u_v);  // 先计算合力矩所需舵量

//	err = 0;
//	for(int i=0; i<4; ++i) {
//		df_4.upperLimits[i] = dir.umin[i];
//		df_4.lowerLimits[i] = dir.umax[i];
//	}
//    Allocator.DPscaled_LPCA(vf, u_vf, err, rho);
//    for(int i=0; i<4 ;++i) u_v[i] = (double)u_vf[i];

	if(check_limits(u_v))  //若舵量可以满足则直接使用
	{
		memcpy(u, u_v, 4 * sizeof(double));
	}

	else	//否则再计算扰动所需舵量
	{
		dir_alloc_mch(v_T, dir.umin, dir.umax, u_v_T);

//		err = 0;
//	    Allocator.DPscaled_LPCA(v_Tf, u_v_Tf, err, rho);
//	    for(int i=0; i<4 ;++i) u_v_T[i] = (double)u_v_Tf[i];

		if(check_limits(u_v_T))	//若扰动可满足，合力矩不能满足，则进行两次分配
		{
			for (i = 0; i < 4; i++)
			{
				dir.umin_res[i] = dir.umin[i] - u_v_T[i];
				dir.umax_res[i] = dir.umax[i] - u_v_T[i];
//				dir.umin_res[i] = fConstrain(dir.umin_res[i], dir.umin[i], dir.umax[i]);
//				dir.umax_res[i] = fConstrain(dir.umax_res[i], dir.umin[i], dir.umax[i]);
			}
			dir_alloc_mch(v_D, dir.umin_res, dir.umax_res, u_v_D);

//			err = 0;
//			for(int i=0; i<4; ++i) {
//				df_4.upperLimits[i] = dir.umin_res[i];
//				df_4.lowerLimits[i] = dir.umax_res[i];
//			}
//		    Allocator.DPscaled_LPCA(v_Df, u_v_Df, err, rho);
//		    for(int i=0; i<4 ;++i) u_v_D[i] = (double)u_v_Df[i];

			for (i = 0; i < 4; i++)
			{
				u[i] = u_v_T[i] + u_v_D[i];
			}
		}
		else	//扰动也不能满足，可以按照合力矩进行分配，也可以按照扰动补偿进行
		{
//			for (i = 0; i < 4; i++)
//			{
//				u[i]=u_v[i];
//			}
			memcpy(u, u_v, 4 * sizeof(double));

		}
	}
}
//-------------------------------------------------------------------------------
/*
 * File trailer for dir_alloc_mch.c
 *
 * [EOF]
 */

void DIR_ALLOC::AllocHex_Init()
{
	float temp_B[3][6] ={ {0.25f,      0.25f,      0.0f,       -0.25f,     -0.25f,      0.0f},
						   {-0.125f,    0.125f,     0.25f,     0.125f,    -0.125f,    -0.25f},
						   { 0.1667f,   0.1667f,    0.1667f,   0.1667f,    0.1667f,    0.1667f}};

	/* Loop over the array to initialize each element. */
  for (u8 idx0 = 0; idx0 < 3; idx0++) {
    for (u8 idx1 = 0; idx1 < 6; idx1++) {
      /* Set the value of the array element.
         Change this value to the value that the application requires. */
      alloc.B[idx0 + 3 * idx1] = temp_B[idx0][idx1];
    }
  }

  for (u8 i = 0; i < 6; i++) {
      alloc.umax[i] = (CS_LIMIT_MAX/RAD_TO_PWM);
	  alloc.umin[i] = -(CS_LIMIT_MAX/RAD_TO_PWM);
    }
}

void DIR_ALLOC::AllocHex_Calc(double output[3], double output1[3], double output2[3])
{
	// controller output
	for (int i = 0; i < 3; i++) {
      alloc.v[i] = output[i];
	  // and maybe calc:
	 alloc.indi_fb[i]  = output1[i];
	 alloc.error_fb[i] = output2[i];
    }

	//-------------PCA------------------
	float u_all[6];//control surface
	float z_all;//optim value
	short iters_all;//is the number of iteration of simplex algorithm loop
//	getTimer_us(&startTimer);
    dir_alloc_six(alloc.umin, alloc.umax, alloc.v, alloc.B, u_all, &z_all, &iters_all); //just use dir_alloc_six dir = dir
//	getTimer_us(&stopTimer);
//	executionTime_us = stopTimer - startTimer;

	if (z_all>1)
	{
		for (u8 i = 0; i < 6; i++)
		{
			alloc.u[i] = fConstrain(u_all[i], (alloc.umin[i]),  (alloc.umax[i]));
		}
	}
	else
	{
		float u_e[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
		float z_e;
		short iters_e;
		dir_alloc_six(alloc.umin, alloc.umax, alloc.indi_fb, alloc.B,u_e, &z_e, &iters_e);
		for (u8 i = 0; i < 3; i++)
		{
			float  temp =  0.0f;
			for(int k = 0 ; k < 6 ; k++)
			{
				temp += alloc.B[i+3*k] * u_e[k];
			}
		}
		if (z_e>1)
		{
			float uMin_new[6];
			float uMax_new[6];
			for (u8 i = 0; i < 6; i++)
			{
				uMin_new[i] = alloc.umin[i] - u_e[i];
				uMax_new[i] = alloc.umax[i] - u_e[i];
			}
			float u_d[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
			float z_d;
			short iters_d;
			dir_alloc_six(uMin_new, uMax_new, alloc.error_fb, alloc.B,u_d, &z_d, &iters_d);
			for (u8 i = 0; i < 3; i++)
			{
				float  temp = 0.0f;
				for(int k = 0 ; k < 6 ; k++)
				{
					temp += alloc.B[i+3*k] * u_d[k];
				}
			}
			for (u8 i = 0; i < 6; i++)
			{
				alloc.u[i] = fConstrain( (u_d[i] + u_e[i]),  (alloc.umin[i]), (alloc.umax[i]));
			}
		}
		else
		{
			for (u8 i = 0; i < 6; i++)
			{
				alloc.u[i] = fConstrain( u_e[i],  (alloc.umin[i]),  (alloc.umax[i]));
			}
		}
	}

}

void DIR_ALLOC::dir_alloc_six(const float umin[6], const float umax[6], const float v[3],
                   const float B[18], float u[6], float *z, short *iters)
{
  static float A[476];
  static float P[289];
  static float b_B[289];
  static float Ad[238];
  static float Ad_eye[238];
  static float c[28];
  static float x[28];
  static const signed char iv[196] = {
      1, 0, 0,  0,  0, 0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0,
      0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, -1, 0,  0,
      0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, -1, 0,  0, 0, 0, 0,  0,  0,
      1, 0, 0,  0,  0, 0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0,
      0, 0, -1, 0,  0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, -1, -1, 0,
      0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0,
      1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0,
      0, 0, 0,  -1, 0, 0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, 0,  -1, 0,
      0, 0, 0,  0,  0, 1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0,
      1, 0, 0,  0,  0, 0, 0, 0,  -1, 0, 0, 0, 0,  0,  0, 1};
  static const signed char iv1[196] = {
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv3[28] = {0, 0, 0, 0, 0, 0, -1, 0, 0, 0,
                                      0, 0, 0, 1, 0, 0, 0,  0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0,  0};
  static const signed char iv2[17] = {1,  2,  3,  15, 16, 17, 18, 19, 20,
                                      21, 22, 23, 24, 25, 26, 27, 28};
  static signed char b_I[289];
  float b_A[28];
  float Aeq[21];
  float b[17];
  float f;
  float temp2;
  float tmp_ii;
  int Aeq_tmp;
  int P_tmp;
  int b_P_tmp;
  int b_div_i_tmp;
  int div_i_tmp;
  int exitg1;
  int i;
  int jj;
  short L;
  short e;
  signed char c_B[17];
  bool exitg2;
  bool flag;
  /*  function [u,z,iters] = dir_alloc_six(umin,umax,v,A,P_inv) */
  /* b求解线性规划 */
  b[0] = 0.0F;
  b[1] = 0.0F;
  b[2] = 0.0F;
  b[9] = 20.0F;
  for (i = 0; i < 6; i++) {
    Aeq[3 * i] = B[3 * i];
    Aeq_tmp = 3 * i + 1;
    Aeq[Aeq_tmp] = B[Aeq_tmp];
    Aeq_tmp = 3 * i + 2;
    Aeq[Aeq_tmp] = B[Aeq_tmp];
    b[i + 3] = umax[i];
    b[i + 10] = -umin[i];
  }
  Aeq[18] = -v[0];
  Aeq[19] = -v[1];
  Aeq[20] = -v[2];
  b[16] = 0.0F;
  /*  构造线性规划标准型 */
  /*  Convert free variables to positively constrained variables */
  for (div_i_tmp = 0; div_i_tmp < 7; div_i_tmp++) {
    f = Aeq[3 * div_i_tmp];
    Ad[17 * div_i_tmp] = f;
    Aeq_tmp = 17 * (div_i_tmp + 7);
    Ad[Aeq_tmp] = -f;
    f = Aeq[3 * div_i_tmp + 1];
    Ad[17 * div_i_tmp + 1] = f;
    Ad[Aeq_tmp + 1] = -f;
    f = Aeq[3 * div_i_tmp + 2];
    Ad[17 * div_i_tmp + 2] = f;
    Ad[Aeq_tmp + 2] = -f;
  }
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      Ad[(Aeq_tmp + 17 * div_i_tmp) + 3] = iv[Aeq_tmp + 14 * div_i_tmp];
    }
  }
  /*  Ad只有第7、第14列的前三行根据v不同而不同，其他固定不变 */
  /*  Ad=[B -v -B v; eye(7) -eye(7);-eye(7) eye(7)]; */
  /*   Ad7=[-v;0;0;0;0;0;0;1;0;0;0;0;0;0;-1]; */
  /*  [mad,~]= size(Ad); */
  /*  先把前三个等式的基找到，并化简 */
  for (div_i_tmp = 0; div_i_tmp < 3; div_i_tmp++) {
    P[17 * div_i_tmp] = Ad[17 * div_i_tmp];
    P_tmp = 17 * div_i_tmp + 1;
    P[P_tmp] = Ad[P_tmp];
    P_tmp = 17 * div_i_tmp + 2;
    P[P_tmp] = Ad[P_tmp];
  }
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    P_tmp = 17 * (div_i_tmp + 3);
    P[P_tmp] = 0.0F;
    P[P_tmp + 1] = 0.0F;
    P[P_tmp + 2] = 0.0F;
  }
  for (div_i_tmp = 0; div_i_tmp < 3; div_i_tmp++) {
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      P_tmp = (Aeq_tmp + 17 * div_i_tmp) + 3;
      P[P_tmp] = Ad[P_tmp];
    }
  }
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      P[(Aeq_tmp + 17 * (div_i_tmp + 3)) + 3] = iv1[Aeq_tmp + 14 * div_i_tmp];
    }
  }
  /*  常量 */
  /*  P_inv=[ -2    -4     6     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           6     4    -6     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*          -4     0     6     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           2     4    -6     1     0     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*          -6    -4     6     0     1     0     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           4     0    -6     0     0     1     0     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     1     0     0     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     0     1     0     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     0     0     1     0     0
   * 0     0     0     0     0     0; */
  /*           0     0     0     0     0     0     0     0     0     1     0
   * 0     0     0     0     0     0; */
  /*          -2    -4     6     0     0     0     0     0     0     0     1
   * 0     0     0     0     0     0; */
  /*           6     4    -6     0     0     0     0     0     0     0     0
   * 1     0     0     0     0     0; */
  /*          -4     0     6     0     0     0     0     0     0     0     0
   * 0     1     0     0     0     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     1     0     0     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     0     1     0     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     0     0     1     0; */
  /*           0     0     0     0     0     0     0     0     0     0     0
   * 0     0     0     0     0     1]; */
  /*  求逆 */
  /*  Ad_eye=P\Ad;% 化简 */
  /*  无关列的逆阵P_inv是常矩阵 */
  /*  对矩阵进行初等行变换求其逆 */
  /*  function Ad_eye=inv_mvh(B_inv,Ad) */
  /*  Ad_eye=B_inv\Ad;% 化简 */
  /*  [row, col] = size(A); */
  /*  B为单位矩阵 */
  memset(&b_I[0], 0, 289U * sizeof(signed char));
  for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
    b_I[Aeq_tmp + 17 * Aeq_tmp] = 1;
  }
  for (div_i_tmp = 0; div_i_tmp < 289; div_i_tmp++) {
    b_B[div_i_tmp] = b_I[div_i_tmp];
  }
  for (i = 0; i < 17; i++) {
    /*  依次将对角行的元素归一化 */
    b_div_i_tmp = 17 * i;
    div_i_tmp = i + b_div_i_tmp;
    tmp_ii = P[div_i_tmp];
    for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
      P_tmp = i + 17 * Aeq_tmp;
      P[P_tmp] /= tmp_ii;
      b_B[P_tmp] /= tmp_ii;
    }
    for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
      tmp_ii = -P[Aeq_tmp + b_div_i_tmp] / P[div_i_tmp];
      if (i + 1 == Aeq_tmp + 1) {
        tmp_ii = 0.0F;
      }
      /*  初等行变换 */
      for (jj = 0; jj < 17; jj++) {
        P_tmp = 17 * jj;
        b_P_tmp = Aeq_tmp + P_tmp;
        P_tmp += i;
        P[b_P_tmp] += tmp_ii * P[P_tmp];
        b_B[b_P_tmp] += tmp_ii * b_B[P_tmp];
      }
    }
  }
  /*  常量 */
  /*  Ad_eye =[ */
  /*   */
  /*      1.0000   -0.0000   -0.0000    1.0000    2.0000    2.0000  0   -1.0000
   * 0.0000    0.0000   -1.0000   -2.0000   -2.0000  0; */
  /*      0.0000    1.0000    0.0000   -2.0000   -3.0000   -2.0000  0   -0.0000
   * -1.0000   -0.0000    2.0000    3.0000    2.0000  0; */
  /*     -0.0000   -0.0000    1.0000    2.0000    2.0000    1.0000  0    0.0000
   * 0.0000   -1.0000   -2.0000   -2.0000   -1.0000  0; */
  /*           0    0.0000    0.0000   -1.0000   -2.0000   -2.0000  0         0
   * -0.0000   -0.0000    1.0000    2.0000    2.0000  0; */
  /*     -0.0000         0   -0.0000    2.0000    3.0000    2.0000  0    0.0000
   * 0    0.0000   -2.0000   -3.0000   -2.0000  0; */
  /*      0.0000    0.0000         0   -2.0000   -2.0000   -1.0000  0   -0.0000
   * -0.0000         0    2.0000    2.0000    1.0000  0; */
  /*           0         0         0    1.0000         0         0  0         0
   * 0         0   -1.0000         0         0  0; */
  /*           0         0         0         0    1.0000         0  0         0
   * 0         0         0   -1.0000         0  0; */
  /*           0         0         0         0         0    1.0000  0         0
   * 0         0         0         0   -1.0000  0; */
  /*           0         0         0         0         0         0  0         0
   * 0         0         0         0         0  0; */
  /*           0   -0.0000   -0.0000    1.0000    2.0000    2.0000  0         0
   * 0.0000    0.0000   -1.0000   -2.0000   -2.0000  0; */
  /*      0.0000         0    0.0000   -2.0000   -3.0000   -2.0000  0   -0.0000
   * 0   -0.0000    2.0000    3.0000    2.0000  0; */
  /*     -0.0000   -0.0000         0    2.0000    2.0000    1.0000  0    0.0000
   * 0.0000         0   -2.0000   -2.0000   -1.0000  0; */
  /*           0         0         0   -1.0000         0         0  0         0
   * 0         0    1.0000         0         0  0; */
  /*           0         0         0         0   -1.0000         0  0         0
   * 0         0         0    1.0000         0  0; */
  /*           0         0         0         0         0   -1.0000  0         0
   * 0         0         0         0    1.0000  0; */
  /*           0         0         0         0         0         0  0         0
   * 0         0         0         0         0  0]; */
  /*   Ad_eye1=[ */
  /*      1.0000   -0.0000   -0.0000    1.0000    2.0000    2.0000  ; */
  /*      0.0000    1.0000    0.0000   -2.0000   -3.0000   -2.0000  ; */
  /*     -0.0000   -0.0000    1.0000    2.0000    2.0000    1.0000  ; */
  /*           0    0.0000    0.0000   -1.0000   -2.0000   -2.0000  ; */
  /*     -0.0000         0   -0.0000    2.0000    3.0000    2.0000  ; */
  /*      0.0000    0.0000         0   -2.0000   -2.0000   -1.0000  ; */
  /*           0         0         0    1.0000         0         0  ; */
  /*           0         0         0         0    1.0000         0  ; */
  /*           0         0         0         0         0    1.0000  ; */
  /*           0         0         0         0         0         0  ; */
  /*           0   -0.0000   -0.0000    1.0000    2.0000    2.0000  ; */
  /*      0.0000         0    0.0000   -2.0000   -3.0000   -2.0000  ; */
  /*     -0.0000   -0.0000         0    2.0000    2.0000    1.0000  ;  */
  /*           0         0         0   -1.0000         0         0  ; */
  /*           0         0         0         0   -1.0000         0  ; */
  /*           0         0         0         0         0   -1.0000  ; */
  /*           0         0         0         0         0         0  ]; */
  /*   Ad_eye2 =[ */
  /*          -1.0000    0.0000    0.0000   -1.0000   -2.0000   -2.0000  ; */
  /*          -0.0000   -1.0000   -0.0000    2.0000    3.0000    2.0000  ; */
  /*           0.0000    0.0000   -1.0000   -2.0000   -2.0000   -1.0000  ; */
  /*                0   -0.0000   -0.0000    1.0000    2.0000    2.0000  ; */
  /*           0.0000         0    0.0000   -2.0000   -3.0000   -2.0000  ; */
  /*          -0.0000   -0.0000         0    2.0000    2.0000    1.0000  ; */
  /*                0         0         0   -1.0000         0         0  ; */
  /*                0         0         0         0   -1.0000         0  ; */
  /*                0         0         0         0         0   -1.0000  ; */
  /*                0         0         0         0         0         0  ; */
  /*                0    0.0000    0.0000   -1.0000   -2.0000   -2.0000  ; */
  /*          -0.0000         0   -0.0000    2.0000    3.0000    2.0000  ; */
  /*           0.0000    0.0000         0   -2.0000   -2.0000   -1.0000  ; */
  /*                0         0         0    1.0000         0         0  ; */
  /*                0         0         0         0    1.0000         0  ; */
  /*                0         0         0         0         0    1.0000  ; */
  /*                0         0         0         0         0         0  ]; */
  /*  根据以上分析，P_inv*Ad只有第5，第10列依v不同而变化。 */
  for (i = 0; i < 17; i++) {
    for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
      f = 0.0F;
      for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
        f += b_B[i + 17 * Aeq_tmp] * Ad[Aeq_tmp + 17 * div_i_tmp];
      }
      Ad_eye[i + 17 * div_i_tmp] = f;
    }
    tmp_ii = 0.0F;
    temp2 = 0.0F;
    for (Aeq_tmp = 0; Aeq_tmp < 17; Aeq_tmp++) {
      f = b_B[i + 17 * Aeq_tmp];
      tmp_ii += f * Ad[Aeq_tmp + 102];
      temp2 += f * Ad[Aeq_tmp + 221];
      /*          temp1=temp1 + P_inv(i,k)*Ad7(k); */
      /*          temp2=temp2 + P_inv(i,k)*-Ad7(k); */
    }
    Ad_eye[i + 102] = tmp_ii;
    Ad_eye[i + 221] = temp2;
    /*      Ak1(i)=temp1; */
    /*      Ak2(i)=temp2; */
  }
  /*  加上松弛变量对应的基 */
  for (div_i_tmp = 0; div_i_tmp < 14; div_i_tmp++) {
    A[17 * div_i_tmp] = Ad_eye[17 * div_i_tmp];
    b_P_tmp = 17 * (div_i_tmp + 14);
    A[b_P_tmp] = 0.0F;
    b_div_i_tmp = 17 * div_i_tmp + 1;
    A[b_div_i_tmp] = Ad_eye[b_div_i_tmp];
    A[b_P_tmp + 1] = 0.0F;
    b_div_i_tmp = 17 * div_i_tmp + 2;
    A[b_div_i_tmp] = Ad_eye[b_div_i_tmp];
    A[b_P_tmp + 2] = 0.0F;
    for (Aeq_tmp = 0; Aeq_tmp < 14; Aeq_tmp++) {
      b_div_i_tmp = (Aeq_tmp + 17 * div_i_tmp) + 3;
      A[b_div_i_tmp] = Ad_eye[b_div_i_tmp];
      A[(Aeq_tmp + b_P_tmp) + 3] = iv1[Aeq_tmp + 14 * div_i_tmp];
    }
  }
  /*  A1=[zeros(3,14); eye(14)]; */
  /*  A=[Ad_eye1 Ak1 Ad_eye2 Ak2 A1] */
  /*  A是Ad_eye的扩充，第7，第14列与P_inv*Ad变化的部分列有关，其他是常数 */
  /*  A(:,7)=Ak1; */
  /*  A(:,14)=Ak2; */
  /*  转C需要特别注意下标的区别 */
  /*  Simplex algorithm */
  /*  Iterate through simplex algorithm main loop */
  for (div_i_tmp = 0; div_i_tmp < 17; div_i_tmp++) {
    c_B[div_i_tmp] = iv2[div_i_tmp];
  }
  /*  (c) mengchaoheng */
  /*  不考虑无解的情形 */
  /*  Last edited 2019-11 */
  /*    min z=c*x   subj. to  A*x (=、 >=、 <=) b */
  /*    x  */
  /*     %% Initialization */
  /*  Iterate through simplex algorithm main loop */
  for (i = 0; i < 28; i++) {
    c[i] = iv3[i];
    x[i] = 0.0F;
  }
  *iters = 0;
  *z = 0.0F;
  /*      [m,n] = size(A); */
  /*      while ~all(c>=0)                      % 3.~isempty(c(c(N)<0)) */
  /*      e = find(c < 0, 1, 'first'); % 进基变量索引    % 4. e =
   * N(find(c(N)<0,1)) */
  do {
    exitg1 = 0;
    flag = false;
    e = 0;
    L = 0;
    i = 0;
    exitg2 = false;
    while ((!exitg2) && (i < 28)) {
      if (c[i] < -1.0E-6F) {
        /*  <0 */
        flag = true;
        e = (short)(i + 1);
        exitg2 = true;
      } else {
        i++;
      }
    }
    if (flag) {
      /*              a_ie=A(:,e); */
      /*              ip=a_ie>(1/tol); */
      /*              delta=tol*ones(m,1); */
      /*              if ~isempty(ip) */
      /*                  delta(ip)=b(ip)./a_ie(ip); */
      /*              end */
      tmp_ii = 1.0E+6F;
      for (i = 0; i < 17; i++) {
        f = A[i + 17 * (e - 1)];
        if (f > 1.0E-6F) {
          f = b[i] / f;
        } else {
          f = 1.0E+6F;
        }
        if (f < tmp_ii) {
          L = (short)(i + 1);
          tmp_ii = f;
        }
      }
      /*              [~,L]=min(delta);%选择离基 (离基在B数组中的行索引) */
      /*          li = B(L);    % 离基变量索引                */
      /*              if delta(L) >= tol     */
      if (tmp_ii >= 1.0E+6F) {
        exitg1 = 1;
      } else {
        /*  此时一定有一个L */
        /*  (c) mengchaoheng */
        /*  Last edited 2019-11 */
        /*    min z=c*x   subj. to  A*x (=、 >=、 <=) b */
        /*    x  */
        /*     %% Compute the coefficients of the equation for new basic
         * variabLe x_e. */
        /*      [m, n] = size(A); */
        /*  row of Leaving var    L = find(B==li,1); */
        /*  Perform pivot operation, exchanging L-row with e-coLumn variabLe */
        jj = 17 * (e - 1);
        tmp_ii = A[(L + jj) - 1];
        b[L - 1] /= tmp_ii;
        /*  4. */
        for (div_i_tmp = 0; div_i_tmp < 28; div_i_tmp++) {
          b_A[div_i_tmp] = A[(L + 17 * div_i_tmp) - 1] / tmp_ii;
        }
        for (div_i_tmp = 0; div_i_tmp < 28; div_i_tmp++) {
          A[(L + 17 * div_i_tmp) - 1] = b_A[div_i_tmp];
        }
        /*     %% Compute the coefficients of the remaining constraints. */
        /*      i=[1:L-1 L+1:m];     %  i = find(B~=li); */
        /*      if ~isempty(i) */
        /*          b(i) = b(i) - A(i,e)*b(L); */
        /*          A(i,1:n) = A(i,1:n) - A(i,e)*A(L,1:n);	 */
        /*      end */
        f = b[L - 1];
        for (i = 0; i < 17; i++) {
          if (i + 1 != L) {
            tmp_ii = A[i + jj];
            b[i] -= tmp_ii * f;
            for (Aeq_tmp = 0; Aeq_tmp < 28; Aeq_tmp++) {
              b_P_tmp = 17 * Aeq_tmp;
              b_div_i_tmp = i + b_P_tmp;
              A[b_div_i_tmp] -= tmp_ii * A[(L + b_P_tmp) - 1];
            }
          }
        }
        /*     %% Compute the objective function */
        tmp_ii = c[e - 1];
        *z -= tmp_ii * b[L - 1];
        for (Aeq_tmp = 0; Aeq_tmp < 28; Aeq_tmp++) {
          c[Aeq_tmp] -= tmp_ii * A[(L + 17 * Aeq_tmp) - 1];
        }
        /*      c(1:n) = c(1:n) - c_e * A(L,1:n);       */
        /*     %% Compute new sets of basic and nonbasic variabLes. */
        /*  N(find(N==e,1)) = li;  */
        c_B[L - 1] = (signed char)e;
        /*   B(find(B==li,1)) = e; */
        /* 换基，即进行初等行变换 */
        (*iters)++;
      }
    } else {
      for (div_i_tmp = 0; div_i_tmp < 17; div_i_tmp++) {
        x[c_B[div_i_tmp] - 1] = b[div_i_tmp];
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  /*  线性规划单纯形法 */
  /*  [x,z,iters]=Simplex_loop_mch(basis, A, b, c, z); */
  for (i = 0; i < 6; i++) {
    u[i] = x[i] - x[i + 7];
  }
  if (*z > 1.0F) {
    /*  放大了倍数，再还原，若小于1，则表示需要缩小，x已经自然到达边界 */
    for (div_i_tmp = 0; div_i_tmp < 6; div_i_tmp++) {
      u[div_i_tmp] /= *z;
    }
  }
}
