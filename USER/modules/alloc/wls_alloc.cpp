/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : wls_alloc.cpp
  * Description        :
  ******************************************************************************
  * Function           :
  *
  ******************************************************************************
  * Author             : Zhiping Fu
  * Creation Date      : Oct 22, 2020
  ******************************************************************************
  */
/* USER CODE END Header */
/*
 * File: wls_alloc_mch.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 23-Nov-2019 12:19:19
 */
 /*
 * File: dir_alloc_mch.c
 * 参考Efficient Active Set Algorithms for Solving Constrained Least Squares Problems in Aircraft Control Allocation
 * 参考网址 http://research.harkegard.se/qcat/
 * 由matlab代码生成
	Generated on:	23-Nov-2019 12:19:25
	Build type:	Static Library
	Output file:	C:\Users\mengc\Desktop\????matlab?C??_11_20\????\codegen\lib\wls_alloc_mch\wls_alloc_mch.lib
	Processor:	ARM Compatible->ARM Cortex
	Version:	MATLAB Coder 4.0 (R2018a)
 * 重新生成记得在头文件添加sWls结构体和extern sWls wls，包含#include "userlib.h"
 * 往 wls_alloc_mch_initialize 添加初始化操作，在c文件添加sWls wls;
 * 最后在调需要的地方调用 wls_alloc_mch_initialize ，和 wls_alloc_mch ，
 * 目前分别在 Control_Init ， Output_To_Motor 调用
 * 修改motor,eso,control使包含正确头文件，舵机幅值，正确扰动补偿量
 */
#include <alloc/wls_alloc.hpp>

/* Function Definitions */

/*
 * Arguments    : const double A_data[]
 *                const int A_size[2]
 *                const double tau_data[]
 *                const int jpvt_data[]
 *                double B[7]
 *                int rankA
 *                double Y_data[]
 *                int Y_size[1]
 * Return Type  : void
 */
void WLS_ALLOC::LSQFromQR(const double A_data[], const int A_size[2], const double
                      tau_data[], const int jpvt_data[], double B[7], int rankA,
                      double Y_data[], int Y_size[1])
{
  int loop_ub;
  int i;
  double wj;
  Y_size[0] = (signed char)A_size[1];
  loop_ub = (signed char)A_size[1];
  if (0 <= loop_ub - 1) {
    memset(&Y_data[0], 0, (unsigned int)(loop_ub * (int)sizeof(double)));
  }

  for (loop_ub = 0; loop_ub < A_size[1]; loop_ub++) {
    if (tau_data[loop_ub] != 0.0) {
      wj = B[loop_ub];
      for (i = loop_ub + 1; i + 1 < 8; i++) {
        wj += A_data[i + A_size[0] * loop_ub] * B[i];
      }

      wj *= tau_data[loop_ub];
      if (wj != 0.0) {
        B[loop_ub] -= wj;
        for (i = loop_ub + 1; i + 1 < 8; i++) {
          B[i] -= A_data[i + A_size[0] * loop_ub] * wj;
        }
      }
    }
  }

  for (i = 0; i < rankA; i++) {
    Y_data[jpvt_data[i] - 1] = B[i];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[loop_ub + A_size[0] * loop_ub];
    for (i = 0; i < loop_ub; i++) {
      Y_data[jpvt_data[i] - 1] -= Y_data[jpvt_data[loop_ub] - 1] * A_data[i +
        A_size[0] * loop_ub];
    }
  }
}

/*
 * Arguments    : const double A_data[]
 *                const int A_size[2]
 *                const double B[7]
 *                double Y_data[]
 *                int Y_size[1]
 * Return Type  : void
 */
void WLS_ALLOC::qrsolve(const double A_data[], const int A_size[2], const double B[7],double Y_data[], int Y_size[1])
{
  int b_A_size[2];
  int n;
  double b_A_data[28];
  int b_n;
  int jpvt_data[4];
  int yk;
  int k;
  double work_data[4];
  double smax;
  int i;
  double tau_data[4];
  double b_B[7];
  double s;
  int i_i;
  int nmi;
  double absxk;
  double vn1_data[4];
  double vn2_data[4];
  double t;
  int pvt;
  b_A_size[0] = 7;
  b_A_size[1] = A_size[1];
  n = A_size[0] * A_size[1];
  if (0 <= n - 1) {
    memcpy(&b_A_data[0], &A_data[0], (unsigned int)(n * (int)sizeof(double)));
  }

  b_n = A_size[1];
  if (A_size[1] < 1) {
    n = 0;
  } else {
    n = A_size[1];
  }

  if (n > 0) {
    jpvt_data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      jpvt_data[k - 1] = yk;
    }
  }

  if (A_size[1] != 0) {
    n = (signed char)A_size[1];
    if (0 <= n - 1) {
      memset(&work_data[0], 0, (unsigned int)(n * (int)sizeof(double)));
    }

    k = 1;
    for (yk = 0; yk < b_n; yk++) {
      smax = 0.0;
      s = 3.3121686421112381E-170;
      for (n = k; n <= k + 6; n++) {
        absxk = fabs(A_data[n - 1]);
        if (absxk > s) {
          t = s / absxk;
          smax = 1.0 + smax * t * t;
          s = absxk;
        } else {
          t = absxk / s;
          smax += t * t;
        }
      }

      smax = s * sqrt(smax);
      vn1_data[yk] = smax;
      vn2_data[yk] = vn1_data[yk];
      k += 7;
    }

    for (i = 0; i < b_n; i++) {
      i_i = i + i * 7;
      nmi = b_n - i;
      if (nmi < 1) {
        n = -1;
      } else {
        n = 0;
        if (nmi > 1) {
          yk = i;
          smax = fabs(vn1_data[i]);
          for (k = 0; k + 2 <= nmi; k++) {
            yk++;
            s = fabs(vn1_data[yk]);
            if (s > smax) {
              n = k + 1;
              smax = s;
            }
          }
        }
      }

      pvt = i + n;
      if (pvt + 1 != i + 1) {
        yk = 7 * pvt;
        n = 7 * i;
        for (k = 0; k < 7; k++) {
          smax = b_A_data[yk];
          b_A_data[yk] = b_A_data[n];
          b_A_data[n] = smax;
          yk++;
          n++;
        }

        n = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[i];
        jpvt_data[i] = n;
        vn1_data[pvt] = vn1_data[i];
        vn2_data[pvt] = vn2_data[i];
      }

      s = b_A_data[i_i];
      absxk = 0.0;
      smax = xnrm2(6 - i, b_A_data, i_i + 2);
      if (smax != 0.0) {
        smax = rt_hypotd(b_A_data[i_i], smax);
        if (b_A_data[i_i] >= 0.0) {
          smax = -smax;
        }

        if (fabs(smax) < 1.0020841800044864E-292) {
          yk = 0;
          n = (i_i - i) + 7;
          do {
            yk++;
            for (k = i_i + 1; k < n; k++) {
              b_A_data[k] *= 9.9792015476736E+291;
            }

            smax *= 9.9792015476736E+291;
            s *= 9.9792015476736E+291;
          } while (!(fabs(smax) >= 1.0020841800044864E-292));

          smax = rt_hypotd(s, xnrm2(6 - i, b_A_data, i_i + 2));
          if (s >= 0.0) {
            smax = -smax;
          }

          absxk = (smax - s) / smax;
          s = 1.0 / (s - smax);
          n = (i_i - i) + 7;
          for (k = i_i + 1; k < n; k++) {
            b_A_data[k] *= s;
          }

          for (k = 1; k <= yk; k++) {
            smax *= 1.0020841800044864E-292;
          }

          s = smax;
        } else {
          absxk = (smax - b_A_data[i_i]) / smax;
          s = 1.0 / (b_A_data[i_i] - smax);
          n = (i_i - i) + 7;
          for (k = i_i + 1; k < n; k++) {
            b_A_data[k] *= s;
          }

          s = smax;
        }
      }

      tau_data[i] = absxk;
      b_A_data[i_i] = s;
      if (i + 1 < b_n) {
        s = b_A_data[i_i];
        b_A_data[i_i] = 1.0;
        xzlarf(7 - i, nmi - 1, i_i + 1, tau_data[i], b_A_data, (i + (i + 1) * 7)
               + 1, work_data);
        b_A_data[i_i] = s;
      }

      for (yk = i + 1; yk < b_n; yk++) {
        if (vn1_data[yk] != 0.0) {
          smax = fabs(b_A_data[i + 7 * yk]) / vn1_data[yk];
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = smax * (s * s);
          if (s <= 1.4901161193847656E-8) {
            vn1_data[yk] = xnrm2(6 - i, b_A_data, (i + 7 * yk) + 2);
            vn2_data[yk] = vn1_data[yk];
          } else {
            vn1_data[yk] *= sqrt(smax);
          }
        }
      }
    }
  }

  n = 0;
  if (A_size[1] > 0) {
    smax = 7.0 * fabs(b_A_data[0]) * 2.2204460492503131E-16;
    while ((n < b_A_size[1]) && (!(fabs(b_A_data[n + 7 * n]) <= smax))) {
      n++;
    }
  }

  for (i = 0; i < 7; i++) {
    b_B[i] = B[i];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, b_B, n, Y_data, Y_size);
}

/*
 * Arguments    : const double x_data[]
 *                const int x_size[1]
 *                const double y_data[]
 *                double z_data[]
 *                int z_size[1]
 * Return Type  : void
 */
void WLS_ALLOC::rdivide(const double x_data[], const int x_size[1], const double y_data[], double z_data[], int z_size[1])
{
  int loop_ub;
  int i0;
  z_size[0] = x_size[0];
  loop_ub = x_size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    z_data[i0] = x_data[i0] / y_data[i0];
  }
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
double WLS_ALLOC::rt_hypotd(double u0, double u1)
{
  double y;
  double a;
  double b;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = b * sqrt(a * a + 1.0);
  } else if (a > b) {
    b /= a;
    y = a * sqrt(b * b + 1.0);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/*
 * Arguments    : int n
 *                const double x_data[]
 *                int ix0
 * Return Type  : double
 */
double WLS_ALLOC::xnrm2(int n, const double x_data[], int ix0)
{
  double y;
  double scale;
  int kend;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabs(x_data[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/*
 * Arguments    : int m
 *                int n
 *                int iv0
 *                double tau
 *                double C_data[]
 *                int ic0
 *                double work_data[]
 * Return Type  : void
 */
void WLS_ALLOC::xzlarf(int m, int n, int iv0, double tau, double C_data[], int ic0,double work_data[])
{
  int lastv;
  int lastc;
  int i;
 unsigned char exitg2;
  int jy;
  int j;
  int i1;
  int ia;
  int exitg1;
  double c;
  int ix;
  if (tau != 0.0) {
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C_data[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      i = ic0 + (lastc - 1) * 7;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    if (lastc != 0) {
      for (i = 1; i <= lastc; i++) {
        work_data[i - 1] = 0.0;
      }

      i = 0;
      i1 = ic0 + 7 * (lastc - 1);
      for (jy = ic0; jy <= i1; jy += 7) {
        ix = iv0;
        c = 0.0;
        j = (jy + lastv) - 1;
        for (ia = jy; ia <= j; ia++) {
          c += C_data[ia - 1] * C_data[ix - 1];
          ix++;
        }

        work_data[i] += c;
        i++;
      }
    }

    if (!(-tau == 0.0)) {
      i = ic0 - 1;
      jy = 0;
      for (j = 1; j <= lastc; j++) {
        if (work_data[jy] != 0.0) {
          c = work_data[jy] * -tau;
          ix = iv0;
          i1 = lastv + i;
          for (ia = i; ia < i1; ia++) {
            C_data[ia] += C_data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        i += 7;
      }
    }
  }
}


/*
 * [u] = wls_alloc_mch(v,u,p_limits,v_limits)
 *  WLS_ALLOC - Control allocation using weighted least squares.
 *   [u,W,iter] = wls_alloc(B,v,umin,umax,[Wv,Wu,ud,gamma,u0,W0,imax])
 *  Solves the weighted, bounded least-squares problem
 *    min ||Wu(u-ud)||^2 + gamma ||Wv(Bu-v)||^2
 *    subj. to  umin <= u <= umax
 *  using an active set method.
 *   Inputs:
 *   -------
 *  v     commanded virtual control (k x 1)
 *  u0    initial point (m x 1)
 *  W0    initial working set (m x 1) [empty]
 *  imax  max no. of iterations [100]
 *   Outputs:
 *   -------
 *  u     optimal control
 *  W     optimal active set
 *  iter  no. of iterations (= no. of changes in the working set + 1)
 *                             0 if u_i not saturated
 *  Working set syntax: W_i = -1 if u_i = umin_i
 *                            +1 if u_i = umax_i
 *  B         control effectiveness matrix (k x m).
 *  umin      lower position limits (m x 1).
 *  umax      upper position limits (m x 1).
 *  Wv        virtual control weighting matrix (k x k) [I].
 *  Wu        control weighting matrix (m x m) [I].
 *  gam       gamma weight (scalar) [1e6].
 *  ud        desired control (m x 1) [0].
 *  imax      maximum iterations.
 *  See also: WLSC_ALLOC, IP_ALLOC, FXP_ALLOC, QP_SIM.
 *  param of ducted fan
 *  使用力矩单位
 *  ======================multi ducted fan param=========================
 *  kc=3.157;
 *  l_1=0.17078793-0.09;% roll,pitch
 *  l_2=0.175;% T
 *  l_3=0.06647954;% yaw1 yaw3
 *  l_4=0.06647954+0.175;% yaw4
 *  l_5=0.175-0.06647954;% yaw2
 *  I_x=0.054593;
 *  I_y=0.017045;
 *  I_z=0.049226;
 *  I=[I_x 0 0;0 I_y 0;0 0 I_z];
 *  =========================10 sureface=================================
 *  K=[-kc*l_1/5 0 kc*l_1/5 0 -kc*l_1/5 0 kc*l_1/5 0 2*l_2/5;
 *      0 -kc*l_1/4 0 kc*l_1/4 0 -kc*l_1/4 0 kc*l_1/4 0;
 *      kc*l_3/8 -kc*l_5/8 kc*l_3/8 kc*l_4/8 kc*l_3/8 kc*l_4/8 kc*l_3/8 -kc*l_5/8 0];
 *  B=I\K;
 *  umin=[[1;1;1;1;1;1;1;1]*-20*pi/180;-0.3*9.788];umax=[[1;1;1;1;1;1;1;1]*20*pi/180;0.3*9.788];
 *  Wu=eye(9);
 *  ud=zeros(9,1);
 *  % Number of variables.
 *  m = 9;
 * ===================================================
 *  =========================8 sureface=================================
 *  K=[-kc*l_1/4 0 kc*l_1/4 0 -kc*l_1/4 0 kc*l_1/4 0;
 *      0 -kc*l_1/4 0 kc*l_1/4 0 -kc*l_1/4 0 kc*l_1/4;
 *      kc*l_3/8 -kc*l_5/8 kc*l_3/8 kc*l_4/8 kc*l_3/8 kc*l_4/8 kc*l_3/8 -kc*l_5/8 ];
 *  B=I\K;
 *  umin=[1;1;1;1;1;1;1;1]*-20*pi/180;umax=[1;1;1;1;1;1;1;1]*20*pi/180;
 *  Wu=eye(8);
 *  ud=zeros(8,1);
 *  % Number of variables.
 *  m = 8;
 * ========================================
 *  K=[-kc*l_1/4    -kc*l_1/4   0           kc*l_1/4    kc*l_1/4    0;
 *      0           0           -kc*l_1/2   0           0           kc*l_1/2;
 *      kc*l_3/6    kc*l_3/6    kc*l_4/6    kc*l_3/6    kc*l_3/8    kc*l_4/6 ];
 *  B=I\K;
 *  umin=[1;1;1;1;1;1]*-20*pi/180;umax=[1;1;1;1;1;1]*20*pi/180;
 *  Wu=eye(6);
 *  ud=zeros(6,1);
 *  % Number of variables.
 *  m = 6;
 *  弧度单位
 * ==============使用等价模型，弧度单位==============
 * Arguments    : const double v[3]
 *                double u[4]
 *                double p_limits
 *                boolean_T v_limits
 * Return Type  : void
 */
void WLS_ALLOC::wls_alloc_mch(const double v[3], const double p_limits, unsigned char v_limits, double u[4])
{
  int i;
  int k;
  double umin[4];
  double dist;
  double umax[4];
  double u_opt;
  double W[4];
  int aoffset;
  double a[3];
  double b_a[7];
  static const short c_a[9] = { 1000, 0, 0, 0, 1000, 0, 0, 0, 1000 };

  double d_a[7];
  unsigned char i_free[4];
  double d[7];
  static const short e_a[28] = { -500, 0, 250, 1, 0, 0, 0, 0, -500, 250, 0, 1, 0,
    0, 500, 0, 250, 0, 0, 1, 0, 0, 500, 250, 0, 0, 0, 1 };

  int iter;
  unsigned char exitg1;
  int trueCount;
  int A_free_size[2];
  int tmp_data[4];
  double A_free_data[28];
  double p_free_data[4];
  int p_free_size[1];
  int b_trueCount;
  double b_u_opt[4];
  int b_tmp_data[4];
  unsigned char y;
  unsigned char x_data[4];
  unsigned char c_u_opt[4];
  double p[4];
  unsigned char exitg2;
  double b_dist[4];
  unsigned char bv0[4];
  int umin_size[1];
  int c_tmp_data[4];
  double p_data[4];
  double d_tmp_data[4];
  int tmp_size[1];
  static const short f_a[28] = { -500, 0, 500, 0, 0, -500, 0, 500, 250, 250, 250,
    250, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  int umax_size[1];
  int e_tmp_data[4];

  /* ====仅幅值约束================ */
  if (!v_limits) {
    for (i = 0; i < 4; i++) {
      umin[i] = -p_limits * 3.1415926535897931 / 180.0;
      umax[i] = p_limits * 3.1415926535897931 / 180.0;
    }
  } else {
    /* ====幅值、速度约束================  */
    for (k = 0; k < 4; k++) {
      dist = -p_limits * 3.1415926535897931 / 180.0;
      u_opt = -0.069813170079773182 + u[k];
      if (dist > u_opt) {
        u_opt = dist;
      }

      umin[k] = u_opt;
      dist = p_limits * 3.1415926535897931 / 180.0;
      u_opt = 0.069813170079773182 + u[k];
      if (dist < u_opt) {
        u_opt = dist;
      }

      umax[k] = u_opt;
    }
  }

  /*  */
  /* ========计算当前有效集============ */
  for (i = 0; i < 4; i++) {
    W[i] = 0.0;
    if (u[i] <= umin[i]) {
      W[i] = -1.0;
    }

    if (u[i] >= umax[i]) {
      W[i] = 1.0;
    }
  }

  /* ===============期望舵机位置======================== */
  /*  可作为输入 */
  /*  Number of variables. */
  /* ================================ */
  /*  加权系数 */
  /*  加权矩阵 */
  /* Wv1=Wv*0.5*y; */
  /*  迭代次数上限 */
  /*  Set default values of optional arguments. */
  /*  Initial residual. */
  for (aoffset = 0; aoffset < 3; aoffset++) {
    a[aoffset] = 0.0;
    for (k = 0; k < 3; k++) {
      a[aoffset] += (double)c_a[aoffset + 3 * k] * v[k];
    }

    b_a[aoffset] = a[aoffset];
  }

  for (aoffset = 0; aoffset < 4; aoffset++) {
    b_a[aoffset + 3] = 0.0;
  }

  for (aoffset = 0; aoffset < 7; aoffset++) {
    d_a[aoffset] = 0.0;
    for (k = 0; k < 4; k++) {
      d_a[aoffset] += (double)e_a[aoffset + 7 * k] * u[k];
    }

    d[aoffset] = b_a[aoffset] - d_a[aoffset];
  }

  /*  Determine indeces of free variables. */
  for (i = 0; i < 4; i++) {
    i_free[i] = (W[i] == 0.0);
  }

  /*  Iterate until optimum is found or maximum number of iterations */
  /*  is reached. */
  iter = 0;
  exitg1 = false;
  while ((!exitg1) && (iter < 100)) {
    /*  ---------------------------------------- */
    /*   Compute optimal perturbation vector p. */
    /*  ---------------------------------------- */
    /*  Eliminate saturated variables. */
    trueCount = 0;
    for (i = 0; i < 4; i++) {
      if (i_free[i]) {
        trueCount++;
      }
    }

    k = 0;
    for (i = 0; i < 4; i++) {
      if (i_free[i]) {
        tmp_data[k] = i + 1;
        k++;
      }
    }

    A_free_size[0] = 7;
    A_free_size[1] = trueCount;
    for (aoffset = 0; aoffset < trueCount; aoffset++) {
      for (k = 0; k < 7; k++) {
        A_free_data[k + 7 * aoffset] = e_a[k + 7 * (tmp_data[aoffset] - 1)];
      }
    }

    /*  Solve the reduced optimization problem for free variables. */
    if (trueCount == 0) {
      p_free_size[0] = 0;
    } else {
      qrsolve(A_free_data, A_free_size, d, p_free_data, p_free_size);
    }

    /*  Zero all perturbations corresponding to active constraints. */
    /*  Insert perturbations from p_free into free the variables. */
    k = 0;

    /*  ---------------------------- */
    /*   Is the new point feasible? */
    /*  ---------------------------- */
    b_trueCount = 0;
    for (i = 0; i < 4; i++) {
      dist = 0.0;
      if (i_free[i]) {
        dist = p_free_data[k];
        k++;
      }

      b_u_opt[i] = u[i] + dist;
      if (i_free[i]) {
        b_trueCount++;
      }

      p[i] = dist;
    }

    k = 0;
    for (i = 0; i < 4; i++) {
      if (i_free[i]) {
        b_tmp_data[k] = i + 1;
        k++;
      }

      c_u_opt[i] = ((b_u_opt[i] < umin[i]) || (b_u_opt[i] > umax[i]));
    }

    for (aoffset = 0; aoffset < b_trueCount; aoffset++) {
      x_data[aoffset] = c_u_opt[b_tmp_data[aoffset] - 1];
    }

    y = false;
    k = 1;
    exitg2 = false;
    while ((!exitg2) && (k <= b_trueCount)) {
      if (x_data[k - 1]) {
        y = true;
        exitg2 = true;
      } else {
        k++;
      }
    }

    if (!y) {
      /*  ---------------------------- */
      /*   Yes, check for optimality. */
      /*  ---------------------------- */
      /*  Update point and residual. */
      for (i = 0; i < 4; i++) {
        u[i] = b_u_opt[i];
      }

      if ((trueCount == 1) || (p_free_size[0] == 1)) {
        for (aoffset = 0; aoffset < 7; aoffset++) {
          b_a[aoffset] = 0.0;
          for (k = 0; k < trueCount; k++) {
            b_a[aoffset] += A_free_data[aoffset + 7 * k] * p_free_data[k];
          }
        }
      } else {
        for (i = 0; i < 7; i++) {
          b_a[i] = 0.0;
        }

        for (k = 0; k < trueCount; k++) {
          if (p_free_data[k] != 0.0) {
            aoffset = k * 7;
            for (i = 0; i < 7; i++) {
              b_a[i] += p_free_data[k] * (double)(short)A_free_data[aoffset + i];
            }
          }
        }
      }

      for (aoffset = 0; aoffset < 7; aoffset++) {
        d[aoffset] -= b_a[aoffset];
      }

      /*  判别推出循环条件 */
      /*          if norm(p)<=eps */
      /*  Compute Lagrangian multipliers. */
      for (aoffset = 0; aoffset < 4; aoffset++) {
        b_u_opt[aoffset] = 0.0;
        for (k = 0; k < 7; k++) {
          b_u_opt[aoffset] += (double)f_a[aoffset + (k << 2)] * d[k];
        }

        b_dist[aoffset] = W[aoffset] * b_u_opt[aoffset];
      }

      /*  Are all lambda non-negative? */
      y = true;
      k = 1;
      exitg2 = false;
      while ((!exitg2) && (k < 5)) {
        if (!(b_dist[k - 1] >= -2.2204460492503131E-16)) {
          y = false;
          exitg2 = true;
        } else {
          k++;
        }
      }

      if (y) {
        /*  / ------------------------ \ */
        /*  | Optimum found, bail out. | */
        /*  \ ------------------------ / */
        exitg1 = true;
      } else {
        /*  -------------------------------------------------- */
        /*   Optimum not found, remove one active constraint. */
        /*  -------------------------------------------------- */
        /*  Remove constraint with most negative lambda from the */
        /*  working set. */
        dist = b_dist[0];
        b_trueCount = 0;
        for (k = 0; k < 3; k++) {
          if (dist > b_dist[k + 1]) {
            dist = b_dist[k + 1];
            b_trueCount = k + 1;
          }
        }

        W[b_trueCount] = 0.0;
        i_free[b_trueCount] = true;

        /*          end */
        iter++;
      }
    } else {
      /*  --------------------------------------- */
      /*   No, find primary bounding constraint. */
      /*  --------------------------------------- */
      /*  Compute distances to the different boundaries. Since alpha < 1 is */
      /*  the maximum step length, initiate with ones. */
      b_trueCount = 0;
      for (i = 0; i < 4; i++) {
        b_dist[i] = 1.0;
        y = (p[i] < 0.0);
        bv0[i] = (p[i] > 0.0);
        if (i_free[i] && y) {
          b_trueCount++;
        }

        c_u_opt[i] = y;
      }

      k = 0;
      for (i = 0; i < 4; i++) {
        if (i_free[i] && c_u_opt[i]) {
          c_tmp_data[k] = i + 1;
          k++;
        }
      }

      umin_size[0] = b_trueCount;
      for (aoffset = 0; aoffset < b_trueCount; aoffset++) {
        b_u_opt[aoffset] = umin[c_tmp_data[aoffset] - 1] - u[c_tmp_data[aoffset]
          - 1];
      }

      for (aoffset = 0; aoffset < b_trueCount; aoffset++) {
        p_data[aoffset] = p[c_tmp_data[aoffset] - 1];
      }

      rdivide(b_u_opt, umin_size, p_data, d_tmp_data, tmp_size);
      k = 0;
      b_trueCount = 0;
      for (i = 0; i < 4; i++) {
        if (i_free[i] && c_u_opt[i]) {
          b_dist[i] = d_tmp_data[k];
          k++;
        }

        if (i_free[i] && bv0[i]) {
          b_trueCount++;
        }
      }

      k = 0;
      for (i = 0; i < 4; i++) {
        if (i_free[i] && bv0[i]) {
          e_tmp_data[k] = i + 1;
          k++;
        }
      }

      umax_size[0] = b_trueCount;
      for (aoffset = 0; aoffset < b_trueCount; aoffset++) {
        b_u_opt[aoffset] = umax[e_tmp_data[aoffset] - 1] - u[e_tmp_data[aoffset]
          - 1];
      }

      for (aoffset = 0; aoffset < b_trueCount; aoffset++) {
        p_data[aoffset] = p[e_tmp_data[aoffset] - 1];
      }

      rdivide(b_u_opt, umax_size, p_data, d_tmp_data, tmp_size);
      k = 0;
      for (i = 0; i < 4; i++) {
        if (i_free[i] && bv0[i]) {
          b_dist[i] = d_tmp_data[k];
          k++;
        }
      }

      /*  Proportion of p to travel */
      dist = b_dist[0];
      b_trueCount = 0;
      for (k = 0; k < 3; k++) {
        if (dist > b_dist[k + 1]) {
          dist = b_dist[k + 1];
          b_trueCount = k + 1;
        }
      }

      /*  Update point and residual. */
      for (aoffset = 0; aoffset < 4; aoffset++) {
        u[aoffset] += dist * p[aoffset];
      }

      /*  Update point and residual. */
      k = 7 * trueCount - 1;
      for (aoffset = 0; aoffset <= k; aoffset++) {
        A_free_data[aoffset] *= dist;
      }

      if ((trueCount == 1) || (p_free_size[0] == 1)) {
        for (aoffset = 0; aoffset < 7; aoffset++) {
          b_a[aoffset] = 0.0;
          for (k = 0; k < trueCount; k++) {
            b_a[aoffset] += A_free_data[aoffset + 7 * k] * p_free_data[k];
          }
        }
      } else {
        for (i = 0; i < 7; i++) {
          b_a[i] = 0.0;
        }

        for (k = 0; k < trueCount; k++) {
          if (p_free_data[k] != 0.0) {
            aoffset = k * 7;
            for (i = 0; i < 7; i++) {
              b_a[i] += p_free_data[k] * A_free_data[aoffset + i];
            }
          }
        }
      }

      for (aoffset = 0; aoffset < 7; aoffset++) {
        d[aoffset] -= b_a[aoffset];
      }

      /*  Add corresponding constraint to working set. */
      dist = p[b_trueCount];
      if (p[b_trueCount] < 0.0) {
        dist = -1.0;
      } else {
        if (p[b_trueCount] > 0.0) {
          dist = 1.0;
        }
      }

      W[b_trueCount] = dist;
      i_free[b_trueCount] = false;
      iter++;
    }
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void WLS_ALLOC::wls_alloc_mch_initialize(void)
{
	for(int i=0;i<4;i++)
	{
		wls.u[i]=0;
		wls.p_limits = (CS_LIMIT_MAX/RAD_TO_PWM)*180.0f/3.1415926535897931f;//单位：度
	}
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void WLS_ALLOC::wls_alloc_mch_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for wls_alloc_mch.c
 *
 * [EOF]
 */


/************************ (C) COPYRIGHT Longmen Drone Team *****END OF FILE****/

