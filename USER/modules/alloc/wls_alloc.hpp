/*
 * WLS_ALLOC.hpp
 *
 *  Created on: 2020年9月2日
 *      Author: 17900
 */

#ifndef WLS_ALLOC_HPP_
#define WLS_ALLOC_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "motor/motor.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	double u[4];//舵量，单位：弧度
	double p_limits;//舵机幅值，单位：度

}sWls;


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus


#include <iostream>
#include <Dense>
using namespace std;
using namespace Eigen;

class WLS_ALLOC
{

public:
	WLS_ALLOC(){}
	~WLS_ALLOC(){}

	void wls_alloc_mch(const double v[3], const double p_limits,unsigned char v_limits, double u[4]);
	void wls_alloc_mch_initialize(void);
	void wls_alloc_mch_terminate(void);
	void xzlarf(int m, int n, int iv0, double tau, double C_data[], int ic0,double work_data[]);
	double xnrm2(int n, const double x_data[], int ix0);
	double rt_hypotd(double u0, double u1);
	void rdivide(const double x_data[], const int x_size[1], const double y_data[], double z_data[], int z_size[1]);
	void qrsolve(const double A_data[], const int A_size[2], const double B[7],double Y_data[], int Y_size[1]);
	void LSQFromQR(const double A_data[], const int A_size[2], const double tau_data[], const int jpvt_data[], double B[7], int rankA,double Y_data[], int Y_size[1]);

	sWls wls;

private:

};


#endif





#endif /* WLS_ALLOC_HPP_ */
