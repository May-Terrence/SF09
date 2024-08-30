/*
 * filters.cpp
 *
 *  Created on: 2022年5月21日
 *      Author: caspar Leo
 */


#include "filters.hpp"
#include "userlib/userlib.hpp"

float ALPHA_FILTER::update(const float raw_val){
	filtered_value = alpha*filtered_value + (1-alpha)*raw_val;
	return filtered_value;
}

void ALPHA_FILTER::set_Ts(float Ts_) {
	Ts = Ts_;
}

void ALPHA_FILTER::set_alpha(float alpha_){
	alpha = alpha_;
	lambda = (Ts*alpha+Ts)/(2-2*alpha);
}


void ALPHA_FILTER::set_lambda(float lambda_) {
	lambda = lambda_;
	alpha = (2*lambda - Ts)/(2*lambda + Ts);
}

float ALPHA_FILTER::get_val() {
	return filtered_value;
}

namespace INDI_FILTER {
namespace{
	ALPHA_FILTER dw_flt[3];
	ALPHA_FILTER ctrl_flt[3];
	float gyro_Ts = 0.001;
	float last_gyro[3] = {0.0, 0.0, 0.0};
}
	void set_lambda(float lam) {
		for(u8 i = 0; i < 3; ++i) {
			dw_flt[i].set_lambda(lam);
			ctrl_flt[i].set_lambda(lam);
		}
	}

	void set_gyro_Ts(float Ts) {
		for(u8 i = 0; i < 3; ++i) {
			dw_flt[i].set_Ts(Ts);
		}
		gyro_Ts = Ts;
	}

	void set_ctrl_Ts(float Ts) {
		for(u8 i = 0; i < 3; ++i) {
			ctrl_flt[i].set_Ts(Ts);
		}
	}

	void get_dw_fltr(float* arr) {
		// arr 是三维数组
		for(u8 i = 0; i < 3; ++i) {
			arr[i] = dw_flt[i].get_val();
		}
	}

	void get_ctrl_fltr(float* arr) {
		// arr 是三维数组
		for(u8 i = 0; i < 3; ++i) {
			arr[i] = ctrl_flt[i].get_val();
		}
	}

	void update_dw_fltr(const float* gyro) {
		// gyro 是三维数组
		for(u8 i = 0; i < 3; ++i) {
			dw_flt[i].update( (gyro[i] - last_gyro[i])/gyro_Ts );
			last_gyro[i] = gyro[i];
		}
	}

	void update_ctrl_fltr(const float* u) {
		// u 是三维数组
		for(u8 i = 0; i < 3; ++i) {
			ctrl_flt[i].update(u[i]);
		}
	}
}
