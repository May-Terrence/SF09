/*
 * filters.hpp
 *
 *  Created on: 2022年5月21日
 *      Author: Caspar Leo
 */

#ifndef DRIVERS_USERLIB_FILTERS_HPP_
#define DRIVERS_USERLIB_FILTERS_HPP_

class ALPHA_FILTER
{
public:
	ALPHA_FILTER(float alpha_, float Ts_):alpha(alpha_), Ts(Ts_),  filtered_value(0), lambda((Ts*alpha+Ts)/(2-2*alpha)) {}
	ALPHA_FILTER(float Ts_):alpha(0), Ts(Ts_),  filtered_value(0), lambda((Ts*alpha+Ts)/(2-2*alpha)) {}
	ALPHA_FILTER():alpha(0), Ts(0.001),  filtered_value(0), lambda((Ts*alpha+Ts)/(2-2*alpha)) {}

	float update(const float raw_val);
	void set_Ts(float Ts_);
	void set_alpha(float alpha_);
	void set_lambda(float lambda_);
	float get_val();
private:
	/*
	 * if used as dirty derivative, 1/lambda defines the bandwidth of the differentiator.
	 * alpha   = (2*lambda - Ts)/(2*lambda + Ts)
	 * 1-alpha = 2*Ts/(2*lambda + Ts)
	 */
	float alpha;
	float Ts;
	float filtered_value;
	float lambda;
};

namespace INDI_FILTER {
	void set_lambda(float lam);
	void set_gyro_Ts(float Ts);
	void set_ctrl_Ts(float Ts);
	void get_dw_fltr(float* arr);
	void get_ctrl_fltr(float* arr);
	void update_dw_fltr(const float* gyro);
	void update_ctrl_fltr(const float* u);
}

#endif /* DRIVERS_USERLIB_FILTERS_HPP_ */
