/*
 * utility.hpp
 *
 *  Created on: 2020年12月10日
 *      Author: 刘成吉
 */

#ifndef DRIVERS_USERLIB_UTILITY_HPP_
#define DRIVERS_USERLIB_UTILITY_HPP_


#ifdef __cplusplus
#include <Eigen>
using namespace std;
using namespace Eigen;

Quaternion<float> Expqua(const Vector3f& theta);	//四元数指数映射
Vector3f Logqua(const Quaternion<float>& q);		//四元数对数映射
Matrix3f crossM(const Vector3f& vec);
Vector3f qua2eul(const Quaternion<float>& q);
Quaternion<float> eul2qua(const Vector3f& eul);


inline Quaternion<float> Expqua(const Vector3f& theta)
{
	float Q = theta.norm();
	Vector3f theta_norm;
	theta_norm = theta.normalized();
	Quaternion<float> uq(cos(Q),theta_norm[0]*sin(Q),theta_norm[1]*sin(Q),theta_norm[2]*sin(Q));
	return uq;
}
inline Vector3f Logqua(const Quaternion<float>& q)
{
	Vector3f qv(q.x(), q.y(), q.z());
	float Q = atan(qv.norm()/q.w());

	qv.normalize();//等效单位旋转向量
	return (Q*qv);
}
inline Vector3f qua2eul(const Quaternion<float>& q)
{
	Vector3f ea;//roll,pitch,yaw
	ea[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()),1-2*(q.x()*q.x()+q.y()*q.y()));
	ea[1] = asin(2*(q.w()*q.y()-q.x()*q.z()));
	ea[2] = atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
	return ea;
}

inline Quaternion<float> eul2qua(const Vector3f& eul)
{
//	欧拉角转四元数
	float cr = cos(eul[0]/2);
	float sr = sin(eul[0]/2);
	float cp = cos(eul[1]/2);
	float sp = sin(eul[1]/2);
	float cy = cos(eul[2]/2);
	float sy = sin(eul[2]/2);
	Quaternion<float> q;
	q.w() = cr*cp*cy+sr*sp*sy;
	q.x() = sr*cp*cy-cr*sp*sy;
	q.y() = cr*sp*cy+sr*cp*sy;
	q.z() = cr*cp*sy-sr*sp*cy;
	return q;
}

inline Matrix3f crossM(const Vector3f& vec)
{
	Matrix3f M(3,3);
	M << 0,     -vec[2], vec[1],
		vec[2],    0,   -vec[0],
	   -vec[1],	 vec[0],  0;
	return M;
}

#endif
#endif /* DRIVERS_USERLIB_UTILITY_HPP_ */
