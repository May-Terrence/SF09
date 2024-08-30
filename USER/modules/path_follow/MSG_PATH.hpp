/*
 * MSG_PATH.hpp
 *
 *  Created on: 2022年5月14日
 *      Author: 刘成吉
 */

#ifndef MODULES_PATH_FOLLOW_MSG_PATH_HPP_
#define MODULES_PATH_FOLLOW_MSG_PATH_HPP_

#include <Eigen>

namespace MSG_PATH
{
// 定义路径消息类型
/*
 **  			   每段曲线要么是圆弧(lambda = ±1), 要么是直线(lambda = 0)
 * (w, n)		: 共同定义dubins曲线中每一段线段的末端半平面，用于判断目标是否已完成一段曲线的飞行
 * w			: 半平面与dubins曲线的交点
 * n			: 半平面的法向量， 若有一点x， 如果， (x-w)·n >= 0 表示已越过半平面，完成某段线段的飞行，否则在半平面背面，未完成飞行
 * n			: 若n = (0,0), 即n.norm()==0, 则该段曲线段无末端半平面(无终点)，可用于标志最后一段曲线，使飞机在最后一段圆弧盘旋
 * rho 			: 转弯半径
 */
	struct Path
	{
		int8_t lambda;	// direction of turning (+1 for CW, -1 for CCW, 0 for straight line)
		float rho;		// radius of orbit
		float zs;		// the z component(Down) at the start node of a path segment
		float ze;		// the z component at the end node of a path segment
		float len;		// the length of the projection of a path segment on x-y plane
		Eigen::Vector2f r;		// inertial position of start of waypoint path
		Eigen::Vector2f q;		// unit vector that defines inertial direction of waypoint path
		Eigen::Vector2f c;		// center of orbit
		Eigen::Vector2f w;		// vector defining half plane H at the end of a path segment
		Eigen::Vector2f n;		// unit vector defining direction of half plane H, n is in the horizontal plane
		Path(const float rho_): lambda(0), rho(rho_), zs(0), ze(0), len(0),
				r(0,0), q(0,0), c(0,0), w(0,0), n(0,0) {}
		Path():lambda(0), rho(87.0), zs(0), ze(0), len(0),
			r(0,0), q(0,0), c(0,0), w(0,0), n(0,0) {}
	};
}


#endif /* MODULES_PATH_FOLLOW_MSG_PATH_HPP_ */
