/*
 * simpaths.cpp
 *
 *  Created on: 2022年1月3日
 *      Author: 刘成吉
 */

#include "simpaths.hpp"

namespace SIMPATHS{
namespace{ /* Anonymous namespace */

	Vector2f rotz(const Vector2f& v, const float theta) {	// rotate v by theta about the axis of z
		Vector2f res;
		res[0] = cos(theta)*v[0] - sin(theta)*v[1];
		res[1] = sin(theta)*v[0] + cos(theta)*v[1];
		return res;
	}

	void orbit(const std::vector<float>& centor, const float rho, const int lambda, std::list<Path>& paths) {
		Path path(rho);
		path.lambda = lambda;
		path.c[0] 	= centor[0];
		path.c[1]	= centor[1];
		path.n 		<< 0, 0;
		path.w 		<< centor[0] + rho, centor[1];
		path.zs 	= centor[2];
		path.ze 	= centor[2];
		path.len 	= rho*2*M_PI;
		paths.push_back(path);
	}

	void straight_orbit(const std::vector<std::vector<float>>& nodes, const float rho, const float lambda, std::list<Path>& paths) {
		Path path(rho);
		path.lambda = 0;
		path.r 		<< nodes[0][0], nodes[0][1];
		path.q 		<< nodes[1][0] - nodes[0][0], nodes[1][1] - nodes[0][1];
		path.len 	= path.q.norm();
		path.q.normalize();
		path.n 		=  path.q;
		path.w 		<< nodes[1][0], nodes[1][1];
		path.zs 	=  nodes[0][2];
		path.ze 	=  nodes[1][2];
		paths.push_back(path);

		Vector2f c;
		if(lambda == 1){
			c = path.w + rho*rotz(path.n,  M_PI/2);
		}else{
			c = path.w + rho*rotz(path.n, -M_PI/2);
		}
		std::vector<float> centor({c[0], c[1], path.ze});
		orbit(centor, rho, lambda, paths);
	}

} /* Anonymous namespace */
//
void left_orbit(const std::vector<float>& centor, const float rho, std::list<Path>& paths){
	/* 左转圆盘旋(CCW)
	 * centor	: (x, y, z) 圆心坐标
	 * rho 		: 转弯半径
	 * paths	: 输出路径(仅有一个圆)
	 */
	paths.clear();
	if(centor.size() < 3) return;
	orbit(centor, rho, -1, paths);
}

void right_orbit(const std::vector<float>& centor, const float rho, std::list<Path>& paths){
	/* 右转圆盘旋(CW)
	 * centor	: (x, y, z) 圆心坐标
	 * rho 		: 转弯半径
	 * paths	: 输出路径(仅有一个圆)
	 */
	paths.clear();
	if(centor.size() < 3) return;
	orbit(centor, rho, 1, paths);
}

void straight_line_CW(const std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths) {
	/* 两点间的直线，飞到终点时在终点盘旋(顺时针右转圆)
	 * waypoints: (x1 y1 z1; x2 y2 z2), 直线上两端点的坐标
	 * rho 		: 转弯半径
	 * paths	: 输出路径链表(仅有两个节点，分别是直线和终点圆)
	 */
	paths.clear();
	if(waypoints.size() < 2 || waypoints[0].size() != 3) return;
	straight_orbit(waypoints, rho, 1, paths);
}

void straight_line_CCW(const std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths) {
	/* 两点间的直线，飞到终点时在终点盘旋(逆时针左转圆)
	 * waypoints: (x1 y1 z1; x2 y2 z2), 直线上两端点的坐标
	 * rho 		: 转弯半径
	 * paths	: 输出路径链表(仅有两个节点，分别是直线和终点圆)
	 */
	paths.clear();
	if(waypoints.size() < 2 || waypoints[0].size() != 3) return;
	straight_orbit(waypoints, rho, -1, paths);
}

} /* namespace SIMPATHS */
