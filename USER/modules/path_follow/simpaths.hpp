/*
 * simpaths.hpp
 *
 *  Created on: 2022年1月3日
 *      Author: 刘成吉
 *       Some simple paths
 */

#ifndef MODULES_PATH_FOLLOW_SIMPATHS_HPP_
#define MODULES_PATH_FOLLOW_SIMPATHS_HPP_

# include "dubins.hpp"
# include "MSG_PATH.hpp"

namespace SIMPATHS{
/*
 *  对外接口：
 * Path			: 返回的曲线格式采取与DUBINS一致的形式，每段曲线要么是圆弧(lambda = ±1), 要么是直线(lambda = 0)
 * (w, n)		: 共同定义dubins曲线中每一段线段的末端半平面，用于判断目标是否已完成一段曲线的飞行
 * w			: 半平面与dubins曲线的交点
 * n			: 半平面的法向量， 若有一点x， 如果， (x-w)·n >= 0 表示已越过半平面，完成某段线段的飞行，否则在半平面背面，未完成飞行
 * n			: 若n = (0,0), 即n.norm()==0, 则该段曲线段无末端半平面(无终点)，可用于标志最后一段曲线，使飞机在最后一段圆弧盘旋
 *
 * left_orbit/right_orbit
 * centor		: 一维数组，centor = (x, y, z) 代表圆心坐标
 * rho 			: 圆弧半径
 *
 * straight_line_CW/straight_line_CCW
 * waypoints 	: 仅有两行的二维数组，每行代表一个节点，仅有两点，飞到末端时在终点盘旋(CW: 顺时针左转， CCW:逆时针右转)
 *
 * paths		: Path为元素的链表，每个节点表示一段直线段或圆弧
 */
	typedef typename MSG_PATH::Path Path;
//	typedef typename DUBINS::DubinsPath Path;
	void left_orbit(const std::vector<float>& centor, const float rho, std::list<Path>& paths);
	void right_orbit(const std::vector<float>& centor, const float rho, std::list<Path>& paths);
	void straight_line_CW(const std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths);
	void straight_line_CCW(const std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths);
}



#endif /* MODULES_PATH_FOLLOW_SIMPATHS_HPP_ */
