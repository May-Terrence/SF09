/*
 * dubins.hpp
 *
 *  Created on: 2021年12月27日
 *      Author: 刘成吉
 */

#ifndef MODULES_PATH_FOLLOW_DUBINS_HPP_
#define MODULES_PATH_FOLLOW_DUBINS_HPP_
#include <list>
#include <vector>
#include <Eigen>
#include <memory>
#include "MSG_PATH.hpp"

using namespace Eigen;


namespace DUBINS
{
/*
 *  对外接口：
 * dubins_planing: 计算出六种dubins曲线中最短的一种，每种dubins曲线由三段线段组成， 三段线段分别由DubinsPath存储表示， 并放入链表paths中
 * waypoints 	: 二维数组，每行代表一个节点
 * waypoints[i] = (x , y, z)或(x, y, z, chi), 前三个是北东地位置，第四个(可选) 是飞机在节点时的航迹角
 * rho 			: 转弯半径
 * paths		: DubinsPath为元素的链表，每种有效的dubins曲线会占据三个链表节点(DubinsPath)
 */
	typedef typename MSG_PATH::Path Path;
	void dubins_planing(std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths);
}

#endif /* MODULES_PATH_FOLLOW_DUBINS_HPP_ */
