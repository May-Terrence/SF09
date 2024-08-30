/*
 * fillets.hpp
 *
 *  Created on: 2022年1月1日
 *      Author: 刘成吉
 */

#ifndef MODULES_PATH_FOLLOW_FILLETS_HPP_
#define MODULES_PATH_FOLLOW_FILLETS_HPP_
#include "MSG_PATH.hpp"

/*
 *  对外接口：
 * Path			: 返回的曲线格式采取与DUBINS一致的形式，每段曲线要么是圆弧(lambda = ±1), 要么是直线(lambda = 0)
 * (w, n)		: 共同定义dubins曲线中每一段线段的末端半平面，用于判断目标是否已完成一段曲线的飞行
 * w			: 半平面与dubins曲线的交点
 * n			: 半平面的法向量， 若有一点x， 如果， (x-w)·n >= 0 表示已越过半平面，完成某段线段的飞行，否则在半平面背面，未完成飞行
 * n			: 若n = (0,0), 即n.norm()==0, 则该段曲线段无末端半平面(无终点)，可用于标志最后一段曲线，使飞机在最后一段圆弧盘旋
 * fillet_planing: 对外函数接口， 计算出鱼眼曲线，直线间的拐角用圆弧过度， 并放入链表paths中
 * waypoints 	: 二维数组，每行代表一个节点
 * waypoints[i] = (x , y, z)， 不含航迹角
 * paths		: Path为元素的链表，每个节点表示一段直线段或圆弧
 */

namespace FILLETS{
	typedef typename MSG_PATH::Path Path;
	void fillet_planing(const std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths);
}
#endif /* MODULES_PATH_FOLLOW_FILLETS_HPP_ */
