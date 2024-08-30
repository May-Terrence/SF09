/*
 * fillets.cpp
 *
 *  Created on: 2022年1月1日
 *      Author: 刘成吉
 */

#include "fillets.hpp"
#include <Eigen>
using namespace Eigen;

namespace FILLETS{
namespace{	/* Anonymous namespace */
	Vector2f q_pre;
	Vector2f q_nxt;
	float z_HP[3]; // the point where the half plane locates to tell whether a curve segment is finished;

	Vector2f rotz(const Vector2f& v, const float theta) {	// rotate v by theta about the axis of z
		Vector2f res;
		res[0] = cos(theta)*v[0] - sin(theta)*v[1];
		res[1] = sin(theta)*v[0] + cos(theta)*v[1];
		return res;
	}

	bool fillet_cal(const std::vector<float>& cur, const std::vector<float>& nxt, float rho, std::list<Path>& paths){
		for(int i = 0; i < 2; ++i){
			q_pre[i] = cur[i] - z_HP[i];
			q_nxt[i] = nxt[i] - cur[i];
		}
		float zdelta_pre = cur[2] - z_HP[2];
		float zdelta_nxt = nxt[2] - cur[2];
		float norm1 = q_pre.norm();
		float norm2 = q_nxt.norm();
		q_pre.normalize();
		q_nxt.normalize();

		float innerPro = 0;
		for(int i = 0; i < 2; ++i){
			innerPro += -q_pre[i]*q_nxt[i];
		}

		if( 1.0 - innerPro  < 1e-2){ 	// 拐角近乎为0， 不可行
			return false;
		}

		float varrho = acosf(innerPro);
		if(M_PI - varrho < 1e-3){	// 三点近乎一条直线,则把两段线段合并，即把中间点(当前点cur)忽略，这种情况不用做其他处理，可直接返回
			return true;
		}

		float censor = rho/tan(varrho/2);

		if(censor > norm1 || censor > norm2){
			return false;
		}

		Path path(rho);
		/* z_HP(前一点) 到 cur 的直线段 */
		path.lambda = 0;
		path.r 		<< z_HP[0], z_HP[1];
		path.zs 	= z_HP[2];
		path.ze 	= cur[2] - zdelta_pre*censor/norm1;
		path.len 	= norm1 - censor;
		path.q 		= q_pre;
		path.n 		= path.q;
		path.w[0]	= cur[0] - censor*q_pre[0];
		path.w[1]  	= cur[1] - censor*q_pre[1];
		paths.push_back(path);

		/* 中间点(cur) 上的过度圆弧 */
		Vector2f bisector = (q_pre - q_nxt);
		bisector.normalize();
		bisector   *= rho/sin(varrho/2);
		path.c[0] 	= cur[0] - bisector[0];
		path.c[1]	= cur[1] - bisector[1];
		path.n 		= q_nxt;
		path.w[0]	= cur[0] + censor*q_nxt[0];
		path.w[1] 	= cur[1] + censor*q_nxt[1];
		path.zs 	= path.ze;
		path.ze 	= cur[2] + zdelta_nxt*censor/norm2;
		path.len 	= rho*(M_PI-varrho);
		z_HP[0]		= path.w[0];
		z_HP[1]		= path.w[1];
		z_HP[2] 	= path.ze;

		if(q_pre[0]*q_nxt[1]-q_pre[1]*q_nxt[0] > 0){
			path.lambda = 1;
		}else{
			path.lambda = -1;
		}
		paths.push_back(path);
		return true;
	}
} /* Anonymous namespace */

void fillet_planing(const std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths){
	/* waypoints 	: 二维数组，每行代表一个节点
	 * waypoints[i] = (x , y, z), 表示航点位置(NED坐标系)，无航迹角
	 * rho 			: 转弯半径
	 * paths 		: 输出路径链表(每个节点是代表一段圆弧或直线)
	 */
	paths.clear();
	int num = waypoints.size();
	if(num < 2 || waypoints[0].size() != 3){	// 最少需要两节点
		return;
	}
	z_HP[0] = waypoints[0][0];
	z_HP[1] = waypoints[0][1];
	z_HP[2] = waypoints[0][2];
	for(int i = 1; i < num - 1; ++i){
		if(!fillet_cal(waypoints[i], waypoints[i+1], rho, paths)){
			paths.clear();	// 航点不可行
			return;
		}
	}

	Path path(rho);
	/*最后一段直线*/
	path.lambda = 0;
	path.r		<< z_HP[0], z_HP[1];
	path.zs  	= z_HP[2];
	path.ze 	= waypoints.back()[2];
	path.q[0] 	= waypoints.back()[0] - z_HP[0];
	path.q[1] 	= waypoints.back()[1] - z_HP[1];
	path.len 	= path.q.norm();
	path.q.normalize();
	path.n 		= path.q;
	path.w[0] 	= waypoints.back()[0];
	path.w[1] 	= waypoints.back()[1];
	paths.push_back(path);

	/* 在终点接一个圆弧，使飞机在终点盘旋 */
	Vector2f cre(path.w + rho*rotz(path.n,  M_PI/2));
	Vector2f cle(path.w + rho*rotz(path.n, -M_PI/2));
	if(cre.norm() < cle.norm()){	// 右转离原点更近
		path.lambda = 1;
		path.c 		= cre;
		path.n 		<< 0, 0;		// 无末端半平面，即飞机永远飞不出这个半平面，实现飞机不断在终点圆上盘旋
		path.zs 	= path.ze;
		path.len 	= rho*2*M_PI;	// 因在此圆上不断盘旋，所以这里的path.len无实际意义
		paths.push_back(path);
	}else{
		path.lambda = -1;
		path.c 		= cle;
		path.n 		<< 0, 0;		// 无末端半平面，即飞机永远飞不出这个半平面，实现飞机不断在终点圆上盘旋
		path.zs 	= path.ze;
		path.len 	= rho*2*M_PI;
		paths.push_back(path);
	}
}

} /* namespace FILLETS */
