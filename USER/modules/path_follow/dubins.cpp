/*
 * dubins.cpp
 *
 *  Created on: 2021年12月27日
 *      Author: 刘成吉
 */

#include "dubins.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include "userlib/userlib.hpp"

namespace DUBINS{

namespace{ //匿名空间
	/*
	 *  以下物理量若涉及长度， 则以转弯半径为单位（dubin曲线中圆弧归一化为单位圆）
	 */
	float t = 0;				// 第一段圆弧弧度
	float p = 0;				// 第二段圆弧弧度/直线长度
	float q = 0;				// 第三段圆弧弧度
	float dist = 3.4028235E38;	// dubins曲线总长
	enum Dubins_set
	{
		LSL, LSR, RSL, RSR, RLR, LRL, INVALID
	};
	Dubins_set path_type = INVALID;

	Vector2f rotz(const Vector2f& v, const float theta)
	{	// rotate v by theta about the axis of z
		Vector2f res;
		res[0] = cos(theta)*v[0] - sin(theta)*v[1];
		res[1] = sin(theta)*v[0] + cos(theta)*v[1];
		return res;
	}

	void dubins_LSL(const float alpha, const float beta, const float d)
	{
		float tmp0 = d + sin(alpha) - sin(beta);
	    float p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(alpha) - sin(beta)));
	    if( p_squared < 0.0f )
	    	return;
	    else{
			float t_lsl = -alpha + atan2((cos(beta)-cos(alpha)), tmp0 );
			mod2PI(&t_lsl);
			float p_lsl = sqrt( p_squared );
			float q_lsl = beta - alpha - t_lsl;
			mod2PI(&q_lsl);
			float dist_temp = t_lsl + p_lsl + q_lsl;
			if(dist > dist_temp){
				t = t_lsl;
				p = p_lsl;
				q = q_lsl;
				dist = dist_temp;
				path_type = LSL;
				return;
			}
	    }
	}

	void dubins_LSR(const float alpha, const float beta, const float d)
	{
	    float p_squared = -2 + (d*d) + (2*cos(alpha - beta)) + (2*d*(sin(alpha)+sin(beta)));
	    if( p_squared < 0.0f )
	    	return;
	    else{
			float p_lsr = sqrt( p_squared );
			float t_lsr = -alpha + atan2( (-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)) ) - atan2(-2.0, p_lsr);
			mod2PI(&t_lsr);
			float q_lsr = alpha + t_lsr - beta;
			mod2PI(&q_lsr);
			float dist_temp = t_lsr + p_lsr + q_lsr;
			if(dist>dist_temp){
				t = t_lsr;
				p = p_lsr;
				q = q_lsr;
				dist = dist_temp;
				path_type = LSR;
				return;
			}
	    }
	}

	void dubins_RSL(const float alpha, const float beta, const float d){
	    float p_squared = (d*d) -2 + (2*cos(alpha - beta)) - (2*d*(sin(alpha)+sin(beta)));
	    if( p_squared < 0.0f )
	    	return;
	    else{
			float p_rsl = sqrt( p_squared );
			float t_rsl = alpha - atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta)) ) + atan2(2.0, p_rsl);
			mod2PI(&t_rsl);
			float q_rsl = beta - alpha + t_rsl;
			mod2PI(&q_rsl);
			float dist_temp = t_rsl + p_rsl + q_rsl;
			if(dist>dist_temp){
				t = t_rsl;
				p = p_rsl;
				q = q_rsl;
				dist = dist_temp;
				path_type = RSL;
				return;
			}
	    }
	}

	void dubins_RSR(const float alpha, const float beta, const float d){
		float tmp0 = d-sin(alpha)+sin(beta);
	    float p_squared = 2 + (d*d) -(2*cos(alpha - beta)) + (2*d*(sin(beta)-sin(alpha)));
	    if( p_squared < 0.0f )
	    	return;
	    else{
			float t_rsr = alpha - atan2( (cos(alpha)-cos(beta)), tmp0 );
			mod2PI(&t_rsr);
			float p_rsr = sqrt( p_squared );
			float q_rsr = alpha - beta - t_rsr;
			mod2PI(&q_rsr);
			float dist_temp = t_rsr + p_rsr + q_rsr;
			if(dist>dist_temp){
				t = t_rsr;
				p = p_rsr;
				q = q_rsr;
				dist = dist_temp;
				path_type = RSR;
				return;
			}
	    }
	}

	void dubins_RLR(const float alpha, const float beta, const float d){
		float tmp_rlr = (6.0f - d*d + 2*cos(alpha - beta) + 2*d*(sin(alpha)-sin(beta))) / 8.0f;
	    if(fabs(tmp_rlr) > 1.0f)
	      return;
	    else{
			float p_rlr = acos( tmp_rlr );
			mod2PI(&p_rlr);
			float tmp_var = atan2( (cos(alpha)-cos(beta)),(d-sin(alpha)+sin(beta)) );
			float t_rlr = alpha - tmp_var + p_rlr/2.0f;
	        if( abs( 2*sin(alpha-t_rlr+p_rlr)-2*sin(alpha-t_rlr) - (d-sin(alpha)+sin(beta)) ) > 1e-6 ||
	            abs(-2*cos(alpha-t_rlr+p_rlr)+2*cos(alpha-t_rlr) - (cos(alpha) - cos(beta)) ) > 1e-6 ){
	            tmp_var += M_PI;
	            t_rlr = alpha - tmp_var + p_rlr/2.0;
	    	}
			mod2PI(&t_rlr);
			float q_rlr = (alpha - beta - t_rlr + p_rlr);
			mod2PI(&q_rlr);
			float dist_temp = t_rlr + p_rlr +q_rlr;
			if(dist>dist_temp){
				t = t_rlr;
				p = p_rlr;
				q = q_rlr;
				dist = dist_temp;
				path_type = RLR;
				return;
			}
	    }
	}

	void dubins_LRL(const float alpha, const float beta, const float d)
	{
		float tmp_lrl = (6.0f - d*d + 2*cos(alpha - beta) - 2*d*(sin(alpha) - sin(beta))) / 8.0f;
	    if(fabs(tmp_lrl) > 1.0f)//判断是否存在可行的 LRL 型杜宾曲线
	      return;
	    else
	    {
			float p_lrl = acos( tmp_lrl );
			mod2PI(&p_lrl);
			float tmp_var = atan2( (-cos(alpha)+cos(beta)),(d+sin(alpha)-sin(beta)));
			float t_lrl = -alpha + tmp_var + p_lrl/2.0f;
	        if( abs(-2*sin(alpha+t_lrl-p_lrl)+2*sin(alpha+t_lrl) - (d+sin(alpha)-sin(beta) ) ) > 1e-6 ||
	            abs( 2*cos(alpha+t_lrl-p_lrl)-2*cos(alpha+t_lrl) - (-cos(alpha) + cos(beta)) ) > 1e-6 )
	        {
	            tmp_var = tmp_var + M_PI;
	            t_lrl = -alpha + tmp_var + p_lrl/2.0;
	    	}//检查计算结果是否满足一定的精度要求。如果不满足精度要求，则通过调整 tmp_var 和重新计算 t_lrl 来纠正结果。
			mod2PI(&t_lrl);
			float q_lrl = (-alpha + beta - t_lrl + p_lrl);
			mod2PI(&q_lrl);
			float dist_temp = t_lrl+p_lrl+q_lrl;//表示整个 LRL 曲线的长度
			if(dist>dist_temp)
			{
				t = t_lrl;
				p = p_lrl;
				q = q_lrl;
				dist = dist_temp;
				path_type = LRL;
				return;
			}
	    }
	}

	void dubins_search(const float alpha, const float beta, const float d)
	{
		dist = 3.4028235E38;
		path_type = INVALID;

		float md = sqrt(4 - (abs(cos(alpha)) + abs(cos(beta)))*(abs(cos(alpha)) + abs(cos(beta))) ) + abs(sin(alpha)) + abs(sin(beta));
		if(d > md)
		{
			dubins_LSL(alpha, beta, d);
			dubins_LSR(alpha, beta, d);
			dubins_RSL(alpha, beta, d);
			dubins_RSR(alpha, beta, d);
		}
		else
		{
			dubins_LRL(alpha, beta, d);
			dubins_RLR(alpha, beta, d);
		}
	}

	void dubins_cal(const std::vector<float>& start, const std::vector<float>& end, const float rho, std::list<Path>& paths)
	{
		/*
		 * start = (xs, ys, zs, chis)
		 * end   = (xe, ye, ze, chie)
		 */
		float dx 	 = end[0] - start[0];
		float dy 	 = end[1] - start[1];
		float zstart = start[2];
		float zend 	 = end[2];
		float zdelta = zend - zstart;
		float chis 	 = start[3];
		float chie 	 = end[3];

		Vector2f ps(start[0], start[1]);
		Vector2f pe(end[0], end[1]);


		float Dist = sqrt(dx*dx + dy*dy);
		/*
		  *  以rho为单位，归一化，并旋转坐标到东北天坐标系，计算6种dubins曲线的t p q时，曲线端点及转弯半径满足
		 * 1. Pi = (0, 0, alpha)
		 * 2. Pf = (d, 0, beta)
		 * 3. Rmin = 1
		 */
		float d = Dist/rho;//距离/转弯半径
		float theta = atan2(dy, dx);//方位角
		float alpha = theta - chis;
		float beta  = theta - chie;
		mod2PI(&alpha);
		mod2PI(&beta);//转换到值域为（0，2PI）

		dubins_search(alpha, beta, d);//计算六种Dubins曲线，算出最短的一种

		/*
		 * 输出
		 */

		Path path(rho);

		Vector2f vchis(cos(chis), sin(chis));
		Vector2f vchie(cos(chie), sin(chie));

		switch(path_type){
			case LSL:
			{
				Vector2f cls(ps + rho*rotz(vchis, -M_PI/2)); //第一段圆弧的圆心坐标
				Vector2f cle(pe + rho*rotz(vchie, -M_PI/2));//第二段圆弧的圆心坐标

				/* first orbit */
				path.lambda = -1;//逆时针旋转
				path.c 		= cls; //第一段圆弧的圆心
				path.n 		= (cle - cls).normalized(); //圆心间方向向量
				path.w 		= cls + rho*rotz(path.n, M_PI/2);//第一段路径结束的点
				path.zs 	= zstart; //起点高度
				path.ze 	= path.zs + zdelta*t/(t+p+q); //第一段圆弧结束时的高度
				path.len 	= t*rho; //第一段圆弧长度
				paths.push_back(path);

				/* straight line(the second segment of LSL) */
				path.lambda = 0; //直线
				path.r 		= path.w; //走直线的起点位置
				path.q 		= path.n; //走直线的方向
				path.w 		= cle + rho*rotz(path.n, M_PI/2); //走直线的终点位置
				path.zs 	= path.ze; //起点高度
				path.ze 	= path.zs + zdelta*p/(t+p+q); //终点高度
				path.len 	= p*rho; //直线长度
				paths.push_back(path);

				/* last orbit of LSL*/
				path.lambda = -1; //逆时针旋转
				path.c 		= cle; //第二段圆弧的圆心
				path.n 		= vchie; //第二段圆弧结束时的方向
				path.w 		= pe; //第二段路径结束的点
				path.zs 	= path.ze; //起点高度
				path.ze 	= zend; //终点高度
				path.len 	= q*rho; //第二段圆弧长度
				paths.push_back(path);
			}
				break;
			case LSR:
			{
				Vector2f cls(ps + rho*rotz(vchis, -M_PI/2));
				Vector2f cre(pe + rho*rotz(vchie,  M_PI/2));

				/* first orbit */
				path.lambda = -1;
				path.c		= cls;
				path.n		= rotz(vchis, -t);
				path.w		= cls + rotz(ps - cls, -t);
				path.zs 	= zstart;
				path.ze 	= path.zs + zdelta*t/(t+p+q);
				path.len 	= t*rho;
				paths.push_back(path);

				/* straight line */
				path.lambda = 0;
				path.r 		= path.w;
				path.q 		= path.n;
				path.w 		= cre + rotz(pe - cre, -q);
				path.zs 	= path.ze;
				path.ze 	= path.zs + zdelta*p/(t+p+q);
				path.len 	= p*rho;
				paths.push_back(path);

				/* last orbit */
				path.lambda = 1;
				path.c 		= cre;
				path.n 		= vchie;
				path.w 		= pe;
				path.zs 	= path.ze;
				path.ze 	= zend;
				path.len 	= q*rho;
				paths.push_back(path);
			}
				break;
			case RSL:
			{
				Vector2f crs(ps + rho*rotz(vchis,  M_PI/2));
				Vector2f cle(pe + rho*rotz(vchie, -M_PI/2));

				/* first orbit R */
				path.lambda = 1;
				path.c 		= crs;
				path.n 		= rotz(vchis, t);
				path.w 		= crs + rotz(ps - crs, t);
				path.zs 	= zstart;
				path.ze 	= path.zs + zdelta*t/(t+p+q);
				path.len 	= t*rho;
				paths.push_back(path);

				/* straight line S*/
				path.lambda = 0;
				path.r 		= path.w;
				path.q 		= path.n;
				path.w 		= cle + rotz(pe - cle, q);
				path.zs 	= path.ze;
				path.ze 	= path.zs + zdelta*p/(t+p+q);
				path.len 	= p*rho;
				paths.push_back(path);

				/* last orbit L */
				path.lambda = -1;
				path.c 		= cle;
				path.n 		= vchie;
				path.w 		= pe;
				path.zs 	= path.ze;
				path.ze 	= zend;
				path.len 	= q*rho;
				paths.push_back(path);
			}
				break;
			case RSR:
			{
				Vector2f crs(ps + rho*rotz(vchis,  M_PI/2));
				Vector2f cre(pe + rho*rotz(vchie,  M_PI/2));

				/* first orbit R */
				path.lambda = 1;
				path.c 		= crs;
				path.n 		= (cre - crs).normalized();
				path.w 		= crs + rho*rotz(path.n, -M_PI/2);
				path.zs 	= zstart;
				path.ze 	= path.zs + zdelta*t/(t+p+q);
				path.len 	= t*rho;
				paths.push_back(path);

				/* straight line */
				path.lambda = 0;
				path.r 		= path.w;
				path.q		= path.n;
				path.w 		= cre + rho*rotz(path.n, -M_PI/2);
				path.zs 	= path.ze;
				path.ze 	= path.zs + zdelta*p/(t+p+q);
				path.len 	= p*rho;
				paths.push_back(path);

				/* last orbit R */
				path.lambda = 1;
				path.c 		= cre;
				path.n 		= vchie;
				path.w 		= pe;
				path.zs 	= path.ze;
				path.ze 	= zend;
				path.len 	= q*rho;
				paths.push_back(path);
			}
				break;
			case RLR:
			{
				Vector2f crs(ps + rho*rotz(vchis,  M_PI/2));
				Vector2f cre(pe + rho*rotz(vchie,  M_PI/2));

				/* first orbit R */
				path.lambda = 1;
				path.c 		= crs;
				path.n 		= rotz(vchis, t);
				path.w 		= crs + rotz(ps - crs, t);
				path.zs 	= zstart;
				path.ze 	= path.zs + zdelta*t/(t+p+q);
				path.len 	= t*rho;
				paths.push_back(path);

				/* second orbit L */
				path.lambda = -1;
				path.c 		= path.w + rho*rotz(path.n, -M_PI/2);
				path.n 		= rotz(vchie, -q);
				path.w 		= cre + rotz(pe - cre, -q);
				path.zs 	= path.ze;
				path.ze 	= path.zs + zdelta*p/(t+p+q);
				path.len 	= p*rho;
				paths.push_back(path);

				/* last orbit R*/
				path.lambda = 1;
				path.c 		= cre;
				path.n 		= vchie;
				path.w 		= pe;
				path.zs 	= path.ze;
				path.ze 	= zend;
				path.len 	= q*rho;
				paths.push_back(path);
			}
				break;
			case LRL:
			{
				Vector2f cls(ps + rho*rotz(vchis, -M_PI/2));
				Vector2f cle(pe + rho*rotz(vchie, -M_PI/2));

				/* first orbit L */
				path.lambda = -1;
				path.c		= cls;
				path.n 		= rotz(vchis, -t);
				path.w 		= cls + rotz(ps - cls, -t);
				path.zs 	= zstart;
				path.ze 	= path.zs + zdelta*t/(t+p+q);
				path.len 	= t*rho;
				paths.push_back(path);

				/* second orbit R */
				path.lambda = 1;
				path.c 		= path.w + rho*rotz(path.n, M_PI/2);
				path.n 		= rotz(vchie, q);
				path.w 		= cle + rotz(pe - cle, q);
				path.zs 	= path.ze;
				path.ze 	= path.zs + zdelta*p/(t+p+q);
				path.len 	= p*rho;
				paths.push_back(path);

				/* last orbit L */
				path.lambda = -1;
				path.c 		= cle;
				path.n 		= vchie;
				path.w 		= pe;
				path.zs 	= path.ze;
				path.ze 	= zend;
				path.len 	= q*rho;
				paths.push_back(path);
			}
				break;
			case INVALID:
				break;
		}
	}

} /*Anonymous namespace*/


void dubins_planing(std::vector<std::vector<float>>& waypoints, const float rho, std::list<Path>& paths)
{
	/* waypoints 	: 二维数组，每行代表一个节点
	 * waypoints[i] = (x , y, z)或(x, y, z, chi), 前三个是航点位置(NED坐标系)，第四个(可选) 是飞机在节点上的航迹角
	 * rho 			: 转弯半径
	 * paths 		: 输出路径链表(每个节点是代表一段圆弧或直线)
	 */
	paths.clear();
	int num = waypoints.size();
	if(num < 2)
	{ // dubins 曲线最少有两节点
		return;
	}
	int col = waypoints[0].size();
	if(col == 4)
	{	// waypoints[i] = (x, y, z, chi);
		for(int i = 1; i < num; ++i)
		{
			dubins_cal(waypoints[i-1], waypoints[i], rho, paths);
			//传入参数:当前点以及下一个点和转弯半径;并将算出的 Dubins 曲线信息传输paths消息
		}
	}
	else if(col == 3)
	{	//waypoints[i] = (x, y, z)
		float chi_cur = atan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0]);
		waypoints[0].push_back(chi_cur);//将当前方位角置于第一个向量的末尾
		for(int i = 1; i < num; ++i)
		{
			chi_cur = atan2(waypoints[i][1] - waypoints[i-1][1], waypoints[i][0] - waypoints[i-1][0]);
			waypoints[i].push_back(chi_cur);
			dubins_cal(waypoints[i-1], waypoints[i], rho, paths);
		}
	}
	else
	{
		return;	// 输入参数格式有误
	}

	/* 增加最后一段圆弧， 标志最后一段圆弧无末端半平面(无终点)，使飞机在最后一节点盘旋*/
//	Path end_path(paths.back());//将paths中的最后一个路径复制给end_path
//
//	end_path.zs = end_path.ze;
//	end_path.n  <<0, 0;
//	paths.push_back(end_path);
}
} /*DUBINS namespace*/










