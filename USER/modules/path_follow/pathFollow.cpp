/*
 * orbitfollow.cpp
 *
 *  Created on: 2021年3月24日
 *      Author: 刘成吉
 */

#include "pathFollow.hpp"

namespace PATHFOLLOW{

namespace{	/* Anonymous namespace */
	float Va_c;		// airspeed command
	float H_c;		// altitude command
	float chi_c;	// heading command
	float phi_ff;	// feed forward roll command
	float chi_infty;// approach angle for large distance from straight-line path
	float k_path;	// proportional gain for path following
	float k_orbit;	// proportional gain for orbit following
	float phi_max;
	float e_py;
	float KL2;
	float ascmd;
	float ascmd_max;

	Vector3f r_path;	// initial position of start of waypoint path
	Vector3f q_path; 	// unit vector that defines inertial direction of waypoint path
	Vector3f c_orbit;	// center of orbit
	Vector2f halfp_w; 	// location of half plane
	Vector2f halfp_n; 	// direction of half plane
	float path_climb_rate;   // command climb rate
	float rho_orbit;// radius of orbit
	float lam_orbit;// direction of orbit (+1 for CW, -1 for CCW)

	float pn;
	float pe;
	float pd;
	float vn;
	float ve;
	float vd;
	float chi;	// current course angle(ground speed heading)
	eskf_msg eskf;

//	enum Stateflag
//	{
//		Start, ReadNxt, OrbitStart, OrbitWait, Straight
//	};
	Stateflag state_transition = Start;
	std::list<Path> paths;		// paths to follow, comprises orbit and straight line
	std::list<Path>::iterator it = paths.begin();
/*std::list<Path> 是一个双向链表，保存了一系列 Path 对象。Path 可能是一个自定义的类或类型，用于表示路径信息。

std::list<Path>::iterator 是 std::list<Path> 的迭代器类型，用于遍历和操作链表中的元素。

通过 paths.begin() 方法，获取到链表的起始位置的迭代器，并将其赋值给 it 迭代器。*/

	void state_update()
	{
		xQueuePeek(queueESKF,&eskf,0);
		pn = eskf.Pos[0];
		pe = eskf.Pos[1];
		pd = eskf.Pos[2];
		vn = eskf.Ned_spd[0];
		ve = eskf.Ned_spd[1];
		vd = eskf.Ned_spd[2];
		chi = atan2(ve, vn);
	}
	bool isInHalfPlane(const Vector2f& w, const Vector2f& n)
	{
		if(n.norm() == 0) return false; // 无末端半平面，用于在末端盘旋
		float proj = (pn - w[0])*n[0] + (pe - w[1])*n[1];
		if(proj >= 0.0)
			return true;
		return false;
	}

	void init()
	{
		state_update(); 		// 读取更新状态
		chi_infty = M_PI/4; 	// 无穷远处航机偏角(见Small unmanned aircraft 第10章)
	//	k_path = 0.02;
		k_path = 1/rho_orbit;	// Rule of thumb
		k_orbit = 3.0; 			// 向量场路径跟随算法增益

	//	Va_c = sqrt(eskf.Ned_spd[0]*eskf.Ned_spd[0] + eskf.Ned_spd[1]*eskf.Ned_spd[1] + eskf.Ned_spd[2]*eskf.Ned_spd[2]);
		Va_c = 2;				// 参考飞行速度
		phi_max = 20*M_PI/180;  // 参考最大滚转角
		lam_orbit = 1; 		    // direction of orbit (+1 for CW, -1 for CCW)
		rho_orbit = SQR(Dubins_Vel)/OneG/tan(DubinsTiltAng); //

		KL2    = 1.20; 		 	// L2 增益
		ascmd_max = 9.788*tan(60*M_PI/180); // 最大向心加速度
	}

	void straight_line_follow()
	{
		float chi_q = atan2(q_path(1), q_path(0));
		while(chi_q - chi < -M_PI){
			chi_q += 2*M_PI;
		}
		while(chi_q - chi > M_PI){
			chi_q -= 2*M_PI;
		}
		Vector3f e_pi(pn - r_path(0), pe - r_path(1), pd - r_path(2));  // relative horizontal path error expressed in the inertial frame

		e_py = -sin(chi_q)*e_pi(0) + cos(chi_q)*e_pi(1);	// relative path error expressed in the path frame whose x-axis aligns with q_path

		// heading command
		chi_c = chi_q - chi_infty*2/M_PI*atan(k_path*e_py);

		Vector3f nnn(q_path(1), -q_path(0), 0); // cross product btw q_path and [0 0 1]';
		nnn.normalize();						// the unit vector normal to the q_path-ki plane
		Vector3f si;
		si = e_pi - e_pi.dot(nnn)*nnn;
		H_c = -r_path(2) + sqrt(si(0)*si(0) + si(1)*si(1))*(-q_path(2)/sqrt(q_path(0)*q_path(0)+q_path(1)*q_path(1)));
		phi_ff = 0;
	}

	void orbit_follow()
	{
		float d_horiz[2];
		d_horiz[0] = c_orbit(0) - pn;
		d_horiz[1] = c_orbit(1) - pe;
		float d = sqrt(d_horiz[0]*d_horiz[0] + d_horiz[1]*d_horiz[1]);	 // distance from orbit center
		d_horiz[0] /= d;
		d_horiz[1] /= d;
		// compute wrapped version of angular position on orbit
		float varphi = atan2(pe-c_orbit(1),pn-c_orbit(0));
	    while (varphi - chi < -M_PI){
	    	varphi = varphi + 2*M_PI;
	    }
	    while (varphi - chi > +M_PI){
	    	varphi = varphi - 2*M_PI;
	    }
	    e_py	= d - rho_orbit;
	    float orbit_error = e_py/rho_orbit;

	    // heading command
	    chi_c = varphi + lam_orbit*(M_PI/2+atan(k_orbit*orbit_error));
	    H_c = -c_orbit(2);

	    // roll angle feedforward command
	//    float ea = atan2(-d_horiz[0]*sin(chi) + d_horiz[1]*cos(chi), d_horiz[0]*cos(chi) + d_horiz[1]*sin(chi));	// angle btw heading and d_horiz;
	//    float ff_k = tanh(ea)*exp(-orbit_error*orbit_error/(2*0.08*0.08));
	//    phi_ff = ff_k*atan(Va_c*Va_c/(9.7883*rho_orbit));
	    phi_ff = 0;

	}


	float arc_height()
	{
		if(abs(path_climb_rate) < 1e-6 || halfp_n.norm() == 0){
			return -c_orbit[2];
		}
		float phi_cur   = atan2(pe - c_orbit[1], pn - c_orbit[0]);
		float phi_tip   = atan2(halfp_n[1], halfp_n[0]) - lam_orbit*M_PI/2;
		float phi_delta = lam_orbit*(phi_tip - phi_cur);
		mod2PI(&phi_delta);
		return -c_orbit[2] + phi_delta*rho_orbit*path_climb_rate;
	}

	void L2_straight_line_guidance()
	{
		float Q = M_PI/6;  // maximum intercept angle
		Vector3f F(pn - r_path[0], pe - r_path[1], pd - r_path[2]);
		Vector3f prj(r_path + q_path*(F.dot(q_path)));
		/*F.dot(q_path)代表向量F与向量q_path的点积，表示F在q_path方向上的投影长度。然后将这个投影长度乘以q_path
		 ，得到一个平行于q_path的向量。最后，将这个向量与r_path相加，得到投影点prj**/
		H_c = -prj[2];

		/* to get aim point(pa) in horizontal plane(x-y) */
		Vector2f qh(q_path[0], q_path[1]); qh.normalize();
		Vector2f ev(vn, ve); ev.normalize();

		e_py = qh[0]*F[1] - qh[1]*F[0];
		float V  = sqrt(vn*vn + ve*ve);
		float ep = abs(e_py);
		float L2 = KL2*V;
		float Dh = 0.0;
		if( V < 0.1*Va_c  )
		{
			ascmd = 0;
			return;
		}
		if(ep <= L2*sin(Q))
		{
			Dh = sqrt(L2*L2 - ep*ep);
		}
		else
		{
			Dh = MIN(ep/tan(Q), 1.8*L2);
		}
		Vector2f pa(prj.head<2>() + Dh*qh);
		Vector2f L(pa[0] - pn, pa[1] - pe);
		L.normalize();

		float sinYita = ev[0]*L[1] - ev[1]*L[0];
		if(ev.dot(L) < 0)
		{ // 背离目标(速度与pa点夹角大于pi/2)
			ascmd = Sign(sinYita)*ascmd_max;
			return;
		}
		ascmd = 2*V*V/L2*sinYita;
	}

	void L2_orbit_guidance()
	{
		float Q = PI/6;
		H_c = arc_height();

		Vector2f ev(vn, ve);  ev.normalize();

		float dh = sqrt( (c_orbit[0] - pn)*(c_orbit[0] - pn) + (c_orbit[1] - pe)*(c_orbit[1] - pe) );
		e_py     = dh - rho_orbit;
		float V  = sqrt(vn*vn + ve*ve);
		float L2 = KL2*V;
		float L2plus = L2;
		float Lh = 0;

		if(V < 0.1*Va_c  ) {
			ascmd = 0;
			return;
		}

		float ell = 0;
		if(e_py > 0) {
			ell = sqrt(dh*dh - rho_orbit*rho_orbit);
			float CPQ = cos(M_PI/2 + Q);
			Lh = rho_orbit*CPQ + sqrt( dh*dh - rho_orbit*rho_orbit + rho_orbit*rho_orbit*CPQ*CPQ );
		}

		if(e_py > 0 && ell > L2){
			L2plus = MAX(Lh, L2);
		}else {
			L2plus = L2;
		}
		L2plus = MIN(L2plus, dh + rho_orbit); //满足三角形约束
		float varphi = atan2(pe - c_orbit[1], pn - c_orbit[0]);
		if(abs(dh-rho_orbit) < L2plus){ //三角形约束
			float B = acos( (dh*dh + rho_orbit*rho_orbit - L2plus*L2plus)/(2*dh*rho_orbit) );
			varphi += B*lam_orbit;
		}

		Vector2f pa(c_orbit[0] + rho_orbit*cos(varphi), c_orbit[1] + rho_orbit*sin(varphi));
		Vector2f L(pa[0] - pn, pa[1] - pe);
		L.normalize();

		float sinYita = ev[0]*L[1] - ev[1]*L[0];
		if(ev.dot(L) < 0 && e_py > 0){ // 背离目标(速度与pa点夹角大于pi/2)
			ascmd = Sign(sinYita)*ascmd_max;
			return;
		}
		ascmd = 2*V*V/L2*sinYita;
	}

} /* Anonymous namespace */

//float dbwps[6][4];

void path_planing(const std::vector<std::vector<float>>& waypoints, const Scheme scheme)
{
	/* 根据航路点和规划方案规划处对应路径paths
	 * waypoints: 航路点
	 * scheme: 	    规划方案
	 */
	init();
	int num = waypoints.size();
	//返回 waypoints 中一级向量的大小，此指waypoints向量的行数
	if(num < 1 || waypoints[0].size() != 3) return;
	std::vector<std::vector<float>> wps;
	wps.emplace_back(waypoints[0]);
	/*wps 是一个二维浮点数向量的容器 ，wps.emplace_back(waypoints[0]) 用于将 waypoints 容器中的第一个向量（waypoints[0]）添加到 wps 容器的末尾*/
	for(int i = 1; i < num - 1; ++i)
	{	// 遍历中间点，并去掉不必要的航点(三点共线的中间点)
		if( abs( atan2(waypoints[i][1] - waypoints[i-1][1], waypoints[i][0] - waypoints[i-1][0]) -\
				 atan2(waypoints[i+1][1] - waypoints[i][1], waypoints[i+1][0] - waypoints[i][0]) ) < 1e-4 )
		{
			continue;
		}
		wps.emplace_back(waypoints[i]);
	}
	if(num > 1)
	{
		wps.emplace_back(waypoints[num-1]);
	}

	switch(scheme)
	{
		case Dubins:
		{
			DUBINS::dubins_planing(wps, rho_orbit, paths);
			//传入参数：二维浮点数向量，用于保存航点信息；转弯半径；输出路径消息链表
		}
			break;
		case Fillet:
		{
			FILLETS::fillet_planing(wps, rho_orbit, paths);
		}
			break;
		case Left_Orbit:
		{
			SIMPATHS::left_orbit(wps[0], rho_orbit, paths);
		}
			break;
		case Right_Orbit:
		{
			SIMPATHS::right_orbit(wps[0], rho_orbit, paths);
		}
			break;
		case Straight_CW:
		{
			SIMPATHS::straight_line_CW(wps, rho_orbit, paths);
		}
			break;
		case Straight_CCW:
		{
			SIMPATHS::straight_line_CCW(wps, rho_orbit, paths);
		}
			break;
	}
	state_transition = Start;
}



void path_manager(PathCMD& pathcmd)
{
	/* 先由path_planing() 接口函数规划得到的路径链表paths。
	 *  path_manager 遍历跟踪paths中的圆弧和直线(vector field control)
	 *  pathcmd：控制输出，返回给control
	 */

	static bool the_last = false;
	state_update();
	if(state_transition == Start){
		if(paths.empty()) return;

		it = paths.begin();
		the_last = false;
		state_transition = ReadNxt;
	}
	if(state_transition == ReadNxt){
		/* 如果it == paths.end()，则说明规划有误，将发生严重错误，在规划时要避免这种情况发生，方法是在paths链表的最后一个节点盘旋，
		 * 令最后一个节点(必须是圆弧)的半平面法向量n = (0,0)，此时isInHalfPlane(w, n)判断飞机永远未完成最后一段线段飞行，即可实现最后盘旋
		 * 除非path_planing()被重新调用初始化state_transition，即可解除盘旋
		 */
		if(it == paths.end()) return;
		if(it->lambda == 0){
			if(abs(it->len) < 1e-10)
				path_climb_rate = 0;
			else
				path_climb_rate = (it->ze - it->zs)/(it->len);
			r_path << it->r, it->zs;
			q_path << it->q, (it->ze - it->zs)/(it->len);
			q_path.normalize();
			halfp_w = it->w;
			halfp_n = it->n;
			state_transition = Straight;

			pathcmd.initiation_of_path[0] = r_path(0);
			pathcmd.initiation_of_path[1] = r_path(1);
			pathcmd.initiation_of_path[2] = r_path(2);
			pathcmd.direction_of_path[0]  = q_path(0);
			pathcmd.direction_of_path[1]  = q_path(1);
			pathcmd.direction_of_path[2]  = q_path(2);
			pathcmd.path_type = 0;
		}else{
			if(abs(it->len) < 1e-10)
				path_climb_rate = 0;
			else
				path_climb_rate = (it->ze - it->zs)/(it->len);
			lam_orbit = it->lambda;
			rho_orbit = it->rho;
			c_orbit	  << it->c, it->ze;
			halfp_w = it->w;
			halfp_n = it->n;
			state_transition = OrbitStart;

			pathcmd.turning_radius		  = rho_orbit;
			pathcmd.turning_centor[0]	  = c_orbit(0);
			pathcmd.turning_centor[1]	  = c_orbit(1);
			pathcmd.turning_centor[2]	  = c_orbit(2);
			pathcmd.path_type = lam_orbit;

		}
		++it;
		if(it == paths.end()) the_last = true;
	}
	if(state_transition == Straight){
		straight_line_follow();
		if(isInHalfPlane(halfp_w, halfp_n) && !the_last){
			state_transition = ReadNxt;
		}
	}
	if(state_transition == OrbitStart){
		orbit_follow();
		if(!isInHalfPlane(halfp_w, halfp_n) )	// the aircraft has to start in the opposite side of the half plane(w, n)
			state_transition = OrbitWait;
	}
	if(state_transition == OrbitWait){
		orbit_follow();
		if(isInHalfPlane(halfp_w, halfp_n) && !the_last){
			state_transition = ReadNxt;
		}
	}

    pathcmd.Va_c 	 = Va_c;
    pathcmd.chi_c 	 = chi_c;
    pathcmd.h_c 	 = H_c;
    pathcmd.phi_ff 	 = phi_ff;
    pathcmd.e_py 	 = e_py;
    pathcmd.the_last = the_last;
}

void path_managerL2(PathCMD& pathcmd)
{
	/* 先由path_planing() 接口函数规划得到的路径链表paths。
	 *  path_manager 遍历跟踪paths中的圆弧和直线(L1/L2control)
	 *  pathcmd：控制输出，返回给control
	 */

	static bool the_last = false;
	state_update();//获取当前位置速度信息
	if(state_transition == Start)
	{
		if(paths.empty()) return;

		it = paths.begin();
		the_last = false;
		state_transition = ReadNxt;
	}
	if(state_transition == ReadNxt)
	{
		/* 如果it == paths.end()，则说明规划有误，将发生严重错误，在规划时要避免这种情况发生，方法是在paths链表的最后一个节点盘旋，
		 * 令最后一个节点(必须是圆弧)的半平面法向量n = (0,0)，此时isInHalfPlane(w, n)判断飞机永远未完成最后一段线段飞行，即可实现最后盘旋
		 * 除非path_planing()被重新调用初始化state_transition，即可解除盘旋
		 */
		if(it == paths.end()) return;
		if(it->lambda == 0)//直线
		{
			if(abs(it->len) < 1e-10)//走直线的长度
				path_climb_rate = 0;
			else
				path_climb_rate = (it->ze - it->zs)/(it->len); //高度变化的速率
			r_path << it->r, it->zs; //走直线的路径起点和起点高度
			q_path << it->q, path_climb_rate; //走直线的方向和高度变化率
			q_path.normalize();
			halfp_w = it->w; //走直线的终点
			halfp_n = it->n; //走直线的方向？
			state_transition = Straight;

			pathcmd.initiation_of_path[0] = r_path(0); //路径起点
			pathcmd.initiation_of_path[1] = r_path(1);
			pathcmd.initiation_of_path[2] = r_path(2);
			pathcmd.destination_of_path[0] = halfp_w(0); //路径终点
			pathcmd.destination_of_path[1] = halfp_w(1);
			pathcmd.destination_of_path[2] = it->ze;
			pathcmd.direction_of_path[0]  = q_path(0); //路径方向
			pathcmd.direction_of_path[1]  = q_path(1);
			pathcmd.direction_of_path[2]  = q_path(2);
			pathcmd.path_type = 0;
		}
		else
		{
			if(abs(it->len) < 1e-10)
				path_climb_rate = 0;
			else
				path_climb_rate = (it->ze - it->zs)/(it->len);
			lam_orbit = it->lambda; //圆弧类型
			rho_orbit = it->rho; //转弯半径
			c_orbit	  << it->c, it->ze; //圆心和圆弧终点高度
			halfp_w = it->w; //圆弧结束的点
			halfp_n = it->n; //圆弧结束的方向
			state_transition = OrbitStart;

			pathcmd.turning_radius		  = rho_orbit; //半径
			pathcmd.turning_centor[0]	  = c_orbit(0); //圆心
			pathcmd.turning_centor[1]	  = c_orbit(1);
			pathcmd.turning_centor[2]	  = c_orbit(2);
			pathcmd.path_type = lam_orbit;
		}
		++it;
		if(it == paths.end()) the_last = true;
	}
	if(state_transition == Straight)
	{
		L2_straight_line_guidance();
		if(isInHalfPlane(halfp_w, halfp_n) && !the_last) //判断w是否位于向量n形成的半平面内部
		{
			state_transition = ReadNxt;
		}
	}
	if(state_transition == OrbitStart)
	{
		L2_orbit_guidance();
		if(!isInHalfPlane(halfp_w, halfp_n) )	// the aircraft has to start in the opposite side of the half plane(w, n)
			state_transition = OrbitWait;
	}
	if(state_transition == OrbitWait)
	{
		L2_orbit_guidance();
		if(isInHalfPlane(halfp_w, halfp_n) && !the_last)
		{
			state_transition = ReadNxt;
		}
	}

    pathcmd.Va_c 	 = Va_c;//飞行速度2
    pathcmd.h_c 	 = H_c;//高度
    pathcmd.e_py 	 = e_py;//飞行器与目标点在水平平面上的偏差量
    pathcmd.ascmd 	 = ascmd;//向心加速度
    pathcmd.the_last = the_last;//判断是否为最后一个节点
}

void get_paths(std::list<Path>& paths_)
{
	/* 获取规划路径链表 */
	paths_ = paths;
}
void get_Stateflag(Stateflag state_transition_)
{
	state_transition_ = state_transition;
}
void restart()
{
	/* 重新跟踪规划好的轨迹，前提是轨迹已经规划好了 */
	state_transition = Start;
}

void generate_waypoints(std::vector<std::vector<float>>& waypoints, const float entry_ang)
{
/*
  *  生成正多边形路径，waypoints 二维数组长度为5时：正5边形
  * 各边夹角 varrho = 180 - 108 = 72°，跟随路劲在距离航点顶点 R/tan(varrho/2)时切入圆弧
 */
	state_update();
	waypoints.resize(5, std::vector<float>(3,0));
	waypoints[0][0] = pn;
	waypoints[0][1] = pe;
	waypoints[0][2] = pd;

	float varrho = 72*D2R;
	float dist = 80;	// 36/tan(54) = 26;

	float q[3]; // unit vector that defines the direction of next point W.R.T current point
	q[0] = cos(chi+entry_ang);
	q[1] = sin(chi+entry_ang);
	q[2] = 0;

	for(int i = 1; i < 5; ++i){
		q[0] = cos(chi + entry_ang + (i-1)*varrho);
		q[1] = sin(chi + entry_ang + (i-1)*varrho);
		for(int j = 0; j < 3; ++j){
			waypoints[i][j] = waypoints[i-1][j] + dist*q[j];
		}
	}
}

} /* namespace PATHFOLLOW */
