/*
 * pathFollow.hpp
 *
 *  Created on: 2021年3月24日
 *      Author: 刘成吉
 */

#ifndef MODULES_PATH_FOLLOW_PATHFOLLOW_HPP_
#define MODULES_PATH_FOLLOW_PATHFOLLOW_HPP_

#include "system/system.hpp"
#include "userlib/userlib.hpp"
#include "dubins.hpp"
#include "fillets.hpp"
#include "simpaths.hpp"
#include <list>


#ifdef __cplusplus

#include <Eigen>

using namespace std;
using namespace Eigen;

namespace PATHFOLLOW
{
/* 接口：
 * PathCMD 			: 控制输出， 通过path_manager()函数返回给control
 * path_planing		: 输入：航路点waypoints, 规划方案scheme; 输出： 匿名空间中的paths路劲链表，用于path_manager
 * path_manager 	: 遍历跟踪链表paths中的圆弧和直线(向量场方法)
 * path_managerL2 	: 遍历跟踪链表paths中的圆弧和直线(L2/L1视线角法)
 * get 				: 获取规划得到的paths接口函数
 * restart 			: 重新跟踪规划好的轨迹，前提是轨迹已经规划好了
 */

	struct PathCMD
	{
		int8_t path_type;
		bool the_last;
		float Va_c;		// commanded air speed
		float h_c;
		float chi_c;	// commanded heading
		float phi_ff;	// roll feedforward command
		float e_py;
		float ascmd; 	// lateral acceleration(only used for L2/L1 guidance control)

		// path_management
		float initiation_of_path[3];
		float destination_of_path[3];
		float direction_of_path[3];
		float turning_centor[3];
		float turning_radius;
	};
	typedef typename MSG_PATH::Path Path;
	enum Scheme{Dubins, Fillet, Left_Orbit, Right_Orbit, Straight_CW, Straight_CCW};
	enum Stateflag
	{
		Start, ReadNxt, OrbitStart, OrbitWait, Straight
	};
	void path_planing(const std::vector<std::vector<float>>& waypoints, const Scheme scheme);

	void path_manager(PathCMD& pathcmd);

	void path_managerL2(PathCMD& pathcmd);

	void get_paths(std::list<Path>& paths_);

	void get_Stateflag(Stateflag state_transition_);

	void restart();

	void generate_waypoints(std::vector<std::vector<float>>& waypoints, const float entry_ang);
}

#endif /*__cplusplus*/
#endif /* MODULES_PATH_FOLLOW_PATHFOLLOW_HPP_ */
