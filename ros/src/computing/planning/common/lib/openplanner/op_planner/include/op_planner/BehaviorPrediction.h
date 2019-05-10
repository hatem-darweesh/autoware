
/// \file  BehaviorPrediction.h
/// \brief Predict detected vehicles's possible trajectories, these trajectories extracted from the vector map.
/// \author Hatem Darweesh
/// \date Jul 6, 2017


#ifndef BEHAVIORPREDICTION_H_
#define BEHAVIORPREDICTION_H_

#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>
#include "op_planner/PlanningHelpers.h"
#include "PlannerH.h"
#include "op_utility/UtilityH.h"
#include "PassiveDecisionMaker.h"

class PredictionParams
{
public:

int MAX_PARTICLES_NUM;
int MIN_PARTICLES_NUM;
double KEEP_PERCENTAGE;

double  POSE_FACTOR;
double  DIRECTION_FACTOR;
double  VELOCITY_FACTOR ;
double  ACCELERATE_FACTOR;
double  INDICATOR_FACTOR;

bool bEnableParking;

std::string experiment_name;

	PredictionParams()
	{
		MAX_PARTICLES_NUM = 30;
		MIN_PARTICLES_NUM = 1;
		KEEP_PERCENTAGE = 0.5;

		POSE_FACTOR = 0.1;
		DIRECTION_FACTOR = 0.0;
		VELOCITY_FACTOR = 0.35;
		ACCELERATE_FACTOR = 0.35;
		INDICATOR_FACTOR = 0.2;

		bEnableParking = false;
	}
};

namespace PlannerHNS
{

#define MOTION_POSE_ERROR 0.7 // 50 cm pose error
#define MOTION_ANGLE_ERROR 0.05 // 0.05 rad angle error
#define MOTION_VEL_ERROR 0.2
#define MOTION_ACC_ERROR 0.1
#define MOTION_IND_ERROR 0.1

#define MEASURE_POSE_ERROR 0.15
#define MEASURE_ANGLE_ERROR 0.05
#define MEASURE_VEL_ERROR 0.1
#define MEASURE_ACC_ERROR 0.1
#define MEASURE_IND_ERROR 0.1

#define PREDICTION_DISTANCE_PERCENTAGE 0.25


#define FIXED_MIN_PLANNING_DISTANCE 10

#define MIN_PREDICTION_TIME 5
#define USE_OPEN_PLANNER_MOVE 0


#define ACCELERATION_CALC_TIME 0.5
#define VELOCITY_DECISION_VALUE 0.2
#define ACCELERATION_DECISION_VALUE 0.25

#define ENABLE_STOP_BEHAVIOR_GEN 1
#define WEIGHT_CRITICAL_DIFF 0.1

#define THREE_PERC_POINTS(x) (int)(x * 1000.0) / 1000.0
#define TWO_PERC_POINTS(x) (int)(x * 100.0) / 100.0

typedef boost::mt19937 ENG;
typedef boost::normal_distribution<double> NormalDIST;
typedef boost::variate_generator<ENG, NormalDIST> VariatGEN;



class TrajectoryTracker;


class Particle
{
public:
	BEH_STATE_TYPE beh; //[Stop, Yielding, Forward, Branching]
	double vel; //[0 -> Stop,1 -> moving]
	double vel_rand;
	double ang;
	double vel_prev_big;
	double prev_time_diff;
	double prev_vel_diff;
	int acc; //[-1 ->Slowing, 0, Stopping, 1 -> accelerating, 2 -> both stopping and accelerating, 3 -> both slowing and stopping  ]
	double acc_raw;
	int indicator; //[0 -> No, 1 -> Left, 2 -> Right, 3 -> both, 4 -> left and both]
	WayPoint pose;
	WayPoint car_curr_pose;
	WayPoint car_left_pose;
	WayPoint car_prev_pose;
	bool bStopLine;
	bool bNewParticle;
	double w;
	double w_real;
	double pose_w_real;
	double dir_w_real;
	double w_raw;
	double pose_w;
	double dir_w;
	double vel_w;
	double acl_w;
	double ind_w;
	TrajectoryTracker* pTraj;
	int original_index;
	RelativeInfo info_to_path;

	Particle()
	{
	  bNewParticle = true;
	  vel_rand = 0;
	  ang = 0;
	  prev_vel_diff = 0;
	  prev_time_diff = 0;
	  vel_prev_big = 0;
	  original_index = 0;
	  bStopLine = false;
	  pTraj = nullptr;
	  w = 0;
	  w_real = 0;
	  w_raw = 0;
	  pose_w = 0;
	  pose_w_real = 0;
	  dir_w = 0;
	  dir_w_real = 0;
	  vel_w = 0;
	  acl_w = 0;
	  ind_w = 0;
	  beh = BEH_STOPPING_STATE;
	  vel = 0;
	  acc = 0;
	  acc_raw = 0;
	  indicator = 0;
	}
};

class TrajectoryTracker
{
public:
	unsigned int index;
	std::string id_;
	BEH_STATE_TYPE beh;
	BEH_STATE_TYPE best_beh_by_p;
	BEH_STATE_TYPE best_beh_by_w;
	double best_w;
	double best_p;
	double all_p;
	double all_w;
	double all_w_real;
	std::vector<int> ids;
	std::vector<int> path_ids;
	WayPoint path_last_pose;
	WayPoint followPoint;
	double rms_error;
	std::vector<WayPoint> trajectory;
	PlannerHNS::RelativeInfo m_CurrRelativeInf;

	std::vector<Particle> m_CurrParts;
	//std::vector<Particle> m_ForwardPart;
	//std::vector<Particle> m_StopPart;
	//std::vector<Particle> m_YieldPart;
	//std::vector<Particle> m_LeftPart;
	//std::vector<Particle> m_RightPart;
	BehaviorState m_CurrBehavior;

	int nAliveStop;
	int nAliveYield;
	int nAlivePark;
	int nAliveForward;
	int nAliveLeft;
	int nAliveRight;

	double pStop;
	double pYield;
	double pPark;
	double pForward;
	double pLeft;
	double pRight;

	double w_avg_forward;
	double w_avg_stop;
	double w_avg_yield;
	double w_avg_park;
	double w_avg_left;
	double w_avg_right;

	static int max_particles_number;
	static int min_particles_number;
	static int active_intentions_number;
	static int total_particles_number;

	PassiveDecisionMaker m_SinglePathDecisionMaker;

	TrajectoryTracker()
	{
		beh = BEH_STOPPING_STATE;
		rms_error = 0;
		index = 0;
		nAliveStop = 0;
		nAliveYield = 0;
		nAlivePark = 0;
		nAliveForward = 0;
		nAliveLeft = 0;
		nAliveRight = 0;

		pStop  = 0;
		pYield = 0;
		pPark = 0;
		pForward = 0;
		pLeft = 0;
		pRight = 0;
		best_beh_by_p = PlannerHNS::BEH_UNKNOWN_STATE;
		best_beh_by_w = PlannerHNS::BEH_UNKNOWN_STATE;
		best_p = 0;
		best_w = 0;
		all_p = 0;
		all_w = 0;
		all_w_real = 0;

		w_avg_forward = 0;
		w_avg_stop = 0;
		w_avg_yield = 0;
		w_avg_park = 0;
		w_avg_left = 0;
		w_avg_right = 0;
	}

	virtual ~TrajectoryTracker()
	{
	}

	TrajectoryTracker(const TrajectoryTracker& obj)
	{

		rms_error = 0;
		ids = obj.ids;
		id_ = obj.id_;
		path_ids = obj.path_ids;
		path_last_pose = obj.path_last_pose;
		beh = obj.beh;
		index = obj.index;
		trajectory = obj.trajectory;
		nAliveStop = obj.nAliveStop;
		nAliveYield = obj.nAliveYield;
		nAlivePark = obj.nAlivePark;
		nAliveForward = obj.nAliveForward;
		nAliveLeft = obj.nAliveLeft;
		nAliveRight = obj.nAliveRight;

		pStop  = obj.pStop;
		pYield = obj.pYield;
		pPark = obj.pPark;
		pForward = obj.pForward;
		pLeft = obj.pLeft;
		pRight = obj.pRight;
		best_beh_by_p = obj.best_beh_by_p;
		best_beh_by_w = obj.best_beh_by_w;
		best_p = obj.best_p;
		best_w = obj.best_w;
		all_p = obj.all_p;
		all_w = obj.all_w;
		all_w_real = obj.all_w_real;
		m_SinglePathDecisionMaker = obj.m_SinglePathDecisionMaker;

		m_CurrParts = obj.m_CurrParts;
//		m_ForwardPart = obj.m_ForwardPart;
//		m_StopPart = obj.m_StopPart;
//		m_YieldPart = obj.m_YieldPart;
//		m_LeftPart = obj.m_LeftPart;
//		m_RightPart = obj.m_RightPart;
		m_CurrBehavior = obj.m_CurrBehavior;

		w_avg_forward = obj.w_avg_forward;
		w_avg_stop = obj.w_avg_stop;
		w_avg_yield = obj.w_avg_yield;
		w_avg_park = obj.w_avg_park;
		w_avg_left = obj.w_avg_left;
		w_avg_right = obj.w_avg_right;
	}

	TrajectoryTracker(std::vector<PlannerHNS::WayPoint>& path, const unsigned int& _index)
	{

		if(path.size()>0)
		{
			beh = path.at(0).beh_state;
			//std::cout << "New Path: Beh: " << beh << ", index: " << _index << ", LaneID_0: " << path.at(0).laneId << ", LaneID_1: "<< path.at(1).laneId << std::endl;
		}

		id_ = PlannerHNS::PlanningHelpers::MakePathDirectionID(path);
		index = _index;
		trajectory = path;
		int prev_id = -10;
		int curr_id = -10;
		ids.clear();
		path_ids.clear();
		for(unsigned int i = 0; i < path.size(); i++)
		{
			curr_id = path.at(i).laneId;
			path_ids.push_back(curr_id);

			if(curr_id != prev_id)
			{
				ids.push_back(curr_id);
				prev_id = curr_id;
			}
		}

		path_last_pose = path.at(path.size()-1);

		nAliveStop = 0;
		nAliveYield = 0;
		nAlivePark = 0;
		nAliveForward = 0;
		nAliveLeft = 0;
		nAliveRight = 0;

		pStop  = 0;
		pYield = 0;
		pPark = 0;
		pForward = 0;
		pLeft = 0;
		pRight = 0;
		best_beh_by_p = PlannerHNS::BEH_UNKNOWN_STATE;
		best_beh_by_w = PlannerHNS::BEH_UNKNOWN_STATE;
		best_p = 0;
		all_p = 0;
		all_w_real = 0;

		w_avg_forward = 0;
		w_avg_stop = 0;
		w_avg_yield = 0;
		w_avg_park = 0;
		w_avg_left = 0;
		w_avg_right = 0;
		best_w = 0;
		all_w = 0;
		rms_error = 0;
	}

	static bool sort_weights_des(const Particle& p1, const Particle& p2)
	{
		return p1.w > p2.w;
	}

	void UpdatePathAndIndex(std::vector<PlannerHNS::WayPoint>& _path, const unsigned int& _index)
	{
		if(_path.size() == 0) return;

		beh = _path.at(0).beh_state;
		index = _index;
		std::string prev_id_str = id_;
		id_ = PlannerHNS::PlanningHelpers::MakePathDirectionID(_path);
		if(prev_id_str.compare(id_) != 0)
		  std::cout << "Trajectory Prediction Wrong Matching, trying to match: " << id_ <<", with: " << prev_id_str << std::endl;
		trajectory = _path;
		int prev_id = -10;
		int curr_id = -10;
		ids.clear();
		path_ids.clear();
		for(unsigned int i = 0; i < _path.size(); i++)
		{
			curr_id = _path.at(i).laneId;
			path_ids.push_back(curr_id);
			if(curr_id != prev_id)
			{
				ids.push_back(curr_id);
				prev_id = curr_id;
			}
		}

		path_last_pose = _path.at(_path.size()-1);
	}

	double CalcMatchingPercentage(const std::vector<PlannerHNS::WayPoint>& _path)
	{
		if(_path.size() == 0) return 0;

		if(beh != _path.at(0).beh_state) return 0;

		int nCount = 0, nIds = 0;
		int prev_id = -10;
		int curr_id = -10;
		std::vector<int> _ids;

		for(unsigned int i = 0; i < _path.size(); i++)
		{
			curr_id = _path.at(i).laneId;
			if(i < path_ids.size())
			{
				nCount++;
				if(curr_id == path_ids.at(i))
					nIds++;
			}

			if(curr_id != prev_id)
			{
				_ids.push_back(curr_id);
				prev_id = curr_id;
			}
		}

		int nEqualities = 0;
		for(unsigned int i=0; i < _ids.size(); i++)
		{
			for(unsigned int j=0; j < ids.size(); j++)
			{
				if(_ids.at(i) == ids.at(j))
				{
					nEqualities++;
					break;
				}
			}
		}

		double rms_val = 0;
		    for(unsigned int i=0; i < _path.size(); i++)
		      {
			if(i < trajectory.size())
			  {
			    rms_val += hypot(_path.at(i).pos.y - trajectory.at(i).pos.y, _path.at(i).pos.y - trajectory.at(i).pos.y);
			  }
		      }

		    rms_error = rms_val;

		if(rms_error < 5.0)
		  return 1;

		if(_ids.size() == ids.size() && ids.size() == nEqualities && rms_error < 5.0) // perfect match
			return 1;

		WayPoint curr_last_pose = _path.at(_path.size()-1);
		double nMatch = (double)nIds/(double)nCount;
		double _d = hypot(path_last_pose.pos.y-curr_last_pose.pos.y, path_last_pose.pos.x-curr_last_pose.pos.x);

		double dCost = _d/FIXED_MIN_PLANNING_DISTANCE;
		if(dCost > 1.0)
			dCost = 1.0;
		double dMatch = 1.0 - dCost;

		double _a_diff = UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(path_last_pose.pos.a, curr_last_pose.pos.a);
		double aCost = _a_diff/M_PI;
		if(aCost > 1.0)
			aCost = 1.0;
		double aMatch = 1.0 - aCost;

		double totalMatch = (nMatch + dMatch + aMatch)/3.0;

		return totalMatch;
	}

	void InsertNewParticle(const Particle& p)
	{
		if(m_CurrParts.size() >= total_particles_number) return;

		m_CurrParts.push_back(p);

		if(p.beh == PlannerHNS::BEH_STOPPING_STATE)
			nAliveStop++;
		else if(p.beh == PlannerHNS::BEH_YIELDING_STATE)
			nAliveYield++;
		else if(p.beh == PlannerHNS::BEH_PARKING_STATE)
			nAlivePark++;
		else if(p.beh == PlannerHNS::BEH_FORWARD_STATE)
			nAliveForward++;
		else if(p.beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
			nAliveLeft++;
		else if(p.beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
			nAliveRight++;

//		if(p.beh == PlannerHNS::BEH_STOPPING_STATE && nAliveStop < BEH_PARTICLES_NUM)
//		{
//			m_StopPart.push_back(p);
//			m_StopPart.at(m_StopPart.size()-1).pTraj = this;
//			nAliveStop++;
//		}
//		else if(p.beh == PlannerHNS::BEH_YIELDING_STATE && nAliveYield < BEH_PARTICLES_NUM)
//		{
//			m_YieldPart.push_back(p);
//			m_YieldPart.at(m_YieldPart.size()-1).pTraj = this;
//			nAliveYield++;
//		}
//		else if(p.beh == PlannerHNS::BEH_FORWARD_STATE && nAliveForward < BEH_PARTICLES_NUM)
//		{
//			m_ForwardPart.push_back(p);
//			m_ForwardPart.at(m_ForwardPart.size()-1).pTraj = this;
//			nAliveForward++;
//		}
//		else if(p.beh == PlannerHNS::BEH_BRANCH_LEFT_STATE && nAliveLeft < BEH_PARTICLES_NUM)
//		{
//			m_LeftPart.push_back(p);
//			m_LeftPart.at(m_LeftPart.size()-1).pTraj = this;
//			nAliveLeft++;
//		}
//		else if(p.beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE && nAliveRight < BEH_PARTICLES_NUM)
//		{
//			m_RightPart.push_back(p);
//			m_RightPart.at(m_RightPart.size()-1).pTraj = this;
//			nAliveRight++;
//		}
	}

	void ClearParticles()
	{
		m_CurrParts.clear();
		nAliveStop = 0;
		nAliveYield = 0;
		nAlivePark = 0;
		nAliveForward = 0;
		nAliveLeft = 0;
		nAliveRight = 0;

//		m_ForwardPart.clear();
//		m_LeftPart.clear();
//		m_RightPart.clear();
//		m_YieldPart.clear();
	}

	void SortParticlesByWeight()
	{
		std::sort(m_CurrParts.begin(), m_CurrParts.end(), sort_weights_des);
	}

	void DeleteParticlesWithPercentageThan(std::vector<Particle>& parts_list, double percentage, const bool& bForce = false)
	{
		int i_erase = ((double)parts_list.size() * percentage) - 1;

		for(int i = (int)parts_list.size()-1; i >= i_erase && i >= 0; i--)
		{
			if(parts_list.at(i).beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				if(nAliveStop == min_particles_number && bForce == true) continue;
				nAliveStop--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_YIELDING_STATE)
			{
				if(nAliveYield == min_particles_number && bForce == true) continue;
				nAliveYield--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_PARKING_STATE)
			{
				if(nAlivePark == min_particles_number && bForce == true) continue;
				nAlivePark--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_FORWARD_STATE)
			{
				if(nAliveForward == min_particles_number  && bForce == true) continue;
				nAliveForward--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
			{
				if(nAliveLeft == min_particles_number && bForce == true) continue;
				nAliveLeft--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
			{
				if(nAliveRight == min_particles_number && bForce == true) continue;
				nAliveRight--;
			}

			parts_list.erase(parts_list.begin()+i);
			//i++;
		}
	}

	void DeleteParticles(double cut_off, double percent, const bool& bForce = false)
	{
		SortParticlesByWeight();

		if(m_CurrParts.size() > 2 && fabs(m_CurrParts.at(0).w - m_CurrParts.at(m_CurrParts.size()-1).w) < 0.0001)
		  return;

		if(percent > 0 && percent < 1)
			DeleteParticlesWithPercentageThan(m_CurrParts, percent, bForce);

		if(cut_off > 0 && cut_off < 1)
			DeleteParticlesWithWeightLessThan(m_CurrParts, cut_off, bForce);
	}

	void DeleteParticlesWithWeightLessThan(std::vector<Particle>& parts_list, double cut_off, const bool& bForce = false)
	{
		//std::cout << "Before Delete: ";
		int i_erase = -1;
		for(unsigned int i = 0; i < parts_list.size(); i++)
		{
			//std::cout << parts_list.at(i).w << ", ";
			if(parts_list.at(i).w < cut_off)
			{
				i_erase = i;
				break;
			}
		}
		//std::cout << i_erase << std::endl;

		for(int i = (int)parts_list.size()-1; i >= i_erase && i >= 0; i--)
		{
			if(parts_list.at(i).beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				if(nAliveStop == min_particles_number && bForce == true) continue;
				nAliveStop--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_YIELDING_STATE)
			{
				if(nAliveYield == min_particles_number && bForce == true) continue;
				nAliveYield--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_PARKING_STATE)
			{
				if(nAlivePark == min_particles_number && bForce == true) continue;
				nAlivePark--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_FORWARD_STATE)
			{
				if(nAliveForward == min_particles_number  && bForce == true) continue;
				nAliveForward--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
			{
				if(nAliveLeft == min_particles_number && bForce == true) continue;
				nAliveLeft--;
			}
			else if(parts_list.at(i).beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
			{
				if(nAliveRight == min_particles_number && bForce == true) continue;
				nAliveRight--;
			}

			parts_list.erase(parts_list.begin()+i);
			//i++;
		}
	}

	void CalcAveragesAndProb()
	{
		double s_sum = 0;
		double f_sum = 0;
		double y_sum = 0;
		double p_sum = 0;
		double l_sum = 0;
		double r_sum = 0;
		double all_sum = 0;
		double all_sum_real = 0;
		w_avg_forward = 0;
		w_avg_stop = 0;
		w_avg_yield = 0;
		w_avg_park = 0;
		w_avg_left = 0;
		w_avg_right = 0;
		pStop = 0;
		pYield = 0;
		pPark = 0;
		pForward = 0;
		pLeft = 0;
		pRight = 0;

		for(unsigned int i = 0; i < m_CurrParts.size(); i++)
		{
			all_sum += m_CurrParts.at(i).w;
			all_sum_real += m_CurrParts.at(i).w_real;

			if(m_CurrParts.at(i).beh == PlannerHNS::BEH_STOPPING_STATE)
				s_sum += m_CurrParts.at(i).w;
			else if(m_CurrParts.at(i).beh == PlannerHNS::BEH_YIELDING_STATE)
				y_sum += m_CurrParts.at(i).w;
			else if(m_CurrParts.at(i).beh == PlannerHNS::BEH_PARKING_STATE)
				p_sum += m_CurrParts.at(i).w;
			else if(m_CurrParts.at(i).beh == PlannerHNS::BEH_FORWARD_STATE)
				f_sum += m_CurrParts.at(i).w;
			else if(m_CurrParts.at(i).beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
				l_sum += m_CurrParts.at(i).w;
			else if(m_CurrParts.at(i).beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
				r_sum += m_CurrParts.at(i).w;
		}

		if(nAliveForward > 0)
			w_avg_forward = f_sum/(double)nAliveForward;
		if(nAliveStop > 0)
			w_avg_stop = s_sum/(double)nAliveStop;
		if(nAliveYield > 0)
			w_avg_yield = y_sum/(double)nAliveYield;
		if(nAlivePark > 0)
			w_avg_park = p_sum/(double)nAlivePark;
		if(nAliveLeft > 0)
			w_avg_left = l_sum/(double)nAliveLeft;
		if(nAliveRight > 0)
			w_avg_right = r_sum/(double)nAliveRight;

		if(m_CurrParts.size() > 0)
		{
			all_w = all_sum/(double)m_CurrParts.size();
			all_w_real = all_sum_real/(double)m_CurrParts.size();
		}


		pStop  = (double)nAliveStop/(double)max_particles_number;
		pYield = (double)nAliveYield/(double)max_particles_number;
		pPark = (double)nAlivePark/(double)max_particles_number;
		pForward = (double)nAliveForward/(double)max_particles_number;
		pLeft = (double)nAliveLeft/(double)max_particles_number;
		pRight = (double)nAliveRight/(double)max_particles_number;

		all_p = (double)(nAliveStop + nAliveForward + nAliveYield + nAliveLeft + nAliveRight + nAlivePark) / (double)(total_particles_number);

//		for(unsigned int i = 0; i < m_ForwardPart.size(); i++)
//		{
//			avg_sum += m_ForwardPart.at(i).w;
//		}
//		if(m_ForwardPart.size() > 0)
//			w_avg_forward = avg_sum/(double)m_ForwardPart.size();
//
//		w_avg_left = 0;
//		avg_sum = 0;
//		for(unsigned int i = 0; i < m_LeftPart.size(); i++)
//		{
//			avg_sum += m_LeftPart.at(i).w;
//		}
//		if(m_LeftPart.size() > 0)
//			w_avg_left = avg_sum/(double)m_LeftPart.size();
//
//		w_avg_right = 0;
//		avg_sum = 0;
//		for(unsigned int i = 0; i < m_RightPart.size(); i++)
//		{
//			avg_sum += m_RightPart.at(i).w;
//		}
//		if(m_RightPart.size() > 0)
//			w_avg_right = avg_sum/(double)m_RightPart.size();
//
//		w_avg_stop = 0;
//		avg_sum = 0;
//		for(unsigned int i = 0; i < m_StopPart.size(); i++)
//		{
//			avg_sum += m_StopPart.at(i).w;
//		}
//		if(m_StopPart.size() > 0)
//			w_avg_stop = avg_sum/(double)m_StopPart.size();
//
//		w_avg_yield = 0;
//		avg_sum = 0;
//		for(unsigned int i = 0; i < m_YieldPart.size(); i++)
//		{
//			avg_sum += m_YieldPart.at(i).w;
//		}
//		if(m_YieldPart.size() > 0)
//			w_avg_yield = avg_sum/(double)m_YieldPart.size();
	}

	void CalcBest()
	{
		best_p = 0.0;
		if(pStop > best_p)
		{
			best_p = pStop;
			best_beh_by_p = PlannerHNS::BEH_STOPPING_STATE;
		}

		if(pYield > best_p)
		{
			best_p = pYield;
			best_beh_by_p = PlannerHNS::BEH_YIELDING_STATE;
		}

		if(pPark > best_p)
		{
			best_p = pPark;
			best_beh_by_p = PlannerHNS::BEH_PARKING_STATE;
		}

		if(pForward > best_p)
		{
			best_p = pForward;
			best_beh_by_p = PlannerHNS::BEH_FORWARD_STATE;
		}

		if(pLeft > best_p)
		{
			best_p = pLeft;
			best_beh_by_p = PlannerHNS::BEH_BRANCH_LEFT_STATE;
		}

		if(pRight > best_p)
		{
			best_p = pRight;
			best_beh_by_p = PlannerHNS::BEH_BRANCH_RIGHT_STATE;
		}

		best_w = 0.0;
		if(w_avg_stop > best_w)
		{
			best_w = w_avg_stop;
			best_beh_by_w = PlannerHNS::BEH_STOPPING_STATE;
		}

		if(w_avg_yield > best_w)
		{
			best_w = w_avg_yield;
			best_beh_by_w = PlannerHNS::BEH_YIELDING_STATE;
		}

		if(w_avg_park > best_w)
		{
			best_w = w_avg_park;
			best_beh_by_w = PlannerHNS::BEH_PARKING_STATE;
		}

		if(w_avg_forward > best_w)
		{
			best_w = w_avg_forward;
			best_beh_by_w = PlannerHNS::BEH_FORWARD_STATE;
		}

		if(w_avg_left > best_w)
		{
			best_w = w_avg_left;
			best_beh_by_w = PlannerHNS::BEH_BRANCH_LEFT_STATE;
		}

		if(w_avg_right > best_w)
		{
			best_w = w_avg_right;
			best_beh_by_w = PlannerHNS::BEH_BRANCH_RIGHT_STATE;
		}
	}
};

class ObjParticles
{
public:
	DetectedObject obj;
	std::vector<TrajectoryTracker*> m_TrajectoryTracker;
	std::vector<TrajectoryTracker*> m_TrajectoryTracker_temp;

	std::vector<Particle*> m_AllParticles;
	std::vector<Particle> m_AllGeneratedParticles;

	TrajectoryTracker* best_behavior_track;
	TrajectoryTracker* best_forward_track;
	int i_best_beh_track;
	int i_best_for_track;

	//PlannerHNS::BehaviorState m_beh;
	double m_PredictionDistance;

	//double all_w;
	//double all_w_real;
	double max_w;
	double min_w;
	double max_w_raw;
	double min_w_raw;

	double pose_w_real_t;
	double pose_w_t;
	double dir_w_t;
	double dir_w_real_t;
	double vel_w_t;
	double acl_w_t;
	double ind_w_t;

	double pose_w_max;
	double pose_w_real_max;
	double dir_w_max;
	double dir_w_real_max;
	double vel_w_max;
	double acl_w_max;
	double ind_w_max;

	double pose_w_min;
	double pose_w_real_min;
	double dir_w_min;
	double dir_w_real_min;
	double vel_w_min;
	double acl_w_min;
	double ind_w_min;

//	double pose_w_max_norm;
//	double dir_w_max_norm;
//	double vel_w_max_norm;
//	double acl_w_max_norm;
//	double ind_w_max_norm;
//
//	double pose_w_min_norm;
//	double dir_w_min_norm;
//	double vel_w_min_norm;
//	double acl_w_min_norm;
//	double ind_w_min_norm;

	double pose_diff_raw;
	double pose_real_diff_raw;
	double dir_diff_raw ;
	double dir_real_diff_raw ;
	double vel_diff_raw ;
	double ind_diff_raw ;
	double acl_diff_raw ;
	double total_diff_raw ;

	double pose_factor;
	double pose_factor_real;
	double dir_factor;
	double dir_factor_real;
	double vel_factor;
	double acl_factor;
	double ind_factor;

	bool bCanDecide;

	double m_t;
	timespec m_LogTime;
	std::vector<std::string>  m_LogData;


	virtual ~ObjParticles()
	{
		DeleteTheRest(m_TrajectoryTracker);
		m_TrajectoryTracker_temp.clear();
	}

	ObjParticles(PredictionParams _predParams)
	{
		m_t = 0;
		bCanDecide = true;
		m_PredictionDistance = 0;
		best_behavior_track = nullptr;
		best_forward_track = nullptr;
		i_best_beh_track = -1;
		i_best_for_track = -1;

		InitWeightsVariables(_predParams);
	}

	void InitWeightsVariables(PredictionParams _predParams)
	{
		//all_w = 0;
		//all_w_real = 0;
		pose_w_t = 0;
		pose_w_real_t = 0;
		dir_w_t = 0;
		dir_w_real_t = 0;
		vel_w_t = 0;
		acl_w_t = 0;
		ind_w_t = 0;
		max_w = DBL_MIN;
		min_w = DBL_MAX;
		max_w_raw = DBL_MIN;
		min_w_raw = DBL_MAX;

		pose_w_max = DBL_MIN;
		pose_w_real_max = DBL_MIN;
		dir_w_max = DBL_MIN;
		dir_w_real_max = DBL_MIN;
		vel_w_max = DBL_MIN;
		acl_w_max = DBL_MIN;
		ind_w_max = DBL_MIN;

		pose_w_min = DBL_MAX;
		pose_w_real_min = DBL_MAX;
		dir_w_min = DBL_MAX;
		dir_w_real_min = DBL_MAX;
		vel_w_min = DBL_MAX;
		acl_w_min = DBL_MAX;
		ind_w_min = DBL_MAX;

//		pose_w_max_norm = DBL_MIN;
//		dir_w_max_norm = DBL_MIN;
//		vel_w_max_norm = DBL_MIN;
//		acl_w_max_norm = DBL_MIN;
//		ind_w_max_norm = DBL_MIN;
//
//		pose_w_min_norm = DBL_MAX;
//		dir_w_min_norm = DBL_MAX;
//		vel_w_min_norm = DBL_MAX;
//		acl_w_min_norm = DBL_MAX;
//		ind_w_min_norm = DBL_MAX;


		pose_diff_raw = 0.0;
		pose_real_diff_raw = 0.0;
		dir_diff_raw = 0.0;
		dir_real_diff_raw = 0.0;
		vel_diff_raw = 0.0;
		ind_diff_raw = 0.0;
		acl_diff_raw = 0.0;
		total_diff_raw = 0.0;

		pose_factor = _predParams.POSE_FACTOR;
		pose_factor_real = _predParams.POSE_FACTOR;
		dir_factor = _predParams.DIRECTION_FACTOR;
		dir_factor_real = _predParams.DIRECTION_FACTOR;
		vel_factor = _predParams.VELOCITY_FACTOR;
		acl_factor = _predParams.ACCELERATE_FACTOR;
		ind_factor = _predParams.INDICATOR_FACTOR;
	}

	void BalanceFactorsToOne()
	{
		std::vector<double> factors_list = {pose_factor, dir_factor, vel_factor, acl_factor, ind_factor};
		int nNonZero = 0;
		for(unsigned int i=0;i < factors_list.size(); i++)
		{
			if(factors_list.at(i) > 0.0)
				nNonZero++;
		}
		double all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

		while(all_factors > 1.01 || all_factors < 0.99)
		{
			//std::cout << "Current Factors: ";
//			for(unsigned int i=0;i < factors_list.size(); i++)
//				std::cout << factors_list.at(i) << ",";
			//std::cout << std::endl;


			double every_one_share = (1.0 - all_factors)/(double)nNonZero;

			//std::cout << "All Factors: " << all_factors << ", NonZero: " << nNonZero << ", Shares: " << every_one_share << std::endl;

			for(unsigned int i=0;i < factors_list.size(); i++)
			{
				if(factors_list.at(i) > 0.0)
				{
					factors_list.at(i) += every_one_share;
				}

				if(factors_list.at(i) < 0.0)
					factors_list.at(i) = 0.0;
			}

			nNonZero = 0;
			for(unsigned int i=0;i < factors_list.size(); i++)
			{
				if(factors_list.at(i) > 0.0)
					nNonZero++;
			}
			all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

			if(all_factors == 0)
				break;
		}

//		std::cout << "Current Factors: ";
//		for(unsigned int i=0;i < factors_list.size(); i++)
//			std::cout << factors_list.at(i) << ",";
//		std::cout << std::endl;

		pose_factor = factors_list.at(0);
		dir_factor = factors_list.at(1);
		vel_factor = factors_list.at(2);
		acl_factor = factors_list.at(3);
		ind_factor = factors_list.at(4);

	}

	void BalanceFactorsToOneReal()
	{
		std::vector<double> factors_list = {pose_factor_real, dir_factor_real, vel_factor, acl_factor, ind_factor};
		int nNonZero = 0;
		for(unsigned int i=0;i < factors_list.size(); i++)
		{
			if(factors_list.at(i) > 0.0)
				nNonZero++;
		}
		double all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

		while(all_factors > 1.01 || all_factors < 0.99)
		{
			double every_one_share = (1.0 - all_factors)/(double)nNonZero;
			for(unsigned int i=0;i < factors_list.size(); i++)
			{
				if(factors_list.at(i) > 0.0)
				{
					factors_list.at(i) += every_one_share;
				}

				if(factors_list.at(i) < 0.0)
					factors_list.at(i) = 0.0;
			}

			nNonZero = 0;
			for(unsigned int i=0;i < factors_list.size(); i++)
			{
				if(factors_list.at(i) > 0.0)
					nNonZero++;
			}
			all_factors = std::accumulate(factors_list.begin(), factors_list.end(), 0.0);

			if(all_factors == 0)
				break;
		}

		pose_factor_real = factors_list.at(0);
		dir_factor_real = factors_list.at(1);
		vel_factor = factors_list.at(2);
		acl_factor = factors_list.at(3);
		ind_factor = factors_list.at(4);

	}

	class LLP
	{
	public:
		int new_index;
		double match_percent;
		TrajectoryTracker* pTrack;

		LLP()
		{
			new_index = -1;
			match_percent = -1;
			pTrack = 0;

		}
	};

	void DeleteFromList(std::vector<TrajectoryTracker*>& delete_me_track, const TrajectoryTracker* track)
	{
		for(unsigned int k = 0; k < delete_me_track.size(); k++)
		{
			if(delete_me_track.at(k) == track)
			{
				delete_me_track.erase(delete_me_track.begin()+k);
				return;
			}
		}
	}

	void DeleteTheRest(std::vector<TrajectoryTracker*>& delete_me_track)
	{
		for(unsigned int k = 0; k < delete_me_track.size(); k++)
		{
			delete delete_me_track.at(k);
		}

		delete_me_track.clear();
	}

	static bool IsBeggerPercentage(const LLP& p1, const LLP& p2)
	{
		return p1.match_percent > p2.match_percent;
	}

	void MatchWithMax(std::vector<LLP>& matching_list, std::vector<TrajectoryTracker*>& delete_me_track, std::vector<TrajectoryTracker*>& check_list)
	{
		if(matching_list.size() == 0 ) return;

		std::sort(matching_list.begin(), matching_list.end(), IsBeggerPercentage);

		while(matching_list.size()>0)
		{
			LLP f = matching_list.at(0);
			f.pTrack->UpdatePathAndIndex(obj.predTrajectories.at(f.new_index), f.new_index);

			bool bFound = false;
			for(unsigned int k=0; k < check_list.size(); k++)
			{
				if(check_list.at(k) == f.pTrack)
				{
					bFound = true;
					break;
				}
			}

			if(!bFound)
				check_list.push_back(f.pTrack);

			DeleteFromList(delete_me_track, f.pTrack);
			for(int i=0; i < matching_list.size(); i++)
			{
				if(matching_list.at(i).new_index == f.new_index || matching_list.at(i).pTrack == f.pTrack)
				{
					matching_list.erase(matching_list.begin()+i);
					i--;
				}
			}
		}
	}

	void MatchTrajectories()
	{
		m_TrajectoryTracker_temp.clear();
		std::vector<LLP> matching_list;
		std::vector<TrajectoryTracker*> delete_me_track = m_TrajectoryTracker;
		for(unsigned int t = 0; t < obj.predTrajectories.size();t++)
		{
			bool bMatched = false;
			LLP match_item;
			match_item.new_index = t;

			for(int i = 0; i < m_TrajectoryTracker.size(); i++)
			{
			    TrajectoryTracker* pTracker = m_TrajectoryTracker.at(i);

				double vMatch = pTracker->CalcMatchingPercentage(obj.predTrajectories.at(t));
				if(vMatch == 1.0) // perfect match
				{
				    pTracker->UpdatePathAndIndex(obj.predTrajectories.at(t), t);
				    bool bFound = false;
				    for(unsigned int k=0; k < m_TrajectoryTracker_temp.size(); k++)
				    {
					    if(m_TrajectoryTracker_temp.at(k) == pTracker)
					    {
						    bFound = true;
						    break;
					    }
				    }

					if(!bFound)
					  m_TrajectoryTracker_temp.push_back(pTracker);

					DeleteFromList(delete_me_track, pTracker);

					for(unsigned int k=0; k < matching_list.size(); k++)
					{
						if(matching_list.at(k).pTrack == pTracker)
						{
							matching_list.erase(matching_list.begin()+k);
							break;
						}
					}

					m_TrajectoryTracker.erase(m_TrajectoryTracker.begin()+i);
					bMatched = true;
					i--;
					break;
				}
				else if(vMatch > 0.5) // any matching less than 50%, the trajectory will be considered new
				{
					bMatched = true;
					match_item.match_percent = vMatch;
					match_item.pTrack = pTracker;
					matching_list.push_back(match_item);
				}
			}

			if(!bMatched)
			{
				m_TrajectoryTracker_temp.push_back(new TrajectoryTracker(obj.predTrajectories.at(t), t));
			}
		}

		MatchWithMax(matching_list,delete_me_track, m_TrajectoryTracker_temp);
		m_TrajectoryTracker.clear();
		DeleteTheRest(delete_me_track);
		m_TrajectoryTracker = m_TrajectoryTracker_temp;
	}

	static bool IsBeggerRealW(const TrajectoryTracker* T1, const TrajectoryTracker* T2)
	{
		return T1->all_w_real > T2->all_w_real;
	}

	void FindBestTracks()
	{
		for(unsigned int t=0; t < m_TrajectoryTracker.size(); t++)
		{
			m_TrajectoryTracker.at(t)->CalcBest();
		}

//		if(m_TrajectoryTracker.size() == 1)
//		{
//			best_forward_track = m_TrajectoryTracker.at(0);
//			i_best_for_track = 0;
//		}
//		else
//		{
//			best_forward_track = nullptr;
//			i_best_for_track = -1;
//			std::vector<TrajectoryTracker*> trajectoryTracker_srt = m_TrajectoryTracker;
//			std::sort(trajectoryTracker_srt.begin(), trajectoryTracker_srt.end(), IsBeggerRealW);
//			if(trajectoryTracker_srt.at(0)->all_w_real - trajectoryTracker_srt.at(1)->all_w_real > 0.2)
//			{
//				best_forward_track = trajectoryTracker_srt.at(0);
//				i_best_for_track = trajectoryTracker_srt.at(0)->index;
//			}
//			else // check if can't find the previous in the current list
//			{
//				bool bFound = false;
//				for(unsigned int t = 0; t < m_TrajectoryTracker.size(); t++)
//				{
//					if(best_forward_track == m_TrajectoryTracker.at(t))
//					{
//						bFound = true;
//						break;
//					}
//				}
//
//				if(!bFound)
//				{
//					best_forward_track = nullptr;
//					i_best_for_track = -1;
//				}
//			}
//		}

		best_behavior_track = nullptr;
		best_forward_track = nullptr;
		i_best_beh_track = -1;
		i_best_for_track = -1;

		if(m_TrajectoryTracker.size() > 0)
		{
			best_behavior_track = m_TrajectoryTracker.at(0);
			i_best_beh_track = 0;
			best_forward_track = m_TrajectoryTracker.at(0);
			i_best_for_track = 0;
		}

		for(unsigned int t = 1; t < m_TrajectoryTracker.size(); t++)
		{
			if(m_TrajectoryTracker.at(t)->all_w_real > best_forward_track->all_w_real)
			{
				best_forward_track = m_TrajectoryTracker.at(t);
				i_best_for_track = t;
			}

			if(m_TrajectoryTracker.at(t)->best_w > best_behavior_track->best_w)
			{
				best_behavior_track = m_TrajectoryTracker.at(t);
				i_best_beh_track = t;
			}

//			if(m_TrajectoryTracker.at(t)->all_p > best_behavior_track->all_p || (m_TrajectoryTracker.at(t)->all_p == best_behavior_track->all_p && m_TrajectoryTracker.at(t)->best_w > best_behavior_track->best_w))
//			{
//				best_behavior_track = m_TrajectoryTracker.at(t);
//				i_best_beh_track = t;
//			}
		}
	}

	std::string GetPredictionLogDataLine(std::vector<PlannerHNS::TrajectoryTracker*> trajectoryTrackers, std::string path_id)
	{
	  for(unsigned int i=0; i < trajectoryTrackers.size(); i++)
	  {
	    if(trajectoryTrackers.at(i)->id_.compare(path_id) == 0)
	    {
	      std::ostringstream dataLine;
	      dataLine << trajectoryTrackers.at(i)->id_ << ",";
	      dataLine << trajectoryTrackers.at(i)->nAliveForward << "," << trajectoryTrackers.at(i)->pForward << "," << trajectoryTrackers.at(i)->w_avg_forward <<",";
	      dataLine << trajectoryTrackers.at(i)->nAliveStop << "," << trajectoryTrackers.at(i)->pStop << "," << trajectoryTrackers.at(i)->w_avg_stop <<",";
	      dataLine << trajectoryTrackers.at(i)->nAliveYield << "," << trajectoryTrackers.at(i)->pYield << "," << trajectoryTrackers.at(i)->w_avg_yield <<",";
	      dataLine << trajectoryTrackers.at(i)->nAlivePark << "," << trajectoryTrackers.at(i)->pPark << "," << trajectoryTrackers.at(i)->w_avg_park <<",";
	      dataLine << trajectoryTrackers.at(i)->best_beh_by_p <<",";
	      dataLine << trajectoryTrackers.at(i)->all_p << "," << trajectoryTrackers.at(i)->all_w << "," << trajectoryTrackers.at(i)->best_p <<"," << trajectoryTrackers.at(i)->best_w <<"," << trajectoryTrackers.at(i)->all_w_real <<",";
	      return dataLine.str();
	    }
	  }

	  return "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,";
	}

	std::string GetPredictionLogDataRealWeightItem(std::vector<PlannerHNS::TrajectoryTracker*> trajectoryTrackers, std::string path_id)
	{
	  for(unsigned int i=0; i < trajectoryTrackers.size(); i++)
	  {
	    if(trajectoryTrackers.at(i)->id_.compare(path_id) == 0)
	    {
	      std::ostringstream dataLine;
	      dataLine << trajectoryTrackers.at(i)->all_w_real <<",";
	      return dataLine.str();
	    }
	  }

	  return "0,";
	}

	void LogDataRow()
	{

		std::ostringstream dataLine;
		dataLine << m_t << "," << obj.center.pos.x << "," <<  obj.center.pos.y << "," << obj.center.pos.a << "," << obj.center.v << "," << obj.acceleration_desc
					  << "," << obj.indicator_state << ",";

		if(best_forward_track != nullptr)
		{
		if(best_forward_track->id_.compare("F")==0)
		  dataLine << "1" << ",";
		else if(best_forward_track->id_.compare("L")==0)
		  dataLine << "2" << ",";
		else if(best_forward_track->id_.compare("R")==0)
		  dataLine << "3" << ",";
		else
		  dataLine << "0" << ",";
		}
		else
		dataLine << "0" << ",";


		dataLine << GetPredictionLogDataRealWeightItem(m_TrajectoryTracker, "F");
		dataLine << GetPredictionLogDataRealWeightItem(m_TrajectoryTracker, "L");
		dataLine << GetPredictionLogDataRealWeightItem(m_TrajectoryTracker, "R");

		if(best_behavior_track != nullptr)
		{
		  dataLine << best_behavior_track->best_beh_by_p << ",";
		  dataLine << best_behavior_track->best_beh_by_w << ",";
		  dataLine << best_behavior_track->w_avg_forward << ",";
		  dataLine << best_behavior_track->w_avg_stop << ",";
		  dataLine << best_behavior_track->w_avg_yield << ",";
		  dataLine << best_behavior_track->w_avg_park << ",";
		}
		else
			dataLine << "-1,-1,0,0,0,0,";

		dataLine << GetPredictionLogDataLine(m_TrajectoryTracker, "F");
		dataLine << GetPredictionLogDataLine(m_TrajectoryTracker, "L");
		dataLine << GetPredictionLogDataLine(m_TrajectoryTracker, "R");
		dataLine << GetPredictionLogDataLine(m_TrajectoryTracker, "U");

		m_LogData.push_back(dataLine.str());

		if(m_t==0)
		{
			  UtilityHNS::UtilityH::GetTickCount(m_LogTime);
			  m_t = 0.01;
		}
		else
		{
			  m_t += UtilityHNS::UtilityH::GetTimeDiffNow(m_LogTime);
			  UtilityHNS::UtilityH::GetTickCount(m_LogTime);
		}
	}
};

class BehaviorPrediction
{
public:
	BehaviorPrediction();
	virtual ~BehaviorPrediction();
	void DoOneStep(const std::vector<DetectedObject>& obj_list, const WayPoint& currPose, const double& minSpeed, const double& maxDeceleration, RoadNetwork& map);

public:
	std::vector<PassiveDecisionMaker*> m_d_makers;
	double m_PredictionHorizon;
	double m_MaxLaneDetectionDistance;
	double m_MaxPredictionDistance;
	bool m_bGenerateBranches;
	bool m_bStepByStep;
	bool m_bParticleFilter;

	std::vector<ObjParticles*> m_temp_list;
	std::vector<ObjParticles*> m_ParticleInfo;

	struct timespec m_GenerationTimer;
	timespec m_ResamplingTimer;

	//bool m_bCanDecide;
	bool m_bFirstMove;
	bool m_bDebugOut;
	bool m_bDebugMotion;
	bool m_bDebugOutWeights;


protected:
	//int GetTrajectoryPredictedDirection(const std::vector<WayPoint>& path, const PlannerHNS::WayPoint& pose, const double& pred_distance);
	int FromIndicatorToNumber(const PlannerHNS::LIGHT_INDICATOR& ind);
	PlannerHNS::LIGHT_INDICATOR FromNumbertoIndicator(const int& num);
	double CalcIndicatorWeight(int p_ind, int obj_ind);
	double CalcAccelerationWeight(int p_acl, int obj_acl);

	void CalPredictionTimeForObject(ObjParticles* pCarPart, const double& max_pre_distance);
	void PredictCurrentTrajectory(RoadNetwork& map, ObjParticles* pCarPart);
	void ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& obj_list, RoadNetwork& map, std::vector<ObjParticles*>& old_list);
	void CalculateCollisionTimes(const double& minSpeed);

	void ParticleFilterSteps(std::vector<ObjParticles*>& part_info);

	void SamplesFreshParticles(ObjParticles* pParts);
	void MoveParticles(ObjParticles* parts);
	void CalculateWeights(ObjParticles* pParts);

	void CalOnePartWeight(ObjParticles* pParts,Particle& p);
	void NormalizeOnePartWeight(ObjParticles* pParts,Particle& p);

	void CollectParticles(ObjParticles* pParts);

	void RemoveWeakParticles(ObjParticles* pParts);
	void FindBest(ObjParticles* pParts);
	void CalculateAveragesAndProbabilities(ObjParticles* pParts);
	void CalculateAccelerationDESC(double dt, Particle* pPart);

	static bool sort_trajectories(const std::pair<int, double>& p1, const std::pair<int, double>& p2)
	{
		return p1.second > p2.second;
	}


public:
	PredictionParams g_PredParams;
	std::vector<std::pair<int, std::vector<std::string> > >  m_AllLogData;

	void SetForTrajTracker()
	{
		TrajectoryTracker::max_particles_number = g_PredParams.MAX_PARTICLES_NUM;
		TrajectoryTracker::min_particles_number = g_PredParams.MIN_PARTICLES_NUM;
		TrajectoryTracker::total_particles_number = TrajectoryTracker::max_particles_number * TrajectoryTracker::active_intentions_number;
	}

	//move to CPP later
	void DeleteFromList(std::vector<ObjParticles*>& delete_me, const ObjParticles* pElement)
	{
		for(unsigned int k = 0; k < delete_me.size(); k++)
		{
			if(delete_me.at(k) == pElement)
			{
				delete_me.erase(delete_me.begin()+k);
				return;
			}
		}
	}

	void DeleteTheRest(std::vector<ObjParticles*>& delete_me)
	{
		for(unsigned int k = 0; k < delete_me.size(); k++)
		{
			m_AllLogData.push_back(std::make_pair(delete_me.at(k)->obj.id, delete_me.at(k)->m_LogData));
			delete delete_me.at(k);
		}

		delete_me.clear();
	}

	void GetBestParticleWeight(std::vector<Particle>& part_list, Particle& pBestF, Particle& pBestS, Particle& pBestY, Particle& pBestP)
	{

		if(part_list.size() < 1) return;

		double f_max_w = DBL_MIN;
		unsigned int f_max_i = -1;

		double s_max_w = DBL_MIN;
		unsigned int s_max_i = -1;

		double y_max_w = DBL_MIN;
		unsigned int y_max_i = -1;

		double p_max_w = DBL_MIN;
		unsigned int p_max_i = -1;

		for(unsigned int i=0; i < part_list.size(); i++)
		{
			if(part_list.at(i).beh == BEH_FORWARD_STATE)
			{
				if(part_list.at(i).w > f_max_w)
				{
					f_max_w = part_list.at(i).w;
					f_max_i = i;
				}
			}
			else if(part_list.at(i).beh == BEH_STOPPING_STATE)
			{
				if(part_list.at(i).w > s_max_w)
				{
					s_max_w = part_list.at(i).w;
					s_max_i = i;
				}
			}
			else if(part_list.at(i).beh == BEH_YIELDING_STATE)
			{
				if(part_list.at(i).w > y_max_w)
				{
					y_max_w = part_list.at(i).w;
					y_max_i = i;
				}
			}
			else if(part_list.at(i).beh == BEH_PARKING_STATE)
			{
				if(part_list.at(i).w > p_max_w)
				{
					p_max_w = part_list.at(i).w;
					p_max_i = i;
				}
			}
		}

		if(f_max_i >= 0)
		{
			pBestF = part_list.at(f_max_i);
		}

		if(s_max_i >= 0)
		{
			pBestS = part_list.at(s_max_i);
		}

		if(y_max_i >= 0)
		{
			pBestY = part_list.at(y_max_i);
		}
		if(p_max_i >= 0)
		{
			pBestP = part_list.at(p_max_i);
		}



	}

	Particle* GetBestParticleWeight(std::vector<Particle>& part_list)
	{
		if(part_list.size() < 1) return nullptr;

		double max_w = part_list.at(0).w;
		unsigned int max_i = 0;

		for(unsigned int i=1; i < part_list.size(); i++)
		{
			if(part_list.at(i).w > max_w)
			{
				max_w = part_list.at(i).w;
				max_i = i;
			}
		}

		return &part_list.at(max_i);
	}

};



} /* namespace PlannerHNS */

#endif /* BEHAVIORPREDICTION_H_ */
