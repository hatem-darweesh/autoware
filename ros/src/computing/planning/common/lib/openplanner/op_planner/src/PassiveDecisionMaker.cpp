
/// \file  PassiveDecisionMaker.cpp
/// \brief Decision Maker for surrounding vehicle, to be integrated with particle filter for intention prediction
/// \author Hatem Darweesh
/// \date Jan 10, 2018

#include "op_planner/PassiveDecisionMaker.h"
#include "op_planner/PlanningHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"


namespace PlannerHNS
{

constexpr double PLANNING_HORIZON = 120;
constexpr double ADD_BREAK_DISTANCE = 5.0;
constexpr double MIN_INDICATOR_DISTANCE = 15.0;

PassiveDecisionMaker::PassiveDecisionMaker()
{
	m_MinStoppingDistance = PLANNING_HORIZON;
	m_VelocityFromPath = 0;
	m_DistanceToCarEdge = 0;
}

PassiveDecisionMaker& PassiveDecisionMaker::operator=(const PassiveDecisionMaker& obj)
{
	return *this;
}

PassiveDecisionMaker::PassiveDecisionMaker(const PassiveDecisionMaker& obj)
{
	m_MinStoppingDistance = PLANNING_HORIZON;
	m_VelocityFromPath = 0;
	m_DistanceToCarEdge = 0;
}

PassiveDecisionMaker::~PassiveDecisionMaker()
{
}

 double PassiveDecisionMaker::GetVelocity(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo, const RelativeInfo& info)
 {
	int prev_index = 0;
	double velocity = PlannerHNS::PlanningHelpers::GetVelocityAhead(path, info, prev_index, m_MinStoppingDistance);
	if(velocity > carInfo.max_speed_forward)
		velocity = carInfo.max_speed_forward;
	else if(velocity < carInfo.min_speed_forward)
		velocity = carInfo.min_speed_forward;

	return velocity;
 }

 double PassiveDecisionMaker::GetSteerAngle(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const RelativeInfo& info)
  {
     unsigned int point_index = 0;

     PlannerHNS::WayPoint pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(path, info, 2, point_index);
     double current_a = UtilityHNS::UtilityH::SplitPositiveAngle(currPose.pos.a);
     double target_a = atan2(pursuite_point.pos.y - currPose.pos.y, pursuite_point.pos.x - currPose.pos.x);
     double a_positive =  UtilityHNS::UtilityH::SplitPositiveAngle(target_a - current_a);

     return a_positive;

  }

 bool PassiveDecisionMaker::CheckForStopLine(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
 {
	int stopLineID = -1;
	int stopSignID = -1;
	int trafficLightID = -1;
	double distanceToClosestStopLine = 0;

	distanceToClosestStopLine = PlanningHelpers::GetDistanceToClosestStopLineAndCheck(path, currPose, 0, stopLineID, stopSignID, trafficLightID) - m_DistanceToCarEdge;
	if(distanceToClosestStopLine > -2 && distanceToClosestStopLine < m_MinStoppingDistance)
	{
		return true;
	}

	return false;
 }

 void PassiveDecisionMaker::CalculateBasicDecisionParams(PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
 {
	 m_MinStoppingDistance = -pow(currPose.v, 2)/(carInfo.max_deceleration) + ADD_BREAK_DISTANCE;
	 m_DistanceToCarEdge = carInfo.length/2.0 + ADD_BREAK_DISTANCE;

	 PlanningHelpers::GetRelativeInfoLimited(path, currPose, m_RelativeInfo);
	 int prev_index = 0;
	 m_VelocityFromPath = PlannerHNS::PlanningHelpers::GetVelocityAhead(path, m_RelativeInfo, prev_index, m_MinStoppingDistance);
 }

 PlannerHNS::ParticleInfo PassiveDecisionMaker::MoveStepII(const double& dt, PlannerHNS::WayPoint& currPose, PlannerHNS::WayPoint& transPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
 {
	 BehaviorState beh;
	 PlannerHNS::ParticleInfo part_info;

	 if(path.size() == 0) return part_info;

	 CalculateBasicDecisionParams(currPose, path, carInfo);

	 bool bStopLine = CheckForStopLine(currPose, path, carInfo);
	 if(bStopLine)
	   {
		 beh.state = PlannerHNS::STOPPING_STATE;
	   }
	 else
	   {
		 beh.state = PlannerHNS::FORWARD_STATE;
	   }

	beh.indicator = PlanningHelpers::GetIndicatorsFromPath(path, currPose, m_MinStoppingDistance + MIN_INDICATOR_DISTANCE);

	//double planning_velocity = GetExpectedVelocity(path, currPose, beh, carInfo, dt);
	double steer = GetSteerAngle(currPose, path, m_RelativeInfo);

	if(steer > carInfo.max_steer_angle)
		steer = carInfo.max_steer_angle;
	if(steer < carInfo.min_steer_angle)
		steer = carInfo.min_steer_angle;

//	PlannerHNS::Mat3 rotationMat(-transPose.pos.a);
//	PlannerHNS::Mat3 translationMat(-transPose.pos.x, -transPose.pos.y);
//
//	PlannerHNS::GPSPoint updated_point;
//	updated_point = translationMat*currPose.pos;
//	updated_point = rotationMat*updated_point;
//	currPose.pos = updated_point;

	currPose.pos.x += transPose.pos.x;
	currPose.pos.y += transPose.pos.y;
	currPose.pos.a += transPose.pos.a;

//	currPose.pos.x += currPose.v * dt * cos(currPose.pos.a);
//	currPose.pos.y += currPose.v * dt * sin(currPose.pos.a);
//	currPose.pos.a += currPose.v * dt * tan(steer)  / carInfo.wheel_base;

	int prev_index = 0;
	part_info.vel = PlannerHNS::PlanningHelpers::GetVelocityAhead(path, m_RelativeInfo, prev_index, m_MinStoppingDistance)*0.75;
	part_info.indicator = beh.indicator;
	part_info.state = beh.state;
	part_info.info_to_path = m_RelativeInfo;

	return part_info;

 }

 double PassiveDecisionMaker::GetExpectedVelocity(const std::vector<WayPoint>& path, PlannerHNS::WayPoint& state, const BehaviorState& beh, const CAR_BASIC_INFO& carInfo, const double& dt)
  {
 	if(path.size() == 0) return 0;

 	if(beh.state == STOPPING_STATE || beh.state == TRAFFIC_LIGHT_STOP_STATE || beh.state == STOP_SIGN_STOP_STATE)
 	{
 		double deceleration_critical = 0;
 		double distance_to_stop = beh.stopDistance ;
 		if(distance_to_stop != 0)
 			deceleration_critical = (-state.v*state.v)/distance_to_stop;

 		if(deceleration_critical >= 0)
 			deceleration_critical = carInfo.max_deceleration;

 		double desiredVelocity = deceleration_critical * dt + state.v;

 		//std::cout << "Stopping : V: " << state.v << ", A: " << deceleration_critical << ", dt: " << dt << std::endl;

// 		if(desiredVelocity > carInfo.max_speed_forward)
// 			desiredVelocity = carInfo.max_speed_forward;
 		if(desiredVelocity < carInfo.min_speed_forward)
 			desiredVelocity = 0;

 		return desiredVelocity;
 	}
 	else if(beh.state == FOLLOW_STATE)
 	{

 		double deceleration_critical = 0;
 		double distance_to_stop = beh.followDistance -  m_DistanceToCarEdge - ADD_BREAK_DISTANCE;
 		double sudden_stop_distance = -pow((state.v - beh.followVelocity), 2)/carInfo.max_deceleration;

 		if(distance_to_stop != 0)
 			deceleration_critical = (-state.v*state.v)/distance_to_stop;

 		if(deceleration_critical >= 0)
 			deceleration_critical = carInfo.max_deceleration;

 		double desiredVelocity = deceleration_critical * dt + state.v;

// 		if(desiredVelocity > carInfo.max_speed_forward)
// 			desiredVelocity = carInfo.max_speed_forward;
 		if(desiredVelocity < 0)
 			desiredVelocity = 0;

 		return desiredVelocity;

 	}
 	else if(beh.state == FORWARD_STATE || beh.state == OBSTACLE_AVOIDANCE_STATE )
 	{
 		double desiredVelocity = carInfo.max_acceleration * dt + state.v;

 		//std::cout << "Velocity From Move Step: Curr=" <<  state.v << ", MaxFromPath: " << m_VelocityFromPath << ", Calculated: " << desiredVelocity <<std::endl;

// 		if(desiredVelocity > m_VelocityFromPath)
// 			desiredVelocity = m_VelocityFromPath;
 		if(desiredVelocity < 0)
 			desiredVelocity = 0;

 		return desiredVelocity;
 	}
 	else if(beh.state == STOP_SIGN_WAIT_STATE || beh.state == TRAFFIC_LIGHT_WAIT_STATE)
 	{
 		double desiredVelocity = 0;
 		return desiredVelocity;
 	}
 	else
 	{
 		double desiredVelocity = 0;
 		return desiredVelocity;
 	}
  }

// PlannerHNS::BehaviorState PassiveDecisionMaker::MoveStep(const double& dt, PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
// {
//	 PlannerHNS::BehaviorState beh;
//	 if(path.size() == 0) return beh;
//
//	 RelativeInfo info;
//	 PlanningHelpers::GetRelativeInfo(path, currPose, info);
//
//	 bool bStopLine = CheckForStopLine(currPose, path, carInfo);
//	 if(bStopLine)
//		 beh.state = PlannerHNS::STOPPING_STATE;
//	 else
//		 beh.state = PlannerHNS::FORWARD_STATE;
//
//	 double average_braking_distance = -pow(currPose.v, 2)/(carInfo.max_deceleration) + 1.0;
//
//	if(average_braking_distance  < 10)
//		average_braking_distance = 10;
//
//	beh.indicator = PlanningHelpers::GetIndicatorsFromPath(path, currPose, average_braking_distance);
//
//	currPose.v = beh.maxVelocity = GetVelocity(currPose, path, carInfo, info);
//
//	double steer = GetSteerAngle(currPose, path, info);
//
//	currPose.pos.x += currPose.v * dt * cos(currPose.pos.a);
//	currPose.pos.y += currPose.v * dt * sin(currPose.pos.a);
//	currPose.pos.a += currPose.v * dt * tan(steer)  / carInfo.wheel_base;
//
//	return beh;
//
// }
//
// PlannerHNS::ParticleInfo PassiveDecisionMaker::MoveStepSimple(const double& dt, PlannerHNS::WayPoint& currPose, const std::vector<WayPoint>& path, const CAR_BASIC_INFO& carInfo)
//  {
// 	 PlannerHNS::ParticleInfo beh;
// 	 if(path.size() == 0) return beh;
//
// 	 RelativeInfo info;
// 	 PlanningHelpers::GetRelativeInfo(path, currPose, info);
//
// 	 bool bStopLine = CheckForStopLine(currPose, path, carInfo);
// 	 if(bStopLine)
// 		 beh.state = PlannerHNS::STOPPING_STATE;
// 	 else
// 		 beh.state = PlannerHNS::FORWARD_STATE;
//
// 	m_MinStoppingDistance = -pow(currPose.v, 2)/(carInfo.max_deceleration) + ADD_BREAK_DISTANCE;
//
// 	PlannerHNS::WayPoint startPose = path.at(0);
//
// 	beh.indicator = PlanningHelpers::GetIndicatorsFromPath(path, startPose, m_MinStoppingDistance + MIN_INDICATOR_DISTANCE);
//
// 	if(info.iFront < path.size())
// 	{
// 		beh.vel = path.at(info.iFront).v;
// 	}
// 	else
// 		beh.vel = 0;
//
// 	double steer = GetSteerAngle(currPose, path, info);
//
// 	currPose.pos.x += currPose.v * dt * cos(currPose.pos.a);
// 	currPose.pos.y += currPose.v * dt * sin(currPose.pos.a);
// 	currPose.pos.a += currPose.v * dt * tan(steer)  / carInfo.wheel_base;
//
// 	return beh;
//
//  }

} /* namespace PlannerHNS */
