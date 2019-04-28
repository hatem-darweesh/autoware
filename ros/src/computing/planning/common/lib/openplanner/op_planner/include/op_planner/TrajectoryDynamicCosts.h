
/// \file TrajectoryDynamicCosts.h
/// \brief Calculate collision costs for roll out trajectory for free trajectory evaluation for OpenPlanner local planner version 1.5+
/// \author Hatem Darweesh
/// \date Jan 14, 2018


#ifndef TRAJECTORYDYNAMICCOSTS_H_
#define TRAJECTORYDYNAMICCOSTS_H_

#include "RoadNetwork.h"
#include "PlannerCommonDef.h"
#include "PlanningHelpers.h"

using namespace std;

namespace PlannerHNS
{

class TrajectoryDynamicCosts
{
public:
	TrajectoryDynamicCosts();
	virtual ~TrajectoryDynamicCosts();

	TrajectoryCost DoOneStepStatic(const vector<vector<WayPoint> >& rollOuts, const vector<WayPoint>& totalPaths,
			const WayPoint& currState, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState,
			const std::vector<PlannerHNS::DetectedObject>& obj_list, const int& iCurrentIndex = -1);

	TrajectoryCost DoOneStepDynamic(const vector<vector<WayPoint> >& rollOuts, const vector<WayPoint>& totalPaths,
			const WayPoint& currState, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState,
			const std::vector<PlannerHNS::DetectedObject>& obj_list, const int& iCurrentIndex = -1);

public:
	int m_PrevCostIndex;
	int m_PrevIndex;
	vector<TrajectoryCost> m_TrajectoryCosts;
	PlanningParams m_Params;
	PolygonShape m_SafetyBorder;
	vector<WayPoint> m_AllContourPoints;
	vector<WayPoint> m_AllTrajectoryPoints;
	vector<WayPoint> m_CollisionPoints;
	double m_WeightPriority;
	double m_WeightTransition;
	double m_WeightLong;
	double m_WeightLat;
	double m_WeightLaneChange;
	double m_LateralSkipDistance;
	double m_CollisionTimeDiff;



private:
	void NormalizeCosts(vector<TrajectoryCost>& trajectoryCosts);
	void CalculateLateralAndLongitudinalCostsStatic(vector<TrajectoryCost>& trajectoryCosts, const vector<vector<WayPoint> >& rollOuts, const vector<WayPoint>& totalPaths, const WayPoint& currState, const vector<WayPoint>& contourPoints, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState);
	void CalculateTransitionCosts(vector<TrajectoryCost>& trajectoryCosts, const int& currTrajectoryIndex, const PlanningParams& params);
	void CollectContoursAndTrajectories(const std::vector<PlannerHNS::DetectedObject>& obj_list, vector<WayPoint>& contourPoints, vector<WayPoint>& trajectoryPoints);
	
	void CalculateIntersectionVelocities(const std::vector<WayPoint>& path, const DetectedObject& obj, const WayPoint& currPose, const CAR_BASIC_INFO& carInfo, const double& c_lateral_d, WayPoint& collisionPoint, TrajectoryCost& trajectoryCosts);
	void CalculateIntersectionVelocitiesII(const std::vector<WayPoint>& path, const DetectedObject& obj, const WayPoint& currPose, const PlanningParams& params, const CAR_BASIC_INFO& carInfo, const double& c_lateral_d, WayPoint& collisionPoint, TrajectoryCost& trajectoryCosts);
	int GetCurrentRollOutIndex(const std::vector<WayPoint>& path, const WayPoint& currState, const PlanningParams& params);
	void InitializeCosts(const vector<vector<WayPoint> >& rollOuts, const PlanningParams& params);
	void InitializeSafetyPolygon(const WayPoint& currState, const CAR_BASIC_INFO& carInfo, const VehicleState& vehicleState, const double& c_lateral_d, const double& c_long_front_d, const double& c_long_back_d);
	void CalculateLateralAndLongitudinalCostsDynamic(const std::vector<PlannerHNS::DetectedObject>& obj_list, const vector<vector<WayPoint> >& rollOuts, const vector<WayPoint>& totalPaths,
			const WayPoint& currState, const PlanningParams& params, const CAR_BASIC_INFO& carInfo,
			const VehicleState& vehicleState, const double& c_lateral_d, const double& c_long_front_d, const double& c_long_back_d );

};

}

#endif /* TRAJECTORYDYNAMICCOSTS_H_ */
