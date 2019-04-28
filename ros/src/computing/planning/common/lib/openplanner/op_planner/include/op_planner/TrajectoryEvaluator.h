/// \file TrajectoryEvaluator.h
/// \brief Calculate collision costs for roll out trajectory for free trajectory evaluation for OpenPlanner local planner version 1.5+ (Dynamic Obstacles Prediction)
/// \author Hatem Darweesh
/// \date Jan 3, 2019

#ifndef TRAJECTORY_EVALUATOR_H_
#define TRAJECTORY_EVALUATOR_H_

#include "PlanningHelpers.h"
#include "PlannerCommonDef.h"

namespace PlannerHNS
{

class EvaluationParams
{

public:
  double priority_weight_;
  double transition_weight_;
  double longitudinal_weight_;
  double lateral_weight_;
  double lane_change_weight_;
  double collision_time_;

  EvaluationParams()
  {
    priority_weight_ = 0.25;
    transition_weight_ = 0.25;
    longitudinal_weight_ = 0.25;
    lateral_weight_ = 0.25;
    lane_change_weight_ = 0;
    collision_time_ = 3;
  }

  EvaluationParams(double periority_w, double transition_w, double logitudinal_w, double lateral_w,
                    double lane_change_w, double collision_t)
  {
    priority_weight_ = periority_w;
    transition_weight_ = transition_w;
    longitudinal_weight_ = logitudinal_w;
    lateral_weight_ = lateral_w;
    lane_change_weight_ = lane_change_w;
    collision_time_ = collision_t;
  }
};

class TrajectoryEvaluator
{
public:
  TrajectoryEvaluator();
  virtual ~TrajectoryEvaluator();

  TrajectoryCost doOneStep(const std::vector<std::vector<WayPoint> >& roll_outs,
                           const std::vector<WayPoint>& total_paths, const WayPoint& curr_state,
                           const PlanningParams& params, const CAR_BASIC_INFO& car_info,
                           const VehicleState& vehicle_state, const std::vector<DetectedObject>& obj_list,
                           const bool& b_static_only = false,
                           const int& prev_curr_index = -1);

public:
  std::vector<WayPoint> all_contour_points_;
  std::vector<WayPoint> all_trajectories_points_;
  std::vector<WayPoint> collision_points_;
  PolygonShape safety_border_;
  std::vector<std::vector<WayPoint> > local_roll_outs_;
  std::vector<TrajectoryCost> trajectory_costs_;

private:
  EvaluationParams eval_params_;

private:
  void normalizeCosts(std::vector<TrajectoryCost>& trajectory_costs);

  void calculateTransitionCosts(std::vector<TrajectoryCost>& trajectory_costs, const int& curr_index,
                                const PlanningParams& params);

  void collectContoursAndTrajectories(const std::vector<PlannerHNS::DetectedObject>& obj_list, PolygonShape& ego_car_border,
                                      std::vector<WayPoint>& contour_points, std::vector<WayPoint>& trajectory_points, const bool& b_static_only = false);

  int getCurrentRollOutIndex(const std::vector<WayPoint>& total_path, const WayPoint& curr_state,
                             const PlanningParams& params);

  void initializeCosts(const std::vector<std::vector<WayPoint> >& roll_outs, const PlanningParams& params,
                       std::vector<TrajectoryCost>& trajectory_costs);

  void initializeSafetyPolygon(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info,
                               const VehicleState& vehicle_state, const double& c_lateral_d,
                               const double& c_long_front_d, const double& c_long_back_d, PolygonShape& car_border);

  void initializeLocalRollOuts(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info, const PlanningParams& params, const double& c_long_back_d, const std::vector<std::vector<WayPoint> >& original_roll_outs, std::vector<std::vector<WayPoint> >& local_roll_outs);

  void calculateDistanceCosts(const PlanningParams& params, const double& c_lateral_d, const std::vector<std::vector<WayPoint> >& roll_outs, const std::vector<WayPoint>& contour_points, const std::vector<WayPoint>& trajectory_points, std::vector<TrajectoryCost>& trajectory_costs, std::vector<WayPoint>& collision_points);

  TrajectoryCost findBestTrajectory(const PlanningParams& params, std::vector<TrajectoryCost> trajectory_costs);

  static bool sortCosts(const TrajectoryCost& c1, const TrajectoryCost& c2)
  {
    return c1.cost < c2.cost;
  }

};

}

#endif /* TRAJECTORY_EVALUATOR_H_ */
