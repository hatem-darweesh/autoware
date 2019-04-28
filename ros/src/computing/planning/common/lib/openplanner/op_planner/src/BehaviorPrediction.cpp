
/// \file  BehaviorPrediction.cpp
/// \brief Predict detected vehicles's possible trajectories, these trajectories extracted from the vector map.
/// \author Hatem Darweesh
/// \date Jul 6, 2017



#include "op_planner/BehaviorPrediction.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"
#include <float.h>


namespace PlannerHNS
{


constexpr double LOOK_AHEAD_DISTANCE = 0.5;
constexpr double PREDICTED_PATH_DENSITY = 0.5;
constexpr double PARKING_LEFT_MARGIN = 1.5; // meters
int TrajectoryTracker::max_particles_number = 30;
int TrajectoryTracker::min_particles_number = 0;
int TrajectoryTracker::active_intentions_number = 3;
int TrajectoryTracker::total_particles_number = 90;

BehaviorPrediction::BehaviorPrediction()
{
	m_PredictionHorizon = 100;
	m_MaxLaneDetectionDistance = 0.5;
	m_MaxPredictionDistance = 20.0;
	m_bGenerateBranches = false;
	//m_bUseFixedPrediction = true;
	m_bStepByStep = false;
	//m_bCanDecide = true;
	m_bParticleFilter = false;
	UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
	UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);
	m_bFirstMove = true;
	m_bDebugOut = false;
	m_bDebugOutWeights = false;
	m_bDebugMotion = false;
}

BehaviorPrediction::~BehaviorPrediction()
{
	DeleteTheRest(m_ParticleInfo);
	m_temp_list.clear();

	//save log data
	std::ostringstream fileName;
	if(g_PredParams.experiment_name.size() == 0)
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::PredictionFolderName;
	else
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + g_PredParams.experiment_name + UtilityHNS::DataRW::PredictionFolderName;

	for(unsigned int i=0; i < m_AllLogData.size(); i++)
	{
		std::ostringstream car_num;
		car_num << "pred_log_car_" << m_AllLogData.at(i).first << "__";

		if(m_AllLogData.at(i).second.size() > 2)
		{
			  UtilityHNS::DataRW::WriteLogData(fileName.str(),
				  car_num.str(),
				  "time,x,y,heading,Velocity,Acceleration,Indicator,Best_Traj,real_W_F,real_W_L,real_W_R,Best_Beh_P,Best_Beh_W,Best_w_f,Best_w_s,Best_w_y,Best_w_p,"
				  "id_F,n_part_forward_F,p_forward_F,w_forward_F,"
				  "n_part_stopping_F,p_stopping_F,w_stopping_F,"
				  "n_part_yielding_F,p_yielding_F,w_yielding_F,"
				  "n_part_parking_F,p_parking_F,w_parking_F,"
				  "best_beh_F,all_p_F,all_w_F,best_p_F,best_w_F,real_w_F,"

				  "id_L,n_part_forward_L,p_forward_L,w_forward_L,"
				  "n_part_stopping_L,p_stopping_L,w_stopping_L,"
				  "n_part_yielding_L,p_yielding_L,w_yielding_L,"
				  "n_part_parking_L,p_parking_L,w_parking_L,"
				  "best_beh_L,all_p_L,all_w_L,best_p_L,best_w_L,real_w_L,"

				  "id_R,n_part_forward_R,p_forward_R,w_forward_R,"
				  "n_part_stopping_R,p_stopping_R,w_stopping_R,"
				  "n_part_yielding_R,p_yielding_R,w_yielding_R,"
				  "n_part_parking_R,p_parking_R,w_parking_R,"
				  "best_beh_R,all_p_R,all_w_R,best_p_R,best_w_R,real_w_R,"

				  "id_U,n_part_forward_U,p_forward_U,w_forward_U,"
				  "n_part_stopping_U,p_stopping_U,w_stopping_U,"
				  "n_part_yielding_U,p_yielding_U,w_yielding_U,"
				  "n_part_parking_U,p_parking_U,w_parking_U,"
				  "best_beh_U,all_p_U,all_w_U,best_p_U,best_w_U,real_w_U," , m_AllLogData.at(i).second);
		}
	}
}

void BehaviorPrediction::DoOneStep(const std::vector<DetectedObject>& obj_list, const WayPoint& currPose, const double& minSpeed, const double& maxDeceleration, RoadNetwork& map)
{
//	if(!m_bUseFixedPrediction && maxDeceleration !=0)
//		m_MaxPredictionDistance = -pow(currPose.v, 2)/(maxDeceleration);

	ExtractTrajectoriesFromMap(obj_list, map, m_ParticleInfo);
	CalculateCollisionTimes(minSpeed);

	if(m_bParticleFilter)
	{
		ParticleFilterSteps(m_ParticleInfo);
	}
}

void BehaviorPrediction::ExtractTrajectoriesFromMap(const std::vector<DetectedObject>& curr_obj_list,RoadNetwork& map, std::vector<ObjParticles*>& old_obj_list)
{
	PlannerH planner;
	m_temp_list.clear();

	std::vector<ObjParticles*> delete_me_list = old_obj_list;

	for(unsigned int i=0; i < curr_obj_list.size(); i++)
	{
		bool bMatch = false;
		for(unsigned int ip=0; ip < old_obj_list.size(); ip++)
		{
			if(old_obj_list.at(ip)->obj.id == curr_obj_list.at(i).id)
			{
				bool bFound = false;
				for(unsigned int k=0; k < m_temp_list.size(); k++)
				{
					if(m_temp_list.at(k) == old_obj_list.at(ip))
					{
						bFound = true;
						break;
					}
				}

				if(!bFound)
				{
					old_obj_list.at(ip)->obj = curr_obj_list.at(i);
					m_temp_list.push_back(old_obj_list.at(ip));
				}

				DeleteFromList(delete_me_list, old_obj_list.at(ip));

				old_obj_list.erase(old_obj_list.begin()+ip);
				bMatch = true;
				break;
			}
		}

		if(!bMatch)
		{
			ObjParticles* pNewObj = new  ObjParticles(g_PredParams);
			pNewObj->obj = curr_obj_list.at(i);
			m_temp_list.push_back(pNewObj);
		}
	}

	DeleteTheRest(delete_me_list);
	old_obj_list.clear();
	old_obj_list = m_temp_list;

	//m_PredictedObjects.clear();
	for(unsigned int ip=0; ip < old_obj_list.size(); ip++)
	{
		PredictCurrentTrajectory(map, old_obj_list.at(ip));
		//m_PredictedObjects.push_back(old_obj_list.at(ip)->obj);
		old_obj_list.at(ip)->MatchTrajectories();
	}

}

void BehaviorPrediction::PredictCurrentTrajectory(RoadNetwork& map, ObjParticles* pCarPart)
{
	pCarPart->obj.predTrajectories.clear();
	PlannerH planner;

	CalPredictionTimeForObject(pCarPart, m_MaxPredictionDistance);

	if(pCarPart->obj.bDirection && pCarPart->obj.bVelocity)
	{
		pCarPart->obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(pCarPart->obj.center, map, m_MaxLaneDetectionDistance, pCarPart->obj.bDirection);
		planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, pCarPart->m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection, PREDICTED_PATH_DENSITY);
	}
	else
	{
		bool bLocalDirectionSearch = false;
		pCarPart->obj.pClosestWaypoints = MappingHelpers::GetClosestWaypointsListFromMap(pCarPart->obj.center, map, m_MaxLaneDetectionDistance, pCarPart->obj.bDirection);
		if(pCarPart->obj.pClosestWaypoints.size()>0)
		{
			pCarPart->obj.center.pos.a = pCarPart->obj.pClosestWaypoints.at(0)->pos.a;
			bLocalDirectionSearch = true;
		}
		planner.PredictTrajectoriesUsingDP(pCarPart->obj.center, pCarPart->obj.pClosestWaypoints, pCarPart->m_PredictionDistance, pCarPart->obj.predTrajectories, m_bGenerateBranches, pCarPart->obj.bDirection, PREDICTED_PATH_DENSITY);
	}

	for(unsigned int t = 0; t < pCarPart->obj.predTrajectories.size(); t ++)
	{
//		std::ostringstream path_name;
//		path_name << "/home/hatem/autoware_openplanner_logs/TempPredLog/";
//		path_name << t ;
//		path_name << "_";
//		PlanningHelpers::GenerateRecommendedSpeed(pCarPart->obj.predTrajectories.at(t), 10, 1.0);
//		PlannerHNS::PlanningHelpers::WritePathToFile(path_name.str(), pCarPart->obj.predTrajectories.at(t));
		if(pCarPart->obj.predTrajectories.at(t).size() > 0)
			pCarPart->obj.predTrajectories.at(t).at(0).collisionCost = 0;
	}
}

void BehaviorPrediction::CalculateCollisionTimes(const double& minSpeed)
{
	for(unsigned int i=0; i < m_ParticleInfo.size(); i++)
	{
		for(unsigned int j=0; j < m_ParticleInfo.at(i)->obj.predTrajectories.size(); j++)
		{
			PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_ParticleInfo.at(i)->obj.predTrajectories.at(j), m_ParticleInfo.at(i)->obj.center, minSpeed, m_ParticleInfo.at(i)->m_PredictionDistance);
//			PlannerHNS::PlanningHelpers::CalcAngleAndCost(m_PredictedObjects.at(i).predTrajectories.at(j));
		}
	}
}

void BehaviorPrediction::ParticleFilterSteps(std::vector<ObjParticles*>& part_info)
{
	for(unsigned int i=0; i < part_info.size(); i++)
	{
		SamplesFreshParticles(part_info.at(i));
		CollectParticles(part_info.at(i));
		MoveParticles(part_info.at(i));
		CalculateWeights(part_info.at(i));
		RemoveWeakParticles(part_info.at(i));
		CalculateAveragesAndProbabilities(part_info.at(i));
		FindBest(part_info.at(i));
		part_info.at(i)->LogDataRow();
	}
}

void BehaviorPrediction::SamplesFreshParticles(ObjParticles* pParts)
{
	timespec _time;
	UtilityHNS::UtilityH::GetTickCount(_time);
	srand(_time.tv_nsec);

	ENG eng(_time.tv_nsec);
	NormalDIST dist_x(0, MOTION_POSE_ERROR);
	VariatGEN gen_x(eng, dist_x);
	NormalDIST vel(0, MOTION_VEL_ERROR);
	VariatGEN gen_v(eng, vel);
	NormalDIST ang(0, MOTION_ANGLE_ERROR);
	VariatGEN gen_a(eng, ang);
//	NormalDIST acl(0, MEASURE_ACL_ERROR);
//	VariatGEN gen_acl(eng, acl);

	Particle p;
	p.pose = pParts->obj.center;
	p.vel = 0;
	p.acc = 0;
	p.indicator = 0;
	bool bRegenerate = true;
	WayPoint left_p;

	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_GenerationTimer) > 2)
	{
		UtilityHNS::UtilityH::GetTickCount(m_GenerationTimer);
		bRegenerate = true;
	}

//	for(int i =0 ; i < pParts->m_TrajectoryTracker.size(); i++)
//	{
//		pParts->m_TrajectoryTracker.at(i)->ClearParticles();
//	}
//	pParts->m_AllParticles.clear();

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		srand(_time.tv_nsec+1);
		unsigned int point_index = 0;
		PlanningHelpers::GetRelativeInfo(pParts->m_TrajectoryTracker.at(t)->trajectory, pParts->obj.center, pParts->m_TrajectoryTracker.at(t)->m_CurrRelativeInf);
		pParts->m_TrajectoryTracker.at(t)->followPoint = PlanningHelpers::GetFollowPointOnTrajectory(pParts->m_TrajectoryTracker.at(t)->trajectory, pParts->m_TrajectoryTracker.at(t)->m_CurrRelativeInf, pParts->obj.center.v, point_index);
		p.pose = PlanningHelpers::GetFollowPointOnTrajectory(pParts->m_TrajectoryTracker.at(t)->trajectory, pParts->m_TrajectoryTracker.at(t)->m_CurrRelativeInf, LOOK_AHEAD_DISTANCE, point_index);
		left_p = p.pose;
		left_p.pos.x += PARKING_LEFT_MARGIN * cos(left_p.pos.a+M_PI_2);
		left_p.pos.y += PARKING_LEFT_MARGIN * sin(left_p.pos.a+M_PI_2);

		//missing particles number
		int nPs = TrajectoryTracker::total_particles_number - pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size();

		for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size(); i++)
		{
			Particle* pTempP = &pParts->m_TrajectoryTracker.at(t)->m_CurrParts.at(i);
			pTempP->car_curr_pose = p.pose;
			pTempP->car_curr_pose = p.pose;
			pTempP->car_left_pose = left_p;
			pTempP->pose.v = pParts->obj.center.v + gen_v();
		}

		if(nPs > 0)
		{
//			Particle pBestF, pBestS, pBestY, pBestP;
//			GetBestParticleWeight(pParts->m_TrajectoryTracker.at(t)->m_CurrParts,pBestF, pBestS, pBestY, pBestP);
//			std::cout << "---------> " << pBestF.bNewParticle << pBestS.bNewParticle << pBestY.bNewParticle << pBestP.bNewParticle <<std::endl;

			int n_ps_f = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAliveForward;
			int n_ps_s = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAliveStop;
			int n_ps_y = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAliveYield;
			if(g_PredParams.bEnableParking)
				int n_ps_p = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAlivePark;

			std::cout << ">>>>> Total Size Before: " <<pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size() << std::endl;
			for(unsigned int i=0; i < TrajectoryTracker::total_particles_number; i++)
			{
				Particle p_new = p;
				if(i <= TrajectoryTracker::max_particles_number/TrajectoryTracker::active_intentions_number)
					p_new.beh = PlannerHNS::BEH_FORWARD_STATE;
				else if(i <= 2.0*TrajectoryTracker::max_particles_number/TrajectoryTracker::active_intentions_number)
					p_new.beh = PlannerHNS::BEH_STOPPING_STATE;
				else if(i <= 3.0*TrajectoryTracker::max_particles_number/TrajectoryTracker::active_intentions_number)
				  p_new.beh = PlannerHNS::BEH_YIELDING_STATE;
				else if(g_PredParams.bEnableParking)
					p_new.beh = PlannerHNS::BEH_PARKING_STATE;

				p_new.car_curr_pose = p.pose;
				p_new.car_prev_pose = p.pose;
				p_new.pose.pos.x += gen_x();
				p_new.pose.pos.y += gen_x();
				//p_new.pose.pos.a += gen_a();
				p_new.pose.v = pParts->obj.center.v + gen_v();
				if(p_new.pose.v < 0) p_new.pose.v = 0;
				p_new.vel_rand = gen_v();
				p_new.pTraj = pParts->m_TrajectoryTracker.at(t);
				pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
			}

			std::cout << ">>>>> Total Size After: " <<pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size() << std::endl;
			//if(pBestF == nullptr && pBestS == nullptr && pBestY == nullptr && pBestP == nullptr)
//			{
//			    Particle p_new = p;
//			    p_new.bNewParticle = true;
//                              for(unsigned int i=0; i < n_ps_f; i++)
//                              {
//                                      p_new.beh = PlannerHNS::BEH_FORWARD_STATE;
//                                      pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//                              }
//                              for(unsigned int i=0; i < n_ps_s; i++)
//                              {
//                                      p_new.beh = PlannerHNS::BEH_STOPPING_STATE;
//                                      pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//                              }
//                              for(unsigned int i=0; i < n_ps_y; i++)
//                              {
//                                      p_new.beh = PlannerHNS::BEH_YIELDING_STATE;
//                                      pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//                              }
//                              for(unsigned int i=0; i < n_ps_p; i++)
//			      {
//				      p_new.beh = PlannerHNS::BEH_PARKING_STATE;
//				      pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//			      }
//
//                              for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size(); i++)
//                              {
//                                Particle* p_part = &pParts->m_TrajectoryTracker.at(t)->m_CurrParts.at(i);
//                                if(p_part->bNewParticle == true)
//                                  {
//                                    p_part->bNewParticle = false;
//				  p_part->car_curr_pose = p.pose;
//				  p_part->car_prev_pose = p.pose;
//				  p_part->car_left_pose = left_p;
//				  p_part->pose.pos.x += gen_x();
//				  p_part->pose.pos.y += gen_x();
//				  //p_part->pose.pos.a += gen_a();
//				  p_part->pose.v = pParts->obj.center.v + gen_v();
//				  if(p_part->pose.v < 0) p_part->pose.v = 0;
//				  p_part->vel_rand = gen_v();
//				  p_part->pTraj = pParts->m_TrajectoryTracker.at(t);
//                                  }
//                              }

//			}
//			else
//			{
//			  int n_ps_f = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAliveForward;
//			  int n_ps_s = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAliveStop;
//			  int n_ps_y = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAliveYield;
//			  int n_ps_p = TrajectoryTracker::max_particles_number - pParts->m_TrajectoryTracker.at(t)->nAlivePark;
//
//
//			  Particle p_new;
//			  if(pBestF != nullptr)
//			  {
//                            for(unsigned int i=0; i < n_ps_f; i++)
//                            {
//                                p_new = *pBestF;
//                                p_new.car_curr_pose = p.pose;
//                                p_new.pose.pos.x += gen_x();
//                                p_new.pose.pos.y += gen_x();
//                                //p_new.pose.pos.a += gen_a();
//                                p_new.vel_rand = gen_v();
//                                p_new.pose.v += gen_v();
//                                if(p_new.pose.v < 0) p_new.pose.v = 0;
//                                pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//                            }
//			  }
//
//                          if(pBestS != nullptr)
//                            {
//                              for(unsigned int i=0; i < n_ps_s; i++)
//                              {
//                                  p_new = *pBestS;
//                                  p_new.car_curr_pose = p.pose;
//                                  p_new.pose.pos.x += gen_x();
//                                  p_new.pose.pos.y += gen_x();
//                                  //p_new.pose.pos.a += gen_a();
//                                  p_new.vel_rand = gen_v();
//                                  p_new.pose.v += gen_v();
//                                  if(p_new.pose.v < 0) p_new.pose.v = 0;
//                                  pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//                              }
//                            }
//
//                          if(pBestY != nullptr)
//                            {
//                              for(unsigned int i=0; i < n_ps_y; i++)
//                              {
//                                  p_new = *pBestY;
//                                  p_new.car_curr_pose = p.pose;
//                                  p_new.pose.pos.x += gen_x();
//                                  p_new.pose.pos.y += gen_x();
//                                  //p_new.pose.pos.a += gen_a();
//                                  p_new.vel_rand = gen_v();
//                                  p_new.pose.v += gen_v();
//                                  if(p_new.pose.v < 0) p_new.pose.v = 0;
//                                  pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//                              }
//                            }
//                          if(pBestP != nullptr)
//                            {
//                              for(unsigned int i=0; i < n_ps_p; i++)
//                              {
//                                  p_new = *pBestP;
//                                  p_new.car_curr_pose = p.pose;
//                                  p_new.pose.pos.x += gen_x();
//                                  p_new.pose.pos.y += gen_x();
//                                  //p_new.pose.pos.a += gen_a();
//                                  p_new.vel_rand = gen_v();
//                                  p_new.pose.v += gen_v();
//                                  if(p_new.pose.v < 0) p_new.pose.v = 0;
//                                  pParts->m_TrajectoryTracker.at(t)->InsertNewParticle(p_new);
//                              }
//                            }
//			}
		}
	}
}

void BehaviorPrediction::CollectParticles(ObjParticles* pParts)
{
	pParts->m_AllParticles.clear();
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		for(unsigned int i=0; i < pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size(); i++)
		{
			pParts->m_TrajectoryTracker.at(t)->m_CurrParts.at(i).original_index = i;
			pParts->m_AllParticles.push_back(&pParts->m_TrajectoryTracker.at(t)->m_CurrParts.at(i));
		}
	}
}

void BehaviorPrediction::MoveParticles(ObjParticles* pParts)
{
	double dt = 0.04;
	if(m_bStepByStep)
	{
		dt = 0.02;
	}
	else
	{
		dt = UtilityHNS::UtilityH::GetTimeDiffNow(m_ResamplingTimer);
		UtilityHNS::UtilityH::GetTickCount(m_ResamplingTimer);
		if(m_bFirstMove)
		{
			m_bFirstMove  = false;
			return;
		}
	}

	PlannerHNS::BehaviorState curr_behavior;
	PlannerHNS::ParticleInfo curr_part_info;
	PlannerHNS::VehicleState control_u;
	PassiveDecisionMaker decision_make;
	PlannerHNS::CAR_BASIC_INFO carInfo;
	carInfo.width = pParts->obj.w;
	carInfo.length = pParts->obj.l;
	carInfo.max_acceleration = 2.0;
	carInfo.max_deceleration = -1.5;
	carInfo.max_speed_forward = 6;
	carInfo.min_speed_forward = 0;
	carInfo.max_steer_angle = 0.4;
	carInfo.min_steer_angle = -0.4;
	carInfo.turning_radius = 7.2;
	carInfo.wheel_base = carInfo.length*0.75;

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		PlanningHelpers::GenerateRecommendedSpeed(pParts->m_TrajectoryTracker.at(t)->trajectory, carInfo.max_speed_forward, 1.0);
	}

	if(m_bDebugMotion)
		std::cout << "Motion Status------ " << std::endl;


	for(unsigned int i=0; i < pParts->m_AllParticles.size(); i++)
	{
		Particle* p = pParts->m_AllParticles.at(i);
//		p->pose.v = pParts->obj.center.v;
//		p->pose.pos.a = pParts->obj.center.pos.a;

		PlannerHNS::WayPoint trans_diff;
		trans_diff.pos.x = p->car_curr_pose.pos.x - p->car_prev_pose.pos.x;
		trans_diff.pos.y = p->car_curr_pose.pos.y - p->car_prev_pose.pos.y;
		trans_diff.pos.a = p->car_curr_pose.pos.a - p->car_prev_pose.pos.a;

		curr_part_info = decision_make.MoveStepII(dt, p->pose, trans_diff, p->pTraj->trajectory, carInfo);
		p->car_prev_pose = p->car_curr_pose;
		p->indicator = FromIndicatorToNumber(curr_part_info.indicator);

		switch(p->beh)
		{
		case BEH_FORWARD_STATE:
		{
			//follow the planned velocity only if it is less than the current motion velocity, not all drivers follow the max speed of the road, but they have to brake in the corners (unless they driving a GTR)

			p->vel = curr_part_info.vel + p->vel_rand;

			if(pParts->obj.center.v > curr_part_info.vel)
				p->acc = 0;
			else
				p->acc = 1;

			if(m_bDebugMotion)
			{
				std::cout << ">> FF Traj(" <<  p->pTraj->index << ") Velocity (" << p->vel << ", " << pParts->obj.center.v << ") , Acceleration(" << p->acc <<", " << pParts->obj.acceleration_desc << ")" << std::endl;
			}
		}
		break;
		case BEH_STOPPING_STATE:
		{
			p->vel = 0 + p->vel_rand;
			//p->vel = curr_part_info.vel + p->vel_rand;
			p->acc = 3;

			if(m_bDebugMotion)
			{
				std::cout << ">> SS Traj(" <<  p->pTraj->index << ") Velocity (" << p->vel << ", " << pParts->obj.center.v << ") , Acceleration(" << p->acc <<", " << pParts->obj.acceleration_desc << ")" << std::endl;
			}
		}
		break;
		case BEH_YIELDING_STATE:
		{
//			if(curr_part_info.state == PlannerHNS::STOPPING_STATE)
//			  p->vel = 0 + p->vel_rand;
//			else
			  p->vel = curr_part_info.vel/2.0 + p->vel_rand;

			p->acc = -1;

			if(m_bDebugMotion)
			{
				std::cout << ">> YY Traj(" <<  p->pTraj->index << ") Velocity (" << p->vel << ", " << pParts->obj.center.v << ") , Acceleration(" << p->acc <<", " << pParts->obj.acceleration_desc << ")" << std::endl;
			}
		}
		break;
		case BEH_PARKING_STATE:
		{
//		  if(curr_part_info.state != PlannerHNS::STOPPING_STATE)
//		    {
		      p->indicator = 4;
//		    }

			p->vel = 0 + p->vel_rand;
			p->acc = 0;

			if(m_bDebugMotion)
			{
				std::cout << ">> PP Traj(" <<  p->pTraj->index << ") Velocity (" << p->vel << ", " << pParts->obj.center.v << ") , Acceleration(" << p->acc <<", " << pParts->obj.acceleration_desc << ")" << std::endl;
			}
		}
		break;
		default:
			break;
		}

		p->info_to_path = curr_part_info.info_to_path;
	}

	if(m_bDebugMotion)
		std::cout << "End Motion Status ------ " << std::endl;
}

void BehaviorPrediction::CalculateAccelerationDESC(double dt, Particle* pPart)
{
	pPart->prev_time_diff += dt;
	if(pPart->prev_time_diff > ACCELERATION_CALC_TIME)
	{
		pPart->prev_vel_diff = (pPart->pose.v - pPart->vel_prev_big);
		pPart->acc_raw = pPart->prev_vel_diff/pPart->prev_time_diff;
		pPart->vel_prev_big = pPart->pose.v;
		pPart->prev_time_diff = 0;
	}

	if(pPart->prev_vel_diff > VELOCITY_DECISION_VALUE)
		pPart->acc = 1;
	else if(pPart->prev_vel_diff < -VELOCITY_DECISION_VALUE)
		pPart->acc = -1;
	else
		pPart->acc = 0;
}

void BehaviorPrediction::CalculateWeights(ObjParticles* pParts)
{
	pParts->InitWeightsVariables(g_PredParams);
	pParts->BalanceFactorsToOne();

	for(unsigned int i = 0 ; i < pParts->m_AllParticles.size(); i++)
	{
	    CalOnePartWeight(pParts, *pParts->m_AllParticles.at(i));
	}

	pParts->pose_diff_raw  = pParts->pose_w_max-pParts->pose_w_min;
	pParts->pose_real_diff_raw  = pParts->pose_w_real_max-pParts->pose_w_real_min;
	pParts->dir_diff_raw = pParts->dir_w_max-pParts->dir_w_min;
	pParts->dir_real_diff_raw = pParts->dir_w_real_max-pParts->dir_w_real_min;
	pParts->vel_diff_raw = pParts->vel_w_max-pParts->vel_w_min;
	pParts->ind_diff_raw = pParts->ind_w_max-pParts->ind_w_min;
	pParts->acl_diff_raw = pParts->acl_w_max-pParts->acl_w_min;
	pParts->total_diff_raw = pParts->max_w_raw -pParts->min_w_raw;

	bool b_change_real = false;
	if(pParts->pose_w_real_min < DBL_MAX && pParts->pose_real_diff_raw < WEIGHT_CRITICAL_DIFF)
	{
		b_change_real = true;
		pParts->pose_factor_real = 0;
	}

	if(pParts->dir_w_real_min < DBL_MAX && pParts->dir_real_diff_raw < WEIGHT_CRITICAL_DIFF)
	{
		b_change_real = true;
		pParts->dir_factor_real = 0;
	}

	bool b_change = false;
	if(pParts->pose_w_min < DBL_MAX && pParts->pose_diff_raw < WEIGHT_CRITICAL_DIFF)
	{
		b_change = true;
		pParts->pose_factor = 0;
	}

	if(pParts->dir_w_min < DBL_MAX && pParts->dir_diff_raw < WEIGHT_CRITICAL_DIFF)
	{
		b_change = true;
		pParts->dir_factor = 0;
	}

	if(pParts->vel_w_min < DBL_MAX && pParts->vel_diff_raw < WEIGHT_CRITICAL_DIFF)
	{
		b_change = true;
		pParts->vel_factor = 0;
	}

	if(pParts->ind_w_min < DBL_MAX && pParts->ind_diff_raw < WEIGHT_CRITICAL_DIFF)
	{
		b_change = true;
		pParts->ind_factor = 0;
	}

	if(pParts->acl_w_min < DBL_MAX && pParts->acl_diff_raw < WEIGHT_CRITICAL_DIFF)
	{
		b_change = true;
		pParts->acl_factor = 0;
	}

	if(b_change)
	{
		pParts->BalanceFactorsToOne();
	}

	if(b_change_real)
	{
		pParts->BalanceFactorsToOneReal();
	}

//	if((pParts->max_w_raw == 0 || fabs(pParts->total_diff_raw) < WEIGHT_CRITICAL_DIFF) && pParts->m_TrajectoryTracker.size() > 1)
//		m_bCanDecide = false;
//	else
//		m_bCanDecide = true;


	//Normalize
	pParts->max_w = DBL_MIN;
	pParts->min_w = DBL_MAX;
	//pParts->all_w = 0;
	//pParts->all_w_real = 0;

	for(unsigned int i = 0 ; i < pParts->m_AllParticles.size(); i++)
	{
		NormalizeOnePartWeight(pParts, *pParts->m_AllParticles.at(i));
	}
}

void BehaviorPrediction::RemoveWeakParticles(ObjParticles* pParts)
{

	pParts->m_AllGeneratedParticles.clear();

	if(m_bDebugOutWeights)
	{
		std::cout << "Behavior Weights ------------------------------------------------ " << std::endl;

		for(int i =0 ; i < pParts->m_AllParticles.size(); i++)
		{
			pParts->m_AllGeneratedParticles.push_back(*pParts->m_AllParticles.at(i));
			if(pParts->m_AllParticles.at(i)->beh == BEH_FORWARD_STATE)
				std::cout << "F, ";
			else if(pParts->m_AllParticles.at(i)->beh == BEH_STOPPING_STATE)
				std::cout << "S, ";
			else if(pParts->m_AllParticles.at(i)->beh == BEH_YIELDING_STATE)
				std::cout << "Y, ";
			else if(pParts->m_AllParticles.at(i)->beh == BEH_PARKING_STATE)
				std::cout << "P, ";

			std::cout << pParts->m_AllParticles.at(i)->pTraj->index <<" => ";
			std::cout << "W:" << THREE_PERC_POINTS(pParts->m_AllParticles.at(i)->w)
					<< ", Pose_W:" << THREE_PERC_POINTS(pParts->m_AllParticles.at(i)->pose_w)
					<< ", Dire_W:" << THREE_PERC_POINTS(pParts->m_AllParticles.at(i)->dir_w)
					<< ", SpeedW:" << THREE_PERC_POINTS(pParts->m_AllParticles.at(i)->vel_w)
					<< ", AccelW:" << THREE_PERC_POINTS(pParts->m_AllParticles.at(i)->acl_w)
					<< ", Indi_W:" << THREE_PERC_POINTS(pParts->m_AllParticles.at(i)->ind_w)
					<< ", Angles:(" << THREE_PERC_POINTS(pParts->m_AllParticles.at(i)->pose.pos.a) << "," <<  THREE_PERC_POINTS(pParts->obj.center.pos.a) << ")"
					<< ", Angle_Diff: " << THREE_PERC_POINTS(fabs(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(pParts->m_AllParticles.at(i)->pose.pos.a,  pParts->obj.center.pos.a)))
					<< std::endl;
		}

		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
		{
			std::cout << "T (" << t << "): " << " All Before:" << pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size()
					<< ", F_N:" << pParts->m_TrajectoryTracker.at(t)->nAliveForward
					<< ", S_N:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop
					<< ", Y_N:" << pParts->m_TrajectoryTracker.at(t)->nAliveYield
					<< ", P_N:" << pParts->m_TrajectoryTracker.at(t)->nAlivePark
					<< ", Percentage: " << g_PredParams.KEEP_PERCENTAGE << std::endl;
		}
	}

	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		pParts->m_TrajectoryTracker.at(t)->DeleteParticles(0, g_PredParams.KEEP_PERCENTAGE, true);
	}

	if(m_bDebugOutWeights)
	{
		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
		{
			std::cout << "T (" << t << "): " << " All After:" << pParts->m_TrajectoryTracker.at(t)->m_CurrParts.size()
					<< ", F_N:" << pParts->m_TrajectoryTracker.at(t)->nAliveForward
					<< ", S_N:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop
					<< ", Y_N:" << pParts->m_TrajectoryTracker.at(t)->nAliveYield
					<< ", P_N:" << pParts->m_TrajectoryTracker.at(t)->nAlivePark
					<< ", Percentage: " << g_PredParams.KEEP_PERCENTAGE << std::endl;
		}
		std::cout << "End Behavior Weights ------ " << std::endl;
	}


	CollectParticles(pParts);
}

void BehaviorPrediction::CalculateAveragesAndProbabilities(ObjParticles* pParts)
{
	for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size(); t++)
	{
		pParts->m_TrajectoryTracker.at(t)->CalcAveragesAndProb();
	}
}

void BehaviorPrediction::FindBest(ObjParticles* pParts)
{
	pParts->FindBestTracks();

	if(m_bDebugOut )
	{
		std::cout << "Behavior Prob ------------------------------------------------ : " << pParts->m_TrajectoryTracker.size() << std::endl;
		std::cout << "PoseW, Raw: Max: " <<  pParts->pose_w_max << ", Min: " << pParts->pose_w_min << std::endl;
		std::cout << "VeloW, Raw: Max: " <<  pParts->vel_w_max << ", Min: " << pParts->vel_w_min << std::endl;
		std::cout << "AcceW, Raw: Max: " <<  pParts->acl_w_max << ", Min: " << pParts->acl_w_min << std::endl;
		std::cout << "DireW, Raw: Max: " <<  pParts->dir_w_max << ", Min: " << pParts->dir_w_min << std::endl;
		std::cout << "IndcW, Raw: Max: " <<  pParts->ind_w_max << ", Min: " << pParts->ind_w_min << std::endl;
		std::cout << "Total, Raw: Max: " <<  pParts->max_w_raw << ", Min: " << pParts->min_w_raw << std::endl;

		std::cout << std::endl;

		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
		{
			std::cout << "ID(" << t <<"," <<pParts->m_TrajectoryTracker.at(t)->id_ << ")"
					<< " | Total Avg W: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->all_w)
					<< " | Total Ral W: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->all_w_real)
					<< " | Forwd Avg W: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->w_avg_forward)
					<< " | Stop  Avg W: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->w_avg_stop)
					<< " | Yield Avg W: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->w_avg_yield)
					<< " | Park  Avg W: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->w_avg_park)
					<< " >>> Best W: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->best_w)
					<< std::endl;
		std::cout << "ID(" << t <<"," <<pParts->m_TrajectoryTracker.at(t)->id_ << ")"
						<< " | Total Avg P: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->all_p)
						<< " | Total Avg P: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->all_p)
						<< " | Forwd Avg P: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->pForward)
						<< " | Stop  Avg P: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->pStop)
						<< " | Yield Avg P: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->pYield)
						<< " | Park  Avg P: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->pPark)
						<< " >>> Best P: " << TWO_PERC_POINTS(pParts->m_TrajectoryTracker.at(t)->best_p)
						<< std::endl;

		std::cout << std::endl;

		}
	}

	for(unsigned int org_t=0; org_t < pParts->obj.predTrajectories.size() ; org_t++)
	{
		pParts->obj.predTrajectories.at(org_t).at(0).collisionCost = 0.0;
	}

	for(unsigned int org_t=0; org_t < pParts->obj.predTrajectories.size() ; org_t++)
	{
//		if(pParts->obj.predTrajectories.at(org_t).size()>0)
//		{
//			pParts->obj.predTrajectories.at(org_t).at(0).collisionCost = 0.25;
//		}

		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
		{
			if(org_t == pParts->m_TrajectoryTracker.at(t)->index)
			{
				if(t == pParts->i_best_for_track)
					pParts->obj.predTrajectories.at(org_t).at(0).collisionCost = 1.0;
				else
					pParts->obj.predTrajectories.at(org_t).at(0).collisionCost = 0.0;

				for(unsigned int i=1; i < pParts->obj.predTrajectories.at(org_t).size(); i++)
				{
					pParts->obj.predTrajectories.at(org_t).at(i).collisionCost = pParts->m_TrajectoryTracker.at(t)->all_w_real;
				}
				break;
			}
		}
	}

	//if(m_bCanDecide && pParts->best_behavior_track != nullptr && pParts->best_forward_track != nullptr)

	if(pParts->best_behavior_track != nullptr)
	{
		std::string str_beh = "Unknown";
		if(pParts->best_behavior_track->best_beh_by_p == BEH_STOPPING_STATE)
			str_beh = "Stopping";
		else if(pParts->best_behavior_track->best_beh_by_p == BEH_FORWARD_STATE)
			str_beh = "Forward";
		else if(pParts->best_behavior_track->best_beh_by_p == BEH_YIELDING_STATE)
			str_beh = "Yielding";
		else if(pParts->best_behavior_track->best_beh_by_p == BEH_PARKING_STATE)
			str_beh = "Parking";

		if(m_bDebugOut )
		{
			std::cout << "Best Behavior (" << pParts->i_best_beh_track << ", " << str_beh << "), bestP: " << pParts->best_behavior_track->best_p << ", allP:" << pParts->best_behavior_track->all_p << ", bestW: " << pParts->best_behavior_track->best_w << std::endl;
		}
	}

	if(pParts->best_forward_track != nullptr)
	{
		if(m_bDebugOut )
		{
			std::cout << "Best Trajectory (" << pParts->i_best_for_track << ", " << pParts->best_forward_track->id_ << "), Real W: " << pParts->best_forward_track->all_w_real	<< std::endl;
		}
	}

	if(pParts->best_behavior_track == nullptr && pParts->best_forward_track == nullptr)
	{
		if(m_bDebugOut )
			std::cout << "Trajectory (" << -1 << "), P: " << 0 << " , Beh (" << -1 << ", " << "Can't Decide" << ")" << std::endl;
	}

	if(m_bDebugOut )
	{
		for(unsigned int t=0; t < pParts->m_TrajectoryTracker.size() ; t++)
		{
			std::cout << "t value = " << t << ", index value: " << pParts->m_TrajectoryTracker.at(t)->index  <<std::endl;
			if(pParts->m_TrajectoryTracker.at(t)->nAliveForward > 0)
			{
				std::cout << "ID(" << t <<"," <<pParts->m_TrajectoryTracker.at(t)->id_ << ")" << " | BestP: " << pParts->m_TrajectoryTracker.at(t)->best_p <<
				", BestW: " << pParts->m_TrajectoryTracker.at(t)->best_w <<
				", AllP: " << pParts->m_TrajectoryTracker.at(t)->all_p <<
				" -> FF Parts:" << pParts->m_TrajectoryTracker.at(t)->nAliveForward << std::endl;
			}
			if(pParts->m_TrajectoryTracker.at(t)->nAliveStop > 0)
			{
				std::cout << "ID(" << t <<"," <<pParts->m_TrajectoryTracker.at(t)->id_ << ")" << " | BestP: " << pParts->m_TrajectoryTracker.at(t)->best_p <<
				", BestW: " << pParts->m_TrajectoryTracker.at(t)->best_w <<
				", AllP: " << pParts->m_TrajectoryTracker.at(t)->all_p <<
				" -> SS Parts:" << pParts->m_TrajectoryTracker.at(t)->nAliveStop << std::endl;
			}

			if(pParts->m_TrajectoryTracker.at(t)->nAliveYield > 0)
			{
				std::cout << "ID(" << t <<"," <<pParts->m_TrajectoryTracker.at(t)->id_ << ")" << " | BestP: " << pParts->m_TrajectoryTracker.at(t)->best_p <<
				", BestW: " << pParts->m_TrajectoryTracker.at(t)->best_w <<
				", AllP: " << pParts->m_TrajectoryTracker.at(t)->all_p <<
				" -> YY Parts:" << pParts->m_TrajectoryTracker.at(t)->nAliveYield << std::endl;
			}

			if(pParts->m_TrajectoryTracker.at(t)->nAlivePark > 0)
			{
				std::cout << "ID(" << t <<"," <<pParts->m_TrajectoryTracker.at(t)->id_ << ")" << " | BestP: " << pParts->m_TrajectoryTracker.at(t)->best_p <<
				", BestW: " << pParts->m_TrajectoryTracker.at(t)->best_w <<
				", AllP: " << pParts->m_TrajectoryTracker.at(t)->all_p <<
				" -> PP Parts:" << pParts->m_TrajectoryTracker.at(t)->nAlivePark << std::endl;
			}
		}

		std::cout << "------------------------------------------------ --------------" << std::endl<< std::endl;
	}
}

void BehaviorPrediction::CalPredictionTimeForObject(ObjParticles* pCarPart, const double& max_pre_distance)
{
//	double d = (MIN_PREDICTION_TIME * pCarPart->obj.center.v) + pCarPart->obj.l;
//	if(d < max_pre_distance)
//		pCarPart->m_PredictionDistance = d;
//	else
		pCarPart->m_PredictionDistance = max_pre_distance;
}

int BehaviorPrediction::FromIndicatorToNumber(const PlannerHNS::LIGHT_INDICATOR& indi)
{
  if(indi == PlannerHNS::INDICATOR_NONE)
  {
      return 0;
  }
  else if(indi == PlannerHNS::INDICATOR_LEFT)
  {
      return 1;
  }
  else if(indi == PlannerHNS::INDICATOR_RIGHT)
  {
      return 2;
  }
  else if(indi == PlannerHNS::INDICATOR_BOTH)
  {
      return 3;
  }
  else
  {
      return 0;
  }
}

PlannerHNS::LIGHT_INDICATOR BehaviorPrediction::FromNumbertoIndicator(const int& num)
{
	if(num == 0)
		return PlannerHNS::INDICATOR_NONE;
	else if(num == 1)
		return PlannerHNS::INDICATOR_LEFT;
	else if(num == 2)
		return PlannerHNS::INDICATOR_RIGHT;
	else if(num == 3)
		return PlannerHNS::INDICATOR_BOTH;
	else
		return PlannerHNS::INDICATOR_NONE;
}

double BehaviorPrediction::CalcIndicatorWeight(int p_ind, int obj_ind)
{
	if(p_ind == obj_ind)
	  return 1.0 - MEASURE_IND_ERROR;
	else if(p_ind == 4 &&  (obj_ind == 1 || obj_ind == 3))
	  return 1.0 - MEASURE_IND_ERROR;
	else
	  return MEASURE_IND_ERROR;
}

double BehaviorPrediction::CalcAccelerationWeight(int p_acl, int obj_acl)
{
	if(p_acl == obj_acl)
	  return 1.0 - MEASURE_ACC_ERROR;
	else if(p_acl == 2 && (obj_acl == 0 || obj_acl == 1))
	  return 1.0 - MEASURE_ACC_ERROR;
	else if(p_acl == 3 && (obj_acl == 0 || obj_acl == -1))
	  return 1.0 - MEASURE_ACC_ERROR;
	else
	  return MEASURE_ACC_ERROR;
}

void BehaviorPrediction::CalOnePartWeight(ObjParticles* pParts,Particle& p)
{
	double x_diff = p.pose.pos.x - p.car_curr_pose.pos.x;
	double y_diff = p.pose.pos.y - p.car_curr_pose.pos.y;

	double x_diff_real = p.pose.pos.x - pParts->obj.center.pos.x;
	double y_diff_real = p.pose.pos.y - pParts->obj.center.pos.y;

	double distance = hypot(y_diff, x_diff);
	if(distance < MEASURE_POSE_ERROR) distance = MEASURE_POSE_ERROR;

	double distance_real = hypot(y_diff_real, x_diff_real);
	if(distance_real < MEASURE_POSE_ERROR) distance_real = MEASURE_POSE_ERROR;

	double vel_diff = fabs(p.vel - pParts->obj.center.v);
	if(vel_diff < MEASURE_VEL_ERROR) vel_diff = MEASURE_VEL_ERROR;

	double a_diff = fabs(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(p.pose.pos.a,  p.car_curr_pose.pos.a));
	if(a_diff < MEASURE_ANGLE_ERROR) a_diff = MEASURE_ANGLE_ERROR;

	double a_diff_real = fabs(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(p.pose.pos.a,  pParts->obj.center.pos.a));
	if(a_diff_real < MEASURE_ANGLE_ERROR) a_diff_real = MEASURE_ANGLE_ERROR;

	//p.pose_w = exp(-(0.5*pow((p.pose.pos.x - pParts->obj.center.pos.x),2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)+ pow((p.pose.pos.y - pParts->obj.center.pos.y),2)/(2*MEASURE_POSE_ERROR*MEASURE_POSE_ERROR)));
	//p.dir_w  = exp(-(pow(fabs(UtilityHNS::UtilityH::AngleBetweenTwoAnglesPositive(p.pose.pos.a,  pParts->obj.center.pos.a)),2)/(2*MEASURE_ANGLE_ERROR*MEASURE_ANGLE_ERROR)));
	//p.vel_w  = exp(-(pow((p.vel - pParts->obj.center.v),2)/(2*MEASURE_VEL_ERROR*MEASURE_VEL_ERROR)));

	p.pose_w = 1.0/distance;
	p.pose_w_real = 1.0/distance_real;
	p.dir_w  = 1.0/a_diff;
	p.dir_w_real  = 1.0/a_diff_real;
	p.vel_w = 1.0/vel_diff;
	p.ind_w = CalcIndicatorWeight(p.indicator, FromIndicatorToNumber(pParts->obj.indicator_state));
	p.acl_w = CalcAccelerationWeight(p.acc, pParts->obj.acceleration_desc);


	pParts->pose_w_t += p.pose_w;
	pParts->pose_w_real_t += p.pose_w_real;
	pParts->dir_w_t += p.dir_w;
	pParts->dir_w_real_t += p.dir_w_real;
	pParts->vel_w_t += p.vel_w;
	pParts->ind_w_t += p.ind_w;
	pParts->acl_w_t += p.acl_w;

	if(p.pose_w > pParts->pose_w_max && pParts->pose_factor> 0)
		pParts->pose_w_max = p.pose_w;
	if(p.pose_w < pParts->pose_w_min && pParts->pose_factor > 0)
		pParts->pose_w_min = p.pose_w;

	if(p.pose_w_real > pParts->pose_w_real_max && pParts->pose_factor> 0)
		pParts->pose_w_real_max = p.pose_w_real;
	if(p.pose_w_real < pParts->pose_w_real_min && pParts->pose_factor > 0)
		pParts->pose_w_real_min = p.pose_w_real;

	if(p.dir_w > pParts->dir_w_max && pParts->dir_factor > 0)
		pParts->dir_w_max = p.dir_w;
	if(p.dir_w < pParts->dir_w_min && pParts->dir_factor > 0)
		pParts->dir_w_min = p.dir_w;

	if(p.dir_w_real > pParts->dir_w_real_max && pParts->dir_factor > 0)
		pParts->dir_w_real_max = p.dir_w_real;
	if(p.dir_w_real < pParts->dir_w_real_min && pParts->dir_factor > 0)
		pParts->dir_w_real_min = p.dir_w_real;

	if(p.vel_w > pParts->vel_w_max && pParts->vel_factor > 0)
		pParts->vel_w_max = p.vel_w;
	if(p.vel_w < pParts->vel_w_min && pParts->vel_factor > 0)
		pParts->vel_w_min = p.vel_w;

	if(p.ind_w > pParts->ind_w_max && pParts->ind_factor > 0)
		pParts->ind_w_max = p.ind_w;
	if(p.ind_w < pParts->ind_w_min && pParts->ind_factor > 0)
		pParts->ind_w_min = p.ind_w;

	if(p.acl_w > pParts->acl_w_max && pParts->acl_factor > 0)
		pParts->acl_w_max = p.acl_w;
	if(p.acl_w < pParts->acl_w_min && pParts->acl_factor > 0)
		pParts->acl_w_min = p.acl_w;

	p.w_raw = p.pose_w*pParts->pose_factor + p.dir_w*pParts->dir_factor + p.vel_w*pParts->vel_factor + p.ind_w*pParts->ind_factor + p.acl_w*pParts->acl_factor;

	if(p.w_raw > pParts->max_w_raw)
		pParts->max_w_raw = p.w_raw;

	if(p.w_raw < pParts->min_w_raw)
		pParts->min_w_raw = p.w_raw;
}

void BehaviorPrediction::NormalizeOnePartWeight(ObjParticles* pParts,Particle& p)
{
//	std::cout << "Before .......... " << std::endl;
//	std::cout << "ACL_W Raw : " << p.acl_w << ", ACC DIFF : " << pParts->acl_diff_raw << ", ACC MIN:" << pParts->acl_w_min<<", ACC Max: "<< pParts->acl_w_max << ", Fact: " <<pParts->acl_factor <<  std::endl;
//	std::cout << "Pos_W Raw : " << p.pose_w << ", Pose DIFF : " << pParts->pose_diff_raw << ", Pose MIN:" << pParts->pose_w_min<<", Pose Max: "<< pParts->pose_w_max << ",Facr: " << pParts->pose_factor << std::endl;

	double epsilon = 0.0001;
	if(pParts->pose_diff_raw > epsilon)
		p.pose_w =  (p.pose_w - pParts->pose_w_min)/pParts->pose_diff_raw;
	else
		p.pose_w = 0;

	if(pParts->pose_real_diff_raw > epsilon)
		p.pose_w_real =  (p.pose_w_real - pParts->pose_w_real_min)/pParts->pose_real_diff_raw;
	else
		p.pose_w_real = 0;

	if(pParts->dir_diff_raw > epsilon)
		p.dir_w = (p.dir_w - pParts->dir_w_min)/pParts->dir_diff_raw;
	else
		p.dir_w = 0;

	if(pParts->dir_real_diff_raw > epsilon)
		p.dir_w_real = (p.dir_w_real - pParts->dir_w_real_min)/pParts->dir_real_diff_raw;
	else
		p.dir_w_real = 0;

	if(pParts->vel_diff_raw > epsilon)
		p.vel_w = (p.vel_w - pParts->vel_w_min)/pParts->vel_diff_raw;
	else
		p.vel_w = 0;

//	if(m_bDebugOutWeights)
//		std::cout << "Vel Norm W: " << p.vel_w  << ", p.vel: " << p.vel << ", CurrV: " <<  pParts->obj.center.v << ", Diff: " << pParts->vel_diff_raw << std::endl;

	if(pParts->ind_diff_raw > epsilon)
		p.ind_w = (p.ind_w - pParts->ind_w_min)/pParts->ind_diff_raw;
	else
		p.ind_w = 0;

	if(pParts->acl_diff_raw > epsilon)
		p.acl_w = (p.acl_w - pParts->acl_w_min)/pParts->acl_diff_raw;
	else
		p.acl_w = 0;

	p.w = p.pose_w*pParts->pose_factor + p.dir_w*pParts->dir_factor + p.vel_w*pParts->vel_factor + p.ind_w*pParts->ind_factor + p.acl_w*pParts->acl_factor;

	p.w_real = p.pose_w_real*pParts->pose_factor_real + p.dir_w_real*pParts->dir_factor_real + p.vel_w*pParts->vel_factor + p.ind_w*pParts->ind_factor + p.acl_w*pParts->acl_factor;

	if(p.w > pParts->max_w)
		pParts->max_w = p.w;

	if(p.w < pParts->min_w)
		pParts->min_w = p.w;

	  //pParts->all_w += p.w;
	  //pParts->all_w_real += p.w_real;
}

} /* namespace PlannerHNS */
