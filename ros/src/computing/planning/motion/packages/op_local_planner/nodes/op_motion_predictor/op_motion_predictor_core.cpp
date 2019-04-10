/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "op_motion_predictor_core.h"
#include "op_planner/MappingHelpers.h"
#include "op_ros_helpers/op_RosHelpers.h"

namespace MotionPredictorNS
{

MotionPrediction::MotionPrediction()
{
	m_t = 0;
	bMap = false;
	bNewCurrentPos = false;
	bVehicleStatus = false;
	bTrackedObjects = false;
	m_bEnableCurbObstacles = false;
	m_DistanceBetweenCurbs = 1.0;
	m_VisualizationTime = 0.25;
	m_bGoNextStep = false;

	ros::NodeHandle _nh;
	UpdatePlanningParams(_nh);

	tf::StampedTransform transform;
	PlannerHNS::RosHelpers::GetTransformFromTF("map", "world", transform);
	m_OriginPos.position.x  = transform.getOrigin().x();
	m_OriginPos.position.y  = transform.getOrigin().y();
	m_OriginPos.position.z  = transform.getOrigin().z();

	pub_predicted_objects_trajectories = nh.advertise<autoware_msgs::DetectedObjectArray>("/predicted_objects", 1);
	pub_PredictedTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("/predicted_trajectories_rviz", 1);
	pub_CurbsRviz					= nh.advertise<visualization_msgs::MarkerArray>("/map_curbs_rviz", 1);
	pub_ParticlesRviz = nh.advertise<visualization_msgs::MarkerArray>("prediction_particles", 1);
	pub_GeneratedParticlesRviz = nh.advertise<visualization_msgs::MarkerArray>("generated_particles", 1);
	pub_BehaviorStateRviz = nh.advertise<visualization_msgs::MarkerArray>("prediction_behaviors", 1);
	pub_TargetPointsRviz = nh.advertise<visualization_msgs::MarkerArray>("target_points_on_trajs", 1);

	sub_StepSignal = nh.subscribe("/pred_step_signal", 		1, &MotionPrediction::callbackGetStepForwardSignals, 		this);
	sub_tracked_objects	= nh.subscribe("/tracked_objects", 			1,		&MotionPrediction::callbackGetTrackedObjects, 		this);
	sub_current_pose 	= nh.subscribe("/current_pose", 10,	&MotionPrediction::callbackGetCurrentPose, 		this);

	int bVelSource = 1;
	_nh.getParam("/op_motion_predictor/velocitySource", bVelSource);
	if(bVelSource == 0)
		sub_robot_odom = nh.subscribe("/odom", 10, &MotionPrediction::callbackGetRobotOdom, this);
	else if(bVelSource == 1)
		sub_current_velocity = nh.subscribe("/current_velocity", 10, &MotionPrediction::callbackGetVehicleStatus, this);
	else if(bVelSource == 2)
		sub_can_info = nh.subscribe("/can_info", 10, &MotionPrediction::callbackGetCanInfo, this);

	UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
	PlannerHNS::RosHelpers::InitPredMarkers(100, m_PredictedTrajectoriesDummy);
	PlannerHNS::RosHelpers::InitCurbsMarkers(100, m_CurbsDummy);
	PlannerHNS::RosHelpers::InitPredParticlesMarkers(1000, m_PredictedParticlesDummy);
	PlannerHNS::RosHelpers::InitPredParticlesMarkers(2000, m_GeneratedParticlesDummy, true);

	//Mapping Section
	if(m_MapType == PlannerHNS::MAP_AUTOWARE)
	{
		sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &MotionPrediction::callbackGetVMLanes,  this);
		sub_points = nh.subscribe("/vector_map_info/point", 1, &MotionPrediction::callbackGetVMPoints,  this);
		sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &MotionPrediction::callbackGetVMdtLanes,  this);
		sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &MotionPrediction::callbackGetVMIntersections,  this);
		sup_area = nh.subscribe("/vector_map_info/area", 1, &MotionPrediction::callbackGetVMAreas,  this);
		sub_lines = nh.subscribe("/vector_map_info/line", 1, &MotionPrediction::callbackGetVMLines,  this);
		sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &MotionPrediction::callbackGetVMStopLines,  this);
		sub_signals = nh.subscribe("/vector_map_info/signal", 1, &MotionPrediction::callbackGetVMSignal,  this);
		sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &MotionPrediction::callbackGetVMVectors,  this);
		sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &MotionPrediction::callbackGetVMCurbs,  this);
		sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &MotionPrediction::callbackGetVMRoadEdges,  this);
		sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &MotionPrediction::callbackGetVMWayAreas,  this);
		sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &MotionPrediction::callbackGetVMCrossWalks,  this);
		sub_nodes = nh.subscribe("/vector_map_info/node", 1, &MotionPrediction::callbackGetVMNodes,  this);
	}

	std::cout << "OpenPlanner Motion Predictor initialized successfully " << std::endl;
}

MotionPrediction::~MotionPrediction()
{
	std::ostringstream fileName;
	if(m_ExperimentFolderName.size() == 0)
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::PredictionFolderName;
	else
		fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentFolderName + UtilityHNS::DataRW::PredictionFolderName;

	if(m_LogDataCar0.size() > 2)
	{
          UtilityHNS::DataRW::WriteLogData(fileName.str(),
                          "PredictionLogCar0_",
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
                          "best_beh_U,all_p_U,all_w_U,best_p_U,best_w_U,real_w_U," , m_LogDataCar0);
	}

        if(m_LogDataCar1.size() > 2)
        {
          UtilityHNS::DataRW::WriteLogData(fileName.str(),
                          "PredictionLogCar1_",
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
                          "best_beh_U,all_p_U,all_w_U,best_p_U,best_w_U,real_w_U," , m_LogDataCar1);
        }

        if(m_LogDataCar2.size() > 2)
        {
          UtilityHNS::DataRW::WriteLogData(fileName.str(),
                          "PredictionLogCar2_",
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
                          "best_beh_U,all_p_U,all_w_U,best_p_U,best_w_U,real_w_U," , m_LogDataCar2);
        }
}

void MotionPrediction::UpdatePlanningParams(ros::NodeHandle& _nh)
{
	_nh.getParam("/op_common_params/enableSwerving", m_PlanningParams.enableSwerving);
	if(m_PlanningParams.enableSwerving)
		m_PlanningParams.enableFollowing = true;
	else
		_nh.getParam("/op_common_params/enableFollowing", m_PlanningParams.enableFollowing);

	_nh.getParam("/op_common_params/enableTrafficLightBehavior", m_PlanningParams.enableTrafficLightBehavior);
	_nh.getParam("/op_common_params/enableStopSignBehavior", m_PlanningParams.enableStopSignBehavior);

	_nh.getParam("/op_common_params/maxVelocity", m_PlanningParams.maxSpeed);
	_nh.getParam("/op_common_params/minVelocity", m_PlanningParams.minSpeed);
	_nh.getParam("/op_common_params/maxLocalPlanDistance", m_PlanningParams.microPlanDistance);


	_nh.getParam("/op_common_params/pathDensity", m_PlanningParams.pathDensity);
	_nh.getParam("/op_common_params/rollOutDensity", m_PlanningParams.rollOutDensity);
	if(m_PlanningParams.enableSwerving)
		_nh.getParam("/op_common_params/rollOutsNumber", m_PlanningParams.rollOutNumber);
	else
		m_PlanningParams.rollOutNumber = 0;

	std::cout << "Rolls Number: " << m_PlanningParams.rollOutNumber << std::endl;

	_nh.getParam("/op_common_params/horizonDistance", m_PlanningParams.horizonDistance);
	_nh.getParam("/op_common_params/minFollowingDistance", m_PlanningParams.minFollowingDistance);
	_nh.getParam("/op_common_params/minDistanceToAvoid", m_PlanningParams.minDistanceToAvoid);
	_nh.getParam("/op_common_params/maxDistanceToAvoid", m_PlanningParams.maxDistanceToAvoid);
	_nh.getParam("/op_common_params/speedProfileFactor", m_PlanningParams.speedProfileFactor);

	_nh.getParam("/op_common_params/horizontalSafetyDistance", m_PlanningParams.horizontalSafetyDistancel);
	_nh.getParam("/op_common_params/verticalSafetyDistance", m_PlanningParams.verticalSafetyDistance);

	_nh.getParam("/op_common_params/enableLaneChange", m_PlanningParams.enableLaneChange);

	_nh.getParam("/op_common_params/width", m_CarInfo.width);
	_nh.getParam("/op_common_params/length", m_CarInfo.length);
	_nh.getParam("/op_common_params/wheelBaseLength", m_CarInfo.wheel_base);
	_nh.getParam("/op_common_params/turningRadius", m_CarInfo.turning_radius);
	_nh.getParam("/op_common_params/maxSteerAngle", m_CarInfo.max_steer_angle);
	_nh.getParam("/op_common_params/maxAcceleration", m_CarInfo.max_acceleration);
	_nh.getParam("/op_common_params/maxDeceleration", m_CarInfo.max_deceleration);
	m_CarInfo.max_speed_forward = m_PlanningParams.maxSpeed;
	m_CarInfo.min_speed_forward = m_PlanningParams.minSpeed;

	int iSource = 0;
	_nh.getParam("/op_common_params/mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("/op_common_params/mapFileName" , m_MapPath);
	_nh.getParam("/op_common_params/experimentName" , m_ExperimentFolderName);
	if(m_ExperimentFolderName.size() > 0)
	{
		if(m_ExperimentFolderName.at(m_ExperimentFolderName.size()-1) != '/')
			m_ExperimentFolderName.push_back('/');
	}

	UtilityHNS::DataRW::CreateLoggingMainFolder();
	if(m_ExperimentFolderName.size() > 1)
		UtilityHNS::DataRW::CreateExperimentFolder(m_ExperimentFolderName);

	_nh.getParam("/op_motion_predictor/enableGenrateBranches" , m_PredictBeh.m_bGenerateBranches);
	_nh.getParam("/op_motion_predictor/max_distance_to_lane" , m_PredictBeh.m_MaxLaneDetectionDistance);
	_nh.getParam("/op_motion_predictor/prediction_distance" , m_PredictBeh.m_MaxPredictionDistance);
	_nh.getParam("/op_motion_predictor/enableCurbObstacles"	, m_bEnableCurbObstacles);
	_nh.getParam("/op_motion_predictor/distanceBetweenCurbs", m_DistanceBetweenCurbs);
	_nh.getParam("/op_motion_predictor/visualizationTime", m_VisualizationTime);
	_nh.getParam("/op_motion_predictor/enableStepByStepSignal", 	m_PredictBeh.m_bStepByStep );
	if(m_PredictBeh.m_bStepByStep)
		m_PredictBeh.m_bDebugOut = true;

	_nh.getParam("/op_motion_predictor/enableParticleFilterPrediction", 	m_PredictBeh.m_bParticleFilter);

	std::cout << "Particles Num Before : " <<  m_PredictBeh.g_PredParams.MAX_PARTICLES_NUM << std::endl;
	_nh.getParam("/op_motion_predictor/pose_weight_factor", 	m_PredictBeh.g_PredParams.POSE_FACTOR);
	_nh.getParam("/op_motion_predictor/dir_weight_factor", 	m_PredictBeh.g_PredParams.DIRECTION_FACTOR);
	_nh.getParam("/op_motion_predictor/vel_weight_factor", 	m_PredictBeh.g_PredParams.VELOCITY_FACTOR);
	_nh.getParam("/op_motion_predictor/acc_weight_factor", 	m_PredictBeh.g_PredParams.ACCELERATE_FACTOR);
	_nh.getParam("/op_motion_predictor/ind_weight_factor", 	m_PredictBeh.g_PredParams.INDICATOR_FACTOR);

	_nh.getParam("/op_motion_predictor/particles_number", 	m_PredictBeh.g_PredParams.MAX_PARTICLES_NUM);
	_nh.getParam("/op_motion_predictor/min_particles_num", 	m_PredictBeh.g_PredParams.MIN_PARTICLES_NUM);
	_nh.getParam("/op_motion_predictor/keep_percentage", 	m_PredictBeh.g_PredParams.KEEP_PERCENTAGE);
	m_PredictBeh.SetForTrajTracker();

	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);
}

void MotionPrediction::callbackGetStepForwardSignals(const geometry_msgs::TwistStampedConstPtr& msg)
{
	if(msg->twist.linear.x == 1)
		m_bGoNextStep = true;
	else
		m_bGoNextStep = false;
}

void MotionPrediction::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
	bNewCurrentPos = true;
}

void MotionPrediction::callbackGetVehicleStatus(const geometry_msgs::TwistStampedConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.linear.x;
	m_CurrentPos.v = m_VehicleStatus.speed;
	if(fabs(msg->twist.linear.x) > 0.25)
		m_VehicleStatus.steer = atan(m_CarInfo.wheel_base * msg->twist.angular.z/msg->twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
	m_VehicleStatus.speed = msg->speed/3.6;
	m_VehicleStatus.steer = msg->angle * m_CarInfo.max_steer_angle / m_CarInfo.max_steer_value;
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetRobotOdom(const nav_msgs::OdometryConstPtr& msg)
{
	m_VehicleStatus.speed = msg->twist.twist.linear.x;
	m_VehicleStatus.steer += atan(m_CarInfo.wheel_base * msg->twist.twist.angular.z/msg->twist.twist.linear.x);
	UtilityHNS::UtilityH::GetTickCount(m_VehicleStatus.tStamp);
	bVehicleStatus = true;
}

void MotionPrediction::callbackGetTrackedObjects(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
	UtilityHNS::UtilityH::GetTickCount(m_SensingTimer);
	m_TrackedObjects.clear();
	bTrackedObjects = true;

	PlannerHNS::DetectedObject obj;

	for(unsigned int i = 0 ; i <msg->objects.size(); i++)
	{
		if(msg->objects.at(i).id > 0)
		{
			PlannerHNS::RosHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(msg->objects.at(i), obj);
			m_TrackedObjects.push_back(obj);
		}
	}

	if(bMap)
	{
		if(m_PredictBeh.m_bStepByStep && m_bGoNextStep)
		{
			m_bGoNextStep = false;
			m_PredictBeh.DoOneStep(m_TrackedObjects, m_CurrentPos, m_PlanningParams.minSpeed, m_CarInfo.max_deceleration,  m_Map);
		}
		else if(!m_PredictBeh.m_bStepByStep)
		{
			m_PredictBeh.DoOneStep(m_TrackedObjects, m_CurrentPos, m_PlanningParams.minSpeed, m_CarInfo.max_deceleration,  m_Map);
		}


		m_PredictedResultsResults.objects.clear();
		autoware_msgs::DetectedObject pred_obj;
		for(unsigned int i = 0 ; i <m_PredictBeh.m_ParticleInfo_II.size(); i++)
		{
			PlannerHNS::RosHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(m_PredictBeh.m_ParticleInfo_II.at(i)->obj, false, pred_obj);
			if(m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track)
				pred_obj.behavior_state = m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->best_beh_by_p;
			m_PredictedResultsResults.objects.push_back(pred_obj);
		}

		if(m_bEnableCurbObstacles)
		{
			curr_curbs_obstacles.clear();
			GenerateCurbsObstacles(curr_curbs_obstacles);
			//std::cout << "Curbs No: " << curr_curbs_obstacles.size() << endl;
			for(unsigned int i = 0 ; i <curr_curbs_obstacles.size(); i++)
			{
				PlannerHNS::RosHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(curr_curbs_obstacles.at(i), false, pred_obj);
				m_PredictedResultsResults.objects.push_back(pred_obj);
			}
		}

		m_PredictedResultsResults.header.stamp = ros::Time().now();
		pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);

		LogDataRaw();
	}
}

void MotionPrediction::GenerateCurbsObstacles(std::vector<PlannerHNS::DetectedObject>& curb_obstacles)
{
	if(!bNewCurrentPos) return;

	for(unsigned int ic = 0; ic < m_Map.curbs.size(); ic++)
	{

		if(m_Map.curbs.at(ic).points.size() > 0)
		{
			PlannerHNS::DetectedObject obj;
			obj.center.pos = m_Map.curbs.at(ic).points.at(0);

			if(curb_obstacles.size()>0)
			{
				double distance_to_prev = hypot(curb_obstacles.at(curb_obstacles.size()-1).center.pos.y-obj.center.pos.y, curb_obstacles.at(curb_obstacles.size()-1).center.pos.x-obj.center.pos.x);
				if(distance_to_prev < m_DistanceBetweenCurbs)
					continue;
			}

			double longitudinalDist = hypot(m_CurrentPos.pos.y -obj.center.pos.y, m_CurrentPos.pos.x-obj.center.pos.x);

			if(longitudinalDist > m_PlanningParams.horizonDistance)
				continue;

			obj.bDirection = false;
			obj.bVelocity = false;
			obj.id = -1;
			obj.t  = PlannerHNS::SIDEWALK;
			obj.label = "curb";
			for(unsigned int icp=0; icp< m_Map.curbs.at(ic).points.size(); icp++)
			{
				obj.contour.push_back(m_Map.curbs.at(ic).points.at(icp));
			}

			curb_obstacles.push_back(obj);
		}
	}
}

std::string MotionPrediction::GetPredictionLogDataLine(std::vector<PlannerHNS::TrajectoryTracker*> trajectoryTrackers, std::string path_id)
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

std::string MotionPrediction::GetPredictionLogDataRealWeightItem(std::vector<PlannerHNS::TrajectoryTracker*> trajectoryTrackers, std::string path_id)
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

void MotionPrediction::LogDataRaw()
{
	for(unsigned int i=0; i < m_PredictBeh.m_ParticleInfo_II.size(); i++)
	{
          std::ostringstream dataLine;
          PlannerHNS::DetectedObject* pObj = &m_PredictBeh.m_ParticleInfo_II.at(i)->obj;
          dataLine << m_t << "," << pObj->center.pos.x << "," <<  pObj->center.pos.y << "," << pObj->center.pos.a << "," << pObj->center.v << "," << pObj->acceleration_desc
                          << "," << pObj->indicator_state << ",";

          if(m_PredictBeh.m_ParticleInfo_II.at(i)->best_forward_track != nullptr)
          {
            if(m_PredictBeh.m_ParticleInfo_II.at(i)->best_forward_track->id_.compare("F")==0)
              dataLine << "1" << ",";
            else if(m_PredictBeh.m_ParticleInfo_II.at(i)->best_forward_track->id_.compare("L")==0)
              dataLine << "2" << ",";
            else if(m_PredictBeh.m_ParticleInfo_II.at(i)->best_forward_track->id_.compare("R")==0)
              dataLine << "3" << ",";
            else
              dataLine << "0" << ",";
          }
          else
            dataLine << "0" << ",";


          dataLine << GetPredictionLogDataRealWeightItem(m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker, "F");
          dataLine << GetPredictionLogDataRealWeightItem(m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker, "L");
          dataLine << GetPredictionLogDataRealWeightItem(m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker, "R");

          if(m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track != nullptr)
          {
              dataLine << m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->best_beh_by_p << ",";
              dataLine << m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->best_beh_by_w << ",";
              dataLine << m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->w_avg_forward << ",";
              dataLine << m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->w_avg_stop << ",";
              dataLine << m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->w_avg_yield << ",";
              dataLine << m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->w_avg_park << ",";
          }
          else
            dataLine << "-1,-1,0,0,0,0,";

          dataLine << GetPredictionLogDataLine(m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker, "F");
          dataLine << GetPredictionLogDataLine(m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker, "L");
          dataLine << GetPredictionLogDataLine(m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker, "R");
          dataLine << GetPredictionLogDataLine(m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker, "U");

          if(i==0)
            m_LogDataCar0.push_back(dataLine.str());
          else if(i == 1)
            m_LogDataCar1.push_back(dataLine.str());
          else if(i == 2)
            m_LogDataCar2.push_back(dataLine.str());

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
}

void MotionPrediction::VisualizePrediction()
{
//	m_all_pred_paths.clear();
//	for(unsigned int i=0; i< m_PredictBeh.m_PredictedObjects.size(); i++)
//		m_all_pred_paths.insert(m_all_pred_paths.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.begin(), m_PredictBeh.m_PredictedObjects.at(i).predTrajectories.end());
//
//	PlannerHNS::RosHelpers::ConvertPredictedTrqajectoryMarkers(m_all_pred_paths, m_PredictedTrajectoriesActual, m_PredictedTrajectoriesDummy);
//	pub_PredictedTrajectoriesRviz.publish(m_PredictedTrajectoriesActual);
//
	PlannerHNS::RosHelpers::ConvertCurbsMarkers(curr_curbs_obstacles, m_CurbsActual, m_CurbsDummy);
	pub_CurbsRviz.publish(m_CurbsActual);

	m_all_pred_paths.clear();
	m_particles_points.clear();
	visualization_msgs::MarkerArray behavior_rviz_arr;


	m_TargetPointsOnTrajectories.markers.clear();

	for(unsigned int i=0; i< m_PredictBeh.m_ParticleInfo_II.size(); i++)
	{
		m_all_pred_paths.insert(m_all_pred_paths.begin(), m_PredictBeh.m_ParticleInfo_II.at(i)->obj.predTrajectories.begin(), m_PredictBeh.m_ParticleInfo_II.at(i)->obj.predTrajectories.end());

		for(unsigned int t=0; t < m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker.size(); t++)
		{
			PlannerHNS::WayPoint tt_wp = m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker.at(t)->followPoint;
			visualization_msgs::Marker targetPoint = PlannerHNS::RosHelpers::CreateGenMarker(tt_wp.pos.x,tt_wp.pos.y,tt_wp.pos.z,0,0,0.0,1,0.5,t,"target_trajectory_point", visualization_msgs::Marker::SPHERE);
			m_TargetPointsOnTrajectories.markers.push_back(targetPoint);

			PlannerHNS::WayPoint p_wp;
			for(unsigned int j=0; j < m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker.at(t)->m_CurrParts.size(); j++)
			{
				PlannerHNS::Particle* pPart = &m_PredictBeh.m_ParticleInfo_II.at(i)->m_TrajectoryTracker.at(t)->m_CurrParts.at(j);
				p_wp = pPart->pose;
				if(pPart->beh == PlannerHNS::BEH_STOPPING_STATE)
					p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
				else if(pPart->beh == PlannerHNS::BEH_PARKING_STATE)
					p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
				else if(pPart->beh == PlannerHNS::BEH_YIELDING_STATE)
					p_wp.bDir = PlannerHNS::BACKWARD_DIR;
				else if(pPart->beh == PlannerHNS::BEH_FORWARD_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_DIR;
				else if(pPart->beh == PlannerHNS::BEH_BRANCH_LEFT_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
				else if(pPart->beh == PlannerHNS::BEH_BRANCH_RIGHT_STATE)
					p_wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;

				m_particles_points.push_back(p_wp);
			}
		}

		if(m_PredictBeh.m_ParticleInfo_II.at(i) != nullptr && m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track != nullptr)
		{
			visualization_msgs::Marker behavior_rviz;
			std::ostringstream ns_beh;
			ns_beh << "pred_beh_state_" << i;
			PlannerHNS::RosHelpers::VisualizeIntentionState(m_PredictBeh.m_ParticleInfo_II.at(i)->obj.center, m_PredictBeh.m_ParticleInfo_II.at(i)->best_behavior_track->best_beh_by_p, behavior_rviz, ns_beh.str(), 3);
			behavior_rviz_arr.markers.push_back(behavior_rviz);
		}
	}
	pub_BehaviorStateRviz.publish(behavior_rviz_arr);


	PlannerHNS::RosHelpers::ConvertParticles(m_particles_points,m_PredictedParticlesActual, m_PredictedParticlesDummy);
	pub_ParticlesRviz.publish(m_PredictedParticlesActual);

	//std::cout << "Start Tracking of Trajectories : " <<  m_all_pred_paths.size() << endl;
	PlannerHNS::RosHelpers::ConvertPredictedTrqajectoryMarkers(m_all_pred_paths, m_PredictedTrajectoriesActual, m_PredictedTrajectoriesDummy);
	pub_PredictedTrajectoriesRviz.publish(m_PredictedTrajectoriesActual);

	m_generated_particles_points.clear();
	for(unsigned int i=0; i< m_PredictBeh.m_ParticleInfo_II.size(); i++)
	{
		PlannerHNS::WayPoint p_wp;
		for(unsigned int t=0; t < m_PredictBeh.m_ParticleInfo_II.at(i)->m_AllGeneratedParticles.size(); t++)
		{
			p_wp = m_PredictBeh.m_ParticleInfo_II.at(i)->m_AllGeneratedParticles.at(t).pose;
			if(m_PredictBeh.m_ParticleInfo_II.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_STOPPING_STATE)
			{
				p_wp.bDir = PlannerHNS::STANDSTILL_DIR;
			}
			else if(m_PredictBeh.m_ParticleInfo_II.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_FORWARD_STATE)
			{
				p_wp.bDir = PlannerHNS::FORWARD_DIR;
			}
			else if(m_PredictBeh.m_ParticleInfo_II.at(i)->m_AllGeneratedParticles.at(t).beh == PlannerHNS::BEH_YIELDING_STATE)
			{
				p_wp.bDir = PlannerHNS::BACKWARD_DIR;
			}
			m_generated_particles_points.push_back(p_wp);
		}
	}
	PlannerHNS::RosHelpers::ConvertParticles(m_generated_particles_points,m_GeneratedParticlesActual, m_GeneratedParticlesDummy, true);
	pub_GeneratedParticlesRviz.publish(m_GeneratedParticlesActual);


	pub_TargetPointsRviz.publish(m_TargetPointsOnTrajectories);

	UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
}

void MotionPrediction::MainLoop()
{

	ros::Rate loop_rate(25);

	while (ros::ok())
	{
		ros::spinOnce();

		if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_MapPath, m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_MapPath, m_Map, true);
		}
		else if (m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap)
		{
			std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;

			if(m_MapRaw.GetVersion()==2)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
						m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_PlanningParams.enableLaneChange, m_bEnableCurbObstacles);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map V2 Is Loaded successfully from the Motion Predictor !! " << std::endl;
				}
			}
			else if(m_MapRaw.GetVersion()==1)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true, m_PlanningParams.enableLaneChange, m_bEnableCurbObstacles);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map V1 Is Loaded successfully from the Motion Predictor !! " << std::endl;
				}
			}
		}

		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_VisualizationTimer) > m_VisualizationTime)
		{
			VisualizePrediction();
			UtilityHNS::UtilityH::GetTickCount(m_VisualizationTimer);
		}

		//For the debugging of prediction
//		if(UtilityHNS::UtilityH::GetTimeDiffNow(m_SensingTimer) > 5)
//		{
//			ROS_INFO("op_motion_prediction sensing timeout, can't receive tracked object data ! Reset .. Reset");
//			m_PredictedResultsResults.objects.clear();
//			pub_predicted_objects_trajectories.publish(m_PredictedResultsResults);
//		}

		loop_rate.sleep();
	}
}

//Mapping Section

void MotionPrediction::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void MotionPrediction::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void MotionPrediction::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void MotionPrediction::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void MotionPrediction::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void MotionPrediction::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void MotionPrediction::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void MotionPrediction::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void MotionPrediction::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void MotionPrediction::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void MotionPrediction::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void MotionPrediction::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void MotionPrediction::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void MotionPrediction::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

}
