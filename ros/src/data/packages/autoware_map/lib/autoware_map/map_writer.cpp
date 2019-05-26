/*
 * opendrive2op_map_converter_core.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/map_writer.h"
#include "op_planner/RoadNetwork.h"
#include <fstream>

#include <autoware_map_msgs/LaneArray.h>
#include <autoware_map_msgs/LaneAttrRelationArray.h>
#include <autoware_map_msgs/LaneRelationArray.h>
#include <autoware_map_msgs/LaneSignalLightRelationArray.h>
#include <autoware_map_msgs/LaneChangeRelationArray.h>
#include <autoware_map_msgs/OppositeLaneRelationArray.h>
#include <autoware_map_msgs/RouteArray.h>
#include <autoware_map_msgs/PointArray.h>
#include <autoware_map_msgs/AreaArray.h>
#include <autoware_map_msgs/SignalArray.h>
#include <autoware_map_msgs/SignalLightArray.h>
#include <autoware_map_msgs/WayareaArray.h>
#include <autoware_map_msgs/WaypointArray.h>
#include <autoware_map_msgs/WaypointLaneRelationArray.h>
#include <autoware_map_msgs/WaypointRelationArray.h>
#include <autoware_map_msgs/WaypointSignalRelationArray.h>

#include "autoware_map/util.h"

namespace autoware_map
{
	
	bool exists(autoware_map_msgs::LaneArray &lanes, int lane_id){
		for(const auto lane: lanes.data){
			if(lane_id == lane.lane_id) return true;
		}
		return false;
	}

	bool exists(autoware_map_msgs::WaypointArray &waypoints, int waypoint_id){
		for(const auto waypoint: waypoints.data){
			if(waypoint_id == waypoint.waypoint_id) return true;
		}
		return false;
	}

MapWriter::MapWriter()
{
}

MapWriter::~MapWriter()
{
}

template <class T>
void MapWriter::writeCSVFile(const std::string& folder, const std::string& title, const std::string& header, const std::vector<T>& data_list)
{
	if(data_list.size() < 2)
			return;

	std::ostringstream file_name;
	file_name << folder;
	file_name << title;
	file_name << ".csv";

	std::ofstream f(file_name.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < data_list.size(); i++)
			f << data_list.at(i) << "\r\n";
	}

	f.close();
}

void MapWriter::writeAutowareMap(std::string folder_name, PlannerHNS::RoadNetwork& map)
{
	if(map.roadSegments.size() == 0)
	{
		std::cout << "Can't Write Empty Map (no lanes) into autoware format ! " << std::endl;
		return;
	}

	autoware_map_msgs::PointArray points_list;
	autoware_map_msgs::LaneArray lanes_list;
	autoware_map_msgs::LaneRelationArray lane_relations_list;
	autoware_map_msgs::SignalArray signals_list;
	autoware_map_msgs::SignalLightArray lights_list;
	autoware_map_msgs::WaypointSignalRelationArray wp_lights_relations_list;
	autoware_map_msgs::WaypointArray waypoints_list;
	autoware_map_msgs::WaypointRelationArray wp_relations_list;
	autoware_map_msgs::WaypointLaneRelationArray wp_lanes_relations_list;

	int points_id_seq = 1;
	int signal_id_seq = 1;


	for(unsigned int i=0; i<map.roadSegments.at(0).Lanes.size(); i++)
	{
		PlannerHNS::Lane* p_lane = &map.roadSegments.at(0).Lanes.at(i);

		if(p_lane->points.size() < 2)
		{
			std::cout << "Writing Autoware format, Skip lane with less than 2 waypoints, with ID: " << p_lane->id << std::endl;
			continue;
		}

		autoware_map_msgs::Lane l;
		l.lane_id = p_lane->id;
		l.lane_number = p_lane->num; //TODO check in document
		l.length = p_lane->length;
		l.num_of_lanes = p_lane->num; //TODO check in document
		l.speed_limit = p_lane->speed;
		l.start_waypoint_id = p_lane->points.at(0).id;
		l.end_waypoint_id = p_lane->points.at(p_lane->points.size()-1).id;
		l.height_limit = 0;
		l.weight_limit = 0;
		l.width_limit = p_lane->width;
		lanes_list.data.push_back(l);

		autoware_map_msgs::LaneRelation lr;
		lr.lane_id = p_lane->id;
		for(unsigned int k=0; k< p_lane->toIds.size(); k++)
		{
			lr.blinker = 0; //TODO check in document for later support
			lr.next_lane_id = p_lane->toIds.at(k);
			lane_relations_list.data.push_back(lr);
		}

		for(unsigned int j=0; j< p_lane->points.size(); j++)
		{
			autoware_map_msgs::Point p;
			p.mgrs = 0; //TODO check in document
			p.epsg = 0;  //TODO check in document
			p.lat = 0;
			p.lng = 0;
			p.pcd = "";
			p.point_id = points_id_seq++;
			p.x = p_lane->points.at(j).pos.x;
			p.y = p_lane->points.at(j).pos.y;
			p.z = p_lane->points.at(j).pos.z;
			points_list.data.push_back(p);

			autoware_map_msgs::Waypoint wp;
			wp.point_id = p.point_id;
			wp.height = 0; //TODO check in document
			wp.left_width = p_lane->points.at(j).collisionCost/2.0;
			wp.right_width = p_lane->points.at(j).collisionCost/2.0;
			wp.stop_line = p_lane->points.at(j).stopLineID;
			wp.velocity = p_lane->points.at(j).v;
			wp.waypoint_id = p_lane->points.at(j).id;
			waypoints_list.data.push_back(wp);

			autoware_map_msgs::WaypointRelation wpr;
			wpr.waypoint_id = wp.waypoint_id;
			wpr.yaw = p_lane->points.at(j).pos.a;
			wpr.blinker = 0; //TODO check in document

			if(j < p_lane->points.size()-1)
			{
				wpr.distance = hypot(p_lane->points.at(j+1).pos.y - p.y, p_lane->points.at(j+1).pos.x - p.x);
			}
			else
			{
				wpr.distance = 0;
			}

			for(unsigned int k=0; k < p_lane->points.at(j).toIds.size(); k++)
			{
				wpr.next_waypoint_id =  p_lane->points.at(j).toIds.at(k);
				wp_relations_list.data.push_back(wpr);
			}

			autoware_map_msgs::WaypointLaneRelation wplr;
			wplr.waypoint_id = wp.waypoint_id;
			wplr.lane_id = l.lane_id;
			wplr.order = j;
			wp_lanes_relations_list.data.push_back(wplr);
		}
	}

	//remove invalid relations
	auto erase_itr = std::remove_if(lane_relations_list.data.begin(),lane_relations_list.data.end(),
 								[&](autoware_map_msgs::LaneRelation lr){return (!exists(lanes_list, lr.next_lane_id)) ;});
  for(auto itr = erase_itr; itr != lane_relations_list.data.end() ; itr++ ){
		std::cout << "deleting: " << itr->lane_id << "->" << itr->next_lane_id << " " << (!exists(lanes_list, itr->lane_id)) << std::endl;
	}
	lane_relations_list.data.erase(erase_itr, lane_relations_list.data.end());

	auto erase_itr2 = std::remove_if(wp_relations_list.data.begin(),wp_relations_list.data.end(),
 								[&](autoware_map_msgs::WaypointRelation wr){return !exists(waypoints_list, wr.waypoint_id) || !exists(waypoints_list, wr.next_waypoint_id) ;});
	wp_relations_list.data.erase(erase_itr2, wp_relations_list.data.end());


	for(unsigned int i=0; i<map.trafficLights.size(); i++)
	{

		PlannerHNS::TrafficLight* p_light = &map.trafficLights.at(i);

		autoware_map_msgs::Point p;
		p.mgrs = 0; //TODO check in document
		p.epsg = 0;  //TODO check in document
		p.lat = 0;
		p.lng = 0;
		p.pcd = "";
		p.point_id = points_id_seq++;
		p.x = p_light->pos.x;
		p.y = p_light->pos.y;
		p.z = p_light->pos.z;
		points_list.data.push_back(p);

		autoware_map_msgs::Signal s;
		s.signal_id = signal_id_seq++;
		signals_list.data.push_back(s);

		autoware_map_msgs::SignalLight sl;
		sl.arrow_type = 0; //TODO support standard
		sl.color_type = 0;
		sl.point_id = p.point_id;
		sl.signal_id = s.signal_id;
		sl.signal_light_id = p_light->id;
		sl.horizontal_angle = p_light->pos.a;

		//sl.vertical_angle = p_light->rot.y; //TODO support standard and opendrive
		lights_list.data.push_back(sl);

		for(unsigned int j=0; j < p_light->laneIds.size(); j++)
		{
			autoware_map_msgs::WaypointSignalRelation wp_s_r;
			PlannerHNS::Lane* p_op_lane = getLaneFromID(map, p_light->laneIds.at(j));
			if(p_op_lane != nullptr)
			{
				wp_s_r.signal_id = sl.signal_id;
				wp_s_r.waypoint_id = p_op_lane->points.at(p_op_lane->points.size()-1).id;
				wp_lights_relations_list.data.push_back(wp_s_r);
			}
		}
	}

	for(unsigned int i=0; i<map.signs.size(); i++)
	{
		PlannerHNS::TrafficSign* p_sign = &map.signs.at(i);


		autoware_map_msgs::Point p;
		p.mgrs = 0; //TODO check in document
		p.epsg = 0;  //TODO check in document
		p.lat = 0;
		p.lng = 0;
		p.pcd = "";
		p.point_id = points_id_seq++;
		p.x = p_sign->pos.x;
		p.y = p_sign->pos.y;
		p.z = p_sign->pos.z;
		points_list.data.push_back(p);


	}


	writeCSVFile(folder_name, "points", "point_id,x,y,z,lat,lng,pcd,mgrs,epsg", points_list.data);

	writeCSVFile(folder_name, "lanes", "lane_id,start_waypoint_id,end_waypoint_id,"
			"lane_number,num_of_lanes,speed_limit,length,width_limit,height_limit,weight_limit", lanes_list.data);

	writeCSVFile(folder_name, "lane_relations", "lane_id,next_lane_id,blinker", lane_relations_list.data);

	writeCSVFile(folder_name, "waypoints", "waypoint_id,point_id,velocity,stop_line,left_width,right_width,height", waypoints_list.data);

	writeCSVFile(folder_name, "waypoint_relations", "waypoint_id,next_waypoint_id,yaw,blinker,distance", wp_relations_list.data);

	writeCSVFile(folder_name, "waypoint_lane_relations", "waypoint_id,lane_id,order", wp_lanes_relations_list.data);

	writeCSVFile(folder_name, "signals", "signal_id", signals_list.data);

	writeCSVFile(folder_name, "signal_lights", "signal_light_id,signal_id,point_id,horizontal_angle,vertical_angle,color_type,arrow_type", lights_list.data);

	writeCSVFile(folder_name, "waypoint_signal_relations", "waypoint_id,signal_id", wp_lights_relations_list.data);

	std::cout << "Finish Writing map files .csv to folder: " << folder_name << std::endl;
}

}
