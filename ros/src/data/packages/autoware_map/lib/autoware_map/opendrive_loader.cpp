/*
 * opendrive2autoware_converter_core.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/opendrive_loader.h"
#include <fstream>
#include <ros/ros.h>

namespace autoware_map
{



OpenDriveLoader::OpenDriveLoader():
keep_right_(true)
{
}

OpenDriveLoader::~OpenDriveLoader()
{
}

void OpenDriveLoader::getFileNameInFolder(const std::string& path, std::vector<std::string>& out_list)
{
	out_list.clear();
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL)
	{
	  while ((ent = readdir (dir)) != NULL)
	  {
		  std::string str(ent->d_name);
		  if(str.compare(".") !=0 && str.compare("..") !=0)
			  out_list.push_back(path+str);
	  }
	  closedir (dir);
	}
}

void OpenDriveLoader::loadCountryCods(const std::string& codes_csv_folder)
{
	if(codes_csv_folder.size() > 0)
	{
		std::vector<std::string> files_names;
		getFileNameInFolder(codes_csv_folder, files_names);
		country_signal_codes_.clear();

		for(unsigned int i=0; i < files_names.size();i++)
		{
			CSV_Reader reader(files_names.at(i));
			int i_ext_dot = files_names.at(i).find('.');
			int i_last_folder = files_names.at(i).find_last_of('/')+1;
			std::string country_code = files_names.at(i).substr(i_last_folder, i_ext_dot - i_last_folder);

			std::cout << "Reading Country Codes file: " << country_code << std::endl;

			std::vector<CSV_Reader::LINE_DATA> country_data;
			reader.ReadAllData(country_data);

			country_signal_codes_.push_back(std::make_pair(country_code, country_data));
		}
	}
	else
	{
		std::cout << "No Signs will be recognized, no Country Code files at: " << codes_csv_folder << std::endl;
	}
}

void OpenDriveLoader::loadOpenDRIVE(const std::string& xodr_file, const std::string& codes_folder, PlannerHNS::RoadNetwork& map, double resolution)
{
	std::ifstream f(xodr_file.c_str());
	if(!f.good())
	{
		std::cout << "Can't Open OpenDRIVE Map File: (" << xodr_file << ")" << std::endl;
		return;
	}

	std::cout << " >> Loading OpenDRIVE Map file ... " << std::endl;

	TiXmlDocument doc(xodr_file);
	try
	{
		doc.LoadFile();
	}
	catch(std::exception& e)
	{
		std::cout << "OpenDRIVE Custom Reader Error, Can't Load .xodr File, path is: "<<  xodr_file << std::endl;
		std::cout << e.what() << std::endl;
		return;
	}

	loadCountryCods(codes_folder);

	std::cout << " >> Reading Header Data from OpenDRIVE map file ... " << std::endl;
	std::vector<TiXmlElement*> elements;
	XmlHelpers::findFirstElement("header", doc.FirstChildElement(), elements);

	std::cout << "Final Results, Num:" << elements.size() <<std::endl;

	if(elements.size() > 0)
	{
		OpenDriveHeader header(elements.at(0));
		std::cout << "Final Results, Num:" << elements.size() << ", main element: " <<  elements.at(0)->Value() << std::endl;
	}

	std::cout << " >> Reading Data from OpenDRIVE map file ... " << std::endl;
	elements.clear();
	XmlHelpers::findElements("road", doc.FirstChildElement(), elements);
	std::cout << "Final Results Roads, Num:" << elements.size() << std::endl;

	ros::NodeHandle pnh("~");
	pnh.param("keep_right", keep_right_, true);
	roads_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		roads_list_.push_back(OpenDriveRoad(elements.at(i), &country_signal_codes_, keep_right_));
	}


	elements.clear();
	XmlHelpers::findElements("junction", doc.FirstChildElement(), elements);

	std::cout << "Final Results Junctions, Num:" << elements.size() << std::endl;

	junctions_list_.clear();
	for(unsigned int i=0; i < elements.size(); i++)
	{
		junctions_list_.push_back(Junction(elements.at(i)));
	}

	//Connect Roads
	connectRoads();
	
	// for(unsigned int i=0; i < roads_list_.size(); i++)
	// {
	// 	std::cout << "Road ID: " << roads_list_.at(i).id_ << std::endl;
	// 	std::cout << "From: ";
	// 	for(unsigned int j=0; j < roads_list_.at(i).from_roads_.size(); j++)
	// 	{
	// 		std::cout << "("  << roads_list_.at(i).from_roads_.at(j).incoming_road_ << "|";
	// 		for(unsigned int k=0; k < roads_list_.at(i).from_roads_.at(j).lane_links.size(); k++)
	// 		{
	// 			std::cout << roads_list_.at(i).from_roads_.at(j).lane_links.at(k).first << ", " << roads_list_.at(i).from_roads_.at(j).lane_links.at(k).second << " ; ";
	// 		}
	// 		std::cout << ")";
	// 	}
	// 
	// 	std::cout << std::endl;
	// 	std::cout << "To : " ;
	// 
	// 	for(unsigned int j=0; j < roads_list_.at(i).to_roads_.size(); j++)
	// 	{
	// 		std::cout << "("  << roads_list_.at(i).to_roads_.at(j).outgoing_road_ <<"|";
	// 		for(unsigned int k=0; k < roads_list_.at(i).to_roads_.at(j).lane_links.size(); k++)
	// 		{
	// 			std::cout << roads_list_.at(i).to_roads_.at(j).lane_links.at(k).first << ", " << roads_list_.at(i).to_roads_.at(j).lane_links.at(k).second << " ; ";
	// 		}
	// 		std::cout << ")";
	// 	}
	// 
	// 	std::cout << std::endl <<std::endl;
	// }

	std::cout << "Finish Linking Road Network .. " << std::endl;

	PlannerHNS::RoadSegment segment;
	segment.id = 1;
	map.roadSegments.push_back(segment);
	getMapLanes(map.roadSegments.at(0).Lanes, resolution);
	std::cout << "Finish Extracting Lanes: " << map.roadSegments.at(0).Lanes.size() << std::endl;

	getTrafficLights(map.trafficLights);
	getTrafficSigns(map.signs);
	getStopLines(map.stopLines);

	std::cout << "Finish Extracting Traffic Objects ... " << std::endl;

	linkWayPoints(map);

	std::cout << "Finish Linking Way Points ... " << std::endl;

}

std::vector<OpenDriveRoad*> OpenDriveLoader::getRoadsBySuccId(int _id)
{
	std::vector<OpenDriveRoad*> _ret_list;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		for(unsigned int j=0; j < roads_list_.at(i).successor_road_.size(); j++)
		{
			if(roads_list_.at(i).successor_road_.at(j).to_road_id_ == _id)
			{
				_ret_list.push_back(&roads_list_.at(i));
			}
		}
	}

	return _ret_list;
}

std::vector<OpenDriveRoad*> OpenDriveLoader::getRoadsByPredId(int _id)
{
	std::vector<OpenDriveRoad*> _ret_list;
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		for(unsigned int j=0; j < roads_list_.at(i).predecessor_road_.size(); j++)
		{
			if(roads_list_.at(i).predecessor_road_.at(j).from_road_id_ == _id)
			{
				_ret_list.push_back(&roads_list_.at(i));
			}
		}
	}

	return _ret_list;
}

OpenDriveRoad* OpenDriveLoader::getRoadById(int _id)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).id_ == _id)
		{
			return &roads_list_.at(i);
		}
	}

	return nullptr;
}

Junction* OpenDriveLoader::getJunctionById(int _id)
{
	for(unsigned int i=0; i < junctions_list_.size(); i++)
	{
		if(junctions_list_.at(i).id_ == _id)
		{
			return &junctions_list_.at(i);
		}
	}

	return nullptr;
}

void OpenDriveLoader::connectRoads()
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).predecessor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).predecessor_road_.at(0).link_type_ == ROAD_LINK)
			{
				OpenDriveRoad* p_pre_road = getRoadById(roads_list_.at(i).predecessor_road_.at(0).from_road_id_);
				if(p_pre_road != nullptr)
				{
					std::vector<Connection> pre_conn_list = roads_list_.at(i).getFirstSectionConnections(p_pre_road);
					for(unsigned k=0; k < pre_conn_list.size(); k++)
					{
						if( !keep_right_)
						{
							pre_conn_list.at(k).flip();
						}
						if(pre_conn_list.at(k).outgoing_road_ == roads_list_.at(i).id_)
						{
							roads_list_.at(i).insertUniqueFromConnection(pre_conn_list.at(k));							
						}
						else if (pre_conn_list.at(k).incoming_road_ == roads_list_.at(i).id_)
						{
							roads_list_.at(i).insertUniqueToConnection(pre_conn_list.at(k));
						}
					}
				}
			}
		}

		if(roads_list_.at(i).successor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).successor_road_.at(0).link_type_ == ROAD_LINK)
			{
				OpenDriveRoad* p_suc_road = getRoadById(roads_list_.at(i).successor_road_.at(0).to_road_id_);
				if(p_suc_road != nullptr)
				{
					std::vector<Connection> suc_conn_list = roads_list_.at(i).getLastSectionConnections(p_suc_road);
					for(unsigned k=0; k < suc_conn_list.size(); k++)
					{
						if(!keep_right_)
						{
							suc_conn_list.at(k).flip();
						}
						if(suc_conn_list.at(k).outgoing_road_ == roads_list_.at(i).id_)
						{
							roads_list_.at(i).insertUniqueFromConnection(suc_conn_list.at(k));							
						}
						else if (suc_conn_list.at(k).incoming_road_ == roads_list_.at(i).id_)
						{
							roads_list_.at(i).insertUniqueToConnection(suc_conn_list.at(k));
						}
					}
				}
			}
		}
	}

	//Link Junctions
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		if(roads_list_.at(i).predecessor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).predecessor_road_.at(0).link_type_ == JUNCTION_LINK)
			{
				Junction* p_junction = getJunctionById(roads_list_.at(i).predecessor_road_.at(0).from_road_id_);
				if(p_junction != nullptr)
				{
					for( const auto junction_connection : p_junction->getConnectionsByRoadId(roads_list_.at(i).id_))
					{
						OpenDriveRoad* incoming_road = getRoadById(junction_connection.outgoing_road_);
						if( incoming_road == nullptr) continue;
	
						RoadSection *outgoing_section = roads_list_.at(i).getLastSection();
						RoadSection *incoming_section = nullptr;
						if(junction_connection.contact_point_ == "end")
							incoming_section = incoming_road->getLastSection();
						else
							incoming_section = incoming_road->getFirstSection();
	
						Connection connection;
						connection.incoming_road_ = junction_connection.outgoing_road_;
						connection.outgoing_road_ = roads_list_.at(i).id_;
						if(incoming_section != nullptr)
							connection.incoming_section_ = incoming_section->id_;
						if(outgoing_section != nullptr)
							connection.outgoing_section_ = roads_list_.at(i).getLastSection()->id_;
						connection.lane_links = junction_connection.lane_links;
	
						if( !connection.lane_links.empty() )
						{
							//flip appropriate lane depending on keep_right flag
							if( (keep_right_ && connection.lane_links.at(0).first > 0 ) || (!keep_right_ && connection.lane_links.at(0).first < 0))
							{
								connection.flipRoad();							
								roads_list_.at(i).insertUniqueToConnection(connection);															
							} 
							else
							{
								roads_list_.at(i).insertUniqueFromConnection(connection);							
							}
						}
					}
				}
			}
		}
	
		if(roads_list_.at(i).successor_road_.size() > 0)
		{
			//connect normal roads , junctions will be handeled alone
			if(roads_list_.at(i).successor_road_.at(0).link_type_ == JUNCTION_LINK)
			{
				Junction* p_junction = getJunctionById(roads_list_.at(i).successor_road_.at(0).to_road_id_);
				if(p_junction != nullptr)
				{
					for( const auto junction_connection : p_junction->getConnectionsByRoadId(roads_list_.at(i).id_))
					{
						OpenDriveRoad* outgoing_road = getRoadById(junction_connection.outgoing_road_);
						if( outgoing_road == nullptr) continue;
	
						RoadSection *incoming_section = roads_list_.at(i).getLastSection();
						RoadSection *outgoing_section = nullptr;
						if(junction_connection.contact_point_ == "end")
							outgoing_section = outgoing_road->getLastSection();
						else
							outgoing_section = outgoing_road->getFirstSection();
						if(incoming_section == nullptr || outgoing_section == nullptr) continue;
	
						Connection connection;
						connection.incoming_road_ = roads_list_.at(i).id_;
						connection.outgoing_road_ = junction_connection.outgoing_road_;
						connection.incoming_section_ = incoming_section->id_;
						connection.outgoing_section_ = outgoing_section->id_;
						connection.lane_links = junction_connection.lane_links;
	
						//flip appropriate lane depending on keep_right flag
						if( !connection.lane_links.empty() )
						{
							if( (keep_right_ && connection.lane_links.at(0).first > 0) || (!keep_right_ && connection.lane_links.at(0).first < 0) )
							{
								connection.flipRoad();
								roads_list_.at(i).insertUniqueFromConnection(connection);															
							}
							else
							{
								roads_list_.at(i).insertUniqueToConnection(connection);							
							}
						}
					}
				}
			}
		}
	}

	// for(unsigned int i=0; i < junctions_list_.size(); i++)
	// {
	// 	for(unsigned int j=0; j < junctions_list_.at(i).connections_.size(); j++)
	// 	{
	// 		//std::cout << "J_ID: " << junctions_list_.at(i).id_ << ", (" << junctions_list_.at(i).connections_.at(j).incoming_road_ << ", " << junctions_list_.at(i).connections_.at(j).outgoing_road_ << " )" <<std::endl;
	// 		OpenDriveRoad* p_from_road = getRoadById(junctions_list_.at(i).connections_.at(j).incoming_road_);
	// 		if(p_from_road != nullptr)
	// 		{
	// 			p_from_road->insertUniqueToConnection(junctions_list_.at(i).connections_.at(j));
	// 		}
	// 
	// 		OpenDriveRoad* p_to_road = getRoadById(junctions_list_.at(i).connections_.at(j).outgoing_road_);
	// 		if(p_to_road != nullptr)
	// 		{
	// 			p_to_road->insertUniqueFromConnection(junctions_list_.at(i).connections_.at(j));
	// 		}
	// 	}
	// }
	// 
	// //Link Missing successors that are linked to junctions
	// for(unsigned int i=0; i < roads_list_.size(); i++)
	// {
	// 	if(roads_list_.at(i).predecessor_road_.size() > 0)
	// 	{
	// 		if(roads_list_.at(i).predecessor_road_.at(0).link_type_ != ROAD_LINK)
	// 		{
	// 			std::vector<OpenDriveRoad*> pred_list = getRoadsBySuccId(roads_list_.at(i).id_);
	// 			for(unsigned int j=0; j < pred_list.size(); j++)
	// 			{
	// 				for(unsigned int k=0; k < pred_list.at(j)->to_roads_.size(); k++)
	// 				{
	// 					if(pred_list.at(j)->to_roads_.at(k).outgoing_road_ == roads_list_.at(i).id_)
	// 					{
	// 						roads_list_.at(i).insertUniqueFromConnection(pred_list.at(j)->to_roads_.at(k));
	// 					}
	// 				}
	// 			}
	// 		}
	// 	}
	// }
}

void OpenDriveLoader::getMapLanes(std::vector<PlannerHNS::Lane>& all_lanes, double resolution)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getRoadLanes(all_lanes, resolution);
	}
}

void OpenDriveLoader::getTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getTrafficLights(all_lights);
	}
}

void OpenDriveLoader::getTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getTrafficSigns(all_signs);
	}
}

void OpenDriveLoader::getStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines)
{
	for(unsigned int i=0; i < roads_list_.size(); i++)
	{
		roads_list_.at(i).getStopLines(all_stop_lines);
	}
}

void OpenDriveLoader::linkWayPoints(PlannerHNS::RoadNetwork& map)
{
	linkLanesPointers(map);

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			PlannerHNS::Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int iwp= 0; iwp < pL->points.size(); iwp++)
			{
				if(iwp < pL->points.size()-1)
				{
					pL->points.at(iwp).toIds.push_back(pL->points.at(iwp+1).id);
				}
				else
				{
					for(unsigned int k=0; k< pL->toLanes.size(); k++)
					{
						if(pL->toLanes.at(k) != nullptr && pL->toLanes.at(k)->points.size()>0)
						{
							pL->points.at(iwp).toIds.push_back(pL->toLanes.at(k)->points.at(0).id);
						}
					}
				}
			}
		}
	}
}

void OpenDriveLoader::linkLanesPointers(PlannerHNS::RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			PlannerHNS::Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
					{
						pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
					{
						pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				pL->points.at(j).pLane  = pL;
			}
		}
	}
}

}
