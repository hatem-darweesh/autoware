/*
 * opendrive_road.cpp
 *
 *  Created on: Feb 11, 2019
 *      Author: hatem
 */

#include "opendrive2autoware_converter/opendrive_road.h"
#include <fstream>
#include <functional>
namespace autoware_map
{

double g_epsilon = 0.0000001;

OpenDriveRoad::OpenDriveRoad(TiXmlElement* main_element, std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA> > >* country_signal_codes, bool keep_right)
{
	name_ = XmlHelpers::getStringAttribute(main_element, "name", "");
	id_ = XmlHelpers::getIntAttribute(main_element, "id", 0);
	junction_id_ = XmlHelpers::getIntAttribute(main_element, "junction", -1);
	length_ = XmlHelpers::getDoubleAttribute(main_element, "length", 0.0);
	p_country_signal_codes_ = country_signal_codes;
	keep_right_ = keep_right;

	//Read Links
	std::vector<TiXmlElement*> sub_elements;
	std::vector<TiXmlElement*> lane_elements;
	std::vector<TiXmlElement*> elements;

	XmlHelpers::findFirstElement("link", main_element, elements);
	if(elements.size() > 0)
	{
		std::vector<TiXmlElement*> pred_elements, succ_elements;

		XmlHelpers::findElements("predecessor", elements.at(0)->FirstChildElement(), pred_elements);
		for(unsigned int j=0; j < pred_elements.size(); j++)
		{
			predecessor_road_.push_back(FromRoadLink(pred_elements.at(j)));
		}

		XmlHelpers::findElements("successor", elements.at(0)->FirstChildElement(), succ_elements);
		for(unsigned int j=0; j < succ_elements.size(); j++)
		{
			successor_road_.push_back(ToRoadLink(succ_elements.at(j)));
		}
	}

	elements.clear();
	XmlHelpers::findFirstElement("planView", main_element, elements);
	if(elements.size() > 0)
	{
		//Get Geometries
		std::vector<TiXmlElement*> geom_elements;
		XmlHelpers::findElements("geometry", elements.at(0), geom_elements);
		for(unsigned int j=0; j < geom_elements.size(); j++)
		{
			geometries_.push_back(Geometry(geom_elements.at(j)));
		}
	}

	elements.clear();
	XmlHelpers::findFirstElement("elevationProfile", main_element, elements);
	if(elements.size() > 0)
	{
		//Get Geometries
		std::vector<TiXmlElement*> elev_elements;
		XmlHelpers::findElements("elevation", elements.at(0), elev_elements);
		for(unsigned int j=0; j < elev_elements.size(); j++)
		{
			elevations_.push_back(Elevation(elev_elements.at(j)));
		}
	}

	int lanes_index_count = 1;
	elements.clear();
	XmlHelpers::findFirstElement("lanes", main_element, elements);
	if(elements.size()>0)
	{
		//laneOffsets
		std::vector<TiXmlElement*> offsets;
		XmlHelpers::findElements("laneOffsets", elements.at(0), offsets);
		for(unsigned int j=0; j < offsets.size(); j++)
		{
			laneOffsets_.push_back(LaneOffset(offsets.at(j)));
		}

		//std::cout << "LaneOffsets Loaded with: " << laneOffsets_.size() <<" offsets."<<std::endl;

		//laneSections and lanes
		std::vector<TiXmlElement*> sections;
		XmlHelpers::findElements("laneSection", elements.at(0), sections);

		for(unsigned int j=0; j < sections.size(); j++)
		{
			double curr_section_s = XmlHelpers::getDoubleAttribute(sections.at(j), "s", 0.0);
			double next_section_s = 0.0;
			double section_length = 0.0;

			if(j+1 < sections.size())
			{
				next_section_s = XmlHelpers::getDoubleAttribute(sections.at(j+1), "s", 0.0);
				section_length = next_section_s - curr_section_s;
			}
			else
			{
				section_length = length_ - curr_section_s;
			}

			if(section_length <= 0)
			{
				std::cout << "Load From OpenDrive with Section length is 0.0 !!" << std::endl;
				continue;
			}

			sections_.push_back(RoadSection(sections.at(j),curr_section_s, section_length, j));

			if(j > 0)
			{
				sections_.at(sections_.size()-1).p_prev_section_ = &sections_.at(sections_.size()-2);
				sections_.at(sections_.size()-2).p_next_section_ = &sections_.at(sections_.size()-1);
			}
		}

		//std::cout << "LaneSections Loaded with: " << sections.size() <<" Sections , And: " << lanes_.size() << " Lanes" <<std::endl;
	}

	elements.clear();
	XmlHelpers::findFirstElement("signals", main_element, elements);
	if(elements.size()>0)
	{
		sub_elements.clear();
		XmlHelpers::findElements("signal", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_signals_.push_back(Signal(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::findElements("signalReference", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_signals_references_.push_back(SignalRef(sub_elements.at(j)));
		}
	}

	elements.clear();
	XmlHelpers::findFirstElement("objects", main_element, elements);
	if(elements.size()>0)
	{
		sub_elements.clear();
		XmlHelpers::findElements("object", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_.push_back(RoadObject(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::findElements("objectReference", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_references_.push_back(RoadObjectRef(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::findElements("tunnel", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_tunnels_.push_back(RoadObjectTunnel(sub_elements.at(j)));
		}

		sub_elements.clear();
		XmlHelpers::findElements("object", main_element, sub_elements);
		for(unsigned int j=0; j < sub_elements.size(); j++)
		{
			road_objects_bridges_.push_back(RoadObjectBridge(sub_elements.at(j)));
		}
	}

	//std::cout << "Road Loaded With ID: " << id_ << " , Length: " << length_  << std::endl;
}

bool OpenDriveRoad::createSingleCenterPoint(double _ds, PlannerHNS::WayPoint& _p)
{
	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		if(_ds >= geometries_.at(i).s && _ds <= (geometries_.at(i).s+geometries_.at(i).length))
		{
			if(geometries_.at(i).getPoint(_ds, _p))
			{
				Elevation* p_elv = getMatchingElevations(_ds);
				if(p_elv != nullptr)
				{
					_p.pos.z = p_elv->getHeigh(_ds);
				}
				return true;
			}
		}
	}

	return false;
}

bool OpenDriveRoad::createRoadCenterPoint(RoadCenterInfo& inf_point, double _s)
{
	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		double end_distance = geometries_.at(i).s + geometries_.at(i).length;

		if(_s >= geometries_.at(i).s && _s <= end_distance)
		{
			RoadCenterInfo inf;
			PlannerHNS::WayPoint p;
			if(geometries_.at(i).getPoint(_s, p))
			{
				Elevation* p_elv = getMatchingElevations(_s);
				if(p_elv != nullptr)
				{
					p.pos.z = p_elv->getHeigh(_s);
				}

				LaneOffset* p_lane_off = getMatchingLaneOffset(_s);
				if(p_lane_off != nullptr)
				{
					inf.offset_width_ = p_lane_off->getOffset(_s);
				}

				inf.ds_ = _s;
				inf.center_p_ = p.pos;
				inf_point = inf;
				return true;
			}
		}
	}

	return false;
}

void OpenDriveRoad::insertRoadCenterInfo(std::vector<RoadCenterInfo>& points_list, RoadCenterInfo& inf_point)
{
	for(unsigned int i = 0; i < points_list.size(); i++)
	{
		if(inf_point.ds_ == points_list.at(i).ds_) // exist
		{
			return;
		}
		else if(inf_point.ds_  < points_list.at(i).ds_)
		{
			points_list.insert(points_list.begin()+i, inf_point);
			return;
		}
	}

	points_list.push_back(inf_point);
}

void OpenDriveRoad::createRoadCenterInfo(std::vector<RoadCenterInfo>& points_list, double resolution)
{
	PlannerHNS::WayPoint p;
	RoadCenterInfo inf;
	points_list.clear();
	for(unsigned int i=0; i < geometries_.size(); i++)
	{
		int n_waypoints = floor(geometries_.at(i).length / resolution) + 1;

//		std::cout << "### WayPoints are: " << n_waypoints << ", for Road Geometry : (" << id_ << "," << i << "), With Res: " << resolution << ", and Length: " << geometries_.at(i).length << std::endl;

		double s_inc = geometries_.at(i).s;
		double remaining_distance = 0.0;
		double end_distance = geometries_.at(i).s + geometries_.at(i).length;
		bool bOneMorePoint = false;

		for(int j=0; j< n_waypoints; j++)
		{
			if(geometries_.at(i).getPoint(s_inc, p))
			{
				Elevation* p_elv = getMatchingElevations(s_inc);
				if(p_elv != nullptr)
				{
					p.pos.z = p_elv->getHeigh(s_inc);
				}

				LaneOffset* p_lane_off = getMatchingLaneOffset(s_inc);
				if(p_lane_off != nullptr)
				{
					inf.offset_width_ = p_lane_off->getOffset(s_inc);
				}

				inf.ds_ = s_inc;
				inf.center_p_ = p.pos;
				points_list.push_back(inf);
			}

			remaining_distance = end_distance - s_inc;

			if(remaining_distance < resolution)
			{
				s_inc += remaining_distance;

				if(remaining_distance > g_epsilon)
				{
					bOneMorePoint = true;
				}
				else if(remaining_distance != 0)
				{
					std::cout << "$$$ Too Small Geometry !: " << "(" << id_ << "," << i << ", " << j << "), With Res: " << resolution << ", and Length: " << geometries_.at(i).length << ", " << remaining_distance << std::endl;
				}
			}
			else
				s_inc += resolution;
		}


		if(bOneMorePoint == true)
		{
			if(geometries_.at(i).getPoint(s_inc, p))
			{
				Elevation* p_elv = getMatchingElevations(s_inc);
				if(p_elv != nullptr)
				{
					p.pos.z = p_elv->getHeigh(s_inc);
				}

				LaneOffset* p_lane_off = getMatchingLaneOffset(s_inc);
				if(p_lane_off != nullptr)
				{
					inf.offset_width_ = p_lane_off->getOffset(s_inc);
				}

				inf.ds_ = s_inc;
				inf.center_p_ = p.pos;
				points_list.push_back(inf);
			}
		}
	}

	for(unsigned int i=0; i< sections_.size(); i++)
	{
		double s_inc = sections_.at(i).s_;
		if(createRoadCenterPoint(inf, s_inc))
		{
			insertRoadCenterInfo(points_list, inf);
		}
		else
		{
			std::cout << ">>>> Can't Find Geometry for Start of Section: " << i << ", Road: " << id_ << std::endl;
		}

		s_inc = sections_.at(i).s_ + sections_.at(i).length_;

		if(createRoadCenterPoint(inf, s_inc))
		{
			insertRoadCenterInfo(points_list, inf);
		}
		else
		{
			std::cout << ">>>> Can't Find Geometry for Start of Section: " << i << ", Road: " << id_ << std::endl;
		}
	}
}

void OpenDriveRoad::insertUniqueFromSectionIds(int from_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < curr_lane->from_lane_.size(); i++)
	{
		int from_lane_id = curr_lane->from_lane_.at(i).from_lane_id;
		int from_gen_id = (id_*100000 + 50000) + from_section_id * 1000 + from_lane_id * 100;

		if(exists(_l.fromIds, from_gen_id))
		{
			std::cout << "Redundant Connection, from road: " << id_ <<", to road: " << id_
						<< ", to section: " << from_section_id+1 << ", GenLaneID: " << from_gen_id << std::endl;
		}
		else
		{
			_l.fromIds.push_back(from_gen_id);
		}
	}
}

void OpenDriveRoad::insertUniqueToSectionIds(int to_section_id, const OpenDriveLane* curr_lane, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < curr_lane->to_lane_.size(); i++)
	{
		int to_lane_id = curr_lane->to_lane_.at(i).to_lane_id;
		int to_gen_id = (id_*100000 + 50000) + to_section_id *1000 + to_lane_id * 100;

		if(exists(_l.toIds, to_gen_id))
		{
			std::cout << "Redundant Connection, from road: " << id_ <<", to road: " << id_
						<< ", from section: " << to_section_id - 1 << ", GenLaneID: " << to_gen_id << std::endl;
		}
		else
		{
			_l.toIds.push_back(to_gen_id);
		}
	}
}

void OpenDriveRoad::insertUniqueFromRoadIds(int curr_section_id, int curr_lane_id, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < from_roads_.size(); i++)
	{
		int from_lane_id = from_roads_.at(i).getFromLane(curr_lane_id);
		if(from_lane_id != 0)
		{
			if(from_roads_.at(i).outgoing_road_ != id_)
				std::cout << " >>>  Something Very Bad Happened in InsertUniqueFromRoadIds, outgoing_road doesn't match current_road, " << from_roads_.at(i).outgoing_road_ << ", " << id_ << std::endl;

			int from_gen_id = (from_roads_.at(i).incoming_road_*100000 + 50000) + from_roads_.at(i).incoming_section_*1000 + from_lane_id * 100;
			if(exists(_l.fromIds, from_gen_id))
			{
				std::cout << "Redundant Connection, from road: " << from_roads_.at(i).incoming_road_ <<", to road: " << id_
							<< ", to section: " << curr_section_id << ", GenLaneID: " << from_gen_id << std::endl;
			}
			else
			{
				_l.fromIds.push_back(from_gen_id);
			}
		}
	}
}

void OpenDriveRoad::insertUniqueToRoadIds(int curr_section_id, int curr_lane_id, PlannerHNS::Lane& _l)
{
	for(unsigned int i=0; i < to_roads_.size(); i++)
	{
		int to_lane_id = to_roads_.at(i).getToLane(curr_lane_id);
		if(to_lane_id != 0)
		{
			if(to_roads_.at(i).incoming_road_ != id_)
				std::cout << " >>>  Something Very Bad Happened in InsertUniqueToRoadIds, incoming_road doesn't match current_road, " << to_roads_.at(i).incoming_road_ << ", " << id_ << std::endl;

			int to_gen_id = (to_roads_.at(i).outgoing_road_*100000 + 50000) + to_roads_.at(i).outgoing_section_*1000 + to_lane_id * 100;
			if(exists(_l.toIds, to_gen_id))
			{
				std::cout << "Redundant Connection, to road: " << to_roads_.at(i).outgoing_road_ <<", from road: " << id_
							<< ", from section: " << curr_section_id << ", GenLaneID: " << to_gen_id << std::endl;
			}
			else
			{
				_l.toIds.push_back(to_gen_id);
			}
		}
	}
}

void OpenDriveRoad::insertUniqueToConnection(const Connection& _connection)
{

	bool bFound = false;
	for(unsigned int j=0; j < to_roads_.size(); j++)
	{
		if(to_roads_.at(j).incoming_road_== _connection.incoming_road_ && to_roads_.at(j).outgoing_road_ == _connection.outgoing_road_)
		{
			for(unsigned int k=0; k < to_roads_.at(j).lane_links.size(); k++)
			{
				for(unsigned int lk=0; lk < _connection.lane_links.size(); lk++)
				{
					if(to_roads_.at(j).lane_links.at(k).first == _connection.lane_links.at(lk).first && to_roads_.at(j).lane_links.at(k).second == _connection.lane_links.at(lk).second)
					{
						bFound = true;
						break;
					}
				}

				if(bFound == true)
					break;
			}

			if(bFound == true)
				break;
		}
	}

	if(!bFound)
	{
		to_roads_.push_back(_connection);
	}
	else
	{
		//std::cout << "To Connection already exists, From : " << _connection.incoming_road_ << ", To: " << _connection.outgoing_road_ << std::endl;
	}
}

void OpenDriveRoad::insertUniqueFromConnection(const Connection& _connection)
{
	bool bFound = false;
	for(unsigned int j=0; j < from_roads_.size(); j++)
	{
		if(from_roads_.at(j).incoming_road_== _connection.incoming_road_ && from_roads_.at(j).outgoing_road_ == _connection.outgoing_road_)
		{
			for(unsigned int k=0; k < from_roads_.at(j).lane_links.size(); k++)
			{
				for(unsigned int lk=0; lk < _connection.lane_links.size(); lk++)
				{
					if(from_roads_.at(j).lane_links.at(k).first == _connection.lane_links.at(lk).first && from_roads_.at(j).lane_links.at(k).second == _connection.lane_links.at(lk).second)
					{
						bFound = true;
						break;
					}
				}

				if(bFound == true)
					break;
			}

			if(bFound == true)
				break;
		}
	}

	if(!bFound)
	{
		from_roads_.push_back(_connection);
	}
	else
	{
		//std::cout << "From Connection already exists, From : " << _connection.incoming_road_ << ", To: " << _connection.outgoing_road_ << std::endl;
	}
}

void OpenDriveRoad::createRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list)
{
	using namespace std::placeholders;
	std::function<void(int, int, PlannerHNS::Lane&)> insertToRoad;
	std::function<void(int, int, PlannerHNS::Lane&)> insertFromRoad;
	std::function<void(int, const OpenDriveLane*, PlannerHNS::Lane&)> insertToSection;
	std::function<void(int, const OpenDriveLane*, PlannerHNS::Lane&)> insertFromSection;

	//flip to and from depending on keep right or keep left rule
	if(keep_right_)
	{
		insertToRoad = std::bind(&OpenDriveRoad::insertUniqueToRoadIds, this, _1, _2, _3);
		insertFromRoad = std::bind(&OpenDriveRoad::insertUniqueFromRoadIds, this, _1, _2, _3);
		insertToSection = std::bind(&OpenDriveRoad::insertUniqueToSectionIds, this, _1, _2, _3);
		insertFromSection = std::bind(&OpenDriveRoad::insertUniqueFromSectionIds, this, _1, _2, _3);
	}
	else
	{
		insertToRoad = std::bind(&OpenDriveRoad::insertUniqueFromRoadIds, this, _1, _2, _3);
		insertFromRoad = std::bind(&OpenDriveRoad::insertUniqueToRoadIds, this, _1, _2, _3);
		insertToSection = std::bind(&OpenDriveRoad::insertUniqueFromSectionIds, this, _1, _2, _3);
		insertFromSection = std::bind(&OpenDriveRoad::insertUniqueToSectionIds, this, _1, _2, _3);		
	}
	
	for(unsigned int i=0; i < sections_.size(); i++)
	{
		RoadSection* p_sec = & sections_.at(i);

		for(unsigned int lj=0; lj < p_sec->left_lanes_.size(); lj++)
		{
			PlannerHNS::Lane op_lane;
			OpenDriveLane* p_l_l = &p_sec->left_lanes_.at(lj);

//			if(p_l_l->type_ != DRIVING_LANE)
//				continue;

			op_lane.id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_l_l->id_ * 100;
			op_lane.num = p_l_l->id_;
			op_lane.roadId = id_;


			if(i == 0){
				insertToRoad(p_sec->id_, p_l_l->id_, op_lane);
			}else{
				insertToSection(p_sec->id_-1, p_l_l, op_lane);
			}
			if( i == sections_.size()-1){
				insertFromRoad(p_sec->id_, p_l_l->id_, op_lane);
			}else{
				insertFromSection(p_sec->id_+1, p_l_l, op_lane);
			}
			lanes_list.push_back(op_lane);
		}

		for(unsigned int rj=0; rj < p_sec->right_lanes_.size(); rj++)
		{
			PlannerHNS::Lane op_lane;
			OpenDriveLane* p_r_l = &p_sec->right_lanes_.at(rj);

			op_lane.id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_r_l->id_ * 100;
			op_lane.num = p_r_l->id_;
			op_lane.roadId = id_;

			if(i == 0){
				insertFromRoad(p_sec->id_, p_r_l->id_, op_lane);
			}else{
				insertFromSection(p_sec->id_-1, p_r_l, op_lane);
			}
			if( i == sections_.size()-1){					
				insertToRoad(p_sec->id_, p_r_l->id_, op_lane);
			}else{
				insertToSection(p_sec->id_+1, p_r_l, op_lane);
			}
			
			lanes_list.push_back(op_lane);
		}
	}
}

void OpenDriveRoad::fixRedundantPointsLanes(PlannerHNS::Lane& _lane)
{
	for(int ip = 1; ip < _lane.points.size(); ip++)
	{
		PlannerHNS::WayPoint* p1 = &_lane.points.at(ip-1);
		PlannerHNS::WayPoint* p2 = &_lane.points.at(ip);
		PlannerHNS::WayPoint* p3 = nullptr;
		if(ip+1 < _lane.points.size())
			p3 = &_lane.points.at(ip+1);

		double d = hypot(p2->pos.y-p1->pos.y, p2->pos.x-p1->pos.x);
		if(d < g_epsilon)
		{
			p1->toIds = p2->toIds;
			p1->originalMapID = p2->originalMapID;
			if(p3 != nullptr)
				p3->fromIds = p2->fromIds;

			_lane.points.erase(_lane.points.begin()+ip);
			ip--;

//			std::cout << "Fixed Redundant Points for Lane:" << _lane.id << ", Current: " << ip << ", Size: " << _lane.points.size() << std::endl;
		}
	}
}

void OpenDriveRoad::createSectionPoints(const RoadCenterInfo& ref_info, std::vector<PlannerHNS::Lane>& lanes_list, RoadSection* p_sec, int& wp_id_seq, std::vector<int> &left_lane_ids,std::vector<int> &right_lane_ids )
{
	if(p_sec == nullptr) return;

	double accum_offset_width = ref_info.offset_width_;

	for(unsigned int lj=0; lj < p_sec->left_lanes_.size(); lj++)
	{
		OpenDriveLane* p_l_l = &p_sec->left_lanes_.at(lj);

		int combined_lane_id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_l_l->id_ * 100;
		PlannerHNS::Lane* p_op_lane = getLaneById(combined_lane_id, lanes_list);


		if(p_op_lane != nullptr)
		{
			double lane_width = p_l_l->getLaneWidth(ref_info.ds_ - p_sec->s_);
			double center_point_margin = accum_offset_width + (lane_width / 2.0);

			PlannerHNS::WayPoint p;
			p.pos = ref_info.center_p_;
			double a = p.pos.a+M_PI_2;
			p.pos.x += center_point_margin * cos(a);
			p.pos.y += center_point_margin * sin(a);
			p.collisionCost = lane_width;
			p.id = wp_id_seq++;
			//TODO apply super elevation later

			p_op_lane->width = lane_width;
			p_op_lane->points.push_back(p);

			accum_offset_width += lane_width;
			
			if(!exists(left_lane_ids, combined_lane_id))
			{
				left_lane_ids.push_back(combined_lane_id);
			}
		}
		else
		{
			std::cout << " >>>>> Can't Find Left Lane:  " << combined_lane_id << std::endl;
		}
	}

	accum_offset_width = ref_info.offset_width_;

	for(unsigned int rj=0; rj < p_sec->right_lanes_.size(); rj++)
	{
		OpenDriveLane* p_r_l = &p_sec->right_lanes_.at(rj);
		int combined_lane_id = (this->id_*100000 + 50000) + p_sec->id_*1000 + p_r_l->id_ * 100;
		PlannerHNS::Lane* p_op_lane = getLaneById(combined_lane_id, lanes_list);

		if(p_op_lane != nullptr)
		{
			double lane_width = p_r_l->getLaneWidth(ref_info.ds_ - p_sec->s_);
			double center_point_margin = accum_offset_width + (lane_width / 2.0);

			PlannerHNS::WayPoint p;
			p.pos = ref_info.center_p_;
			double a = p.pos.a-M_PI_2;
			p.pos.x += center_point_margin * cos(a);
			p.pos.y += center_point_margin * sin(a);
			p.collisionCost = lane_width;
			p.id = wp_id_seq++;
			//TODO apply super elevation later

			p_op_lane->width = lane_width;
			p_op_lane->points.push_back(p);

			accum_offset_width += lane_width;
			
			if(!exists(right_lane_ids, combined_lane_id))
			{
				right_lane_ids.push_back(combined_lane_id);
			}
		}
		else
		{
			std::cout << " >>>>> Can't Find Right Lane:  " << combined_lane_id << std::endl;
		}
	}
}

void OpenDriveRoad::getRoadLanes(std::vector<PlannerHNS::Lane>& lanes_list, double resolution)
{
	std::vector<RoadCenterInfo> ref_info;
	createRoadCenterInfo(ref_info, resolution);
	createRoadLanes(lanes_list);
	static int wp_id_seq = 1;
	std::vector<int> left_lane_ids, right_lane_ids;

	for(unsigned int i=0; i < ref_info.size(); i++)
	{
		RoadSection* p_sec = getExactMatchingSection(ref_info.at(i).ds_);
		if(p_sec != nullptr)
		{
			createSectionPoints(ref_info.at(i), lanes_list, p_sec, wp_id_seq, left_lane_ids, right_lane_ids);
//			std::cout << " >>>>> Found Exact Section:  " << ref_info.at(i).ds_ << std::endl;
		}

		p_sec = getMatchingSection(ref_info.at(i).ds_);
		if(p_sec != nullptr)
		{
			createSectionPoints(ref_info.at(i), lanes_list, p_sec, wp_id_seq, left_lane_ids, right_lane_ids);
		}
		else
		{
			std::cout << " >>>>> Can't Find Section:  " << ref_info.at(i).ds_ << std::endl;
		}
	}
	
	if(keep_right_)
	{
			for( auto id: left_lane_ids )
			{
					PlannerHNS::Lane* p_op_lane = getLaneById(id, lanes_list);
					if(p_op_lane != nullptr)
					{
						std::reverse(	p_op_lane->points.begin(), p_op_lane->points.end() );
					}
			}
	}
	else
	{
		for( auto id: right_lane_ids )
		{
				PlannerHNS::Lane* p_op_lane = getLaneById(id, lanes_list);
				if(p_op_lane != nullptr)
				{
					std::reverse(	p_op_lane->points.begin(), p_op_lane->points.end() );
				}
			}
	}

	for(unsigned int i=0; i < lanes_list.size(); i++)
	{
		fixRedundantPointsLanes(lanes_list.at(i));
	}
}

std::vector<Connection> OpenDriveRoad::getLastSectionConnections(OpenDriveRoad *_p_successor_road)
{	
		std::vector<Connection> connections_list;
		Connection conn;

		if( _p_successor_road == nullptr)
		{
			return connections_list;
		}
		
		//for right lanes
		RoadSection* p_l_sec = getLastSection();
		if(p_l_sec != nullptr)
		{
			conn.incoming_road_ = id_;
			conn.incoming_section_ = p_l_sec->id_;
			conn.outgoing_road_ = _p_successor_road->id_; 
			conn.outgoing_section_ = 0;

			for(unsigned int i =0 ; i < p_l_sec->right_lanes_.size(); i++)
			{
				if(p_l_sec->right_lanes_.at(i).to_lane_.size() > 0)
				{
					int _to_id = p_l_sec->right_lanes_.at(i).to_lane_.at(0).to_lane_id;
					conn.lane_links.push_back(std::make_pair(p_l_sec->right_lanes_.at(i).id_, _to_id));
				}
				
				if(conn.lane_links.size() > 0)
				{
					connections_list.push_back(conn);
					conn.lane_links.clear();
				}
			}
		}
		
		//for left lanes
		if(p_l_sec != nullptr )
		{
			conn.outgoing_road_ = id_;
			conn.outgoing_section_ = p_l_sec->id_;
			conn.incoming_road_ = _p_successor_road->id_; 
			conn.incoming_section_ = 0;

			for(unsigned int i =0 ; i < p_l_sec->left_lanes_.size(); i++)
			{
				if(p_l_sec->left_lanes_.at(i).from_lane_.size() > 0)
				{
					int _from_id = p_l_sec->left_lanes_.at(i).from_lane_.at(0).from_lane_id;
					conn.lane_links.push_back(std::make_pair( _from_id, p_l_sec->left_lanes_.at(i).id_));
				}
				
				if(conn.lane_links.size() > 0)
				{
					connections_list.push_back(conn);
					conn.lane_links.clear();
				}
			}
		}
		
		return connections_list;
}


std::vector<Connection> OpenDriveRoad::getFirstSectionConnections( OpenDriveRoad *_p_predecessor_road )
{
	std::vector<Connection> connections_list;
	Connection conn;

	if(_p_predecessor_road == nullptr)
	{
		return connections_list;
	}
	
	//for right lanes
	RoadSection* p_f_sec =  getFirstSection();
	RoadSection* p_prev_sec = _p_predecessor_road->getLastSection(); 
	if(p_prev_sec != nullptr &&  p_f_sec != nullptr)
	{
		conn.outgoing_road_ = id_;
		conn.outgoing_section_ = p_f_sec->id_;
		conn.incoming_road_ = _p_predecessor_road->id_; 
		conn.incoming_section_ = p_prev_sec->id_;

		for(unsigned int i =0 ; i < p_f_sec->right_lanes_.size(); i++)
		{
			if(p_f_sec->right_lanes_.at(i).from_lane_.size() > 0)
			{
				int _from_id = p_f_sec->right_lanes_.at(i).from_lane_.at(0).from_lane_id;
				conn.lane_links.push_back(std::make_pair(_from_id, p_f_sec->right_lanes_.at(i).id_));
			}
			
			if(conn.lane_links.size() > 0)
			{
				connections_list.push_back(conn);
				conn.lane_links.clear();
			}
		}		
	}

	 
	//left lanes 
	if(p_prev_sec != nullptr && p_f_sec != nullptr)
	{
		conn.incoming_road_ = id_;
		conn.incoming_section_ = p_f_sec->id_;
		conn.outgoing_road_ = _p_predecessor_road->id_; 
		conn.outgoing_section_ = p_prev_sec->id_;

		for(unsigned int i =0 ; i < p_f_sec->left_lanes_.size(); i++)
		{
			if(p_f_sec->left_lanes_.at(i).to_lane_.size() > 0)
			{
				int _to_id = p_f_sec->left_lanes_.at(i).to_lane_.at(0).to_lane_id;
				conn.lane_links.push_back(std::make_pair(p_f_sec->left_lanes_.at(i).id_, _to_id));
			}

			if(conn.lane_links.size() > 0)
			{
				connections_list.push_back(conn);
				conn.lane_links.clear();
			}
		}
	}
	
	return connections_list;
}

OBJECT_TYPE OpenDriveRoad::getObjTypeFromText(const std::string& autoware_type)
{
	if(autoware_type.compare("TRAFFIC_LIGHT"))
	{
		return TRAFFIC_LIGHT;
	}
	else if(autoware_type.compare("ROAD_SIGN"))
	{
		return ROAD_SIGN;
	}
	else if(autoware_type.compare("ROAD_MARK"))
	{
		return ROAD_MARK;
	}
	else
	{
		return UNKNOWN_OBJECT;
	}
}

OBJECT_TYPE OpenDriveRoad::getAutowareMainTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type)
{
	if(p_country_signal_codes_ != nullptr)
	{
		for(unsigned int i=0; i < p_country_signal_codes_->size(); i++)
		{
			if(p_country_signal_codes_->at(i).first.compare(country_code) == 0)
			{
				for(unsigned int j=0; j < p_country_signal_codes_->at(i).second.size(); j++)
				{
					if(p_country_signal_codes_->at(i).second.at(j).id_.compare(type) == 0 &&
							p_country_signal_codes_->at(i).second.at(j).sub_id_.compare(sub_type) == 0)
					{
						return getObjTypeFromText(p_country_signal_codes_->at(i).second.at(j).gen_type_);
					}
				}
			}
		}
	}

	return UNKNOWN_OBJECT;
}

TRAFFIC_LIGHT_TYPE OpenDriveRoad::getLightTypeFromText(const std::string& autoware_type)
{
	if(autoware_type.compare("VERTICAL_DEFAULT_LIGHT"))
	{
		return VERTICAL_DEFAULT_LIGHT;
	}
	else if(autoware_type.compare("HORIZONTAL_DEFAULTLIGHT"))
	{
		return HORIZONTAL_DEFAULTLIGHT;
	}
	else if(autoware_type.compare("PEDESTRIAN_DEFAULT_LIGHT"))
	{
		return PEDESTRIAN_DEFAULT_LIGHT;
	}
	else
	{
		return UNKNOWN_LIGHT;
	}
}

TRAFFIC_LIGHT_TYPE OpenDriveRoad::getAutowareLightTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type)
{
	if(p_country_signal_codes_ != nullptr)
	{
		for(unsigned int i=0; i < p_country_signal_codes_->size(); i++)
		{
			if(p_country_signal_codes_->at(i).first.compare(country_code) == 0)
			{
				for(unsigned int j=0; j < p_country_signal_codes_->at(i).second.size(); j++)
				{
					if(p_country_signal_codes_->at(i).second.at(j).id_.compare(type) == 0 &&
							p_country_signal_codes_->at(i).second.at(j).sub_id_.compare(sub_type) == 0)
					{
						return getLightTypeFromText(p_country_signal_codes_->at(i).second.at(j).gen_sub_type_);
					}
				}
			}
		}
	}

	return UNKNOWN_LIGHT;
}

ROAD_SIGN_TYPE OpenDriveRoad::getSignTypeFromText(const std::string& autoware_type)
{
	if(autoware_type.compare("SPEED_LIMIT_SIGN"))
	{
		return SPEED_LIMIT_SIGN;
	}
	else if(autoware_type.compare("STOP_SIGN"))
	{
		return STOP_SIGN;
	}
	else if(autoware_type.compare("NO_PARKING_SIGN"))
	{
		return NO_PARKING_SIGN;
	}
	else
	{
		return UNKNOWN_SIGN;
	}
}

ROAD_SIGN_TYPE OpenDriveRoad::getAutowareRoadSignTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type)
{
	if(p_country_signal_codes_ != nullptr)
	{
		for(unsigned int i=0; i < p_country_signal_codes_->size(); i++)
		{
			if(p_country_signal_codes_->at(i).first.compare(country_code) == 0)
			{
				for(unsigned int j=0; j < p_country_signal_codes_->at(i).second.size(); j++)
				{
					if(p_country_signal_codes_->at(i).second.at(j).id_.compare(type) == 0 &&
							p_country_signal_codes_->at(i).second.at(j).sub_id_.compare(sub_type) == 0)
					{
						return getSignTypeFromText(p_country_signal_codes_->at(i).second.at(j).gen_sub_type_);
					}
				}
			}
		}
	}

	return UNKNOWN_SIGN;

}

ROAD_MARK_TYPE OpenDriveRoad::getMarkTypeFromText(const std::string& autoware_type)
{
	if(autoware_type.compare("STOP_LINE_MARK"))
	{
		return STOP_LINE_MARK;
	}
	else if(autoware_type.compare("WAITING_LINE_MARK"))
	{
		return WAITING_LINE_MARK;
	}
	else if(autoware_type.compare("FORWARD_DIRECTION_MARK"))
	{
		return FORWARD_DIRECTION_MARK;
	}
	else if(autoware_type.compare("LEFT_DIRECTION_MARK"))
	{
		return LEFT_DIRECTION_MARK;
	}
	else if(autoware_type.compare("RIGHT_DIRECTION_MARK"))
	{
		return RIGHT_DIRECTION_MARK;
	}
	else if(autoware_type.compare("FORWARD_LEFT_DIRECTION_MARK"))
	{
		return FORWARD_LEFT_DIRECTION_MARK;
	}
	else if(autoware_type.compare("FORWARD_RIGHT_DIRECTION_MARK"))
	{
		return FORWARD_RIGHT_DIRECTION_MARK;
	}
	else if(autoware_type.compare("ALL_DIRECTION_MARK"))
	{
		return ALL_DIRECTION_MARK;
	}
	else if(autoware_type.compare("U_TURN_DIRECTION_MARK"))
	{
		return U_TURN_DIRECTION_MARK;
	}
	else if(autoware_type.compare("NO_U_TURN_DIRECTION_MARK"))
	{
		return NO_U_TURN_DIRECTION_MARK;
	}
	else
	{
		return UNKNOWN_ROAD_MARK;
	}
}

ROAD_MARK_TYPE OpenDriveRoad::getAutowareRoadMarksTypeFromCode(const std::string& country_code, const std::string& type, const std::string& sub_type)
{
	if(p_country_signal_codes_ != nullptr)
	{
		for(unsigned int i=0; i < p_country_signal_codes_->size(); i++)
		{
			if(p_country_signal_codes_->at(i).first.compare(country_code) == 0)
			{
				for(unsigned int j=0; j < p_country_signal_codes_->at(i).second.size(); j++)
				{
					if(p_country_signal_codes_->at(i).second.at(j).id_.compare(type) == 0 &&
							p_country_signal_codes_->at(i).second.at(j).sub_id_.compare(sub_type) == 0)
					{
						return getMarkTypeFromText(p_country_signal_codes_->at(i).second.at(j).gen_sub_type_);
					}
				}
			}
		}
	}

	return UNKNOWN_ROAD_MARK;
}

void OpenDriveRoad::getTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights)
{
	for(unsigned int i=0; i<road_signals_.size(); i++)
	{
		if(getAutowareMainTypeFromCode(road_signals_.at(i).country_code_, road_signals_.at(i).type_, road_signals_.at(i).sub_type_) == TRAFFIC_LIGHT)
		{
			PlannerHNS::TrafficLight tl;
			tl.id = (this->id_*1000000) + road_signals_.at(i).id_ * 10;
			PlannerHNS::WayPoint p;
			if(createSingleCenterPoint(road_signals_.at(i).s_, p))
			{
				double a = p.pos.a+M_PI_2;
				p.pos.x += road_signals_.at(i).t_ * cos(a);
				p.pos.y += road_signals_.at(i).t_ * sin(a);
				tl.pos = p.pos;
				tl.laneIds = road_signals_.at(i).valid_lanes_ids_;
				all_lights.push_back(tl);
			}
		}
	}
}

void OpenDriveRoad::getTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs)
{
//	for(unsigned int i=0; i<road_signals_.size(); i++)
//	{
//		PlannerHNS::TrafficLight tl;
//		tl.id = (this->id_*1000000) + road_signals_.at(i).id_ * 10;
//		PlannerHNS::WayPoint p;
//		if(CreateSingleCenterPoint(road_signals_.at(i).s_, p))
//		{
//			double a = p.pos.a+M_PI_2;
//			p.pos.x += road_signals_.at(i).t_ * cos(a);
//			p.pos.y += road_signals_.at(i).t_ * sin(a);
//			tl.pos = p.pos;
//			tl.laneIds = road_signals_.at(i).valid_lanes_ids_;
//			all_lights.push_back(tl);
//		}
//	}
}

void OpenDriveRoad::getStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines)
{
	for(unsigned int i=0; i<road_signals_.size(); i++)
	{
		if(getAutowareMainTypeFromCode(road_signals_.at(i).country_code_, road_signals_.at(i).type_, road_signals_.at(i).sub_type_) ==  ROAD_MARK)
		{
			PlannerHNS::StopLine sl;
			sl.id = (this->id_*1000000) + road_signals_.at(i).id_ * 10;
			PlannerHNS::WayPoint p, p1, p2;
			if(createSingleCenterPoint(road_signals_.at(i).s_, p))
			{
				//assume fixed 2 meter stop line length
				p1 = p;
				double a = p.pos.a+M_PI_2;
				p1.pos.x += (road_signals_.at(i).t_ - 1.0 ) * cos(a);
				p1.pos.y += (road_signals_.at(i).t_ - 1.0 ) * sin(a);

				p2 = p;
				p2.pos.x += (road_signals_.at(i).t_ + 1.0 ) * cos(a);
				p2.pos.y += (road_signals_.at(i).t_ + 1.0 ) * sin(a);
				sl.points.push_back(p1.pos);
				sl.points.push_back(p2.pos);

				if(road_signals_.at(i).valid_lanes_ids_.size() > 0)

				for(unsigned int j=0; j<road_signals_.at(i).valid_lanes_ids_.size(); j++)
				{
					sl.laneId = road_signals_.at(i).valid_lanes_ids_.at(j);
					all_stop_lines.push_back(sl);
					sl.id += 1;
				}
			}
		}
	}
}
}
