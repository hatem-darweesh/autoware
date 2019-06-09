
#include <ros/ros.h>
#include "op_planner/MappingHelpers.h"


void FixDtLaneProblem(const std::string vectoMapPath)
{
	std::string laneLinesDetails = vectoMapPath + "point.csv";
	std::string center_lines_info = vectoMapPath + "dtlane.csv";
	std::string lane_info = vectoMapPath + "lane.csv";
	std::string node_info = vectoMapPath + "node.csv";

	std::cout << " >> Loading vector map data files ... " << std::endl;
	UtilityHNS::AisanCenterLinesFileReader  center_lanes(center_lines_info);
	UtilityHNS::AisanLanesFileReader lanes(lane_info);
	UtilityHNS::AisanPointsFileReader points(laneLinesDetails);
	UtilityHNS::AisanNodesFileReader nodes(node_info);

	std::vector<UtilityHNS::AisanNodesFileReader::AisanNode> nodes_data;
	std::vector<UtilityHNS::AisanLanesFileReader::AisanLane> lanes_data;
	std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints> points_data;
	std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine> dt_data;

	nodes.ReadAllData(nodes_data);
	lanes.ReadAllData(lanes_data);
	points.ReadAllData(points_data);
	center_lanes.ReadAllData(dt_data);

	if(points_data.size() == 0)
	{
		std::cout << std::endl << "## Alert Can't Read Points Data from vector map files in path: " << vectoMapPath << std::endl;
		return;
	}

	std::vector<std::string> dt_data_str;
	std::vector<std::string> lane_data_str;

	std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine> n_dt_data;

	PlannerHNS::RoadNetwork map;
	std::cout << " >> Extract dt_lane from Lane, Node and Points data ... " << std::endl;
	PlannerHNS::MappingHelpers::GenerateDtLaneAndFixLaneForVectorMap(&lanes, &points, &nodes, map, n_dt_data);

	std::cout << " >> Write dt_lane data to file dt_lane_fix.csv ... " << std::endl;
	for(auto x : n_dt_data)
	{
		std::ostringstream str;
		str << x;
		dt_data_str.push_back(str.str());
	}
	UtilityHNS::DataRW::writeCSVFile(vectoMapPath, "dtlane_fix", center_lanes.header_, dt_data_str);

	std::cout << " >> Write updated lane data to file lane_fix.csv ... " << std::endl;
	for(auto y : lanes.m_data_list)
	{
		std::ostringstream str;
		str << y;
		lane_data_str.push_back(str.str());
	}
	UtilityHNS::DataRW::writeCSVFile(vectoMapPath, "lane_fix", lanes.header_, lane_data_str);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_dt_lane");



	std::string src_path;

	if(argc < 2)
	{
		std::cout << " add_dt_lane is a rosnode that read vector map files .csv and fix the dt_lane problem )." << std::endl << std::endl;
		std::cout << "Commands: " << std::endl;
		std::cout << " add_dt_lane source_map_path " <<std::endl <<std::endl;
		return 0;
	}
	else
	{
		src_path = std::string(argv[1]);
		FixDtLaneProblem(src_path);
	}

    //ros::spin();
}
