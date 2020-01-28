
#include <ros/ros.h>
#include <opendrive2autoware_converter/opendrive_loader.h>
#include <opendrive2autoware_converter/map_writer.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "opendrive2autoware_converter");



	std::string src_path;
	std::string dst_path;
	std::string country_codes_path;
	bool keep_right;

	double wp_res = 0.5;

	if(argc < 3)
	{
		std::cout << "opendrive2autoware_converter is a rosnode that convert OpenDRIVE map format (.xodr) to autoware map format (list of .csv files)." << std::endl << std::endl;
		std::cout << "Commands: " << std::endl;
		std::cout << "        opendrive2autoware_converter Source OpenDRIVE .xodr file name,  Countries' codes folder name, Destination folder for Autoware map files, (optional)Waypoints resolution default is 0.5 meters" <<std::endl <<std::endl;
		return 0;
	}
	else
	{
		std::cout << argc << std::endl;

		src_path = std::string(argv[1]);
		country_codes_path = std::string(argv[2]);
		dst_path = std::string(argv[3]);
		if(argc > 4)
			wp_res = atof(argv[4]);

		ros::NodeHandle pnh("~");
		pnh.param("keep_right", keep_right, true);

		autoware_map::OpenDriveLoader map_loader(keep_right);
		PlannerHNS::RoadNetwork map;
		map_loader.loadOpenDRIVE(src_path, country_codes_path, map, wp_res);
		autoware_map::MapWriter map_save;
		map_save.writeAutowareMap(dst_path, map);
		//Write autoware maps files
	}

  //  ros::spin();
}
