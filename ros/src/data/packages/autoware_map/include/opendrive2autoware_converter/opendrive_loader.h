/*
 *  Copyright (c) 2019, Nagoya University
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

#ifndef OPENDRIVE2AUTOWARE_CONVERTER
#define OPENDRIVE2AUTOWARE_CONVERTER

#include "opendrive_road.h"
#include "dirent.h"

namespace autoware_map
{
class OpenDriveLoader
{

public:
	OpenDriveLoader();
    ~OpenDriveLoader();
    void getFileNameInFolder(const std::string& path, std::vector<std::string>& out_list);
    void loadCountryCods(const std::string& codes_csv_folder);
    void loadOpenDRIVE(const std::string& xodr_file, const std::string& codes_folder, PlannerHNS::RoadNetwork& map,double resolution = 0.5);
    void getMapLanes(std::vector<PlannerHNS::Lane>& all_lanes, double resolution = 0.5);
    void getTrafficLights(std::vector<PlannerHNS::TrafficLight>& all_lights);
    void getTrafficSigns(std::vector<PlannerHNS::TrafficSign>& all_signs);
    void getStopLines(std::vector<PlannerHNS::StopLine>& all_stop_lines);
    void connectRoads();

private:
		bool keep_right_;
    std::vector<OpenDriveRoad> roads_list_;
    std::vector<Junction> junctions_list_;
    std::vector<std::pair<std::string, std::vector<CSV_Reader::LINE_DATA> > > country_signal_codes_;


    std::vector<OpenDriveRoad*> getRoadsBySuccId(int _id);
    std::vector<OpenDriveRoad*> getRoadsByPredId(int _id);

    OpenDriveRoad* getRoadById(int _id);
    Junction* getJunctionById(int _id);
    void linkWayPoints(PlannerHNS::RoadNetwork& map);
    void linkLanesPointers(PlannerHNS::RoadNetwork& map);
};

}

#endif // OPENDRIVE2AUTOWARE_CONVERTER
