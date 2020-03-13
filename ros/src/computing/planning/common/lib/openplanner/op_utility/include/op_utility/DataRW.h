
/// \file DataRW.h
/// \brief File operations for loading vector map files, loading kml map files and writing log .csv files
/// \author Hatem Darweesh
/// \date Jun 23, 2016


#ifndef DATARW_H_
#define DATARW_H_

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <ostream>
#include <limits>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/WayAreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/VectorArray.h"
#include "vector_map_msgs/CrossRoadArray.h"
#include "vector_map_msgs/RoadSignArray.h"
#include "vector_map_msgs/CurbArray.h"
#include "vector_map_msgs/RoadEdgeArray.h"
#include "vector_map_msgs/CrossWalkArray.h"
#include "vector_map_msgs/WhiteLineArray.h"
#include "vector_map_msgs/GutterArray.h"
#include "vector_map_msgs/PoleArray.h"
#include "vector_map_msgs/StreetLightArray.h"
#include "vector_map_msgs/UtilityPoleArray.h"
#include "vector_map_msgs/ZebraZoneArray.h"


#include "UtilityH.h"

namespace UtilityHNS {

class DataRW
{
public:
	DataRW();
	virtual ~DataRW();

	static std::string LoggingMainfolderName;
	static std::string ControlLogFolderName;
	static std::string PathLogFolderName;
	static std::string GlobalPathLogFolderName;
	static std::string StatesLogFolderName;
	static std::string SimulationFolderName;
	static std::string KmlMapsFolderName;
	static std::string PredictionFolderName;
	static std::string TrackingFolderName;
	static std::string ExperimentsFolderName;

	static void WriteKMLFile(const std::string& fileName, const std::vector<std::string>& gps_list);
	static void WriteKMLFile(const std::string& fileName, const std::vector<std::vector<std::string> >& gps_list);
	static void WriteLogData(const std::string& logFolder, const std::string& logTitle, const std::string& header, const std::vector<std::string>& logData);
	static void CreateLoggingMainFolder();
	static void CreateLoggingFolders(const std::string& mainFolderName);
	static void CreateExperimentFolder(const std::string& folderName);

	static void writeCSVFile(const std::string& folder, const std::string& title,
			const std::string& header,
			const std::vector<std::string>& data_list);
};

class SimpleReaderBase
{
private:
	std::ifstream* m_pFile;
	std::vector<std::string> m_RawHeaders;
	std::vector<std::string> m_DataTitlesHeader;
	std::vector<std::vector<std::vector<std::string> > > m_AllData;
	int m_nHeders;
	int m_iDataTitles;
	int m_nVarPerObj;
	int m_nLineHeaders;
	std::string m_HeaderRepeatKey;
	char m_Separator;

	void ReadHeaders();
	void ParseDataTitles(const std::string& header);

public:
	/**
	 *
	 * @param fileName log file name
	 * @param nHeaders number of data headers
	 * @param iDataTitles which row contains the data titles
	 * @param nVariablesForOneObject 0 means each row represents one object
	 */
	SimpleReaderBase(const std::string& path, const int& nHeaders = 2, const std::string& csv_file_name = "", const char& separator = ',',
			const int& iDataTitles = 1, const int& nVariablesForOneObject = 0,
			const int& nLineHeaders = 0, const std::string& headerRepeatKey = "...");

	virtual ~SimpleReaderBase();
	virtual int ReadAllData() = 0;
	std::string header_;
	std::string file_name_;

protected:

	bool ReadSingleLine(std::vector<std::vector<std::string> >& line);

};

class GPSDataReader : public SimpleReaderBase
{
public:
	struct GPSBasicData
	{
		double lat;
		double lon;
		double alt;
		double dir;
		double distance;

	};

	public:
	GPSDataReader(const std::string& fileName) : SimpleReaderBase(fileName){}
	virtual ~GPSDataReader(){}

	bool ReadNextLine(GPSBasicData& data);
	int ReadAllData(std::vector<GPSBasicData>& data_list);
	int ReadAllData();
};

class SimulationFileReader : public SimpleReaderBase
{
public:
	struct SimulationPoint
	{
		double x;
		double y;
		double z;
		double a;
		double c;
		double v;
		std::string name;
	};

	struct SimulationData
	{
		SimulationPoint startPoint;
		SimulationPoint goalPoint;
		std::vector<SimulationPoint> simuCars;
	};

	SimulationFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1){}
	virtual ~SimulationFileReader(){}

	bool ReadNextLine(SimulationPoint& data);
	int ReadAllData(SimulationData& data_list);
	int ReadAllData();
};

class LocalizationPathReader : public SimpleReaderBase
{
public:
	struct LocalizationWayPoint
	{
		double t;
		double x;
		double y;
		double z;
		double a;
		double v;
	};

	LocalizationPathReader(const std::string& fileName, const char& separator) : SimpleReaderBase(fileName, 1, "", separator){}
	virtual ~LocalizationPathReader(){}

	bool ReadNextLine(LocalizationWayPoint& data);
	int ReadAllData(std::vector<LocalizationWayPoint>& data_list);
	int ReadAllData();
};

class AisanPointsFileReader : public SimpleReaderBase
{
public:
	struct AisanPoints
	{
		int PID;
		double B;
		double L;
		double H;
		double Bx;
		double Ly;
		int Ref;
		int MCODE1;
		int MCODE2;
		int MCODE3;
	};

	AisanPointsFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "point.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "PID,B,L,H,Bx,Ly,Ref,MCODE1,MCODE2,MCODE3";
	}

	AisanPointsFileReader(const vector_map_msgs::PointArray& _points);
	virtual ~AisanPointsFileReader(){}

	bool ReadNextLine(AisanPoints& data);
	int ReadAllData(std::vector<AisanPoints>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Point& _rec, AisanPoints& data);
	AisanPoints* GetDataRowById(int _pid);
	std::vector<AisanPoints> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanPoints& obj)
	{
	    os << obj.PID << ","
	    << obj.B << ","
	    << obj.L << ","
	    << obj.H << ","
	    << obj.Bx << ","
	    << obj.Ly << ","
	    << obj.Ref << ","
	    << obj.MCODE1 << ","
	    << obj.MCODE2 << ","
	    << obj.MCODE3;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanPoints*> m_data_map;
};

class AisanNodesFileReader : public SimpleReaderBase
{
public:

	struct AisanNode
	{
		int NID;
		int PID;
	};

	AisanNodesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "node.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "NID,PID";
	}

	AisanNodesFileReader(const vector_map_msgs::NodeArray& _nodes);
	virtual ~AisanNodesFileReader(){}

	bool ReadNextLine(AisanNode& data);
	int ReadAllData(std::vector<AisanNode>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Node& _rec, AisanNode& data);
	AisanNode* GetDataRowById(int _nid);
	std::vector<AisanNode> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanNode& obj)
	{
	    os << obj.NID << ","
	    << obj.PID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanNode*> m_data_map;
};

class AisanLinesFileReader : public SimpleReaderBase
{
public:

	struct AisanLine
	{
		int LID;
		int BPID;
		int FPID;
		int BLID;
		int FLID;
	};

	AisanLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "line.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "LID,BPID,FPID,BLID,FLID";
	}
	AisanLinesFileReader(const vector_map_msgs::LineArray & _lines);
	virtual ~AisanLinesFileReader(){}

	bool ReadNextLine(AisanLine& data);
	int ReadAllData(std::vector<AisanLine>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Line& _rec, AisanLine& data);
	AisanLine* GetDataRowById(int _lid);
	std::vector<AisanLine> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanLine& obj)
	{
	    os << obj.LID << ","
	    << obj.BPID << ","
	    << obj.FPID << ","
	    << obj.BLID << ","
	    << obj.FLID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanLine*> m_data_map;
};

class AisanCLinesFileReader : public SimpleReaderBase
{
public:

	struct AisanCLine
	{
		int ID;
		int LID;
		double width;
		char color;
		int type;
		int LinkID;
	};

	AisanCLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "cline.csv")
	{
		header_ = "ID,LID,width,color,type,LinkID";
	}
	virtual ~AisanCLinesFileReader(){}

	bool ReadNextLine(AisanCLine& data);
	int ReadAllData(std::vector<AisanCLine>& data_list);
	int ReadAllData();

	friend std::ostream& operator<<(std::ostream& os, const AisanCLine& obj)
	{
	    os << obj.ID << ","
	    << obj.LID << ","
	    << obj.width << ","
	    << obj.color << ","
	    << obj.type << ","
	    << obj.LinkID;
	    return os;
	}
};

class AisanCenterLinesFileReader : public SimpleReaderBase
{
public:

	struct AisanCenterLine
	{
		int 	DID;
		int 	Dist;
		int 	PID;
		double 	Dir;
		double 	Apara;
		double 	r;
		double 	slope;
		double 	cant;
		double 	LW;
		double 	RW;
	};

	AisanCenterLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "dtlane.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW";
	}
	AisanCenterLinesFileReader(const vector_map_msgs::DTLaneArray& _dtLanes);
	virtual ~AisanCenterLinesFileReader(){}

	bool ReadNextLine(AisanCenterLine& data);
	int ReadAllData(std::vector<AisanCenterLine>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::DTLane& _rec, AisanCenterLine& data);
	AisanCenterLine* GetDataRowById(int _lnid);
	std::vector<AisanCenterLine> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanCenterLine& obj)
	{
	    os << obj.DID << ","
	    << obj.Dist << ","
	    << obj.PID << ","
	    << obj.Dir << ","
	    << obj.Apara << ","
	    << obj.r << ","
	    << obj.slope << ","
	    << obj.cant << ","
	    << obj.LW << ","
	    << obj.RW;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanCenterLine*> m_data_map;
};

class AisanAreasFileReader : public SimpleReaderBase
{
public:

	struct AisanArea
	{
		int 	AID;
		int 	SLID;
		int 	ELID;
	};

	AisanAreasFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "area.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "AID,SLID,ELID";
	}
	AisanAreasFileReader(const vector_map_msgs::AreaArray& _areas);
	virtual ~AisanAreasFileReader(){}

	bool ReadNextLine(AisanArea& data);
	int ReadAllData(std::vector<AisanArea>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Area& _rec, AisanArea& data);
	AisanArea* GetDataRowById(int _lnid);
	std::vector<AisanArea> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanArea& obj)
	{
	    os << obj.AID << ","
	    << obj.SLID << ","
	    << obj.ELID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanArea*> m_data_map;
};

class AisanIntersectionFileReader : public SimpleReaderBase
{
public:

	struct AisanIntersection
	{
		int 	ID;
		int 	AID;
		int 	LinkID;
	};

	AisanIntersectionFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "intersection.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,LinkID";
	}
	AisanIntersectionFileReader(const vector_map_msgs::CrossRoadArray& _inters);
	virtual ~AisanIntersectionFileReader(){}

	bool ReadNextLine(AisanIntersection& data);
	int ReadAllData(std::vector<AisanIntersection>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::CrossRoad& _rec, AisanIntersection& data);
	AisanIntersection* GetDataRowById(int _lnid);
	std::vector<AisanIntersection> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanIntersection& obj)
	{
	    os << obj.ID << ","
	    << obj.AID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanIntersection*> m_data_map;
};

class AisanLanesFileReader : public SimpleReaderBase
{
public:

	struct AisanLane
	{
		int LnID	;
		int DID		;
		int BLID	;
		int FLID	;
		int BNID	;
		int FNID	;
		int JCT		;
		int BLID2	;
		int BLID3	;
		int BLID4	;
		int FLID2	;
		int FLID3	;
		int FLID4	;
		int ClossID	;
	 	double Span	;
		int LCnt	;
		int Lno		;
		int LaneType;
		int LimitVel;
		int RefVel	;
		int RoadSecID;
		int LaneChgFG;
		int LinkWAID;
	    char LaneDir;
	    int  LeftLaneId;
		int RightLaneId;

		int originalMapID;
	};

	AisanLanesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "lane.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4,FLID2,FLID3,"
					"FLID4,ClossID,Span,LCnt,Lno,LaneType,LimitVel,RefVel,RoadSecID,LaneChgFG,LinkWAID";
	}
	AisanLanesFileReader(const vector_map_msgs::LaneArray& _lanes);
	virtual ~AisanLanesFileReader(){}

	bool ReadNextLine(AisanLane& data);
	int ReadAllData(std::vector<AisanLane>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Lane& _rec, AisanLane& data);
	AisanLane* GetDataRowById(int _lnid);
	std::vector<AisanLane> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanLane& obj)
	{
	    os << obj.LnID				<< ","
	       << obj.DID					<< ","
	       << obj.BLID				<< ","
	       << obj.FLID	 			<< ","
	       << obj.BNID	 			<< ","
	       << obj.FNID	 			<< ","
	       << obj.JCT		 			<< ","
	       << obj.BLID2	 			<< ","
	       << obj.BLID3	 			<< ","
	       << obj.BLID4	 			<< ","
	       << obj.FLID2	 			<< ","
	       << obj.FLID3	 			<< ","
	       << obj.FLID4	 			<< ","
	       << obj.ClossID	 			<< ","
	       << obj.Span	 			<< ","
	       << obj.LCnt	 			<< ","
	       << obj.Lno		 			<< ","
	       << obj.LaneType 			<< ","
	       << obj.LimitVel 			<< ","
	       << obj.RefVel	 			<< ","
	       << obj.RoadSecID 			<< ","
	       << obj.LaneChgFG 			<< ","
	       << obj.LinkWAID 			<< ","
	       << obj.LaneDir 			<< ","
	       << obj.LeftLaneId 			<< ","
	       << obj.RightLaneId;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanLane*> m_data_map;
};

class AisanStopLineFileReader : public SimpleReaderBase
{
public:

	struct AisanStopLine
	{
		int 	ID;
		int 	LID;
		int 	TLID;
		int 	SignID;
		int 	LinkID;
	};

	AisanStopLineFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "stopline.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,TLID,SignID,LinkID";
	}
	AisanStopLineFileReader(const vector_map_msgs::StopLineArray& _stopLines);
	virtual ~AisanStopLineFileReader(){}

	bool ReadNextLine(AisanStopLine& data);
	int ReadAllData(std::vector<AisanStopLine>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::StopLine& _rec, AisanStopLine& data);
	AisanStopLine* GetDataRowById(int _lnid);
	std::vector<AisanStopLine> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanStopLine& obj)
	{
	    os << obj.ID << ","
	    << obj.LID << ","
	    << obj.TLID << ","
	    << obj.SignID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanStopLine*> m_data_map;
};

class AisanRoadSignFileReader : public SimpleReaderBase
{
public:

	struct AisanRoadSign
	{
		int 	ID;
		int 	VID;
		int 	PLID;
		int 	Type;
		int 	LinkID;
	};

	AisanRoadSignFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "roadsign.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,VID,PLID,Type,LinkID";
	}
	AisanRoadSignFileReader(const vector_map_msgs::RoadSignArray& _signs);
	virtual ~AisanRoadSignFileReader(){}

	bool ReadNextLine(AisanRoadSign& data);
	int ReadAllData(std::vector<AisanRoadSign>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::RoadSign& _rec, AisanRoadSign& data);
	AisanRoadSign* GetDataRowById(int _lnid);
	std::vector<AisanRoadSign> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanRoadSign& obj)
	{
	    os << obj.ID << ","
	    << obj.VID << ","
	    << obj.PLID << ","
	    << obj.Type << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanRoadSign*> m_data_map;
};

class AisanSignalFileReader : public SimpleReaderBase
{
public:

	struct AisanSignal
	{
		int 	ID;
		int 	VID;
		int 	PLID;
		int 	Type;
		int 	LinkID;
	};

	AisanSignalFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "signaldata.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,VID,PLID,Type,LinkID";
	}
	AisanSignalFileReader(const vector_map_msgs::SignalArray& _signals);
	virtual ~AisanSignalFileReader(){}

	bool ReadNextLine(AisanSignal& data);
	int ReadAllData(std::vector<AisanSignal>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Signal& _rec, AisanSignal& data);
	AisanSignal* GetDataRowById(int _lnid);
	std::vector<AisanSignal> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanSignal& obj)
	{
	    os << obj.ID << ","
	    << obj.VID << ","
	    << obj.PLID << ","
	    << obj.Type << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanSignal*> m_data_map;
};

class AisanVectorFileReader : public SimpleReaderBase
{
public:

	struct AisanVector
	{
		int 	VID;
		int 	PID;
		double 	Hang;
		double 	Vang;
	};

	AisanVectorFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "vector.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "VID,PID,hang,Vang";
	}
	AisanVectorFileReader(const vector_map_msgs::VectorArray& _vectors);
	virtual ~AisanVectorFileReader(){}

	bool ReadNextLine(AisanVector& data);
	int ReadAllData(std::vector<AisanVector>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Vector& _rec, AisanVector& data);
	AisanVector* GetDataRowById(int _lnid);
	std::vector<AisanVector> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanVector& obj)
	{
	    os << obj.VID << ","
	    << obj.PID << ","
	    << obj.Hang << ","
	    << obj.Vang;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanVector*> m_data_map;
};

class AisanCurbFileReader : public SimpleReaderBase
{
public:

	struct AisanCurb
	{
		int 	ID;
		int 	LID;
		double 	Height;
		double 	Width;
		int 	dir;
		int 	LinkID;
	};

	AisanCurbFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "curb.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,Height,Width,dir,LinkID";
	}
	AisanCurbFileReader(const vector_map_msgs::CurbArray& _curbs);
	virtual ~AisanCurbFileReader(){}

	bool ReadNextLine(AisanCurb& data);
	int ReadAllData(std::vector<AisanCurb>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Curb& _rec, AisanCurb& data);
	AisanCurb* GetDataRowById(int _lnid);
	std::vector<AisanCurb> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanCurb& obj)
	{
	    os << obj.ID << ","
	    << obj.LID << ","
	    << obj.Height << ","
	    << obj.Width << ","
	    << obj.dir << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanCurb*> m_data_map;
};

class AisanCrossWalkFileReader : public SimpleReaderBase
{
public:

	struct AisanCrossWalk
	{
		int 	ID;
		int 	AID;
		int 	Type;
		int		BdID;
		int 	LinkID;
	};

	AisanCrossWalkFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "crosswalk.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,Type,BdID,LinkID";
	}
	AisanCrossWalkFileReader(const vector_map_msgs::CrossWalkArray& _crossWalks);
	virtual ~AisanCrossWalkFileReader(){}

	bool ReadNextLine(AisanCrossWalk& data);
	int ReadAllData(std::vector<AisanCrossWalk>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::CrossWalk& _rec, AisanCrossWalk& data);
	AisanCrossWalk* GetDataRowById(int _lnid);
	std::vector<AisanCrossWalk> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanCrossWalk& obj)
	{
	    os << obj.ID << ","
	    << obj.AID << ","
	    << obj.Type << ","
	    << obj.BdID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanCrossWalk*> m_data_map;
};

class AisanWayareaFileReader : public SimpleReaderBase
{
public:

	struct AisanWayarea
	{
		int 	ID;
		int 	AID;
		int 	LinkID;
	};

	AisanWayareaFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "wayarea.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,LinkID";
	}
	AisanWayareaFileReader(const vector_map_msgs::WayAreaArray& _wayArea);
	virtual ~AisanWayareaFileReader(){}

	bool ReadNextLine(AisanWayarea& data);
	int ReadAllData(std::vector<AisanWayarea>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::WayArea& _rec, AisanWayarea& data);
	AisanWayarea* GetDataRowById(int _lnid);
	std::vector<AisanWayarea> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanWayarea& obj)
	{
	    os << obj.ID << ","
	    << obj.AID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanWayarea*> m_data_map;
};

class AisanWhitelinesFileReader : public SimpleReaderBase
{
public:

	struct AisanWhiteline
	{
		int ID;
		int LID;
		double Width;
		std::string Color;
		int type;
		int LinkID;
	};

	AisanWhitelinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "whiteline.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,Width,Color,type,LinkID";
	}

	AisanWhitelinesFileReader(const vector_map_msgs::WhiteLineArray& _white_line);
	virtual ~AisanWhitelinesFileReader(){}

	bool ReadNextLine(AisanWhiteline& data);
	int ReadAllData(std::vector<AisanWhiteline>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::WhiteLine& _rec, AisanWhiteline& data);
	AisanWhiteline* GetDataRowById(int _wlid);
	std::vector<AisanWhiteline> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanWhiteline& obj)
	{
	    os << obj.ID << ","
	    << obj.LID
	    << obj.Width
	    << obj.Color
	    << obj.type
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanWhiteline*> m_data_map;
};

class AisanGutterFileReader : public SimpleReaderBase
{
public:

	struct AisanGutter
	{
		int ID;
		int AID;
		int Type;
		int LinkID;
	};

	AisanGutterFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "gutter.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,Type,LinkID";
	}

	AisanGutterFileReader(const vector_map_msgs::GutterArray& _gutter);
	virtual ~AisanGutterFileReader(){}

	bool ReadNextLine(AisanGutter& data);
	int ReadAllData(std::vector<AisanGutter>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Gutter& _rec, AisanGutter& data);
	AisanGutter* GetDataRowById(int _wlid);
	std::vector<AisanGutter> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanGutter& obj)
	{
	    os << obj.ID << ","
	    << obj.AID
	    << obj.Type
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanGutter*> m_data_map;
};

class AisanIdxFileReader : public SimpleReaderBase
{
public:

	struct AisanIdx
	{
		int ID;
		std::string KIND;
		std::string fname;
	};

	AisanIdxFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "idx.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,KIND,fname";
	}


	virtual ~AisanIdxFileReader(){}

	bool ReadNextLine(AisanIdx& data);
	int ReadAllData(std::vector<AisanIdx>& data_list);
	int ReadAllData();
	//void ParseNextLine(const vector_map_msgs::WhiteLine& _rec, AisanIdx& data);
	AisanIdx* GetDataRowById(int _wlid);
	std::vector<AisanIdx> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanIdx& obj)
	{
	    os << obj.ID << ","
	    << obj.KIND
	    << obj.fname;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanIdx*> m_data_map;
};

class AisanPoleFileReader : public SimpleReaderBase
{
public:

	struct AisanPole
	{
		int PLID;
		int VID;
		double Length;
		double Dim;
	};

	AisanPoleFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "pole.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "PLID,VID,Length,Dim";
	}

	AisanPoleFileReader(const vector_map_msgs::PoleArray& _pole);
	virtual ~AisanPoleFileReader(){}

	bool ReadNextLine(AisanPole& data);
	int ReadAllData(std::vector<AisanPole>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::Pole& _rec, AisanPole& data);
	AisanPole* GetDataRowById(int _wlid);
	std::vector<AisanPole> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanPole& obj)
	{
	    os << obj.PLID << ","
	    << obj.VID
	    << obj.Length
	    << obj.Dim;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanPole*> m_data_map;
};

class AisanPoledataFileReader : public SimpleReaderBase
{
public:

	struct AisanPoledata
	{
		int ID;
		int PLID;
		int LinkID;
	};

	AisanPoledataFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "poledata.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,PLID,LinkID";
	}

	//AisanPoledataFileReader(const vector_map_msgs::Poledata& _Poledata);
	virtual ~AisanPoledataFileReader(){}

	bool ReadNextLine(AisanPoledata& data);
	int ReadAllData(std::vector<AisanPoledata>& data_list);
	int ReadAllData();
	//void ParseNextLine(const vector_map_msgs::Poledata& _rec, AisanPoledata& data);
	AisanPoledata* GetDataRowById(int _wlid);
	std::vector<AisanPoledata> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanPoledata& obj)
	{
	    os << obj.ID << ","
	    << obj.PLID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanPoledata*> m_data_map;
};

class AisanRoadEdgeFileReader : public SimpleReaderBase
{
public:

	struct AisanRoadEdge
	{
		int ID;
		int LID;
		int LinkID;
	};

	AisanRoadEdgeFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "roadedge.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,LinkID";
	}

	AisanRoadEdgeFileReader(const vector_map_msgs::RoadEdgeArray& _roadedge);
	virtual ~AisanRoadEdgeFileReader(){}

	bool ReadNextLine(AisanRoadEdge& data);
	int ReadAllData(std::vector<AisanRoadEdge>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::RoadEdge& _rec, AisanRoadEdge& data);
	AisanRoadEdge* GetDataRowById(int _wlid);
	std::vector<AisanRoadEdge> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanRoadEdge& obj)
	{
	    os << obj.ID << ","
	    << obj.LID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanRoadEdge*> m_data_map;
};

class AisanSurfacemarkFileReader : public SimpleReaderBase
{
public:

	struct AisanSurfacemark
	{
		int ID;
		int AID;
		int Type;
		int LinkID;
	};

	AisanSurfacemarkFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "road_surface_mark.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,Type,LinkID";
	}

	//AisanSurfacemarkFileReader(const vector_map_msgs::& _Surfacemark);
	virtual ~AisanSurfacemarkFileReader(){}

	bool ReadNextLine(AisanSurfacemark& data);
	int ReadAllData(std::vector<AisanSurfacemark>& data_list);
	int ReadAllData();
	//void ParseNextLine(const vector_map_msgs::Surfacemark& _rec, AisanSurfacemark& data);
	AisanSurfacemark* GetDataRowById(int _wlid);
	std::vector<AisanSurfacemark> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanSurfacemark& obj)
	{
	    os << obj.ID << ","
	    << obj.AID
	    << obj.Type
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanSurfacemark*> m_data_map;
};

class AisanStreetlightFileReader : public SimpleReaderBase
{
public:

	struct AisanStreetlight
	{
		int ID;
		int LID;
		int PLID;
		int LinkID;
	};

	AisanStreetlightFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "streetlight.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,PLID,LinkID";
	}

	AisanStreetlightFileReader(const vector_map_msgs::StreetLightArray& _streetlight);
	virtual ~AisanStreetlightFileReader(){}

	bool ReadNextLine(AisanStreetlight& data);
	int ReadAllData(std::vector<AisanStreetlight>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::StreetLight& _rec, AisanStreetlight& data);
	AisanStreetlight* GetDataRowById(int _wlid);
	std::vector<AisanStreetlight> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanStreetlight& obj)
	{
	    os << obj.ID << ","
	    << obj.LID
	    << obj.PLID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanStreetlight*> m_data_map;
};

class AisanUtilitypoleFileReader : public SimpleReaderBase
{
public:

	struct AisanUtilitypole
	{
		int ID;
		int PLID;
		int LinkID;
	};

	AisanUtilitypoleFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "utilitypole.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,PLID,LinkID";
	}

	AisanUtilitypoleFileReader(const vector_map_msgs::UtilityPoleArray& _utilitypole);
	virtual ~AisanUtilitypoleFileReader(){}

	bool ReadNextLine(AisanUtilitypole& data);
	int ReadAllData(std::vector<AisanUtilitypole>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::UtilityPole& _rec, AisanUtilitypole& data);
	AisanUtilitypole* GetDataRowById(int _wlid);
	std::vector<AisanUtilitypole> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanUtilitypole& obj)
	{
	    os << obj.ID << ","
	    << obj.PLID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanUtilitypole*> m_data_map;
};

class AisanZebrazoneFileReader : public SimpleReaderBase
{
public:

	struct AisanZebrazone
	{
		int ID;
		int AID;
		int LinkID;
	};

	AisanZebrazoneFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "zebrazone.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,LinkID";
	}

	AisanZebrazoneFileReader(const vector_map_msgs::ZebraZoneArray& _zebrazone);
	virtual ~AisanZebrazoneFileReader(){}

	bool ReadNextLine(AisanZebrazone& data);
	int ReadAllData(std::vector<AisanZebrazone>& data_list);
	int ReadAllData();
	void ParseNextLine(const vector_map_msgs::ZebraZone& _rec, AisanZebrazone& data);
	AisanZebrazone* GetDataRowById(int _wlid);
	std::vector<AisanZebrazone> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanZebrazone& obj)
	{
	    os << obj.ID << ","
	    << obj.AID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanZebrazone*> m_data_map;
};

class AisanDataConnFileReader : public SimpleReaderBase
{
public:

	struct DataConn
	{
		int 	LID; // lane id
		int 	SLID; // stop line id
		int 	SID; // signal id
		int 	SSID; // stop sign id
	};

	AisanDataConnFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
	{
		header_ = "LID,SLID,SID,SSID";
	}

	virtual ~AisanDataConnFileReader(){}

	bool ReadNextLine(DataConn& data);
	int ReadAllData(std::vector<DataConn>& data_list);
	int ReadAllData();

	friend std::ostream& operator<<(std::ostream& os, const DataConn& obj)
	{
	    os << obj.LID << ","
	    << obj.SLID << ","
	    << obj.SID << ","
	    << obj.SSID;
	    return os;
	}
};

class MapRaw
{
public:
	UtilityHNS::AisanLanesFileReader* pLanes;
	UtilityHNS::AisanPointsFileReader* pPoints;
	UtilityHNS::AisanCenterLinesFileReader* pCenterLines;
	UtilityHNS::AisanIntersectionFileReader* pIntersections;
	UtilityHNS::AisanAreasFileReader* pAreas;
	UtilityHNS::AisanLinesFileReader* pLines;
	UtilityHNS::AisanStopLineFileReader* pStopLines;
	UtilityHNS::AisanSignalFileReader* pSignals;
	UtilityHNS::AisanVectorFileReader* pVectors;
	UtilityHNS::AisanCurbFileReader* pCurbs;
	UtilityHNS::AisanRoadEdgeFileReader* pRoadedges;
	UtilityHNS::AisanWayareaFileReader* pWayAreas;
	UtilityHNS::AisanCrossWalkFileReader* pCrossWalks;
	UtilityHNS::AisanNodesFileReader* pNodes;
	UtilityHNS::AisanWhitelinesFileReader* pWhitelines;
	UtilityHNS::AisanGutterFileReader* pGutter;
	UtilityHNS::AisanIdxFileReader* pIdx;
	UtilityHNS::AisanPoleFileReader* pPole;
	UtilityHNS::AisanPoledataFileReader* pPoledata;
	UtilityHNS::AisanSurfacemarkFileReader* pSurfacemark;
	UtilityHNS::AisanStreetlightFileReader* pStreetLight;
	UtilityHNS::AisanUtilitypoleFileReader* pUtilitypole;
	UtilityHNS::AisanZebrazoneFileReader* pZebrazore;

	std::string map_path;

	struct timespec _time_out;

	MapRaw()
	{
		pLanes = nullptr;
		pPoints = nullptr;
		pCenterLines = nullptr;
		pIntersections = nullptr;
		pAreas = nullptr;
		pLines = nullptr;
		pStopLines = nullptr;
		pSignals = nullptr;
		pVectors = nullptr;
		pCurbs = nullptr;
		pRoadedges = nullptr;
		pWayAreas = nullptr;
		pCrossWalks = nullptr;
		pNodes = nullptr;
		pWhitelines = nullptr;
		pGutter = nullptr;
		pIdx = nullptr;
		pPole = nullptr;
		pPoledata = nullptr;
		pSurfacemark = nullptr;
		pStreetLight = nullptr;
		pUtilitypole = nullptr;
		pZebrazore = nullptr;

		UtilityH::GetTickCount(_time_out);
	}

//	MapRaw(std::string folder_name) : MapRaw()
//	{
//		map_path = folder_name;
//
//		std::cout << " >> Loading vector map data files ... " << std::endl;
//		AisanNodesFileReader 		nodes(map_path);
//		AisanPointsFileReader 		points(map_path);
//		AisanLinesFileReader 		lines(map_path);
//		AisanWhitelinesFileReader 	whitelines(map_path);
//		AisanCenterLinesFileReader  dtlanes(map_path);
//		AisanLanesFileReader 		lanes(map_path);
//		AisanAreasFileReader 		areas(map_path);
//		AisanIntersectionFileReader intersections(map_path);
//		AisanStopLineFileReader 	stoplines(map_path);
//		AisanRoadSignFileReader		roadsigns(map_path);
//		AisanSignalFileReader 		signals(map_path);
//		AisanVectorFileReader 		vectors(map_path);
//		AisanCurbFileReader 		curbs(map_path);
//		AisanCrossWalkFileReader 	crosswalks(map_path);
//		AisanWayareaFileReader 		wayareas(map_path);
//		AisanGutterFileReader		gutters(map_path);
//		AisanIdxFileReader			idxs(map_path);
//		AisanPoleFileReader			poles(map_path);
//		AisanPoledataFileReader		polesdata(map_path);
//		AisanRoadEdgeFileReader 	roadedges(map_path);
//		AisanSurfacemarkFileReader	surfacemarks(map_path);
//		AisanStreetlightFileReader	streetlights(map_path);
//		AisanUtilitypoleFileReader	utilitypoles(map_path);
//		AisanZebrazoneFileReader	zerazones(map_path);
//	}

	virtual ~MapRaw()
	{
		if(pLanes != nullptr)
		{
			delete pLanes;
			pLanes = nullptr;
		}

		if(pPoints != nullptr)
		{
			delete pPoints;
			pPoints = nullptr;
		}

		if(pCenterLines != nullptr)
		{
			delete pCenterLines;
			pCenterLines = nullptr;
		}

		if(pIntersections != nullptr)
		{
			delete pIntersections;
			pIntersections = nullptr;
		}

		if(pAreas != nullptr)
		{
			delete pAreas;
			pAreas = nullptr;
		}

		if(pLines != nullptr)
		{
			delete pLines;
			pLines = nullptr;
		}

		if(pStopLines != nullptr)
		{
			delete pStopLines;
			pStopLines = nullptr;
		}

		if(pSignals != nullptr)
		{
			delete pSignals;
			pSignals = nullptr;
		}

		if(pVectors != nullptr)
		{
			delete pVectors;
			pVectors = nullptr;
		}

		if(pCurbs != nullptr)
		{
			delete pCurbs;
			pCurbs = nullptr;
		}

		if(pRoadedges != nullptr)
		{
			delete pRoadedges;
			pRoadedges = nullptr;
		}

		if(pWayAreas != nullptr)
		{
			delete pWayAreas;
			pWayAreas = nullptr;
		}

		if(pCrossWalks != nullptr)
		{
			delete pCrossWalks;
			pCrossWalks = nullptr;
		}

		if(pNodes != nullptr)
		{
			delete pNodes;
			pNodes = nullptr;
		}

		if(pWhitelines != nullptr)
		{
			delete pWhitelines;
			pWhitelines = nullptr;
		}
	}

	int GetVersion()
	{
		bool bTimeOut = UtilityH::GetTimeDiffNow(_time_out) > 2.0;
		bool bLoaded =  pLanes != nullptr && pPoints != nullptr && pCenterLines  != nullptr && pNodes  != nullptr;
		int iVersion = 0;
		if(bLoaded && bTimeOut)
		{
			iVersion = 2;
			if(pNodes->m_data_list.size() == 0)
			{
				iVersion = 1;
			}
		}
//		else
//		{
//			bLoaded =  pLanes != nullptr && pPoints != nullptr && pCenterLines  != nullptr;
//			if(bLoaded && bTimeOut)
//			{
//				iVersion = 1;
//				if(pNodes  == nullptr)
//					pNodes = new AisanNodesFileReader(vector_map_msgs::NodeArray());
//			}
//		}

		if(bLoaded && bTimeOut)
		{
			if(pIntersections  == nullptr)
				pIntersections  = new AisanIntersectionFileReader(vector_map_msgs::CrossRoadArray());

			if(pLines  == nullptr)
				pLines  = new AisanLinesFileReader(vector_map_msgs::LineArray());

			if(pStopLines  == nullptr)
				pStopLines  = new AisanStopLineFileReader(vector_map_msgs::StopLineArray());

			if(pSignals  == nullptr)
				pSignals  = new AisanSignalFileReader(vector_map_msgs::SignalArray());

			if(pVectors  == nullptr)
				pVectors  = new AisanVectorFileReader(vector_map_msgs::VectorArray());

			if(pCurbs  == nullptr)
				pCurbs  = new AisanCurbFileReader(vector_map_msgs::CurbArray());

			if(pRoadedges  == nullptr)
				pRoadedges  = new AisanRoadEdgeFileReader(vector_map_msgs::RoadEdgeArray());

			if(pWayAreas  == nullptr)
				pWayAreas  = new AisanWayareaFileReader(vector_map_msgs::WayAreaArray());

			if(pCrossWalks  == nullptr)
				pCrossWalks  = new AisanCrossWalkFileReader(vector_map_msgs::CrossWalkArray());
		}

		return iVersion;
	}
};

} /* namespace UtilityHNS */

#endif /* DATARW_H_ */
