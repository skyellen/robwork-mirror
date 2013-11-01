#ifndef IO_HPP
#define IO_HPP


#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>

using namespace std;
using namespace rw::math;

/**
 * @class Pace
 * 
 * @brief Describes the robot & gripper configurations and poses of the objects in the scene at specific time step
 */
class Pace {

	//time
	double time;
	//robot config
	Q robotConfig;
	//gripper config
	Q gripperConfig;
	//object poses
	vector< pair< int, Transform3D<> > > objectPoses;
	//object contacts {[id id #ofPoints points ...]} TODO
	
public:
	/// Default constructor
	Pace();
	
	/// Constructor
	Pace(double _time, Q _robotConfig, Q _gripperConfig, vector<pair<int, Transform3D<> > > _objectPoses);
	
	friend ostream& operator<<(ostream& os, const Pace& pace);
	friend istream& operator>>(istream& is, Pace& pace);
};

/**
 * @class Reader
 * @brief A class to read paces from an ASCII file.
 */
class Reader {
	ifstream currentFile;
public:
	/// Convenience function for reading the whole file at once
	vector<Pace *> readAllFile(string filename);
	
	/// Constructor	
	Reader(string filename);
	
	/// Opens a file for reading
	bool openFile(string filename);
	
	/// Reads next Pace from opened file
	Pace * readNext();
	
	/// Closes the file
	void closeFile();
};

class Recorder {
	vector<Pace *> paces;
public:
	/// Stores another Pace by appending it to a vector
	void addPace(Pace *pace);
	
	/// Saves stored Paces to a specified file
	void save(string filename);
};


#endif
