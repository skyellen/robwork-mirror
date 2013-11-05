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
	Pace();
	Pace(double _time, Q _robotConfig, Q _gripperConfig, vector<pair<int, Transform3D<> > > _objectPoses);
	
	friend ostream& operator<<(ostream& os, const Pace& pace);
	friend istream& operator>>(istream& is, Pace& pace);
};

class Reader {
	ifstream currentFile;
public:
	vector<Pace *> readAllFile(string filename);	
	Reader(string filename);
	bool openFile(string filename);
	Pace * readNext();
	void closeFile();
};

class Recorder {
	vector<Pace *> paces;
public:
	void addPace(Pace *pace);
	void save(string filename);
};


#endif
