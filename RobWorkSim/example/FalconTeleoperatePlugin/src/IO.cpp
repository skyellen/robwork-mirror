#include "IO.hpp"

/**************************    Pace    **************************/
Pace::Pace() {
	
}

Pace::Pace(double _time, Q _robotConfig, Q _gripperConfig, vector< pair< int, Transform3D<> > > _objectPoses){
	time = _time;
	robotConfig = _robotConfig;
	gripperConfig = _gripperConfig;
	objectPoses = _objectPoses;
}
	
ostream& operator<<(ostream& os, const Pace& pace) {
	os << pace.time << ":" << pace.robotConfig << ":" << pace.gripperConfig << ":" << pace.objectPoses.size() ;
	
	for(int i = 0 ; i < pace.objectPoses.size() ; i++)
	{
		os << " " <<pace.objectPoses[i].first << " " << pace.objectPoses[i].second;
	}
	
	/* 	to write contact points
	os << ":" << pace.contactPoints.size();
	
	for (int i = 0; i < contactPoints.size(); ++i)
		os << " " << pace.contactPoints[i].p; //write contact point in the structure you wish 
	*/
	
	return os;
}

istream& operator>>(istream& is, Transform3D<> &t) {

	double p1, p2, p3, r1, r2, r3, r4, r5, r6, r7, r8, r9;
	char c;
	
	do {
		is.get(c);
	} while (c == ' ' || c == '\t' );
	
	
	//read Transform3D
	is >> c;
	while ( c != '(' ) {
		is >> c;
	}
	
	//read Vector3D
	is >> c;
	while ( c != '(' )
		is >> c;
	is >> p1 >> c >> p2 >> c >> p3;
	
	//read Rotation3D
	is >> c;
	while ( c != '(' )
		is >> c;	
	is >> r1 >> c >> r2 >> c >> r3 >> c >> r4 >> c >> r5 >> c >> r6 >> c >> r7 >> c >> r8 >> c >> r9;
	
	t = Transform3D<>(Vector3D<>(p1, p2, p3), Rotation3D<>(r1, r2, r3, r4, r5, r6, r7, r8, r9));

	return is >> c >> c;
}


istream& operator>>(istream& is, Pace& pace) {
	//TODO
	char c;
	int objectPosesCount;
	is >> pace.time >> c; 
	is >> pace.robotConfig >> c >> pace.gripperConfig >> c>> objectPosesCount;
	pace.objectPoses.clear();
	
	for(int i = 0 ; i < objectPosesCount ; i++)
	{
		int id;
		Transform3D<> coor;
		
		is >> id;
		is >> coor;
		pace.objectPoses.push_back(make_pair(id,coor));
	}
	
	/* to read the contact points from file
	int contactPointsCount;
	is >> contactPointsCount;
	for(int i = 0; i < contactPointsCount; ++i)
	{
		//TODO parse the contact point structure here
	}
	*/
	
	return is ;
}

/**************************    Recorder    **************************/
void Recorder::addPace(Pace *pace) {
	paces.push_back(pace);
}

void Recorder::save(string filename) {
	ofstream fout;
	fout.open(filename.c_str());
	
	for (vector<Pace *>::iterator it = paces.begin(); it != paces.end(); ++it) {
		fout << **it << endl;
		delete *it;
	}
		
	fout.close();
	paces.clear();
}

/**************************    Reader    **************************/
Reader::Reader(string filename) {
	openFile(filename);
}

bool Reader::openFile(string filename) {
	if (currentFile.is_open())
		currentFile.close();
		
	currentFile.open(filename.c_str());
}

Pace * Reader::readNext() {
	static stringstream ss;
	static string line;
	
	while( getline(currentFile, line) && line[0] == '#' );

	// make sure there is a line with pace
	if (line[0] != '#') {
	
		// clear the stream
		ss.str( string() );
		ss.clear();
		
		ss << line;
		
		Pace *pace = new Pace();
		ss >> *pace;
		
		return pace;
	}

	return NULL;
}

void Reader::closeFile() {
	currentFile.close();
}

vector<Pace *> Reader::readAllFile(string filename) {
	vector<Pace *> paces;
	Pace *pace;
	
	openFile(filename);
	
	while ( (pace = readNext()) != NULL )
		paces.push_back(pace);
	
	closeFile();
	
	return paces;
}

