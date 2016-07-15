/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <fstream>
#include <vector>

#include <rw/graspplanning/GraspTable.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::math;
using rw::graspplanning::GraspTable;

const int linesize = 8*256;
char tmpStr[linesize];

Vector3D<> getVector3D(std::ifstream& istr){
	double x,y,z;
	//Unused: double rr,rp,ry;
	char tmpC;
	istr >> tmpC >> x >> tmpC >> y >> tmpC >> z;
	//std::cout << "center:" << x << "," << y << "," << z<< std::endl;
	istr.getline(tmpStr,linesize); // arm goal 1
	return Vector3D<>(x,y,z);
}

rw::sensor::TactileArray::ValueMatrix readMatrix(std::ifstream& istr, int xdim, int ydim){
	istr.getline(tmpStr,linesize); // all vals

	std::stringstream sstr;
	sstr << tmpStr;

	rw::sensor::TactileArray::ValueMatrix vals(xdim,ydim);
	for(int y=0;y<ydim;y++)
		for(int x=0;x<xdim;x++)
			sstr >> vals(x,y);

	return vals.transpose();
}

Q getQ(std::ifstream& istr){
	double tmp;
	std::vector<double> vals;
	istr.getline(tmpStr,linesize); // all vals

	std::stringstream sstr;
	sstr << tmpStr;
	while(!sstr.eof()){
		sstr >> tmp;
		if(sstr.fail())
			break;
		vals.push_back(tmp);
	}
	//std::cout << "size of vals: " << vals.size() << std::endl;

	Q tmpQ(vals.size());
	for(size_t i=0;i<vals.size(); i++){
		tmpQ(i)=vals[i];
	}
	if(vals.size()==7){
		tmpQ(2) = vals[0];
		//tmpQ(0) = vals[2];
		tmpQ(1) = vals[2]; // thumb distal
		tmpQ(0) = vals[1];// thumb prox


	}

	return tmpQ*Deg2Rad;
}

int main(int argc, char** argv)
{
	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
		std::cout << "- Arg 1 name of KTH table filen" << std::endl;
	    std::cout << "- Arg 2 name of grasp table" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);
	std::string table_filename(argv[2]);
	//Unused: int groupSize = 1;

	//Unused: GraspTable *gtable = new GraspTable("SchunkHand","Cylinder");

	std::ifstream istr( filename.c_str() );
/*
	[object center]
	(0,950,210)
	[arm goal 1]
	(0,657.689,277.485)
	[arm goal 2]
	(0,716.151,263.988)
	[approach angle]
	13
	[approach vector]
	(0,0.97437,-0.224951)
	[hand preshape]
	0 -90 0 -90 0 -90 0
	[hand target]
	0 45 90 45 90 45 90
	[frame 8103162]
*/
	//Unused: char tmpC;
	//Unused: double x,y,z,rr,rp,ry;
	const int linesize = 8*256;
	char tmpStr[linesize];
	istr.getline(tmpStr,linesize); // object center
	//Unused: Vector3D<> objCenter = getVector3D(istr);
	//Instead:
	getVector3D(istr);

	istr.getline(tmpStr,linesize); // arm goal 1
	//Unused: Vector3D<> armGoal1 = getVector3D(istr);
	//Instead:
	getVector3D(istr);

	istr.getline(tmpStr,linesize); // arm goal 2
	//Unused: Vector3D<> armGoal2 = getVector3D(istr);
	//Instead:
	getVector3D(istr);

	istr.getline(tmpStr,linesize); // approach angle
	double approach_angle;
	istr >> approach_angle;
	istr.getline(tmpStr,linesize); // next line

	istr.getline(tmpStr,linesize); // approach vector
	Vector3D<> approach = getVector3D(istr);

	istr.getline(tmpStr,linesize); // hand preshape
	Q preshape = getQ(istr);

	istr.getline(tmpStr,linesize); // hand target
	Q target = getQ(istr);
	typedef rw::sensor::TactileArray::ValueMatrix VMatrix;
	rw::sensor::TactileArray::ValueMatrix values;
	GraspTable table("SchunkHand","Object");

	// now come all the frames
	while(!istr.eof()){
		GraspTable::GraspData data;
		data.approach = approach;
		data.pq = preshape;
		//data.op = Pose6D<>( objCenter, EAA<>( Rortation3D<>() ) );

		istr.getline(tmpStr,linesize); // frame
		istr.getline(tmpStr,linesize); // nr contacts... or something *only new data(*(*

		//istr.getline(tmpStr,linesize); // proximal
		VMatrix prox1val = readMatrix(istr,6,14); // proximal
		values = readMatrix(istr,6,13);
		data._tactiledata.push_back(values);

		//istr.getline(tmpStr,linesize); // proximal
		VMatrix prox2val = readMatrix(istr,6,14); // proximal
		values = readMatrix(istr,6,13);
		data._tactiledata.push_back(values);

		//istr.getline(tmpStr,linesize); // proximal
		VMatrix prox3val = readMatrix(istr,6,14); // proximal
		values = readMatrix(istr,6,13);
		data._tactiledata.push_back(values);

		data._tactiledata.push_back(prox1val);
		data._tactiledata.push_back(prox2val);
		data._tactiledata.push_back(prox3val);

		istr.getline(tmpStr,linesize); // empty

		//std::cout << tmpStr << std::endl;

		Q currentQ = getQ(istr);
		data.cq = currentQ;

		istr.getline(tmpStr,linesize); // empty
		istr.getline(tmpStr,linesize); // image
		istr.getline(tmpStr,linesize); // empty
		std::cout << currentQ << std::endl;
		table.addGrasp(data);
	}
	table.save(table_filename);
	//std::cout << "Checking!" << std::endl;
	//GraspTable::load(table_filename);
	return 0;
}
