#include <iostream>
#include <rw/rw.hpp>
#include "CSGModel.hpp"



using namespace std;

USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::csg;



int main(int argc, char* argv[])
{
	// let's test some stuff
	CSGModel A = CSGModel::makeCube(1, 1, 1);
	CSGModel AA = CSGModel::makeSphere(0.6);
	//CSGModel B = CSGModel::makeCube(0.1, 0.1, 0.1).translate(-0.5, 0.5, 0.5);
	CSGModel C = CSGModel::makeCylinder(0.25, 1.1);
	CSGModel D = CSGModel::makeCylinder(0.25, 1.1).rotate(0, 90*Deg2Rad, 0);
	CSGModel E = CSGModel::makeCylinder(0.25, 1.1).rotate(0, 0, 90*Deg2Rad);
	//CSGModel F = CSGModel::makeSphere(0.25).translate(0.5, 0.5, 0.5);
	CSGModel P = CSGModel::makePlane(Vector3D<>(0, 0, 0.1), Vector3D<>(1, 1, 1));
	CSGModel W = CSGModel::makeWedge(15*Deg2Rad);
	
	CSGModel X = A * AA;
	//X += B;
	X -= (C + D + E);
	X -= W;
	
	cout << X << endl;
	
	X.saveToStl("result.stl");
	
	cout << "Done." << endl;
	
	return 0;
}
