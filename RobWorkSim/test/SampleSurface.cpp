#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>

#include <vector>

#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <rwsim/dynamics/DynamicUtil.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rw/geometry/GeometryFactory.hpp>
USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace rwsim::dynamics;

using namespace boost::numeric::ublas;

int binSearchRec(const double value, std::vector<double>& surfaceArea, size_t start, size_t end){
    if(start==end)
        return start;
    // choose a int between start and end
    size_t split = (end-start)/2+start;

    if(value<surfaceArea[split])
        return binSearchRec(value, surfaceArea, start, split);
    else
        return binSearchRec(value, surfaceArea, split+1, end);
}

int main(int argc, char** argv)
{
	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of stl, ac3d or 3ds file" << std::endl;
	    std::cout << "- Arg 2 mass of model in kg\n" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);
	double mass = 1.0;
	if(argc>2)
	    mass = std::atof(argv[2]);

	Geometry::Ptr geo = GeometryFactory::getGeometry(filename);
	std::vector<Geometry::Ptr> geoms;
	geoms.push_back(geo);

	Vector3D<> masscenter = GeometryUtil::estimateCOG(geoms);
	InertiaMatrix<> inertia = GeometryUtil::estimateInertia(mass, geoms);

	std::cout << "------- Model properties ----- \n"
			  << "- COG     : " << masscenter << "\n"
			  << "- Inertia : " << inertia << "\n";

	TriMesh::Ptr mesh = geo->getGeometryData()->getTriMesh();
	std::vector<double> surfaceArea(mesh->size());
	double sAreaSum = 0;
	// run through mesh and create the search
	for(size_t i=0; i<mesh->size(); i++){
	    Triangle<> tri = mesh->getTriangle(i);
	    // calculate triangle area
	    sAreaSum += tri.calcArea();
	    surfaceArea[i] = sAreaSum;
	}

	// these should be the object transformation
    Vector3D<> pos(0, 0, 0);
    Rotation3D<> rot(1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);

    rwlibs::task::CartesianTask tasks;
    // first set up the configuration
    Vector3D<> d(0,0,-0.10);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    Q openQ(1,0.0);
    Q closeQ(1,1.0);

    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    tasks.getPropertyMap().set<Transform3D<> >("Nominal", wTe_n);
    tasks.getPropertyMap().set<Transform3D<> >("Home", wTe_home);
    tasks.getPropertyMap().set<Vector3D<> >("Approach", Vector3D<>(0,0,0.02));
    tasks.getPropertyMap().set<Q>("OpenQ", openQ);
    tasks.getPropertyMap().set<Q>("CloseQ", closeQ);

	// now we choose a random number in the total area
	const int NR_OF_SAMPLES = 10000;
	for(int i=0; i<NR_OF_SAMPLES; i++){
	    double rnum = Math::ran(0.0, sAreaSum);
	    int triIds = binSearchRec(rnum, surfaceArea, 0, mesh->size()-1);
	    Triangle<> tri = mesh->getTriangle(triIds);

	    // random sample the triangle
	    double b0 = Math::ran();
	    double b1 = ( 1.0f - b0 ) * Math::ran();
	    double b2 = 1 - b0 - b1;

	    Vector3D<> position = tri[0] * b0 + tri[1] * b1 + tri[2] * b2;
	    // add some randomeness to the position
	    const double RANDIST = 0.02;
	    position += Vector3D<>(Math::ran(-RANDIST,RANDIST),Math::ran(-RANDIST,RANDIST),Math::ran(-RANDIST,RANDIST));

	    // and sample the orientation
        Transform3D<> target( position, Math::ranRotation3D<double>());
        CartesianTarget::Ptr ctarget = ownedPtr( new CartesianTarget(target) );
        tasks.addTarget( ctarget );
	}

    try {
        XMLTaskSaver saver;
        saver.save(&tasks, "SuctionCupTaskFile.xml");
    } catch (const Exception& exp) {
       // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
    }



	return 0;
}
