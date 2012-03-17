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

#include <rwsim/util/SurfacePoseSampler.hpp>

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


/*
class SampleGraspTasks {
public:

    SampleGraspTasks(PoseSampler::Ptr )


    void sample(rwlibs::task::CartesianTask& task){

    }

};
*/

int main(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );

    if( argc < 4 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of stl file" << std::endl;
	    std::cout << "- Arg 2 SCUP, PG70, PG70_SMALL, SDH_BALL, SDH_PAR, SDH_CYL, GS20, GS20_WIDE\n" << std::endl;
	    std::cout << "- Arg 3 name of output xml file\n" << std::endl;
	    return 0;
	}

	std::string filename(argv[1]);
	std::string type(argv[2]);
	std::string outfile(argv[3]);
	Geometry::Ptr geo;

	try{
        geo = GeometryFactory::getGeometry(filename);
	} catch (...){
	    RW_WARN("No such file: \""<< filename << "\"");
	    return 0;
	}

	SurfacePoseSampler ssurf( geo );
	ssurf.setRandomRotationEnabled(false);

	// these should be the object transformation
    Vector3D<> pos(0, 0, 0);
    Rotation3D<> rot(1, 0, 0,
                     0, 1, 0,
                     0, 0, 1);

    rwlibs::task::CartesianTask tasks;
    // first set up the configuration
    Vector3D<> d(0,0,-0.02);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    Q openQ(1,0.0);
    Q closeQ(1,1.0);
    if( type=="PG70" ){
        openQ  = Q(1, 0.034);
        closeQ = Q(1, 0.0);
        tasks.getPropertyMap().set<std::string>("TCP","TCPPG70");
    } else if( type== "PG70_SMALL"){
        openQ  = Q(1, 0.01);
        closeQ = Q(1, 0.0);
        tasks.getPropertyMap().set<std::string>("TCP","TCPPG70");
    } else if( type== "GS20"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        tasks.getPropertyMap().set<std::string>("TCP","TCPGS20");
    } else if( type== "GS20_WIDE"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
        tasks.getPropertyMap().set<std::string>("TCP","TCPGS20");
    } else if( type== "SDH_PAR"){
        openQ = Q(7, -1.571,-1.571,1.571, -1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7,-1.571,-1.571,1.571,0,0.419,0,0.419);
        tasks.getPropertyMap().set<std::string>("TCP","SDHTCP");
    } else if( type== "SDH_BALL"){
        openQ = Q(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 1.047,0.0, 0.349,0, 0.349);
        tasks.getPropertyMap().set<std::string>("TCP","SDHTCP");
    } else if( type== "SDH_CYL"){
        openQ = Q(7, -1.048, 0.174, 0.0, -1.048, 0.174,-1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 0.0, 0.0, 0.349,0, 0.349);
        tasks.getPropertyMap().set<std::string>("TCP","SDHTCP");
    } else if( type== "SCUP"){
        openQ  = Q(1, 0.0);
        closeQ = Q(1, 1.0);
        tasks.getPropertyMap().set<std::string>("TCP","EndFrame");
    } else {
        RW_THROW(" The gripper type is wrong! please specify a valid grippertype: (PG70, SCUP, SDH_PAR, SDH_CYL, SDH_BALL)");
    }
    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    tasks.getPropertyMap().set<std::string >("Gripper", type);
    tasks.getPropertyMap().set<Transform3D<> >("Offset", wTe_n);
    tasks.getPropertyMap().set<Transform3D<> >("Home", wTe_home);
    if( type== "SCUP"){
        tasks.getPropertyMap().set<Transform3D<> >("Approach", Transform3D<>(Vector3D<>(0,0,0.04)) );
        tasks.getPropertyMap().set<Transform3D<> >("Retract", Transform3D<>(Vector3D<>(0,0,-0.04)));
    } else {
        tasks.getPropertyMap().set<Transform3D<> >("Approach", Transform3D<>(Vector3D<>(0,0,0.0)) );
        tasks.getPropertyMap().set<Transform3D<> >("Retract", Transform3D<>(Vector3D<>(0,0,0.0)));
    }
    tasks.getPropertyMap().set<Q>("OpenQ", openQ);
    tasks.getPropertyMap().set<Q>("CloseQ", closeQ);


    if( type!="SCUP" ){
        ssurf.setBoundsD(0.001,0.05);
    } else if( type!="GS20" || type!="GS20_WIDE"){
        ssurf.setBoundsD(-0.03,0.03);
    } else {
        ssurf.setBoundsD(-0.05,0.05);
    }

	// now we choose a random number in the total area
	const int NR_OF_SAMPLES = 10000;
	for(int i=0; i<NR_OF_SAMPLES; i++){
	    Transform3D<> target = ssurf.sample();
        CartesianTarget::Ptr ctarget = ownedPtr( new CartesianTarget(target) );
        tasks.addTarget( ctarget );
	}

    try {
        XMLTaskSaver saver;
        saver.save(&tasks, outfile);
    } catch (const Exception& exp) {
       // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
    }



	return 0;
}
