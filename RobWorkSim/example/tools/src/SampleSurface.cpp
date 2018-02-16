#include <vector>
#include <string>

#include <rw/math/Math.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/TaskSaver.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::geometry;
using rw::loaders::GeometryFactory;
using namespace rw::math;
using namespace rwlibs::task;

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
    Vector3D<> d(0,0,-0.02);
    Transform3D<> wTe_n(pos, rot);
    Transform3D<> wTe_home(pos+inverse(rot)*d, rot);
    Q openQ(1,0.0);
    Q closeQ(1,1.0);
    if( type=="PG70" ){
        openQ  = Q(1, 0.034);
        closeQ = Q(1, 0.0);
    } else if( type== "PG70_SMALL"){
        openQ  = Q(1, 0.01);
        closeQ = Q(1, 0.0);
    } else if( type== "GS20"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
    } else if( type== "GS20_WIDE"){
        openQ  = Q(1, 0.005);
        closeQ = Q(1, 0.0);
    } else if( type== "SDH_PAR"){
        openQ = Q(7, -1.571,-1.571,1.571, -1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7,-1.571,-1.571,1.571,0,0.419,0,0.419);
    } else if( type== "SDH_BALL"){
        openQ = Q(7,-1.048, 0.174, 1.047 ,-1.048, 0.174, -1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 1.047,0.0, 0.349,0, 0.349);
    } else if( type== "SDH_CYL"){
        openQ = Q(7, -1.048, 0.174, 0.0, -1.048, 0.174,-1.048, 0.174);
        closeQ = Q(7, 0.0, 0.349, 0.0, 0.0, 0.349,0, 0.349);
    } else if( type== "SCUP"){
        openQ  = Q(1, 0.0);
        closeQ = Q(1, 1.0);
    } else {
        RW_THROW(" The gripper type is wrong! please specify a valid grippertype: (PG70, SCUP, SDH_PAR, SDH_CYL, SDH_BALL)");
    }
    //wTe_n = Transform3D<>::identity();
    //wTe_home = Transform3D<>::identity();
    tasks.getPropertyMap().set<std::string >("Gripper", type);
    tasks.getPropertyMap().set<Transform3D<> >("Nominal", wTe_n);
    tasks.getPropertyMap().set<Transform3D<> >("Home", wTe_home);
    if( type== "SCUP"){
        tasks.getPropertyMap().set<Vector3D<> >("Approach", Vector3D<>(0,0,0.04));
    } else {
        tasks.getPropertyMap().set<Vector3D<> >("Approach", Vector3D<>(0,0,0));
    }
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
	    //Unused: const double RANDIST = 0.02;
	    //position += Vector3D<>(Math::ran(-RANDIST,RANDIST),Math::ran(-RANDIST,RANDIST),Math::ran(-RANDIST,RANDIST));

	    // and sample the orientation
	    EAA<> eaa(Vector3D<>::z(), -tri.calcFaceNormal());
        Transform3D<> target( position, Math::ranRotation3D<double>());
        //Transform3D<> target( position, eaa.toRotation3D());
        if( type!="SCUP" ){
            target.P() -= (target.R()*Vector3D<>::z())*Math::ran(0.001,0.05);
        } else if( type!="GS20" || type!="GS20_WIDE"){
            target.P() -= (target.R()*Vector3D<>::z())*Math::ran(-0.03,0.03);
        } else {
            target.P() -= (target.R()*Vector3D<>::z())*Math::ran(-0.05,0.05);

        }

        CartesianTarget::Ptr ctarget = ownedPtr( new CartesianTarget(target) );
        tasks.addTarget( ctarget );
	}

    try {
        const TaskSaver::Ptr saver = TaskSaver::Factory::getTaskSaver("xml");
        saver->save(&tasks, outfile);
    } catch (const Exception& exp) {
       // QMessageBox::information(this, "Task Execution Widget", "Unable to save tasks");
    }



	return 0;
}
