#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>
#include <vector>

#include <rw/rw.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rwlibs/task.hpp>

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
    Math::seed(time(NULL));
    srand ( time(NULL) );

    if( argc < 4 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of stl file" << std::endl;
	    std::cout << "- Arg 2 name of output xml file\n" << std::endl;
	    return 0;
	}

	std::string filename(argv[1]);
	std::string outfile(argv[2]);
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
	std::string type = "PG70";
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


    CollisionStrategy::Ptr cstrategy = ProximityStrategyFactory::makeDefaultCollisionStrategy();
    PlainTriMeshF *rayMesh = new PlainTriMeshF(1);
    (*rayMesh)[0] = Triangle<float>( Vector3D<float>(0,(float)-0.001,0),Vector3D<float>(0,(float)0.001,0),Vector3D<float>((float)10,0,0) );

    ProximityModel::Ptr ray = cstrategy->createModel();
    Geometry geom( rayMesh ); // we have to wrap the trimesh in an geom object
    geom.setId("Ray");
    ray->addGeometry(geom);

    // also add the stl object
    ProximityModel::Ptr object = cstrategy->createModel();
    cstrategy->addGeometry(object.get(), geo);
    ProximityStrategyData data;
    data.setCollisionQueryType( CollisionStrategy::AllContacts );

	// now we choose a random number in the total area
	const int NR_OF_SAMPLES = 10000;
	for(int i=0; i<NR_OF_SAMPLES; i++){
	    bool targetFound = false;
	    Transform3D<> target;
	    do {
            double rnum = Math::ran(0.0, sAreaSum);
            int triIds = binSearchRec(rnum, surfaceArea, 0, mesh->size()-1);
            Triangle<> tri = mesh->getTriangle(triIds);

            // random sample the triangle
            double b0 = Math::ran();
            double b1 = ( 1.0f - b0 ) * Math::ran();
            double b2 = 1 - b0 - b1;

            Vector3D<> pos = tri[0] * b0 + tri[1] * b1 + tri[2] * b2;
            Vector3D<> faceNormal = normalize( tri.calcFaceNormal() );
            // create orientation that point in the -z-axis direction
            Vector3D<> tanV = pos-tri[0];
            if(tanV.norm2()<0.000001)
                tanV = pos-tri[1];
            tanV = normalize(tanV);

            Rotation3D<> rot(cross(tanV,-faceNormal), tanV, -faceNormal);
            Transform3D<> rayTrans( pos, rot );
            // now we want to find
            cstrategy->inCollision(object,Transform3D<>::identity(), ray, rayTrans, data);

            typedef std::pair<int,int> PrimID;
            //BOOST_FOREACH( data.getCollisionData()._geomPrimIds


	    } while( !targetFound );
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
