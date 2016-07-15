/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <rw/common/Log.hpp>
#include <rw/graspplanning/GraspTable.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Random.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::common;
using namespace rw::graspplanning;
using namespace rw::math;

int main(int argc, char** argv)
{
	const double EPSILON = 0.000001;
	for(int i=0;i<1000000;i++){
		//generate random vector
		Vector3D<> v(Random::ran(0,1),Random::ran(0,1),Random::ran(0,1));
		if(MetricUtil::norm2(v)<0.001)
			continue;

		Vector3D<> normal = normalize(v);
		// create perpendicular vector
        Vector3D<> tdir;
        if( fabs(normal(0))<EPSILON && fabs(normal(1))<EPSILON ){
            tdir = normalize(  Vector3D<>(0,-normal(2),normal(1)) );
        } else {
            tdir = normalize(  Vector3D<>(-normal(1),normal(0),0) );
        }
        //std::cout << "Angle: " << (angle(tdir,normal)*Rad2Deg) << std::endl;
        double ang = 90-angle(tdir,normal)*Rad2Deg;
        if(fabs(ang)>2)
        	std::cout << "Angle: " << (angle(tdir,normal)*Rad2Deg) << std::endl;
	}
	exit(0);






	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of grasp table" << std::endl;
	    std::cout << "- Arg 2 size of group\n" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);
	int groupSize = 1;
	if(argc>2)
	    groupSize = std::atoi(argv[2]);

	Ptr<GraspTable> gtable = GraspTable::load(filename);
	RW_ASSERT(gtable!=NULL);
	Log::infoLog() << "Table size: " << gtable->size() << std::endl;
	int nrOfGroups = gtable->size()/groupSize;
	int globalStat = 0;
	for(int i=0;i<nrOfGroups; i++){
		GraspTable::GraspData &ngrasp = gtable->getData()[i*groupSize];
		int localStat = 0;
		for(int j=0;j<groupSize-1;j++){
			GraspTable::GraspData &grasp = gtable->getData()[i*groupSize+1+j];
			// test if any of the angles are too far away
			EAA<> eaa1 = grasp.hp.getEAA();
			EAA<> eaa2 = ngrasp.hp.getEAA();
			Vector3D<> v1(eaa1(0),eaa1(1),eaa1(2));
			Vector3D<> v2(eaa2(0),eaa2(1),eaa2(2));
			if( MetricUtil::dist2(v1,v2)>0.5 ){
				grasp = ngrasp;
				localStat++;
				continue;
			}
		}
		globalStat += localStat;
		std::cout << "Stat: " << localStat << "/" << groupSize << std::endl;
	}
	std::cout << "Stat: " << globalStat << "/" << groupSize*nrOfGroups << std::endl;
	gtable->save("grasptable_out.txt");
	return 0;
}
