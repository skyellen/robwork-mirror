/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <vector>

#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rw/geometry/GiftWrapHull3D.hpp>

#include <rw/graspplanning/GraspTable.hpp>
#include <rw/common/Log.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::graspplanning;

using namespace rwsim::dynamics;

using namespace boost::numeric::ublas;

int main(int argc, char** argv)
{
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

	GraspTable *gtable = GraspTable::load(filename);
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
