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

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <rw/geometry/GeometryUtil.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rw/common/Log.hpp>

#include <sandbox/loaders/RWXMLFile.hpp>

#include <rw/loaders.hpp>
#include <rw/math.hpp>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/geometry.hpp>

using namespace boost::numeric;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::models;



int main(int argc, char** argv)
{
	if( argc < 3 ){
		std::cout << "------ Usage: " << std::endl;
	    std::cout << "- Arg 1 name of workcell input" << std::endl;
	    std::cout << "- Arg 2 name of workcell ouput" << std::endl;
	    return 0;
	}
	std::string filename(argv[1]);
	std::string outname(argv[2]);

	Log::infoLog() << "Loading workcell" << std::endl;
	WorkCellPtr wc = WorkCellLoader::load(filename);
	Log::infoLog() << "workcell loadet" << std::endl;

	Log::infoLog() << "saving workcell" << std::endl;


	RWXMLFile::saveWorkCell(*wc, wc->getDefaultState(), outname);
	Log::infoLog() << "workcell saved.." << std::endl;
	return 0;
}
