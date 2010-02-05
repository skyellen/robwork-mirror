/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <rw/geometry/Face.hpp>
#include <rw/geometry/FaceArrayFactory.hpp>
#include <vector>

#include <sandbox/geometry/STLFile.hpp>
#include <sandbox/geometry/Triangle.hpp>
#include <sandbox/geometry/PlainTriMesh.hpp>
#include <sandbox/geometry/TriangleUtil.hpp>
#include <sandbox/geometry/GeometryFactory.hpp>

#include <dynamics/ContactPoint.hpp>
#include <dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <dynamics/DynamicUtil.hpp>

#include <dynamics/ContactManifold.hpp>
#include <sandbox/geometry/GeometryFactory.hpp>

using namespace rw::math;
using namespace boost::numeric;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::geometry::sandbox;
using namespace dynamics;

using namespace boost::numeric::ublas;

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

	Geometry *geo = GeometryFactory::getGeometry(filename);
	std::vector<Geometry*> geoms;
	geoms.push_back(geo);

	Vector3D<> masscenter = DynamicUtil::estimateCOG(mass, geoms);
	InertiaMatrix<> inertia = DynamicUtil::estimateInertia(mass, geoms);

	typedef std::pair<matrix<double>, vector<double> > Result;
	Result res = LinearAlgebra::eigenDecompositionSymmetric( inertia.m() );

	std::cout << "------- Model properties ----- \n"
			  << "- COG     : " << masscenter << "\n"
			  << "- Inertia : " << inertia << "\n";

	std::cout << "- inertia on pricipal from: \n"
			  << "- inertia          : " << res.second << "\n"
			  << "- Body orientation : " << res.first << "\n";

	delete geo;

	return 0;
}
