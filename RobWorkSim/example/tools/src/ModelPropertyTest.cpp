/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <vector>

#include <rw/geometry/GeometryUtil.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rw::geometry;
using rw::loaders::GeometryFactory;
using namespace rw::math;

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

	typedef std::pair<Eigen::MatrixXd, Eigen::VectorXd> Result;
	Result res = LinearAlgebra::eigenDecompositionSymmetric( Eigen::MatrixXd(inertia.e()) );

	std::cout << "------- Model properties ----- \n"
			  << "- COG     : " << masscenter << "\n"
			  << "- Inertia : " << inertia << "\n";

	std::cout << "- inertia on pricipal from: \n"
			  << "- inertia          : " << res.second << "\n"
			  << "- Body orientation : " << res.first << "\n";

	return 0;
}
