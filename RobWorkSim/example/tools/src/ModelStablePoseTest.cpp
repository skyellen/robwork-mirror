/*
 * ModelPropertyTest.cpp
 *
 *  Created on: 14-05-2009
 *      Author: jimali
 */

#include <vector>

#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/QHull3D.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <boost/foreach.hpp>

using namespace rw::geometry;
using namespace rw::loaders;
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
			  << "- Body orientation  : " << res.first << "\n";

	std::cout << "- Calculating invariant axes: \n";

	// now create the convex hull of the geometry
	TriMesh::Ptr mesh = geo->getGeometryData()->getTriMesh();
	//IndexedTriMesh<float> *idxMesh = dynamic_cast<IndexedTriMesh<float>* >(mesh);
	IndexedTriMesh<>::Ptr idxMesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<> >(*mesh,0.00001);
	RW_ASSERT(idxMesh);

	std::cout << "- nr vertices: " << idxMesh->getVertices().size() << std::endl;

	QHull3D hull;
    hull.rebuild( idxMesh->getVertices() );

    PlainTriMesh<TriangleN1<> >::Ptr fmesh = hull.toTriMesh();
    //std::cout << "SIZE of mesh: " << fmesh->size() << std::endl;
    // now project the center of mass onto all triangles in the trimesh
    // If it is inside a triangle then the triangle is a stable pose
    std::vector<TriangleN1<> > result;
    for(size_t i=0;i<fmesh->getSize();i++){
        std::cout << (*fmesh)[i].getVertex(0) << std::endl;
        std::cout << (*fmesh)[i].getVertex(1) << std::endl;
        std::cout << (*fmesh)[i].getVertex(2) << std::endl;
        if( (*fmesh)[i].isInside(masscenter) )
            result.push_back((*fmesh)[i]);
    }
    //std::cout << "NR of result : " << result.size() << std::endl;
    std::vector<TriangleN1<> > result2;
    BOOST_FOREACH(TriangleN1<>& tri, result){
        Vector3D<> n = tri.getFaceNormal();
        bool hasNormal = false;
        BOOST_FOREACH(TriangleN1<> &t, result2){
            if( MetricUtil::dist2(n, t.getFaceNormal())<0.1 ){
                hasNormal=true;
                break;
            }
        }
        if(!hasNormal)
            result2.push_back(tri);
    }

    BOOST_FOREACH(TriangleN1<>& tri, result2){
        // calculate distance from cog to face
        // this is the height to place the object in
        double z = tri.halfSpaceDist( masscenter );
        // calculate the orientation of the object
        EAA<> rot(-tri.getFaceNormal(), Vector3D<>::z());
        RPY<> rpy(rot.toRotation3D());
        std::cout << "pos: " << Vector3D<>(0,0,z) << std::endl;
        std::cout << "rpy: " << rpy << std::endl;

        //std::cout << "- " << -tri.getFaceNormal() << std::endl;


    }
    std::cout << "------- Model properties END ----- \n";
    //std::cout << "write to file..." << std::endl;
    STLFile::save(*fmesh, "resultingHull.stl");
    STLFile::save(*idxMesh, "testSTL.stl");



	return 0;
}
