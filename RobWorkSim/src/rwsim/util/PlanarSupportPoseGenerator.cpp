#include "PlanarSupportPoseGenerator.hpp"

#include <rw/geometry/GiftWrapHull3D.hpp>
#include <rw/common/Ptr.hpp>
#include <boost/foreach.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>

#include <rw/math/Math.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/IntersectUtil.hpp>


using namespace rwsim::util;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;

PlanarSupportPoseGenerator::PlanarSupportPoseGenerator():_hullGenerator(ownedPtr( new GiftWrapHull3D() )){}

PlanarSupportPoseGenerator::PlanarSupportPoseGenerator(ConvexHull3DPtr hullGenerator):_hullGenerator(hullGenerator){}


void PlanarSupportPoseGenerator::analyze(const std::vector<rw::geometry::GeometryPtr>& bodies){
    // first the center of mass
    cleanup();
    Vector3D<> masscenter = GeometryUtil::estimateCOG(bodies);
    _com = masscenter;
    // then we find the convex hull of all vertexes of all geoms
    std::vector<Vector3D<> > vertices;
    BOOST_FOREACH(GeometryPtr geom, bodies){
        GeometryDataPtr geomdata = geom->getGeometryData();
        TriMeshPtr mesh = geomdata->getTriMesh(false);
        Transform3D<> t3d = geom->getTransform();
        if( dynamic_cast<IndexedTriMesh<>*>(mesh.get()) ){
            IndexedTriMesh<>* idxmesh = dynamic_cast<IndexedTriMesh<>*>(mesh.get());
            std::vector<Vector3D<> >& verts = idxmesh->getVertices();
            BOOST_FOREACH(Vector3D<>& v, verts){ vertices.push_back( t3d*v ); }
        } else {
            for(size_t i=0; i<mesh->getSize(); i++){
                Triangle<> t = mesh->getTriangle(i);
                vertices.push_back(t3d*t[0]);
                vertices.push_back(t3d*t[1]);
                vertices.push_back(t3d*t[2]);
            }
        }
    }

    _hullGenerator->rebuild( vertices );
    doAnalysis();

}

void PlanarSupportPoseGenerator::analyze(const rw::geometry::TriMesh& mesh){
    cleanup();
	Vector3D<> masscenter = GeometryUtil::estimateCOG(mesh);
	_com = masscenter;

	IndexedTriMeshDPtr idxMesh;
	if( dynamic_cast<const IndexedTriMeshD*>(&mesh) ){
		idxMesh = (IndexedTriMeshD*)(&mesh); // we still own the object, and the constness.. well we discard that for the time being
	} else {
		IndexedTriMeshD *nmesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0D>(mesh,0.00001);
		idxMesh = ownedPtr(nmesh);
	}

	_hullGenerator->rebuild( idxMesh->getVertices() );
	doAnalysis();
}
#include <rw/geometry/STLFile.hpp>
void PlanarSupportPoseGenerator::doAnalysis(){

	PlainTriMesh<TriangleN1<> > *fmesh = _hullGenerator->toTriMesh();

	STLFile::save(*fmesh,"hull_mesh.stl");
    // now project the center of mass onto all triangles in the trimesh
    // If it is inside a triangle then the triangle is a stable pose
    std::vector<TriangleN1<> > result;
    for(size_t i=0;i<fmesh->getSize();i++){
        if( (*fmesh)[i].isInside(_com) )
            result.push_back((*fmesh)[i]);
    }

    BOOST_FOREACH(TriangleN1<>& tri, result){
        Plane p(tri[0],tri[1],tri[2]);

        bool hasPlane = false;

        for(size_t i =0; i<_supportPlanes.size(); i++){
            Plane &psup = _supportPlanes[i];
            if( fabs(psup.d()-p.d())<0.001 ){
                if( MetricUtil::dist2(psup.normal(),p.normal())<0.001 ){
                    hasPlane = true;
                    break;
                }
            }
        }
        if(!hasPlane){
            _supportPlanes.push_back(p);
            _supportTriangles.push_back(std::vector<TriangleN1<> >());
        }
    }
    std::cout << "NR OF PLANES: " << _supportPlanes.size() << std::endl;

    for(size_t i=0;i<fmesh->getSize();i++){
        TriangleN1<> tri = (*fmesh)[i];
        Plane p(tri[0],tri[1],tri[2]);

        for(size_t j =0; j<_supportPlanes.size(); j++){
            Plane &psup = _supportPlanes[j];
            if( fabs(psup.d()-p.d())<0.001 ){
                if( MetricUtil::dist2(psup.normal(),p.normal())<0.001 ){
                    _supportTriangles[j].push_back(tri);
                }
            }
        }
    }


    std::cout << "Now ";
    BOOST_FOREACH(Plane& p, _supportPlanes){
    	SupportPose pose( -p.normal() );
        std::cout << "- " << -p.normal() << std::endl;

        //Vector3D<> v = tri.getVertex(0);
        //double distance = dot(_com-v, n);
        pose._rotAxes[0] = p.normal();
        //pose._posAxes[0] = masscenter + (- p.normal() )*distance;
        pose._rotAxesTable[0] = Vector3D<>(0,0,1);
        _supportPoses.push_back(pose);
        //_supportTriangles.push_back(tri);

    }
}

namespace {
    Rotation3D<> rand_rotation( float x, float y, float bz)
    {
        float theta = x * Pi*2.0; /* Rotation about the pole (Z).      */
        float phi   = y * Pi*2.0; /* For direction of pole deflection. */
        float z     = bz * 2.0;      /* For magnitude of pole deflection. */

        /* Compute a vector V used for distributing points over the sphere  */
        /* via the reflection I - V Transpose(V).  This formulation of V    */
        /* will guarantee that if x[1] and x[2] are uniformly distributed,  */
        /* the reflected points will be uniform on the sphere.  Note that V */
        /* has length sqrt(2) to eliminate the 2 in the Householder matrix. */

        float r  = sqrt( z );
        float Vx = sin( phi ) * r;
        float Vy = cos( phi ) * r;
        float Vz = sqrt( 2.0 - z );

        /* Compute the row vector S = Transpose(V) * R, where R is a simple */
        /* rotation by theta about the z-axis.  No need to compute Sz since */
        /* it's just Vz.                                                    */

        float st = sin( theta );
        float ct = cos( theta );
        float Sx = Vx * ct - Vy * st;
        float Sy = Vx * st + Vy * ct;

        /* Construct the rotation matrix  ( V Transpose(V) - I ) R, which   */
        /* is equivalent to V S - R.                                        */
        Rotation3D<> M;
        M(0,0) = Vx * Sx - ct;
        M(0,1) = Vx * Sy - st;
        M(0,2) = Vx * Vz;

        M(1,0) = Vy * Sx + st;
        M(1,1) = Vy * Sy - ct;
        M(1,2) = Vy * Vz;

        M(2,0) = Vz * Sx;
        M(2,1) = Vz * Sy;
        M(2,2) = 1.0 - z;   /* This equals Vz * Vz - 1.0 */
        return M;
    }


}

bool PlanarSupportPoseGenerator::isInside(const rw::math::Vector3D<>& v, size_t i){
    for(size_t j=0;j<_supportTriangles[i].size();j++){
        if( _supportTriangles[i][j].isInside(v) ){
            return true;
        }
    }
    return false;
}

void PlanarSupportPoseGenerator::calculateDistribution(int i, std::vector<rw::math::Transform3D<> >& poses){
    std::cout << "a1:"<< i;
    SupportPose supPose = _supportPoses[i];
    std::cout << "a2:";
    Plane supPlane = _supportPlanes[i];
    std::cout << "a2:";
    // translate the cooridnate system such that COM is in the origin and that triSup is defined relative to this
    RW_ASSERT( isInside(_com, i) );

    // find the projection vector from COM(which is now (0,0,0)) to the triangle
    std::cout << "a3";
    Vector3D<> res;
    //RW_ASSERT( IntersectUtil::intersetPtRayPlane(_com, supPlane.normal(), supPlane, res) );
    std::cout << "a4";
    //RW_ASSERT( isInside(res, i) );
    std::cout << "a5";
    // randomly generate a new pose and apply it to the vector v. If v intersects the triangle then add it to the distribution
    double d = 40*Deg2Rad;//Math::ran(0.0,0.01);
    for(int sample=0;sample<5000;sample++){
        //Rotation3D<> rot = rand_rotation(d,1.0,d);
        Rotation3D<> rot = RPY<>(Math::ran(-d,d),Math::ran(-d,d),Math::ran(-d,d)).toRotation3D();

        Vector3D<> rotV = rot*supPlane.normal();
        Vector3D<> result;
        if( IntersectUtil::intersetPtRayPlane(_com, rotV, supPlane, result) )
            if( isInside(result, i) )
                poses.push_back( Transform3D<>(_com,rot) );
    }
    std::cout << "poses within: " <<  poses.size() << std::endl;

}

std::vector<SupportPose> PlanarSupportPoseGenerator::getSupportPoses(){
	return _supportPoses;
}
