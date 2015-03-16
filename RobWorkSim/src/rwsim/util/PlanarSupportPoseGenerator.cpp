#include "PlanarSupportPoseGenerator.hpp"

#include <rw/geometry/QHull3D.hpp>
//#include <rw/geometry/IncrementalHull.hpp>
#include <rw/common/Ptr.hpp>
#include <boost/foreach.hpp>
#include <rw/geometry/GeometryUtil.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>

#include <rw/math/Math.hpp>
#include <rw/geometry/Plane.hpp>
#include <rw/geometry/IntersectUtil.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include <rw/common/Timer.hpp>

using namespace rwsim::util;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rw::loaders;

namespace {

    /**
     * @brief a datastructure for representating a triangle mesh where triangle neighborhood searching
     * is efficient.
     *
     * A neighbor of triangle \b t is a triangle that share at least one vertice with the triangle \b t.
     *
     * A mapping from each vertice id to the triangle ids are maintained to allow efficient neigh search
     *
     * @note make sure the indexed trimesh does not have vertices that are overlapping (have the same position)
     */
    class TriMeshNeighbohrhood {
    public:

        TriMeshNeighbohrhood(rw::geometry::IndexedTriMesh<>::Ptr mesh):
            _mesh(mesh),
            _verticeIdToTriIdMap(mesh->getVertices().size()+mesh->size()*3, 0), // 3 for each triangle and 1 for each vertice (sizE)
            _verticeIdIdxMap(mesh->getVertices().size(),0)
       {

            // first we run through all triangles to figure out the nr of triangles per vertice
            //int startIdx = (int)(_verticeIdToTriIdMap.size()-mesh->getVertices().size());
            for(size_t i=0;i<_mesh->size();i++){
                IndexedTriangle<uint32_t> tri = _mesh->getIndexedTriangle( i );
                _verticeIdToTriIdMap[ tri[0] ]++;
                _verticeIdToTriIdMap[ tri[1] ]++;
                _verticeIdToTriIdMap[ tri[2] ]++;
            }

            // now starting from the first index we make sure to allocate enough room
            int cIdx=0;
            for(size_t i=0;i<mesh->getVertices().size();i++){
                _verticeIdIdxMap[i] = cIdx;
                cIdx += _verticeIdToTriIdMap[cIdx]; // add the number of triangles for this vertice
            }
            std::vector<int> tmpsize(mesh->getVertices().size(),0);
            // and now in the end add all triangle indices to the _verticeIdToTriIdMap
            for(size_t i=0;i<_mesh->size();i++){
                IndexedTriangle<uint32_t> tri = _mesh->getIndexedTriangle( i );
                _verticeIdIdxMap[ tri[0]+1+tmpsize[tri[0]] ];
                tmpsize[tri[0]]++;
                _verticeIdIdxMap[ tri[1]+1+tmpsize[tri[1]] ];
                tmpsize[tri[1]]++;
                _verticeIdIdxMap[ tri[2]+1+tmpsize[tri[2]] ];
                tmpsize[tri[2]]++;
            }

            // in the end we validate that tmpsize and _verticeIdIdxMap is actually the same
            for(size_t i=0;i<mesh->getVertices().size();i++){
                if(_verticeIdIdxMap[i] != tmpsize[i])
                    RW_WARN("Size does not match [" << i << ",{" <<_verticeIdIdxMap[i]<< ","<< tmpsize[i]<<"}]");
            }
        }

        /**
         * @brief returns all neighborhs to triangle triid
         * @param triid
         * @return
         */
        std::vector<int> getTriNeighbors(int triid){
            std::vector<int> neighs;
            IndexedTriangle<uint32_t> tri = _mesh->getIndexedTriangle( triid );
            int size;
            size = _verticeIdToTriIdMap[_verticeIdIdxMap[ tri[0]] ];
            for(int i=0;i<size;i++)
                neighs.push_back(_verticeIdToTriIdMap[_verticeIdIdxMap[ tri[0]]+1+i ]);
            size = _verticeIdToTriIdMap[_verticeIdIdxMap[ tri[1]] ];
            for(int i=0;i<size;i++)
                neighs.push_back(_verticeIdToTriIdMap[_verticeIdIdxMap[ tri[1]]+1+i ]);
            size = _verticeIdToTriIdMap[_verticeIdIdxMap[ tri[2]] ];
            for(int i=0;i<size;i++)
                neighs.push_back(_verticeIdToTriIdMap[_verticeIdIdxMap[ tri[2]]+1+i ]);
            return neighs;
        }

        /**
         * @brief returns all neighborhs to vertice \b triveticeid of triangle \b triid
         * @param triid
         * @param triverticeid
         * @return
         */
        std::vector<int> getTriVerticeNeighbors(int triid, int triverticeid){
            std::vector<int> neighs;
            IndexedTriangle<uint32_t> tri = _mesh->getIndexedTriangle( triid );
            int size;
            size = _verticeIdToTriIdMap[_verticeIdIdxMap[ tri[triverticeid]] ];
            for(int i=0;i<size;i++)
                neighs.push_back(_verticeIdToTriIdMap[_verticeIdIdxMap[ tri[triverticeid]]+1+i ]);
            return neighs;
        }

        /**
         * @brief returns all neighborhs to the triangle edge containing the tri vertices
         * \b triverticeid and \b triverticeid2
         * @param triid
         * @param triverticeid
         * @param triverticeid2
         * @return
         */
        std::vector<int> getTriEdgeNeighbors(int triid, int triverticeid, int triverticeid2){
            std::vector<int> neighs;
            IndexedTriangle<uint32_t> tri = _mesh->getIndexedTriangle( triid );
            int size;
            size = _verticeIdToTriIdMap[_verticeIdIdxMap[ tri[triverticeid]] ];
            for(int i=0;i<size;i++){
                IndexedTriangle<uint32_t> tri2 = _mesh->getIndexedTriangle( _verticeIdIdxMap[ tri[triverticeid]]+1+i );
                // check if tri2 has vertice triverticeid2
                bool hasVertice = false;
                hasVertice |= tri[triverticeid2] == tri2[0];
                hasVertice |= tri[triverticeid2] == tri2[1];
                hasVertice |= tri[triverticeid2] == tri2[2];
                if(hasVertice)
                    neighs.push_back(_verticeIdToTriIdMap[_verticeIdIdxMap[ tri[triverticeid]]+1+i ]);
            }
            return neighs;
        }

    private:
        rw::geometry::IndexedTriMesh<>::Ptr _mesh;
        std::vector<int> _verticeIdToTriIdMap;
        std::vector<int> _verticeIdIdxMap;

    };

}



PlanarSupportPoseGenerator::PlanarSupportPoseGenerator():_hullGenerator(ownedPtr( new QHull3D() )){}

PlanarSupportPoseGenerator::PlanarSupportPoseGenerator(ConvexHull3D::Ptr hullGenerator):_hullGenerator(hullGenerator){}


void PlanarSupportPoseGenerator::analyze(const std::vector<rw::geometry::Geometry::Ptr>& bodies, rw::kinematics::Frame* ref, const rw::kinematics::State& state){
    // first the center of mass
    cleanup();
    Vector3D<> masscenter = GeometryUtil::estimateCOG(bodies, ref, state);
    _com = masscenter;
    // then we find the convex hull of all vertexes of all geoms
    std::vector<Vector3D<> > vertices;
    BOOST_FOREACH(Geometry::Ptr geom, bodies){
        GeometryData::Ptr geomdata = geom->getGeometryData();
        TriMesh::Ptr mesh = geomdata->getTriMesh(false);
        Transform3D<> t3d = geom->getTransform();
        std::cout << t3d << std::endl;
        if( dynamic_cast<IndexedTriMesh<>*>(mesh.get()) ){
            IndexedTriMesh<>* idxmesh = dynamic_cast<IndexedTriMesh<>*>(mesh.get());
            std::vector<Vector3D<> >& verts = idxmesh->getVertices();
            BOOST_FOREACH(Vector3D<>& v, verts){ vertices.push_back( t3d*v ); }
        } else {
            IndexedTriMesh<>::Ptr idxMesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<> >(*mesh,0.00001);
            std::vector<Vector3D<> > verts = idxMesh->getVertices();
            for(size_t i=0; i<verts.size(); i++){
                vertices.push_back(t3d*verts[i]);
            }
        }
    }

    std::cout << "Building hull: vertices:"<< vertices.size() << std::endl;
    Timer time;
    _hullGenerator->rebuild( vertices );
    double buildtime = time.getTime();
    std::cout << "hull build: "<< buildtime << "s" << std::endl;
    double atime = time.getTime();
    doAnalysis();
    std::cout << "analysis: "<< atime << "s" << std::endl;
}

void PlanarSupportPoseGenerator::analyze(const rw::geometry::TriMesh& mesh){
    cleanup();
	Vector3D<> masscenter = GeometryUtil::estimateCOG(mesh);
	_com = masscenter;

	IndexedTriMeshD::Ptr idxMesh;
	if( dynamic_cast<const IndexedTriMeshD*>(&mesh) ){
		idxMesh = (IndexedTriMeshD*)(&mesh); // we still own the object, and the constness.. well we discard that for the time being
	} else {
	    std::cout << "Building indexed tri mesh..." << std::endl;

	    idxMesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0D>(mesh,0.00001);
		std::cout << "Build indexed tri mesh..." << std::endl;
	}

	_hullGenerator->rebuild( idxMesh->getVertices() );
	doAnalysis();
}



void PlanarSupportPoseGenerator::doAnalysis(){

	PlainTriMesh<TriangleN1<> >::Ptr fmesh = _hullGenerator->toTriMesh();
	IndexedTriMeshN0<>::Ptr imesh = TriangleUtil::toIndexedTriMesh<IndexedTriMeshN0<> >(*fmesh);

	STLFile::save(*fmesh,"hull_mesh.stl");
    // now project the center of mass onto all triangles in the trimesh
    // If it is inside a triangle then the triangle is a stable pose

	std::vector<std::pair<bool,int> > stableList(fmesh->size(), std::pair<bool,int>(false,-1) );
    std::vector<int> result;
    for(size_t i=0;i<fmesh->getSize();i++){
        if( (*fmesh)[i].isInside(_com) ){
            result.push_back((int)i);
            stableList[i].first = true;
        }
    }

    BOOST_FOREACH(int triIdx, result){
        TriangleN1<>& tri = (*fmesh)[triIdx];
        Plane p(tri[0],tri[1],tri[2]);

        bool hasPlane = false;

        for(size_t i =0; i<_supportPlanes.size(); i++){
            Plane &psup = _supportPlanes[i];
            if( fabs(psup.d()-p.d())<0.001 ){
                if( MetricUtil::dist2(psup.normal(),p.normal())<0.001 ){
                    hasPlane = true;
                    stableList[triIdx].second = (int)i;
                    break;
                }
            }
        }
        if(!hasPlane){
            _supportPlanes.push_back(p);
            _supportTriangles.push_back(std::vector<TriangleN1<> >());
            stableList[triIdx].second = (int)_supportPlanes.size()-1;
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
                    stableList[i].first = true;
                    stableList[i].second = (int)j;
                }
            }
        }
    }

    // create the neighbor map
    TriMeshNeighbohrhood neigh(imesh);

    // now, all support planes has been found
    // next locate all non-stable triangles and find out into which stable plane
    // the object will slide from a non-stable triangle. We do this using a region growing/coloring aproach
    // where we follow the neighboring triangles
    int tmpColor = (int)result.size()+100;
    typedef std::pair<bool,int> StableData;
    for(size_t i=0;i<stableList.size();i++){
        StableData& st = stableList[i];
        if( !st.first && (st.second ==-1) ){
            // give this a new temporary color and start coloring the neighbors
            // which are located in the same direction of the center mass ray
            st.second = tmpColor;
            std::stack<int> group;
            group.push((int)i);

            while(!group.empty()){
                //int colored = group.top();
                group.pop();
                // now determine which neighbors to follow
                // we follow neighbors according to the projection of the mass center
                // assuming that the mass center is outside the triangle and that we have
                // counter clock wise vertice ordering then the positive cross product between an
                // edge (v1,v2) and the edge (v1,COM_proj) will indicate a neighbor to follow
                //Vector3D<> dir = imesh->getVertex(i, V2)-imesh->getVertex(i, V1);
                //Vector3D<> dir1 = imesh->getVertex(i, V2)-imesh->getVertex(i, V1);




            }
            tmpColor++;

        }
    }

    std::cout << "Now ";
    BOOST_FOREACH(Plane& p, _supportPlanes){
        double height = p.distance(_com);
    	SupportPose pose( -p.normal() , height);
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
        float theta = (float)(x * Pi*2.0); /* Rotation about the pole (Z).      */
        float phi   = (float)(y * Pi*2.0); /* For direction of pole deflection. */
        float z     = bz * 2.0f;      /* For magnitude of pole deflection. */

        /* Compute a vector V used for distributing points over the sphere  */
        /* via the reflection I - V Transpose(V).  This formulation of V    */
        /* will guarantee that if x[1] and x[2] are uniformly distributed,  */
        /* the reflected points will be uniform on the sphere.  Note that V */
        /* has length sqrt(2) to eliminate the 2 in the Householder matrix. */

        float r  = sqrt( z );
        float Vx = sin( phi ) * r;
        float Vy = cos( phi ) * r;
        float Vz = sqrt( 2.0f - z );

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

void PlanarSupportPoseGenerator::calculateDistribution(int i, std::vector<rw::math::Transform3D<> >& poses, std::vector<rw::math::Transform3D<> >& posesMises){

    SupportPose supPose = _supportPoses[i];

    Plane supPlane = _supportPlanes[i];

    // translate the cooridnate system such that COM is in the origin and that triSup is defined relative to this
    RW_ASSERT( isInside(_com, i) );

/*
    Vector3D<> zb = normalize( supPlane.normal() );
    Vector3D<> yb = normalize( cross(Vector3D<>(0,0,1),zb) );
    Vector3D<> xb = normalize( cross(yb,zb) );
    Vector3D<> xa(1,0,0);
    Vector3D<> ya(0,1,0);
    Vector3D<> za(0,0,1);
    Rotation3D<> R(xb,yb,zb);
*/

    EAA<> eaa(supPlane.normal(),Vector3D<>(0,0,1));
    Rotation3D<> R = eaa.toRotation3D();
    //Rotation3D<> rnew = inverse(R)*inverse();

    // find the projection vector from COM(which is now (0,0,0)) to the triangle
    Vector3D<> res;
    //RW_ASSERT( IntersectUtil::intersetPtRayPlane(_com, supPlane.normal(), supPlane, res) );
    //RW_ASSERT( isInside(res, i) );
    // randomly generate a new pose and apply it to the vector v. If v intersects the triangle then add it to the distribution



    double d = 100*Deg2Rad;//Math::ran(0.0,0.01);
    for(int sample=0;sample<20000;sample++){
        //Rotation3D<> rot = rand_rotation(d,1.0,d);
        //Rotation3D<> rot = RPY<>(Math::ran(-d,d),Math::ran(-d,d),Math::ran(-d,d)).toRotation3D();
        Rotation3D<> rot = RPY<>(0,Math::ran(-d,d),Math::ran(-d,d)).toRotation3D()*R;
        Vector3D<> rotV = rot*supPlane.normal();
        Vector3D<> ray = inverse(R)*rotV;

//        Vector3D<> rotV = RPY<>(0,Math::ran(-d,d),Math::ran(-d,d)).toRotation3D()*Vector3D<>(0,0,1);
//        Vector3D<> ray = inverse(R)*rotV;

        Vector3D<> result;
        if( IntersectUtil::intersetPtRayPlane(_com, ray, supPlane, result) ){
            if( isInside(result, i) ){
                poses.push_back( Transform3D<>( _com, rot*inverse(R) ) );
                continue;
            }
        }
        posesMises.push_back( Transform3D<>( _com, rot*inverse(R) ) );


    }
    std::cout << "poses within: " <<  poses.size() << "sup: " << i<< std::endl;

}

std::vector<SupportPose> PlanarSupportPoseGenerator::getSupportPoses(){
	return _supportPoses;
}
