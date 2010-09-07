#ifndef RW_GEOMETRY_INCREMENTALHULL_HPP_
#define RW_GEOMETRY_INCREMENTALHULL_HPP_

#include <rw/math/Vector3D.hpp>
#include <vector>
#include <list>
#include <stack>


#include "Triangle.hpp"
#include "IndexedTriangle.hpp"
#include "IndexedTriMesh.hpp"
#include "PlainTriMesh.hpp"

#include <boost/dynamic_bitset.hpp>
#include <boost/foreach.hpp>

namespace rw {
namespace geometry {
    /** @addtogroup geometry
     *  @{
     *  @file IncrementalHull.hpp
     */

    /**
     * @brief builds the convex hull using an incremental hull algorithm.
     */
    template <class T=double>
    class IncrementalHull {
    private:
        //const static double EPSILON = 0.0000001;
        typedef rw::geometry::IndexedTriangleN1<T> IdxTriN1;
        typedef rw::geometry::IndexedTriangle<T> IdxTriN0;
        typedef rw::geometry::IndexedTriMesh<IdxTriN1 > IdxTriMeshN1;

        typedef std::vector<rw::math::Vector3D<T> > VertexArray;

    public:

        template <class ARRAY>
        struct IndexedTriFace {
            typedef ARRAY VertexArray;
            IndexedTriFace():_visible(false){};
            IndexedTriFace(int v1i, int v2i, int v3i, const VertexArray& verts):_visible(false)
            {
                _v[0] = v1i; _v[1]=v2i; _v[2] = v3i;
                const rw::math::Vector3D<T> &v1=verts[v1i];
                const rw::math::Vector3D<T> &v2=verts[v2i];
                const rw::math::Vector3D<T> &v3=verts[v3i];
                //std::cout << v1 << v2 << v3 << std::endl;
                //_normal = cross((verts[_v[1]]-verts[_v[0]]),(verts[_v[2]]-verts[_v[0]]));
                _normal = cross( v2-v1, v3-v1);
                _normal = normalize(_normal);
                _d = dot(_normal, verts[_v[0]]);
                //std::cout << "Normal: " << _normal << " " << _d << std::endl;
            };

            virtual ~IndexedTriFace(){};

            T halfSpaceDist(const rw::math::Vector3D<T>& x){
                //std::cout<< "dot(_normal,x): " << dot(_normal,x) << " " << _d << std::endl;
                return dot(_normal,x) - _d;
            }

            bool isInside(const rw::math::Vector3D<T>& x, const VertexArray& verts){
                using namespace rw::math;
                // calc vectors
                Vector3D<T> v0 = verts[_v[2]] - verts[_v[0]];
                Vector3D<T> v1 = verts[_v[1]] - verts[_v[0]];
                Vector3D<T> v2 = x - verts[_v[0]];
                // calc dot products
                T dot00 = dot(v0, v0);
                T dot01 = dot(v0, v1);
                T dot02 = dot(v0, v2);
                T dot11 = dot(v1, v1);
                T dot12 = dot(v1, v2);
                // calc barycentric coordinates
                T invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
                T u = (dot11 * dot02 - dot01 * dot12) * invDenom;
                T v = (dot00 * dot12 - dot01 * dot02) * invDenom;

                // Check if point is in triangle
                return (u > 0) && (v > 0) && (u + v < 1);
            }

            void setVisible(bool visible){
                _visible = visible;
            }

            bool isVisible(){
                return _visible;
            }

            rw::geometry::Face<T> toFace(const VertexArray& verts){
                return rw::geometry::Face<T>(verts[_v[0]],verts[_v[1]],verts[_v[2]],_normal);
            }

            rw::geometry::TriangleN1<T> toTriangle(const VertexArray& verts){
                return rw::geometry::TriangleN1<T>(verts[_v[0]],verts[_v[1]],verts[_v[2]],_normal);
            }

            const rw::math::Vector3D<T>& getNormal(){
                return _normal;
            }

            int operator[](size_t i) { return _v[i]; }

            int _v[3];
            bool _visible;
            // halfspace variables
            rw::math::Vector3D<T> _normal;
            double _d;
        };

        struct TriFace {

            TriFace(){};

            TriFace(rw::math::Vector3D<>& v1,
                    rw::math::Vector3D<>& v2,
                    rw::math::Vector3D<>& v3)
            {
                _v[0] = &v1; _v[1]=&v2; _v[2] = &v3;
                _normal = cross((v2-v1),(v3-v1));
                _normal = normalize(_normal);
                _d = dot(_normal, v1);
            };

            bool isInsideHalfSpace(const rw::math::Vector3D<>& x){
                return dot(_normal,x) > _d;
            }

            rw::math::Vector3D<>& operator[](size_t i) { return *_v[i]; }


            // vertice references
            rw::math::Vector3D<>* _v[3];
            // halfspace variables
            rw::math::Vector3D<> _normal;
            double _d;
        };

        struct Edge {

        };

    /*
        static T halfSpaceDist(const rw::math::Vector3D<T>& x, IdxTri& tri,
                               VertexArray& vertices, VertexArray& normals)
        {
            using namespace rw::math;
            //std::cout<< "dot(_normal,x): " << dot(_normal,x) << " " << _d << std::endl;
            const Vector3D<T>& normal = normals[tri.getFaceNormalIdx()];
            T d = dot(normal, vertices[tri.getVertexIdx(0)]);
            return dot(normal,x) - d;
        }
    */

        IncrementalHull(){};

    //	IncrementalHull( FaceList *faces, std::vector<rw::math::Vector3D<T> >* vertices ):
    //		_faces(faces),_vertices(vertices){};

        virtual ~IncrementalHull(){};
    #ifdef BUMBUM
        /**
         * @brief test if the given vertex is inside the convex hull
         */
        bool isInside(const rw::math::Vector3D<T>& vertex){
            const static T EPSILON = 0.0000001;
            if( _faces->size()==0 )
                return 0;
            BOOST_FOREACH( IndexedTriFace &face, *_faces ){
                T dist = face.halfSpaceDist( vertex );
                if( dist < -EPSILON )
                    return false;
            }
            return true;
        }

        /**
         * @brief if the vertex is inside the convex hull the minimum distance
         * to any of the half-spaces of the hull is returned. If its not inside
         * 0 is returned.
         * @param vertex
         * @return
         */
        T getMinDist(const rw::math::Vector3D<T>& vertex){
            const static T EPSILON = 0.0000001;
            if( _faces->size()==0 )
                return 0;
            T minDist = _faces->front().halfSpaceDist( vertex );
            BOOST_FOREACH( IndexedTriFace &face, *_faces ){
                T dist = face.halfSpaceDist( vertex );
                if( dist < -EPSILON )
                    return 0;
                if(dist<minDist){
                    minDist = dist;
                }
            }
            return minDist;
        }


        rw::geometry::PlainTriMesh<T,rw::geometry::N1>* toTriMesh(){
            using namespace rw::math;
            using namespace rw::geometry;
           // TODO: convert face array of indexed triangle faces
            PlainTriMesh<T,N1>* trimesh = new PlainTriMesh<T,N1>();
            BOOST_FOREACH(IndexedTriFace &face, *_faces){
                trimesh->add( face.toTriangle(*_vertices) );
            }
            return trimesh;
        }

    #endif

    /*
        static void build(const rw::geometry::IndexedTriMesh<T,rw::geometry::N0>& triMesh,
                   std::vector<int>& result){
            const std::vector<rw::math::Vector3D<T> >& verts = triMesh.getVertices();
            build<T>( verts, result);
        }
        */
        /*
        static void build(const rw::geometry::IndexedTriMesh<T,rw::geometry::N0>& triMesh,
                   std::vector<int>& result)
        {
            const std::vector<rw::math::Vector3D<T> >& verts = triMesh.getVertices();
            build<T>( verts, result);
        }
        */

    //	void build(const std::vector<rw::math::Vector3D<T> >& vertices,
    //			   std::vector<int>& result){
        //static rw::geometry::PlainTriMesh<T,rw::geometry::N1>*
        //build(const std::vector<rw::math::Vector3D<T> >& verts){

        static void build(const rw::geometry::IndexedTriMesh<T>& triMesh,std::vector<int>& result)
        {
            const std::vector<rw::math::Vector3D<T> >& verts = triMesh.getVertices();
            build< std::vector<rw::math::Vector3D<T> > >( verts, result);
        }



        /**
         * @brief takes a random accisible sequence of unique vertices and calculates the
         * convex hull. Unique vertices mean that no vertice v1 in the list match any other
         * vertice v2 in the list.
         */
        template<class V>
        static rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<T> >*
        build(const V& verts){
            double EPSILON = 0.000001;
            using namespace rw::math;
            using namespace rw::geometry;

            // creates a Indexed Triangle mesh with a reference to the vertices

            //std::cout << "Building... " << vertices.size() << std::endl;
            typedef std::list<IndexedTriFace<V> > FaceList;
            //typedef std::list<Triangle<T,N1> > FaceList;
            FaceList faces;

            // initialize connected array
            const int n = verts.size();
            std::vector<short> connected(n*n);
            for(int i=0;i<n*n;i++)
                connected[i] = 0;

            // first we create a simple convex hull from minx,maxx,miny,maxy and minz,maxz
            //Triangle face0 = Triangle(verts[0],verts[1],verts[2]);
            IndexedTriFace<V> face0 = IndexedTriFace<V>(0,1,2,verts);

            // now locate a new point that is not in the plane of 0,1,2
            size_t baseIdx = 0;
            for(size_t vIdx=3; vIdx< verts.size(); vIdx++){
                float dist = face0.halfSpaceDist(verts[vIdx]);
                //float dist = halfSpaceDist(vertices[vIdx], face0, vertices, normals);
                if( fabs(dist) > EPSILON ){
                    if( dist < 0 ){// change face 0
                        faces.push_back(IndexedTriFace<V>(1,0,2,verts));
                        faces.push_back(IndexedTriFace<V>(0,1,vIdx,verts));
                        faces.push_back(IndexedTriFace<V>(1,2,vIdx,verts));
                        faces.push_back(IndexedTriFace<V>(2,0,vIdx,verts));
                    } else {
                        faces.push_back(face0);
                        //faces.push_back(IndexedTriFace<V>(1,0,2,verts));
                        faces.push_back(IndexedTriFace<V>(1,0,vIdx,verts));
                        faces.push_back(IndexedTriFace<V>(2,1,vIdx,verts));
                        faces.push_back(IndexedTriFace<V>(0,2,vIdx,verts));
                    }
                    baseIdx = vIdx;
                    break;
                }
            }
            bool inPlane = false;
            if( baseIdx==0 ){
                // all points in the vertex array lie in a plane
                // create the 2d convex hull
                faces.push_back(IndexedTriFace<V>(1,0,2,verts));
                faces.push_back(face0);
                inPlane = true;
                //PlainTriMesh<T,N1>* trimesh = new PlainTriMesh<T,N1>();
                //BOOST_FOREACH(IndexedTriFace<V> &face, faces){
                //    trimesh->add( face.toTriangle(verts) );
                //}
                return NULL;
            }

            // next create stack to keep track of new edges
            typedef std::pair<int,int> Edge;
            std::stack<Edge> edgeStack;

            // for each vertex in vertices
            std::cout << "Vertices.size" << verts.size()<< std::endl;
            for(size_t vIdx=3; vIdx< verts.size(); vIdx++){

                if(vIdx == baseIdx)
                    continue;
                // for each triangle where the vertex lies outside create lines between vertex and triangle corners
                const Vector3D<T> &vertex = verts[vIdx];
                bool isInside = true;
                std::cout << "VERTICE INDEX: " << vIdx << std::endl;
                typename FaceList::iterator tIter = faces.begin();
                while(tIter!=faces.end()) {
                    IndexedTriFace<V>& face = *tIter;
                    // calculate the distance between triangle plane and vertex
                    T dist = face.halfSpaceDist( vertex );
                    if( dist < -EPSILON ){
                        face.setVisible(true);
                        //std::cout << "VERTICE INDEX: " << vIdx << std::endl;
                        //std::cout << "Is not inside... " << std::endl;
                        //std::cout << face[0] << " " << face[1] << " " << face[2] << std::endl;
                        isInside = false;
                        // only add edges that have not allready been added
                        int id0=face[0],id1=face[1],index;
                        if( id1<id0 ) index = id1*n + id0;
                        else index = id0*n + id1;
                        if( connected[index] == 0 )
                            edgeStack.push( std::make_pair(id0,id1) );
                        connected[index]++;

                        id0=face[1],id1=face[2];
                        if( id1<id0 ) index = id1*n + id0;
                        else index = id0*n + id1;
                        if( connected[index] == 0 )
                            edgeStack.push( std::make_pair(id0,id1) );
                        connected[index]++;

                        id0=face[2],id1=face[0];
                        if( id1<id0 ) index = id1*n + id0;
                        else index = id0*n + id1;
                        if( connected[index] == 0 )
                            edgeStack.push( std::make_pair(id0,id1) );
                        connected[index]++;

                        // lastly delete face and update iterator
                        tIter = faces.erase(tIter);
                    } else if(dist > EPSILON){
                        face.setVisible(false);
                        //std::cout << "Is inside!" << std::endl;
                        // update iterator
                        ++tIter;
                    } else {
                        face.setVisible(false);
                        ++tIter;
                        if( !inPlane )
                            continue;

                        //
                        // only add edges that are visible
                    }
                }
                std::cout << "Faces size: " << faces.size() << std::endl;
                if( isInside )
                    continue;
                //std::cout << "Handle edges!! " << std::endl;
                while( !edgeStack.empty() ){
                    Edge e = edgeStack.top();
                    edgeStack.pop();
                    int index;
                    if( e.first<e.second ) index = e.first*n + e.second;
                    else index = e.second*n + e.first;
                    if( connected[index]!=1 ){
                        //std::cout << "Too many hits: " << connected[index] << std::endl;
                        continue;
                    } else if( connected[index]>1 ){
                        std::cout << "Too many hits: " << connected[index] << std::endl;
                    }
                    connected[index] = 0;
                    IndexedTriFace<V> face(e.first, e.second, vIdx, verts);
                    faces.push_back(face);
                }

            } // for

            std::cout << "Conv to plain mesh" <<std::endl;
            // TODO: convert face array of indexed triangle faces
            PlainTriMesh<TriangleN1<T> >* trimesh = new PlainTriMesh<TriangleN1<T> >();
            BOOST_FOREACH(IndexedTriFace<V> &face, faces){
                trimesh->add( face.toTriangle(verts) );
            }

            return trimesh;
        };

    };

    #ifdef dummystuff_backup
    /*else if( fabs(dist)<EPSILON ){
                        // the vertex lie in the same plane as the triangle
                        // if the vertex is outside the triangle boundaries then
                        // add the closest line to edgestack

                        // calc vectors
                        Vector3D<T> v0 = vertices[face[1]] - vertices[face[0]];
                        Vector3D<T> vp0 = vertex - vertices[face[0]];
                        Vector3D<T> v1 = vertices[face[2]] - vertices[face[1]];
                        Vector3D<T> vp1 = vertex - vertices[face[1]];
                        Vector3D<T> v2 = vertices[face[0]] - vertices[face[2]];
                        Vector3D<T> vp2 = vertex - vertices[face[2]];

                        bool l0 = dot( cross(v0,vp0), face.getNormal()) >= 0;
                        bool l1 = dot( cross(v1,vp1), face.getNormal()) >= 0;
                        bool l2 = dot( cross(v2,vp2), face.getNormal()) >= 0;

                        size_t index = 0;
                        if( !l0 ) {
                            if(face[0]<face[1]) index = face[0]*n + face[1];
                            else index = face[1]*n + face[0];
                            //if( ++(connected[index]) < 2 )
                                edgeStack.push( std::make_pair(face[0],face[1]) );
                        }
                        if( !l1 ) {
                            if(face[1]<face[2]) index = face[1]*n + face[2];
                            else index = face[2]*n + face[1];
                            //if( ++(connected[index]) < 2 )
                                edgeStack.push( std::make_pair(face[1],face[2]) );
                        }
                        if( !l2 ) {
                            if(face[2]<face[0]) index = face[2]*n + face[0];
                            else index = face[0]*n + face[2];
                            //if( ++(connected[index]) < 2 )
                                edgeStack.push( std::make_pair(face[2],face[0]) );
                        }
                        ++tIter;
                        std::cout << l0 << " " << l1 << " " << l2 << std::endl;
                    } */
    #endif
    //! @}
}
}

#endif /*INCREMENTALHULL_HPP_*/
