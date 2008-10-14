#ifndef RW_GEOMETRY_TRIANGLEUTIL_HPP_
#define RW_GEOMETRY_TRIANGLEUTIL_HPP_

#include <rw/geometry/Face.hpp>

#include "Triangle.hpp"
#include "TriMesh.hpp"
#include "IndexedTriMesh.hpp"
#include "PlainTriMesh.hpp"

#include <rw/math/Vector3D.hpp>
#include <stack>
#include <rw/math/MetricUtil.hpp>
#include <boost/foreach.hpp>

namespace rw { namespace geometry {

	class TriangleUtil
    {
    private:

        template<class T>
        struct VertexCmp {
        public:
            VertexCmp():vertIdx(-1){}
            //VertexCmp(int *axisPtr):_axisPtr(axisPtr){}
            bool operator<(VertexCmp<T> const &other) const {
                return n[*_axisPtr] < other.n[*_axisPtr];
            }
            int triIdx;
            rw::math::Vector3D<T> n;
            int *_axisPtr;
            int vertIdx;
        };

        struct SortJob {
        public:
            SortJob(int a, int f, int t):axis(a),from(f),to(t){}
            int axis;
            int from;
            int to;
            void print(){ std::cout << "Job: " << axis << " " << from << "-->" << to << std::endl;}
        };

	public:
		/**
		 * @brief takes a general triangle mesh and creates an indexed
		 * triangle mesh. All data is copied.
		 * @param triMesh [in] the tri mesh that is to be converted
		 * @param epsilon [in] if two vertices are closer than epsilon they
		 * are considered the equal.
		 */
		template <class TRILIST>
		static TRILIST* toIndexedTriMesh(const TriMesh/*<typename TRILIST::value_type>*/& triMesh,
										 double epsilon=0.00001)
		{
		    typedef typename TRILIST::value_type T;
		    typedef typename TRILIST::tri_type TRI;
		    using namespace rw::math;
		    int axis = 0;

            std::vector<VertexCmp<T> > *verticesIdx =
                new std::vector<VertexCmp<T> >(triMesh.getSize()*3);
            std::vector<Vector3D<T> > *vertices =
                new std::vector<Vector3D<T> >(triMesh.getSize()*3);

            for(size_t i = 0; i<triMesh.getSize(); i++){
                int vIdx = i*3;
                TriangleN0<double> tri = triMesh.getTriangle(i);
                (*verticesIdx)[vIdx].n = cast<T,double>(tri[0]);
                (*verticesIdx)[vIdx].triIdx = i;
                (*verticesIdx)[vIdx].vertIdx = 0;
                (*verticesIdx)[vIdx]._axisPtr = &axis;
                (*verticesIdx)[vIdx+1].n = cast<T>(tri[1]);
                (*verticesIdx)[vIdx+1].triIdx = i;
                (*verticesIdx)[vIdx+1].vertIdx = 1;
                (*verticesIdx)[vIdx+1]._axisPtr = &axis;
                (*verticesIdx)[vIdx+2].n = cast<T>(tri[2]);
                (*verticesIdx)[vIdx+2].triIdx = i;
                (*verticesIdx)[vIdx+2].vertIdx = 2;
                (*verticesIdx)[vIdx+2]._axisPtr = &axis;
            }

            // first sort all vertices into one large array
            axis = 0;
            std::sort(verticesIdx->begin(),verticesIdx->end());

            // allocate enough memory
            std::vector<TRI> *triangles =
                new std::vector<TRI>(triMesh.getSize());

            // run through the semi sorted list and merge vertices that are alike
            //std::stack<VertexCmp<T>*>
            std::stack<SortJob> sjobs;
            sjobs.push(SortJob(0,0,verticesIdx->size()));
            while( !sjobs.empty() ){
                SortJob job = sjobs.top();
                sjobs.pop();
                //job.print();

                // locate the next end
                int j = job.from;
                T axisVal, lastAxisVal = (*verticesIdx)[j].n[job.axis];
                do{
                    //std::cout << "j" << j << "," << job.axis << " ";
                    axisVal = (*verticesIdx)[j].n[job.axis];
                    j++;
                } while(axisVal<lastAxisVal+epsilon && j<job.to);

                axis = job.axis+1;
                if(j<job.to)
                    sjobs.push(SortJob(job.axis, j , job.to));
                if( job.axis==0 )
                    sjobs.push(SortJob(job.axis+1, job.from , j));
                std::sort(verticesIdx->begin()+job.from,verticesIdx->begin()+j);
            }
            //std::cout << "Finished jobs.... " << std::endl;
            int vertCnt = 0;
            Vector3D<T> lastVert = (*verticesIdx)[0].n;
            (*vertices)[vertCnt] = lastVert;
            TRI &itri = (*triangles)[ (*verticesIdx)[0].triIdx ];
            itri[(*verticesIdx)[0].vertIdx] = vertCnt;
            for(int i=1;i<verticesIdx->size(); i++){
                if( MetricUtil::dist2(lastVert, (*verticesIdx)[i].n)>epsilon ){
                    lastVert = (*verticesIdx)[i].n;
                    vertCnt++;
                    (*vertices)[vertCnt] = lastVert;
                }
                int triIdx = (*verticesIdx)[i].triIdx;
                int vertTriIdx = (*verticesIdx)[i].vertIdx;
                // update the triangle index for this vertice
//                std::cout <<  "vertIdx: " << (*verticesIdx)[i].vertIdx << std::endl;
                ((*triangles)[ triIdx ])[vertTriIdx] = vertCnt;
                //std::cout << (*verticesIdx)[i].n << std::endl;
            }
            vertices->resize(vertCnt+1);
/*
            for(size_t i = 0; i<triangles->size(); i++){
                std::cout << "Triangle ("
                          << ((*triangles)[i])[0] << ","
                          << ((*triangles)[i])[1] << ","
                          << ((*triangles)[i])[2] << ")" << std::endl;

            }*/
/*
			// for each triangle check if any of the triangle vertices are allready in vertices array
			int vSize = 0;
			for(size_t i = 0; i<triMesh.getSize(); i++){

				int v0=-1,v1=-1,v2=-1;
				Triangle<T,N0> tri = triMesh.getTriangle(i);
				bool v0found=false, v1found=false, v2found=false;
				for(size_t j=0; j<vSize; j++){
					Vector3D<T> &v = (*vertices)[j];
					if( MetricUtil::dist2<T>(v, tri.getVertex(0))<epsilon ){
						v0 = j;
						break;
					}
					if( MetricUtil::dist2<T>(v, tri.getVertex(1))<epsilon ){
						v1 = j;
						break;
					}
					if( v2<0 && MetricUtil::dist2<T>(v, tri.getVertex(2))<epsilon ){
						v2 = j;
						break;
					}
				}
				if( v0<0 ){
					v0 = vSize;
					(*vertices)[vSize] = tri.getVertex(0);
					vSize++;
				}
				if( v1<0 ){
                    v1 = vSize;
                    (*vertices)[vSize] = tri.getVertex(1);
                    vSize++;
				}
				if( v2<0 ){
                    v2 = vSize;
                    (*vertices)[vSize] = tri.getVertex(2);
                    vSize++;
				}
				(*triangles)[i] = IndexedTriangle(v0,v1,v2);
			}
			vertices->resize(vSize);
            std::cout << "Indexed triangle mesh created: " << vertices->size()
                      << " " << triangles->size() << std::endl;
			std::cout << "BB" << std::endl;
			//for(size_t i=0; i<vertices->size(); i++)
			//	std::cout << (*vertices)[i] << std::endl;

			*/
            delete verticesIdx;
			return new TRILIST(vertices, triangles);
		}


		/**
		 * @brief takes a indexed triangle mesh and creates a generel
		 * triangle mesh. All data is copied.
		 * @param triMesh [in] the tri mesh that is to be converted
		 */
		//template <class TRI=Triangle<double> >
		//static TriMesh<TRI>* toTriMesh( const IndexedTriMesh<TRI>& iTriMesh );


		//template <class TRI=Triangle<double> >
		//static TriMesh<TRI>* toMesh( const IndexedTriMesh<TRI>& iTriMesh );

		// calculate and add vertex normals from/to TriMesh
/*		template <class T>
		static void CalcVertexNormals( const TriMesh<T>& triMesh,
									   PlainTriMesh<T>& result)
		{

		}
*/
		// calculate and add vertex normals from/to TriMesh
/*		template <class T, TriType TRI>
		static void CalcVertexNormals( const IndexedTriMesh<T, TRI>& triMesh,
									   IndexedTriMesh<T>& result)
		{

		}
*/
		template <class TRI>
		static PlainTriMesh< TRI >*
        toTriangleMesh( std::vector<rw::geometry::Face<float> >& faces){
		    using namespace rw::math;
		    typedef typename TRI::value_type T;
		    PlainTriMesh<TRI > *mesh = new PlainTriMesh<TRI >(faces.size());

		    for(size_t i=0;i<faces.size();i++){
		        Vector3D<T> v3d1(faces[i]._vertex1[0],faces[i]._vertex1[1],faces[i]._vertex1[2]);
		        Vector3D<T> v3d2(faces[i]._vertex2[0],faces[i]._vertex2[1],faces[i]._vertex2[2]);
		        Vector3D<T> v3d3(faces[i]._vertex3[0],faces[i]._vertex3[1],faces[i]._vertex3[2]);
		        (*mesh)[i] = TRI(v3d1,v3d2,v3d3);
		    }
		    return mesh;
		}

		template <class T>
		static void recalcNormals(PlainTriMesh<TriangleN1<T> >& trimesh){
		    using namespace rw::math;
		    for(size_t i=0; i<trimesh.size(); i++){
		        Vector3D<T> normal = trimesh[i].calcFaceNormal();
		        trimesh[i].getFaceNormal() = normal;
		    }
		}
	};

}} // end namespaces

#endif // end include guard
