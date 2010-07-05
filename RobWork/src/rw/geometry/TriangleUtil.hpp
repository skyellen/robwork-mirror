/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_GEOMETRY_TRIANGLEUTIL_HPP_
#define RW_GEOMETRY_TRIANGLEUTIL_HPP_

//#include <rw/geometry/Face.hpp>

#include "Triangle.hpp"
#include "TriMesh.hpp"
#include "IndexedTriMesh.hpp"
#include "PlainTriMesh.hpp"

#include <rw/math/Vector3D.hpp>
#include <stack>
#include <rw/math/MetricUtil.hpp>
#include <boost/foreach.hpp>

namespace rw { namespace geometry {
	//! @addtogroup geometry
	// @{

	/**
	 * @brief utility for triangle manipulation
	 */
	class TriangleUtil
    {
    private:

        template<class T>
        struct VertexCmp {
        public:
            VertexCmp():vertIdx(-1){}
            VertexCmp(const rw::math::Vector3D<T>& val, int tidx, int vidx, int *axisp):
            	n(val), triIdx(tidx), vertIdx(vidx),_axisPtr(axisp)
            { }

            bool operator<(VertexCmp<T> const &other) const {
                return n[*_axisPtr] < other.n[*_axisPtr];
            }
            rw::math::Vector3D<T> n;
            int triIdx;
            int vertIdx;
            int *_axisPtr;

        };

        struct SortJob {
        public:
            SortJob(int a, int f, int t):axis(a),from(f),to(t){}
            int axis;
            int from;
            int to;
            void print(){ std::cout << "Job: " << axis << " " << from << "-->" << to << std::endl;}
        };

		/**
		 * @brief creates a sorted indexed verticelist. The vertice list is
		 * indexed into the triangle mesh so that each vertice "knows" the triangle
		 * that its comming from.
		 */
		template <class T>
		static std::vector<VertexCmp<T> >* createSortedVerticeIdxList(
				const TriMesh& triMesh, double epsilon){
			using namespace rw::math;
			std::vector<VertexCmp<T> > *verticesIdx =
                new std::vector<VertexCmp<T> >(triMesh.getSize()*3);
			int axis = 0;
            // now copy all relevant info into the compare list
            for(size_t i = 0; i<triMesh.getSize(); i++){
                int vIdx = i*3;
                Triangle<double> tri = triMesh.getTriangle(i);
                (*verticesIdx)[vIdx+0] = VertexCmp<T>(cast<T,double>(tri[0]), i, 0, &axis);
                (*verticesIdx)[vIdx+1] = VertexCmp<T>(cast<T,double>(tri[1]), i, 1, &axis);
                (*verticesIdx)[vIdx+2] = VertexCmp<T>(cast<T,double>(tri[2]), i, 2, &axis);
            }

            // first sort all vertices into one large array
            axis = 0;
            std::sort(verticesIdx->begin(),verticesIdx->end());

            // run through the semi sorted list and merge vertices that are alike
            //std::stack<VertexCmp<T>*>
            std::stack<SortJob> sjobs;
            sjobs.push(SortJob(0,0,verticesIdx->size()-1));
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
                if( j<job.to )
                    sjobs.push(SortJob(job.axis, j , job.to));
                if( job.axis==0 )
                    sjobs.push(SortJob(job.axis+1, job.from , j-1));

                std::sort(verticesIdx->begin()+job.from, verticesIdx->begin()+j-1);
                /*
                std::cout << axis << job.from << " --> " << j-1 << "   ";
                for(int i=job.from;i<j;i++){
                	std::cout << (*verticesIdx)[i].n << " " << std::endl;;
                }
                 */
            }
            return verticesIdx;
		}


	public:
		/**
		 * @brief takes a general triangle mesh and creates an indexed
		 * triangle mesh. All data is copied.
		 * @param triMesh [in] the tri mesh that is to be converted
		 * @param epsilon [in] if two vertices are closer than epsilon they
		 * are considered the equal.
		 */
		template <class TRILIST>
		static TRILIST* toIndexedTriMesh(const TriMesh& triMesh,
										 double epsilon=0.00001)
		{
		    typedef typename TRILIST::value_type T;
		    typedef typename TRILIST::tri_type TRI;
		    typedef typename TRILIST::index_type S;
		    using namespace rw::math;

		    // create a sorted vertice list with backreference to the triangle list
		    std::vector<VertexCmp<T> >* verticesIdx =
		    		createSortedVerticeIdxList<T>(triMesh, epsilon);

            // Now copy all sorted vertices into the vertice array
            // and make sure vertices that are close to each other are discarded
            std::vector<Vector3D<T> > *vertices =
                new std::vector<Vector3D<T> >(triMesh.getSize()*3);
		    // allocate enough memory
            std::vector<TRI> *triangles =
                new std::vector<TRI>(triMesh.getSize());

            S vertCnt = 0;
            Vector3D<T> lastVert = (*verticesIdx)[0].n;
            (*vertices)[vertCnt] = lastVert;
            TRI &itri = (*triangles)[ (*verticesIdx)[0].triIdx ];
            itri[(*verticesIdx)[0].vertIdx] = vertCnt;
            for(size_t i=1;i<verticesIdx->size(); i++){
            	// check if vertices are too close
                if( MetricUtil::dist2(lastVert, (*verticesIdx)[i].n)>epsilon ){
                    lastVert = (*verticesIdx)[i].n;
                    vertCnt++;
                    (*vertices)[vertCnt] = lastVert;
                }
                S triIdx = (*verticesIdx)[i].triIdx;
                S vertTriIdx = (*verticesIdx)[i].vertIdx;
                // update the triangle index for this vertice
                //std::cout <<  "vertIdx: " << (*verticesIdx)[i].vertIdx << std::endl;
                ((*triangles)[ triIdx ])[vertTriIdx] = vertCnt;
                //std::cout << (*verticesIdx)[i].n << std::endl;
            }
            vertices->resize(vertCnt+1);

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
		/*
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
*/

		template <class T>
		static void recalcNormals(PlainTriMesh<TriangleN1<T> >& trimesh){
		    using namespace rw::math;
		    for(size_t i=0; i<trimesh.getSize(); i++){
		        Vector3D<T> normal = trimesh[i].calcFaceNormal();
		        trimesh[i].getFaceNormal() = normal;
		    }
		}
	private:






    };


	// @}
}
} // end namespaces

#endif // end include guard
