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
#include "Plane.hpp"

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
            void print() const { std::cout << "Job: " << axis << " " << from << "-->" << to << std::endl;}
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
			Triangle<T> tri;
            for(size_t i = 0; i<triMesh.getSize(); i++){

                int vIdx = (int)i*3;

                triMesh.getTriangle(i, tri);

                RW_ASSERT(vIdx+0<(int)verticesIdx->size());
                RW_ASSERT(vIdx+1<(int)verticesIdx->size());
                RW_ASSERT(vIdx+2<(int)verticesIdx->size());

                (*verticesIdx)[vIdx+0] = VertexCmp<T>(tri[0], (int)i, 0, &axis);
                (*verticesIdx)[vIdx+1] = VertexCmp<T>(tri[1], (int)i, 1, &axis);
                (*verticesIdx)[vIdx+2] = VertexCmp<T>(tri[2], (int)i, 2, &axis);

            }

            // first sort all vertices into one large array
            axis = 0;
            std::sort(verticesIdx->begin(),verticesIdx->end());

            // run through the semi sorted list and merge vertices that are alike
            std::stack<SortJob> sjobs;
            sjobs.push(SortJob(0,0,(int)(verticesIdx->size()-1)));
            while( !sjobs.empty() ){
                SortJob job = sjobs.top();
                sjobs.pop();
                if(job.from==job.to)
                    continue;
                // locate the next end
                int j = job.from;
                RW_ASSERT_MSG(j<(int)verticesIdx->size(), j << "<" << verticesIdx->size());
                RW_ASSERT(job.axis<3);
                T axisVal, lastAxisVal = (*verticesIdx)[j].n[job.axis];
                do{
                    j++;
                    axisVal = (*verticesIdx)[j].n[job.axis];
                } while(axisVal<lastAxisVal+epsilon && j<job.to);
                // if j==job.to, then we might have two possible outcomes,
                // we change j to point to the element that is not equal to lastVal
                if(j==job.to && axisVal<lastAxisVal+epsilon)
                    j++;

                // the previus has determined an interval [job.from;j] in which values in job.axis equal
                // now add the unproccessed in a new job [j;job.to]
                if( j<job.to )
                    sjobs.push(SortJob(job.axis, j , job.to));

                if( job.axis==0 )
                    sjobs.push(SortJob(job.axis+1, job.from , j-1));

                axis = job.axis+1;
                std::sort(verticesIdx->begin()+job.from, verticesIdx->begin()+j);
            }

            return verticesIdx;
		}


	public:
		/**
		 * @brief Takes a general triangle mesh and creates an indexed
		 * triangle mesh. All data is copied.
		 *
		 * The order of the triangles in the new mesh will be the same as that of
		 * the old mesh. This is not true for the vertices.
		 * @param triMesh [in] the tri mesh that is to be converted
		 * @param epsilon [in] if two vertices are closer than epsilon they
		 * are considered the equal.
		 */
		template <class TRILIST>
		static rw::common::Ptr<TRILIST> toIndexedTriMesh(const TriMesh& triMesh,
										 double epsilon=0.00001)
		{
		    typedef typename TRILIST::value_type T;
		    typedef typename TRILIST::tri_type TRI;
		    typedef typename TRILIST::index_type S;
		    using namespace rw::math;
		    if( triMesh.getSize()==0)
		        RW_THROW("Size of mesh must be more than 0!");

		    // create a sorted vertice list with backreference to the triangle list
		    std::vector<VertexCmp<T> >* verticesIdx = createSortedVerticeIdxList<T>(triMesh, epsilon);

			// Now copy all sorted vertices into the vertice array
            // and make sure vertices that are close to each other are discarded
            std::vector<Vector3D<T> > *vertices =
                new std::vector<Vector3D<T> >(triMesh.getSize()*3);

		    // allocate enough memory
            std::vector<TRI> *triangles = new std::vector<TRI>(triMesh.getSize());

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
                ((*triangles)[ triIdx ])[vertTriIdx] = vertCnt;
            }

            vertices->resize(vertCnt+1);

            delete verticesIdx;

			return rw::common::ownedPtr( new TRILIST(rw::common::ownedPtr(vertices), rw::common::ownedPtr(triangles)) );
		}


		/**
		 * @brief Recalculate the normals of \b trimesh
		 */
		template <class T>
		static void recalcNormals(PlainTriMesh<TriangleN1<T> >& trimesh){
		    using namespace rw::math;
		    for(size_t i=0; i<trimesh.getSize(); i++){
		        Vector3D<T> normal = trimesh[i].calcFaceNormal();
		        trimesh[i].getFaceNormal() = normal;
		    }
		}


		/**
		 * @brief Divided \b trimesh using the specified plane
		 * 
		 * Triangles is cut by the plane and replaced with new triangles.
		 * Triangles lying in the plane is placed with the triangles behind.
		 *
		 * @return First item in the pair is the triangles in front of the plane and second item is the triangles behind or in the plane.
		 */
		template <class TRI>
                static std::pair<TriMesh::Ptr, TriMesh::Ptr> divide(TriMesh::Ptr trimesh, Plane::Ptr plane) {
                        typedef typename TRI::value_type T;
                        using namespace rw::math;
                        typename PlainTriMesh<TRI>::Ptr front = ownedPtr(new PlainTriMesh<TRI>());
                        typename PlainTriMesh<TRI>::Ptr back = ownedPtr(new PlainTriMesh<TRI>());
                        for (size_t i = 0; i<trimesh->size(); i++) {
                                Triangle<>& tri = trimesh->getTriangle(i);
                                double d0 = plane->distance(tri.getVertex(0));
                                double d1 = plane->distance(tri.getVertex(1));
                                double d2 = plane->distance(tri.getVertex(2));
                                //std::cout<<"d0 = "<<d0<<" d1 = "<<d1<<" d2 = "<<d2<<std::endl;
                                if (d0 <= 0 && d1<= 0 && d2 <= 0) {
                                        back->add(tri);
                                } else if (d0 >= 0 && d1 >= 0 && d2 >= 0) {
                                        front->add(tri);
                                } else {
                                        std::vector<int> behind;
                                        std::vector<int> infront;
                                        if (d0 < 0) 
                                                behind.push_back(0);
                                        else
                                                infront.push_back(0);

                                        if (d1 < 0) 
                                                behind.push_back(1);
                                        else
                                                infront.push_back(1);

                                        if (d2 < 0) 
                                                behind.push_back(2);
                                        else
                                                infront.push_back(2);

                                        if (behind.size() == 2) {
                                                Vector3D<T> b1 = tri.getVertex(behind[0]);
                                                Vector3D<T> b2 = tri.getVertex(behind[1]);
                                                Vector3D<T> f1 = tri.getVertex(infront[0]);

                                                Vector3D<T> i1 = plane->intersection(b1, f1);
                                                Vector3D<T> i2 = plane->intersection(b2, f1);

                                                if (d0 > 0 || d2 > 0) {
                                                        TRI trib1(b1, i2, i1);
                                                        TRI trib2(b1, b2, i2);
                                                        TRI trif1(i1, i2, f1);
                                                        back->add(trib1);
                                                        back->add(trib2);
                                                        front->add(trif1);
                                                } else {
                                                        TRI trib1(b1, i1, i2);
                                                        TRI trib2(b1, i2, b2);
                                                        TRI trif1(i1, f1, i2);
                                                        back->add(trib1);
                                                        back->add(trib2);
                                                        front->add(trif1);
                                                }

                                        } else { //inFront.size() == 2
                                                Vector3D<T> b1 = tri.getVertex(behind[0]);
                                                Vector3D<T> f1 = tri.getVertex(infront[0]);
                                                Vector3D<T> f2 = tri.getVertex(infront[1]);

                                                Vector3D<T> i1 = plane->intersection(b1, f1);
                                                Vector3D<T> i2 = plane->intersection(b1, f2);

                                                if (d0 < 0 || d2 < 0) {
                                                        TRI trif1(f1, i2, i1);
                                                        TRI trif2(f1, f2, i2);
                                                        TRI trib1(b1, i1, i2);

                                                        front->add(trif1);
                                                        front->add(trif2);
                                                        back->add(trib1);
                                                } else {
                                                        TRI trif1(f1, i1, i2);
                                                        TRI trif2(f1, i2, f2);
                                                        TRI trib1(b1, i2, i1);

                                                        front->add(trif1);
                                                        front->add(trif2);
                                                        back->add(trib1);
                                                }
                                        }
                                }
                        }
                        return std::make_pair(front, back);
		
		}
    };


	// @}
}
} // end namespaces

#endif // end include guard
