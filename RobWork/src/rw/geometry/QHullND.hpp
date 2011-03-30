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

#ifndef RW_GEOMETRY_QHULLND_HPP_
#define RW_GEOMETRY_QHULLND_HPP_

#include <stack>
#include <set>
#include <vector>
#include <float.h>
#include <boost/numeric/ublas/vector.hpp>
#include <rw/math/VectorND.hpp>
#include "ConvexHullND.hpp"

namespace rw {
namespace geometry {
    /** @addtogroup geometry
     *  @{
     *  @file QHullND.hpp
     */

    namespace qhull {

        /**
         * @brief calclates the convex hull of a set of vertices \b coords each with dimension  \b dim
         *
         * @param dim [in] nr of dimensions in each vertice
         * @param coords [in] array of vertices
         * @param nrCoords [in] the number of vertices
         * @return
         */
        void build(size_t dim, double *coords, size_t nrCoords, std::vector<int>& vertIdxs, std::vector<int>& faceIdxs, std::vector<double>& faceNormals);
    }

    /**
	 * @brief calculates the convex hull of a set of 3d points.
	 *
	 * The GirftWrap convex hull algorithm is used, hence the
	 * class name.
	 *
	 * @note It is important that there are not multiple vertices at the same coordinates.
	 * Filter these away before using this convex hull calculation.
	 */
    template <std::size_t N>
	class QHullND: public ConvexHullND<N> {
	public:
        //static const std::size_t dimension = N;
	    //typedef boost::numeric::ublas::bounded_vector<double, N> VectorND;
	    //typedef int[N] FaceIdxND;

		/**
		 * @brief constructor
		 */
		QHullND(){};

		/**
		 * @brief destructor
		 */
		virtual ~QHullND(){};

		//! @copydoc ConvexHull3D::rebuild
		void rebuild(const std::vector<rw::math::VectorND<N> >& vertices){
		    using namespace rw::math;
		    // convert the vertice array to an array of double
		    double *vertArray = new double[vertices.size()*N];
		    // copy all data into the vertArray
		    for(size_t i=0;i<vertices.size();i++){
		        const VectorND<N> &vnd = vertices[i];
		        for(size_t j=0;j<N;j++)
		            vertArray[i*N+j] = vnd[j];
		    }
		    // build the hull
		    qhull::build(N, vertArray, vertices.size(), _vertiIdxs, _faceIdxs, _faceNormalsTmp);
		    delete[] vertArray;

		    _hullVertices.resize(_vertiIdxs.size());
		    for(size_t i=0;i<_vertiIdxs.size(); i++){
		        _hullVertices[i] = vertices[_vertiIdxs[i]];
		    }
		    _faceNormals.resize(_faceIdxs.size()/N);
            for(size_t i=0;i<_faceIdxs.size()/N; i++){
                for(size_t j=0; j<N; j++)
                    _faceNormals[i][j] = _faceNormalsTmp[i*N+j];
            }
		}

		//! @copydoc ConvexHull3D::isInside
		bool isInside(const rw::math::VectorND<N>& vertex, const std::vector<rw::math::VectorND<N> >& vertices){
		    using namespace rw::math;
            const static double EPSILON = 0.0000001;
            if( _faceIdxs.size()==0 ){
                //std::cout << "No Tris" << std::endl;
                return 0;
            }

            double minDist = DBL_MAX;
            for(int i=0; i<_faceIdxs.size()/N; i++){
                int faceVerticeIdx = _faceIdxs[i*N];
                RW_ASSERT(faceVerticeIdx< vertices.size());
                VectorND<N> v = vertices[ faceVerticeIdx ];
                RW_ASSERT(i< _faceNormals.size());
                double dist = inner_prod(_faceNormals[i], v);
                minDist = std::min( dist, minDist );
                if(minDist<0)
                    return false;
            }

            return minDist>=0;
		}

		//! @copydoc ConvexHull3D::getMinDistOutside
		double getMinDistOutside(const rw::math::VectorND<N>& vertex){ return 0; }

		//! if negative then point is outside hull
		double getMinDistInside(const rw::math::VectorND<N>& vertex, const std::vector<rw::math::VectorND<N> >& vertices){
		    using namespace rw::math;
		    if( _faceIdxs.size()==0 ){
		        //std::cout << "No Tris" << std::endl;
		        return 0;
		    }

		    double minDist = DBL_MAX;
		    for(size_t i=0; i<_faceIdxs.size()/N; i++){
		        int faceVerticeIdx = _faceIdxs[i*N];
		        //RW_ASSERT(faceVerticeIdx< (int)vertices.size());
		        VectorND<N> v = vertices[ faceVerticeIdx ];
		        RW_ASSERT(i< _faceNormals.size());
		        double dist = inner_prod(_faceNormals[i],v);
		        minDist = std::min( dist, minDist );
		    }

		    return minDist;
		}

		const std::vector<rw::math::VectorND<N> >& getHullVertices(){ return _hullVertices; }

		const std::vector<int>& getFaceIndices(){ return _faceIdxs; }

		const std::vector<rw::math::VectorND<N> >& getFaceNormals(){ return _faceNormals; }

	private:
		std::vector<rw::math::VectorND<N> > _hullVertices, _faceNormals;
		std::vector<int> _vertiIdxs, _faceIdxs;
		std::vector<double> _faceNormalsTmp;
	};
	//! @}
}
}
#endif /* GIFTWRAPHULL_HPP_ */
