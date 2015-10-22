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


#ifndef RW_GEOMETRY_TRIANGLE_HPP_
#define RW_GEOMETRY_TRIANGLE_HPP_

#include "GeometryData.hpp"
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/MetricUtil.hpp>

namespace rw {
namespace geometry {
	//! @addtogroup geometry
	// @{
	/**
	 * @brief plain triangle class. The second template argument specify
	 * the number of normals associated with the triangle.
	 *  The triangle vertices should be arranged counter clock wise.
	 */
	template <class T=double>
	class Triangle {
	private:
	    rw::math::Vector3D<T> _vertices[3];
	public:
	    //! @brief value type of vertices
	    typedef T value_type;

	    //@brief default constructor
	    Triangle(){};

	    /**
	     * @brief constructor
	     * @param p1 [in] vertice 1
	     * @param p2 [in] vertice 2
	     * @param p3 [in] vertice 3
	     */
	    Triangle(const rw::math::Vector3D<T>& p1,
                   const rw::math::Vector3D<T>& p2,
                   const rw::math::Vector3D<T>& p3)
	    {
	        _vertices[0] = p1;
	    	_vertices[1] = p2;
	    	_vertices[2] = p3;
	    };

	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    Triangle(const Triangle<T>& f)
	    {
	        _vertices[0] = f.getVertex(0);
	        _vertices[1] = f.getVertex(1);
	        _vertices[2] = f.getVertex(2);
	    };

	    /**
	     * @brief destructor
	     */
	    virtual ~Triangle(){};

	    /**
	     * @brief get vertex at index i
	     */
		rw::math::Vector3D<T>& getVertex(size_t i){
			return _vertices[i];
		}

	    /**
	     * @brief get vertex at index i
	     */
		const rw::math::Vector3D<T>& getVertex(size_t i) const{
			return _vertices[i];
		}

        /**
         * @brief get vertex at index i
         */
        const rw::math::Vector3D<T>& operator[](size_t i) const{ return getVertex(i);};

        /**
         * @brief get vertex at index i
         */
        rw::math::Vector3D<T>& operator[](size_t i) { return getVertex(i);};

		/**
		 * @brief calculates the face normal of this triangle. It is assumed
		 * that the triangle vertices are arranged counter clock wise.
		 */
		rw::math::Vector3D<T> calcFaceNormal() const {
	        rw::math::Vector3D<T> n =
	        	cross(rw::math::Vector3D<T>(_vertices[1]-_vertices[0]),
	        		  rw::math::Vector3D<T>(_vertices[2]-_vertices[0]));
	        return normalize(n);
		}

		/**
		 * @brief tests wheather the point x is inside the triangle
		 */
		bool isInside(const rw::math::Vector3D<T>& x){
			using namespace rw::math;
			// calc vectors
			const Vector3D<T> &v0 = _vertices[2] - _vertices[0];
			const Vector3D<T> &v1 = _vertices[1] - _vertices[0];
			const Vector3D<T> &v2 = x - _vertices[0];
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

		/**
		 * @brief calculate the area of the triangle
		 * @return area in m^2
		 */
        double calcArea() const {
            rw::math::Vector3D<T> ab = getVertex(1)-getVertex(0);
            rw::math::Vector3D<T> ac = getVertex(2)-getVertex(0);
            return rw::math::MetricUtil::norm2( cross(ab,ac) )/2;
        }

        /**
         * @brief apply a transformation to this triangle
         * @param t3d [in] transform that is to be applied
         */
        void applyTransform(const rw::math::Transform3D<T>& t3d){
            _vertices[0] = t3d*_vertices[0];
            _vertices[1] = t3d*_vertices[1];
            _vertices[2] = t3d*_vertices[2];
        }

        /**
         * @brief Returns Triangle transformed by t3d.
         */
        Triangle<T> transform(const rw::math::Transform3D<T>& t3d) const {
            return Triangle<T>(t3d*_vertices[0],t3d*_vertices[1],t3d*_vertices[2]);
        }

        inline const Triangle<T>& getTriangle() const { return *this; }
        inline Triangle<T>& getTriangle() { return *this; }

        /**
         * @brief Outputs transform to stream
         * @param os [in/out] an output stream
         * @param t [in] the transform that is to be sent to the output stream
         * @return os
         */
        friend std::ostream& operator<<(std::ostream &os, const Triangle<T>& t)
        {
            // This format matches the Lua notation.
            return os
                << "Triangle("
                << t._vertices[0]
                << ", "
                << t._vertices[1]
				 << ", "
				 << t._vertices[2]
				<< ")";
        }


	};

	/**
	 * @brief Triangle facet. triangle class of type N1, which means that
	 * beside the plain triangle the face normal of the triangle is saved
	 * with the facenormal.
	 */
	template <class T=double>
	class TriangleN1
	{
	private:
	    Triangle<T> _triN0;
		rw::math::Vector3D<T> _faceNormal;

	public:
		//! @brief value type of vertices
		typedef T value_type;

	    //@brief default constructor
	    TriangleN1(){};

	    /**
	     * @brief constructor, calculates the face normal from vertex data
	     */
	    TriangleN1(const rw::math::Vector3D<T>& p1,
	    		   const rw::math::Vector3D<T>& p2,
	    		   const rw::math::Vector3D<T>& p3):
	    		       _triN0(p1,p2,p3),
	    		       _faceNormal(_triN0.calcFaceNormal())
	    {}

	    /**
	     * @brief constructor
	     */
	    TriangleN1(const rw::math::Vector3D<T>& p1,
	    		 const rw::math::Vector3D<T>& p2,
	    		 const rw::math::Vector3D<T>& p3,
	    		 const rw::math::Vector3D<T>& n):
	    			   _triN0(p1,p2,p3),
	    			   _faceNormal(n)
	    {}

        /**
         * @brief constructor
         */
        TriangleN1(const Triangle<T>& t,
                   const rw::math::Vector3D<T>& n):
                       _triN0(t),
                       _faceNormal(n)
        {}

	    /**
	     * @brief destructor
	     */
	    virtual ~TriangleN1(){};

	    /**
	     * @brief returns the facenormal of this triangle
	     */
		rw::math::Vector3D<T>& getFaceNormal(){
			return _faceNormal;
		}

	    /**
	     * @brief returns the facenormal of this triangle
	     */
		const rw::math::Vector3D<T>& getFaceNormal() const{
			return _faceNormal;
		}

		/**
		 * @brief calculates the distance to the halfspace of the triangle
		 */
		T halfSpaceDist(const rw::math::Vector3D<T>& x) const {
			T d = dot(_faceNormal, _triN0.getVertex(0));
			return dot(_faceNormal,x) - d;
		}

		// inheritet functions from Triangle
		//! @copydoc Triangle::getVertex
		rw::math::Vector3D<T>& getVertex(size_t i){ return _triN0.getVertex(i); };

		//! @copydoc Triangle::getVertex
        const rw::math::Vector3D<T>& getVertex(size_t i) const { return _triN0.getVertex(i); };

        //! @copydoc Triangle::calcFaceNormal
        rw::math::Vector3D<T> calcFaceNormal() const{
            return _triN0.calcFaceNormal();
        };

        /**
         * @brief get vertex at index i
         */
        const rw::math::Vector3D<T>& operator[](size_t i) const{ return getVertex(i);};

        /**
         * @brief get vertex at index i
         */
        rw::math::Vector3D<T>& operator[](size_t i) { return getVertex(i);};


        /**
         * @brief tests wheather the point x is inside the triangle
         */
        bool isInside(const rw::math::Vector3D<T>& x){
            return _triN0.isInside(x);
        }

        //! @copydoc Triangle::applyTransform
        void applyTransform(const rw::math::Transform3D<T>& t3d){
            _triN0.applyTransform(t3d);
            _faceNormal = t3d.R()*_faceNormal;
        }


        //! @copydoc Triangle::transform
        TriangleN1<T> transform(const rw::math::Transform3D<T>& t3d) const {
            return TriangleN1<T>(_triN0.transform(t3d), t3d.R()*_faceNormal );
        }

        inline const Triangle<T>& getTriangle() const { return _triN0; }
        inline Triangle<T>& getTriangle() { return _triN0; }
	};

    /**
     * @brief Triangle facet. triangle class that stores one normal for each
     * vertex in the triangle.
     */
	template <class T=double>
	class TriangleN3
	{
	private:
	    Triangle<T> _triN0;
		rw::math::Vector3D<T> _vertexNormals[3];

	public:
		//! @brief value type of vertices
		typedef T value_type;

	    //@brief default constructor
	    TriangleN3(){};

	    virtual ~TriangleN3(){};

	    /**
	     * @brief constructor, calculates the face normal from vertex data
	     */
	    TriangleN3(const rw::math::Vector3D<T>& p1,
	    		   const rw::math::Vector3D<T>& p2,
	    		   const rw::math::Vector3D<T>& p3,
	    		   const rw::math::Vector3D<T>& n1,
	    		   const rw::math::Vector3D<T>& n2,
	    		   const rw::math::Vector3D<T>& n3):
	    			   _triN0(p1,p2,p3)
	    {
			   _vertexNormals[0]=n1;
			   _vertexNormals[1]=n2;
			   _vertexNormals[2]=n3;
	    }

	    /**
	     * @brief constructor
	     */
	    TriangleN3(const Triangle<T>& t,
	    		   const rw::math::Vector3D<T>& n1,
	    		   const rw::math::Vector3D<T>& n2,
	    		   const rw::math::Vector3D<T>& n3):
	    			   _triN0(t)
	    {
	 		   _vertexNormals[0]=n1;
	 		   _vertexNormals[1]=n2;
	 		   _vertexNormals[2]=n3;
	    }

	    /**
	     * @brief get normal of vertice \b i
	     * @param i [in] index of vertice
	     * @return normal of the i'th vertice
	     */
		rw::math::Vector3D<T>& getNormal(size_t i){
			return _vertexNormals[i];
		}

		/**
		 * @brief return vertex normal of the vertex specified by i
		 */
		const rw::math::Vector3D<T>& getNormal(size_t i) const{
			return _vertexNormals[i];
		}

      // inheritet functions from Triangle
		//! @copydoc Triangle::getVertex
        virtual rw::math::Vector3D<T>& getVertex(size_t i){ return _triN0.getVertex(i); };
        //! @copydoc Triangle::getVertex
        virtual const rw::math::Vector3D<T>& getVertex(size_t i) const { return _triN0.getVertex(i); };
        //! @copydoc Triangle::operator[]
        const rw::math::Vector3D<T>& operator[](size_t i) const { return _triN0.getVertex(i); };
        //! @copydoc Triangle::calcFaceNormal
        rw::math::Vector3D<T> calcFaceNormal() const{
            return _triN0.calcFaceNormal();
        };

		/**
         * @brief Returns TriangleN2 transformed by t3d.
         */
        TriangleN3<T> transform(const rw::math::Transform3D<T>& t3d) const {
            return TriangleN3<T>(_triN0.transform(t3d), t3d.R()*_vertexNormals[0], t3d.R()*_vertexNormals[1], t3d.R()*_vertexNormals[2] );
        }

        inline const Triangle<T>& getTriangle() const { return _triN0; }
        inline Triangle<T>& getTriangle() { return _triN0; }

	};
	// @}
} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
