/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_GEOMETRY_TRIANGLE_HPP_
#define RW_GEOMETRY_TRIANGLE_HPP_

#include <sandbox/geometry/GeometryData.hpp>
#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {

	//typedef enum {N0,N1,N3,N4} TriType;

    template <class T=double>
    class Triangle : public GeometryData {
    //private:
    //    Triangle(){};

    public:
        typedef T value_type;

        /**
         * @brief get vertex at index i
         */
        virtual rw::math::Vector3D<T>& getVertex(size_t i) = 0;

        /**
         * @brief get vertex at index i
         */
        virtual const rw::math::Vector3D<T>& getVertex(size_t i) const = 0;

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
         * that the triangle vertices are arranged counter lock wise.
         */
        virtual rw::math::Vector3D<T> calcFaceNormal() const = 0;

        GeometryData::GeometryType getType(){
            return GeometryData::TrianglePrim;
        };


        //virtual TriType getType() = 0;
    };

	/**
	 * @brief plain triangle class. The second template argument specify
	 * the number of normals associated with the triangle.
	 *  The triangle vertices should be arranged counter clock whise.
	 */
	template <class T=double>
	class TriangleN0: public Triangle<T> {
	protected:
	    rw::math::Vector3D<T> _vertices[3];

	public:

	    //@brief default constructor
	    TriangleN0(){};

	    /**
	     * @brief constructor
	     * @param
	     */
	    TriangleN0(const rw::math::Vector3D<T>& p1,
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
	    TriangleN0(const Triangle<T>& f)
	    {
	        _vertices[0] = f.getVertex(0);
	        _vertices[1] = f.getVertex(1);
	        _vertices[2] = f.getVertex(2);
	    };

	    /**
	     * @brief destructor
	     */
	    virtual ~TriangleN0(){};

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

		//TriType getType(){ return N0; };

		/**
		 * @brief calculates the face normal of this triangle. It is assumed
		 * that the triangle vertices are arranged counter lock wise.
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

	};

	/**
	 * @brief Triangle facet. triangle class of type N1, which means that
	 * beside the plain triangle the face normal of the triangle is saved
	 * with the facenormal.
	 */
	template <class T=double>
	class TriangleN1 : public Triangle<T>
	{
	protected:
	    TriangleN0<T> _triN0;
		rw::math::Vector3D<T> _faceNormal;

	public:

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
         * @brief Copy constructor
         */
        TriangleN1(const Triangle<T>& t):
            _triN0(t),
            _faceNormal(_triN0.calcFaceNormal())
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
         * @brief copy constructor
         *
         * @param f [in] - The face that is to be copied.
         */
        TriangleN1(const TriangleN1<T>& f):
            _triN0(f),
            _faceNormal(f.getFaceNormal())
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

		rw::math::Vector3D<T>& getVertex(size_t i){ return _triN0.getVertex(i); };

        const rw::math::Vector3D<T>& getVertex(size_t i) const { return _triN0.getVertex(i); };

        rw::math::Vector3D<T> calcFaceNormal() const{
            return _triN0.calcFaceNormal();
        };

	};

	template <class T=double>
	class TriangleN3 : public Triangle<T>
	{
	protected:
	    TriangleN0<T> _triN0;
		rw::math::Vector3D<T> _vertexNormals[3];

	public:

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
	     * @brief
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
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    TriangleN3(const TriangleN3<T>& f):
	        _triN0(f)
	    {
	        _vertexNormals[0] = f.getNormal(0);
	        _vertexNormals[1] = f.getNormal(1);
	        _vertexNormals[2] = f.getNormal(2);
	    };

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

        virtual rw::math::Vector3D<T>& getVertex(size_t i){ return _triN0.getVertex(i); };

        virtual const rw::math::Vector3D<T>& getVertex(size_t i) const { return _triN0.getVertex(i); };

        const rw::math::Vector3D<T>& operator[](size_t i) const { return _triN0.getVertex(i); };

        rw::math::Vector3D<T> calcFaceNormal() const{
            return _triN0.calcFaceNormal();
        };

	};

} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
