#ifndef RW_GEOMETRY_INDEXEDTRIANGLE_HPP_
#define RW_GEOMETRY_INDEXEDTRIANGLE_HPP_

#include <rw/math/Vector3D.hpp>

#include "Triangle.hpp"

namespace rw {
namespace geometry {

    template <class T>
    class IndexedTriangle {
    public:
        typedef T value_type;

        /**
         * @brief returns the index of vertex i of the triangle
         */
        virtual int& getVertexIdx(int i) = 0;

        /**
         * @brief returns the index of vertex i of the triangle
         */
        virtual int getVertexIdx(int i) const = 0;


        /**
         * @brief get vertex at index i
         */
        const int&  operator[](int i) const {
            return getVertexIdx(i);
        }

        /**
         * @brief get vertex at index i
         */
        int& operator[](int i){
            return getVertexIdx(i);
        }

        //virtual rw::math::Vector3D<T> calcFaceNormal()
    };

	template<class T>
	class IndexedTriangleN0 : IndexedTriangle<T> {
	protected:
		int _vertices[3];

	public:
	    //@brief default constructor

	    IndexedTriangleN0(){};

	    /**
	     * @brief
	     */
	    IndexedTriangleN0(int p1, int p2, int p3)
	    {
	    	_vertices[0] = p1;
	    	_vertices[1] = p2;
	    	_vertices[2] = p3;
	    }

	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    IndexedTriangleN0(const IndexedTriangleN0& f){
	        _vertices[0] = f[0];
	        _vertices[1] = f[1];
	        _vertices[2] = f[2];
	    };

	    /**
	     * @brief returns the index of vertex i of the triangle
	     */
	    int& getVertexIdx(int i) {
			return _vertices[i];
		}

	    /**
         * @brief returns the index of vertex i of the triangle
         */
        int getVertexIdx(int i) const  {
            return _vertices[i];
        }

		/**
		 * @brief tests wheather the point x is inside the triangle
		 */
		bool isInside(const rw::math::Vector3D<T>& x, const std::vector<rw::math::Vector3D<T> >& verts){
			using namespace rw::math;
			// calc vectors
			Vector3D<T> v0 = verts[_vertices[2]] - verts[_vertices[0]];
			Vector3D<T> v1 = verts[_vertices[1]] - verts[_vertices[0]];
			Vector3D<T> v2 = x - verts[_vertices[0]];
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

   template<class T>
    class IndexedTriangleN1 : IndexedTriangle<T> {
    protected:
        IndexedTriangleN0<T> _triN0;
        rw::math::Vector3D<T> _faceNormal;

    public:
        //@brief default constructor

        IndexedTriangleN1(){};

        /**
         * @brief
         */
        IndexedTriangleN1(int p1, int p2, int p3)
        {
            _vertices[0] = p1;
            _vertices[1] = p2;
            _vertices[2] = p3;
        }

        /**
         * @brief
         */
        IndexedTriangleN1(int p1, int p2, int p3)
        {
            _vertices[0] = p1;
            _vertices[1] = p2;
            _vertices[2] = p3;
        }

        /**
         * @brief copy constructor
         *
         * @param f [in] - The face that is to be copied.
         */
        IndexedTriangleN1(const IndexedTriangleN1& f){
            _vertices[0] = f[0];
            _vertices[1] = f[1];
            _vertices[2] = f[2];
        };

        /**
         * @brief returns the index of vertex i of the triangle
         */
        int& getVertexIdx(int i) {
            return _vertices[i];
        }

        /**
         * @brief returns the index of vertex i of the triangle
         */
        int getVertexIdx(int i) const  {
            return _vertices[i];
        }

        /**
         * @brief tests wheather the point x is inside the triangle
         */
        bool isInside(const rw::math::Vector3D<T>& x, const std::vector<rw::math::Vector3D<T> >& verts){
            using namespace rw::math;
            // calc vectors
            Vector3D<T> v0 = verts[_vertices[2]] - verts[_vertices[0]];
            Vector3D<T> v1 = verts[_vertices[1]] - verts[_vertices[0]];
            Vector3D<T> v2 = x - verts[_vertices[0]];
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


#ifdef OLD_STUFF
	template<>
	class IndexedTriangle<N1>: public IndexedTriangle<N0> {
	protected:
		size_t _faceNormal;
	public:

	    //@brief default constructor
		IndexedTriangle(){};

	    /**
	     * @brief
	     */
	    IndexedTriangle(size_t p1, size_t p2, size_t p3, size_t n)
	    {
	    	_vertices[0] = p1;
	    	_vertices[1] = p2;
	    	_vertices[2] = p3;
	    	_faceNormal = n;
	    }

	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    IndexedTriangle(const IndexedTriangle<N1>& f){
	        _vertices[0] = f.getVertexIdx(0);
	        _vertices[1] = f.getVertexIdx(1);
	        _vertices[2] = f.getVertexIdx(2);
	        _faceNormal = f.getFaceNormalIdx();
	    };


	    size_t getFaceNormalIdx() const {
			return _faceNormal;
		}

	};

	template<>
	class IndexedTriangle<N3>: public IndexedTriangle<N0> {
	protected:
		size_t _normals[3];
	public:

	    //@brief default constructor
		IndexedTriangle(){};

	    /**
	     * @brief
	     */
	    IndexedTriangle(size_t p1, size_t p2, size_t p3,
	    				size_t n1, size_t n2, size_t n3):
	    					IndexedTriangle<N0>(p1,p2,p3)
	    {
	    	_normals[0] = n1;
	    	_normals[1] = n2;
	    	_normals[2] = n3;
	    }

	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    IndexedTriangle(const IndexedTriangle<N3>& f){
	        _vertices[0] = f.getVertexIdx(0);
	        _vertices[1] = f.getVertexIdx(1);
	        _vertices[2] = f.getVertexIdx(2);
	        _normals[0] = f.getVertexNormalIdx(0);
	        _normals[1] = f.getVertexNormalIdx(1);
	        _normals[2] = f.getVertexNormalIdx(2);
	    };


	    size_t getVertexNormalIdx(size_t i) const {
			return _normals[i];
		}

	};
#endif
} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
