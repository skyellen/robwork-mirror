#ifndef RW_GEOMETRY_TRIANGLE_HPP_
#define RW_GEOMETRY_TRIANGLE_HPP_

#include <rw/math/Vector3D.hpp>

namespace rw {
namespace geometry {

	typedef enum {N0,N1,N3,N4} TriType;

	/**
	 * @brief plain triangle class. The second template argument specify
	 * the number of normals associated with the triangle. 
	 *  The triangle vertices should be arranged counter clock whise.
	 */
	template <class T=double, TriType TRI=N0>
	class Triangle {
	protected:
		rw::math::Vector3D<T> _vertices[3];
		
	public:
		
		typedef T value_type;
		
	    //@brief default constructor
	    Triangle(){};
	
	    /**
	     * @brief constructor
	     * @param  
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
	    Triangle(const Triangle<T,TRI>& f)
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
		const rw::math::Vector3D<T>&  operator[](size_t i) const { 
			return _vertices[i]; 
		}
		
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
	template <class T>
	class Triangle<T,N1> : public Triangle<T,N0> 
	{
	protected:
		rw::math::Vector3D<T> _faceNormal;
		
	public:
		
	    //@brief default constructor
	    Triangle(){};
	    
	    /**
	     * @brief constructor, calculates the face normal from vertex data 
	     */
	    Triangle(const rw::math::Vector3D<T>& p1, 
	    		 const rw::math::Vector3D<T>& p2, 
	    		 const rw::math::Vector3D<T>& p3):
	    		   Triangle<T,N0>(p1,p2,p3)
	    {
	        rw::math::Vector3D<T> n = cross(rw::math::Vector3D<T>(p2-p1), rw::math::Vector3D<T>(p3-p1));
	        n = normalize(n);
	    	_faceNormal = n;
	    }
	    
	    /**
	     * @brief constructor
	     */
	    Triangle(const rw::math::Vector3D<T>& p1, 
	    		 const rw::math::Vector3D<T>& p2, 
	    		 const rw::math::Vector3D<T>& p3,
	    		 const rw::math::Vector3D<T>& n):
	    			   Triangle<T,N0>(p1,p2,p3)
	    {
	    	_faceNormal = n;
	    }
	
	    /**
	     * @brief destructor
	     */
	    virtual ~Triangle(){};
	    
	    /**
	     * @brief Copy constructor
	     */
	    template<class A>
	    Triangle(const Triangle<T>& t):
	    			Triangle<T,N0>(t)
	    {
	    	_faceNormal = Triangle<T,N0>::calcFaceNormal();
	    }
	    
	    /**
	     * @brief constructor
	     */
	    template<class A>
	    Triangle(const Triangle<T>& t,
	    		   const rw::math::Vector3D<A>& n):
	    			   Triangle<T,N0>(t)
	    {
	    	_faceNormal = n;
	    }
	
	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    template<class A>
	    Triangle(Triangle<A,N1> f){
	    	Triangle<T,N0>::_vertices[0] = f.getVertex(0);
	    	Triangle<T,N0>::_vertices[1] = f.getVertex(1);
	    	Triangle<T,N0>::_vertices[2] = f.getVertex(2);
	        _faceNormal = f.getFaceNormal();
	    };
	
	    /**
	     * @brief returns the facenormal of this triangle
	     */
		rw::math::Vector3D<T> getFaceNormal(){
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
			T d = dot(_faceNormal, Triangle<T,N0>::_vertices[0]);
			return dot(_faceNormal,x) - d;
		}

	};

	template <class T>
	class Triangle<T,N3> : public Triangle<T,N0> 
	{
	protected:
		rw::math::Vector3D<T> _vertexNormals[3];
		
	public:
		
	    //@brief default constructor
	    Triangle(){};
	    
	    virtual ~Triangle(){};
	    
	    /**
	     * @brief constructor, calculates the face normal from vertex data 
	     */
	    template<class A>
	    Triangle(const rw::math::Vector3D<A>& p1, 
	    		   const rw::math::Vector3D<A>& p2, 
	    		   const rw::math::Vector3D<A>& p3,
	    		   const rw::math::Vector3D<A>& n1, 
	    		   const rw::math::Vector3D<A>& n2, 
	    		   const rw::math::Vector3D<A>& n3):
	    			   Triangle<T>(p1,p2,p3)
	    {
			   _vertexNormals[0]=n1;
			   _vertexNormals[1]=n2;
			   _vertexNormals[2]= n3;
	    }
	        
	    /**
	     * @brief 
	     */
	    template<class A>
	    Triangle(const Triangle<T>& t,
	    		   const rw::math::Vector3D<A>& n1,
	    		   const rw::math::Vector3D<A>& n2,
	    		   const rw::math::Vector3D<A>& n3):
	    			   Triangle<T>(t)
	    {
	 		   _vertexNormals[0]=n1;
	 		   _vertexNormals[1]=n2;
	 		   _vertexNormals[2]= n3;
	    }
	
	    /**
	     * @brief copy constructor
	     *
	     * @param f [in] - The face that is to be copied.
	     */
	    template<class A>
	    Triangle(Triangle<A,N3> f){
	    	Triangle<T>::_vertices[0] = f.getVertex(0);
	    	Triangle<T>::_vertices[1] = f.getVertex(1);
	    	Triangle<T>::_vertices[2] = f.getVertex(2);
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
	
	};

} // geometry
} // rw

#endif /*TRIANGLE_HPP_*/
