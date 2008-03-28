#ifndef INDEXEDTRIANGLE_HPP_
#define INDEXEDTRIANGLE_HPP_

#include <rw/math/Vector3D.hpp>

template<TriType T>
class IndexedTriangle {
protected:
	size_t _vertices[3];
	
public:
	
    //@brief default constructor
	IndexedTriangle(){};

    /**
     * @brief 
     */
    IndexedTriangle(size_t p1, size_t p2, size_t p3) 
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
    IndexedTriangle(const IndexedTriangle& f){
        _vertices[0] = f.getVertexIdx(0);
        _vertices[1] = f.getVertexIdx(1);
        _vertices[2] = f.getVertexIdx(2);
    };

    
    size_t getVertexIdx(size_t i) const {
		return _vertices[i];
	}
		
};

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


#endif /*TRIANGLE_HPP_*/
