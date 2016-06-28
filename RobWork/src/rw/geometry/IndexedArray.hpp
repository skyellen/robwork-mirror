#ifndef INDEXEDARRAY_HPP_
#define INDEXEDARRAY_HPP_

#include <vector>
#include <cstddef>


template<class OBJ, class T=int>
class IndexedArray {
private:
	const std::vector<OBJ> *_objArr;
	const std::vector<T>& _idxArr;
public:

	IndexedArray(const std::vector<OBJ> *objArr,
				 const std::vector<T>& idxArr):
		_objArr(objArr),
		_idxArr(idxArr)
	{
	}

	virtual ~IndexedArray(){}

	const std::vector<T>& getIndexes() const{
		return _idxArr;
	}

	const OBJ& operator[](T i) const {
		return (*_objArr)[ _idxArr[i] ];
	}

	size_t size() const {
		return _idxArr.size();
	}

};



#endif /*INDEXEDARRAY_HPP_*/
