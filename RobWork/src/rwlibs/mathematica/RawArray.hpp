/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_MATHEMATICA_RAWARRAY_HPP_
#define RWLIBS_MATHEMATICA_RAWARRAY_HPP_

/**
 * @file RawArray.hpp
 *
 * \copydoc rwlibs::mathematica::RawArray
 */

#include "Mathematica.hpp"
#include "List.hpp"

#include <rw/common/macros.hpp>

#include <boost/multi_array.hpp>

namespace rwlibs {
namespace mathematica {
//! @addtogroup mathematica

//! @{
//! @brief Utility for the RawArray type.
class RawArrayUtil {
public:
	/**
	 * @brief Detect the dimensions of an array given as an expression.
	 * @param exp [in] the expression.
	 * @return the dimensions.
	 */
	static std::size_t detectDimensions(const Mathematica::FunctionBase& exp);

	/**
	 * @brief Get the type of array.
	 * @return a symbol with the data type.
	 */
	template<typename T>
	static Mathematica::Symbol::Ptr getType() {
		RW_THROW("Could not determine type for RawArray.");
	}

private:
	RawArrayUtil() {};
	virtual ~RawArrayUtil() {};
};

/**
 * @brief Construct Byte symbol from unsigned char.
 * @return Byte symbol.
 */
template<>
inline Mathematica::Symbol::Ptr RawArrayUtil::getType<unsigned char>() {
	return rw::common::ownedPtr(new Mathematica::Symbol("Byte"));
}

//! @brief Representation of a N-dimensional %Mathematica array with fixed depth.
template <typename T, std::size_t Dim>
class RawArray: public Mathematica::Array<T> {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<RawArray<T,Dim> > Ptr;

	//! @brief The underlying array type.
	typedef boost::multi_array<T, Dim> ArrayType;

	/**
	 * @brief Construct new array.
	 * @param array [in] the array.
	 */
	RawArray(const ArrayType& array):
		_array(array),
		_size(new int[Dim])
	{
		for (std::size_t i = 0; i < Dim; i++) {
			_size[i] = _array.shape()[i];
		}
		_type = RawArrayUtil::getType<T>();
	}

	/**
	 * @brief Construct new array with specific shape.
	 * @param shape [in] the shape.
	 */
	RawArray(boost::array<typename ArrayType::index, Dim> shape):
		_array(shape),
		_size(new int[Dim])
	{
		for (std::size_t i = 0; i < Dim; i++) {
			_size[i] = shape[i];
		}
		_type = RawArrayUtil::getType<T>();
	}

	/**
	 * @brief Construct new array with specific shape.
	 * @param shape [in] the shape.
	 */
	RawArray(const std::vector<std::size_t> shape):
		_size(new int[Dim])
	{
		for (std::size_t i = 0; i < Dim; i++) {
			_size[i] = shape[i];
		}
		boost::array<typename ArrayType::index,Dim> bshape;
		typename boost::array<typename ArrayType::index,Dim>::size_type i;
		for (i = 0; i < Dim; i++) {
			bshape[i] = shape[i];
		}
		_array.resize(bshape);

		_type = RawArrayUtil::getType<T>();
		//boost::array<typename ArrayType::index,Dim> cur;
		//_list = createList(_array,0,cur,size());
	}

	/**
	 * @brief Construct RawArray from native Mathematica array.
	 * @param data [in] the data.
	 * @param shape [in] the shape.
	 */
	RawArray(const T* const data, const int* const shape):
		_size(new int[Dim])
	{
		for (std::size_t i = 0; i < Dim; i++) {
			_size[i] = shape[i];
		}
		boost::array<typename ArrayType::index,Dim> bshape;
		typename boost::array<typename ArrayType::index,Dim>::size_type i;
		int elements = 0;
		for (i = 0; i < Dim; i++) {
			bshape[i] = shape[i];
			if (i == 0)
				elements = shape[0];
			else
				elements *= shape[i];
		}
		_array.resize(bshape);
		std::copy(data,data+elements,_array.data());

		_type = RawArrayUtil::getType<T>();
	}

	//! @brief Destructor.
	virtual ~RawArray() {
		delete[] _size;
	}

	/**
	 * @brief Get the underlying array.
	 * @return a reference to the array.
	 */
	const ArrayType& getArray() const {
		return _array;
	}

	/**
	 * @brief Set a value.
	 * @param indexes [in] the indices.
	 * @param value [in] the value to set.
	 */
	void set(std::vector<std::size_t> indexes, T value) {
		RW_ASSERT(indexes.size() == Dim);
		boost::array<typename ArrayType::index,Dim> cur;
		for (std::size_t i = 0; i < Dim; i++)
			cur[i] = indexes[i];
		_array(cur) = value;
	}

	/**
	 * @brief Construct a new array from an expression.
	 * @param exp [in] the expression to parse.
	 * @return a new array.
	 */
	static RawArray<T, Dim>::Ptr fromExpression(const Mathematica::FunctionBase& exp) {
		if (exp.getName() != "RawArray")
			RW_THROW("Expected function with name RawArray, not " << exp.getName() << ".");
		const std::list<rw::common::Ptr<const Mathematica::Expression> > args = exp.getArguments();
		if (args.size() != 2)
			RW_THROW("Expected two arguments for RawArray, not " << args.size() << ".");

		// Find the shape
		boost::array<typename ArrayType::index,Dim> shape;
		rw::common::Ptr<const Mathematica::Expression> arrayArg = args.back();
		rw::common::Ptr<const Mathematica::FunctionBase> fct;
		bool cont = true;
		typename boost::array<typename ArrayType::index,Dim>::size_type dim = 0;
		while(cont) {
			cont = false;
			fct = arrayArg.cast<const Mathematica::FunctionBase>();
			if (!fct.isNull()) {
				const std::size_t args = fct->getArguments().size();
				if (args > 0) {
					shape[dim] = args;
					dim++;
					arrayArg = fct->getArguments().front();
					cont = true;
				}
			}
		}
		if (dim != Dim) {
			RW_THROW("Dim appear to be wrong.");
		}

		arrayArg = args.back();
		fct = arrayArg.cast<const Mathematica::FunctionBase>();
		const rw::common::Ptr<const List> list = List::fromExpression(*fct);

		RawArray<T,Dim>::Ptr array = rw::common::ownedPtr(new RawArray<T,Dim>(shape));
		const std::vector<std::size_t> size = array->getSize();
		boost::array<typename ArrayType::index,Dim> cur;
		setValues(array->_array,list,0,cur,size);
		return array;
	}

	/**
	 * @brief Set the values of an array recursively.
	 * @param array [in/out] the array.
	 * @param list  [in] the list at the current level.
	 * @param level [in] the current level.
	 * @param cur [in] the current indices.
	 * @param size [in] the shape of the array.
	 */
	static void setValues(ArrayType& array, rw::common::Ptr<const List> list, std::size_t level, boost::array<typename ArrayType::index,Dim> cur, const std::vector<std::size_t>& size) {
		const std::list<rw::common::Ptr<const Mathematica::Expression> >& args = list->getArguments();
		std::list<rw::common::Ptr<const Mathematica::Expression> >::const_iterator it = args.begin();
		if (level == Dim-1) {
			for (std::size_t i = 0; i < size[level]; i++) {
				cur[level] = i;
				const rw::common::Ptr<const Mathematica::Expression> exp = *it;
				const rw::common::Ptr<const Mathematica::Integer> integer = exp.cast<const Mathematica::Integer>();
				if (integer.isNull())
					RW_THROW("Expected integer at this level of the array.");
				array(cur) = (T)integer->value();
				it++;
			}
		} else {
			for (std::size_t i = 0; i < size[level]; i++) {
				cur[level] = i;
				const rw::common::Ptr<const Mathematica::Expression> exp = *it;
				const rw::common::Ptr<const Mathematica::FunctionBase> fct = exp.cast<const Mathematica::FunctionBase>();
				if (fct.isNull())
					RW_THROW("Expected function at this level of the array.");
				if (fct->getName() != "List")
					RW_THROW("Expected function with name \"List\", instead got \"" << fct->getName() << "\".");
				const rw::common::Ptr<const List> listChild = fct.cast<const List>();
				RW_ASSERT(!listChild.isNull());
				setValues(array,listChild,level+1,cur,size);
				it++;
			}
		}
	}

	/**
	 * @brief Create a List expression from an array.
	 * @param array [in] the array.
	 * @param level [in] the current level.
	 * @param cur [in] the current indices.
	 * @param size [in] the shape of the array.
	 * @return a list.
	 */
	static List::Ptr createList(const ArrayType& array, std::size_t level, boost::array<typename ArrayType::index,Dim> cur, const std::vector<std::size_t>& size) {
		const List::Ptr list = rw::common::ownedPtr(new List());
		if (level == Dim-1) {
			for (std::size_t i = 0; i < size[level]; i++) {
				cur[level] = i;
				list->add(rw::common::ownedPtr(new Mathematica::Integer(array(cur))));
			}
		} else {
			for (std::size_t i = 0; i < size[level]; i++) {
				cur[level] = i;
				list->add(createList(array,level+1,cur,size));
			}
		}
		return list;
	}

	/**
	 * @brief Get the shape.
	 * @return a list of sizes for each dimension.
	 */
	std::vector<std::size_t> getSize() const {
		std::vector<std::size_t> shape;
		for (std::size_t i = 0; i < Dim; i++)
			shape.push_back(_array.shape()[i]);
		return shape;
	}

	//! @copydoc Mathematica::Expression::out
	virtual void out(std::ostream& stream) const {
		stream << "RawArray[" << _type->getName() << ", << ";
		for (std::size_t i = 0; i < Dim; i++) {
			stream << _array.shape()[i];
			if (i != Dim-1)
				stream << "x";
		}
		stream << " array >>]";
	}

	//! @copydoc Mathematica::Expression::clone
	virtual Mathematica::Expression::Ptr clone() const {
		return rw::common::ownedPtr(new RawArray<T,Dim>(_array));
	}

	//! @copydoc Mathematica::Array::size
	virtual const int* size() const {
		return _size;
	}

	//! @copydoc Mathematica::Array::data
	virtual const T* data() const {
		return _array.data();
	}

	//! @copydoc Mathematica::Array::dimensions
	virtual int dimensions() const {
		return Dim;
	}

private:
	ArrayType _array;
	int* _size;
	Mathematica::Symbol::Ptr _type;
};

//! @brief Value for the size of a RawArray when size is not known at compile time.
const int Dynamic = -1;

//! @brief Representation of a N-dimensional %Mathematica array with dynamic depth.
template<typename T>
class RawArray<T,Dynamic>: public Mathematica::Array<T> {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<RawArray<T,Dynamic> > Ptr;

	/**
	 * @brief Construct new array with a dynamic dimensionality.
	 * @param data [in] the data.
	 * @param dims [in] the size of each dimension.
	 * @param depth [in] the number of dimensions.
	 */
	RawArray(const T* const data, const int* const dims, const int depth):
		_data(NULL), _dims(new int[depth]), _depth(depth)
	{
		int size = 0;
		if (depth > 0) {
			size = dims[0];
			for (int i = 1; i < depth; i++) {
				size *= dims[i];
			}
		}
		_data = new unsigned char[size];
		std::copy(data,data+size,_data);
		std::copy(dims,dims+depth,_dims);
		_type = RawArrayUtil::getType<T>();
	}

	//! @brief Destructor.
	virtual ~RawArray() {
		delete[] _data;
		delete[] _dims;
	}

	//! @copydoc Mathematica::Expression::out
	void out(std::ostream& stream) const {
		stream << "RawArray[" << _type->getName() << ", << ";
		for (std::size_t i = 0; i < _depth; i++) {
			stream << _dims[i];
			if (i != _depth-1)
				stream << "x";
		}
		stream << " array >>]";
	}

	//! @copydoc Mathematica::Expression::clone
	Mathematica::Expression::Ptr clone() const {
		return rw::common::ownedPtr(new RawArray<T,Dynamic>(_data,_dims,_depth));
	}

	//! @copydoc Mathematica::Array::size
	const int* size() const {
		return _dims;
	}

	//! @copydoc Mathematica::Array::data
	const T* data() const {
		return _data;
	}

	//! @copydoc Mathematica::Array::dimensions
	int dimensions() const {
		return _depth;
	}

private:
	T* _data;
	int* _dims;
	const int _depth;
	Mathematica::Symbol::Ptr _type;
};

//! @}
} /* namespace mathematica */
} /* namespace rwlibs */
#endif /* RWLIBS_MATHEMATICA_RAWARRAY_HPP_ */
