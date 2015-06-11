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

#ifndef RWLIBS_MATHEMATICA_LIST_HPP_
#define RWLIBS_MATHEMATICA_LIST_HPP_

/**
 * @file List.hpp
 *
 * \copydoc rwlibs::mathematica::List
 */

#include "Mathematica.hpp"

#include <rw/common/macros.hpp>

namespace rwlibs {
namespace mathematica {
//! @addtogroup mathematica

//! @{
//! @brief Representation of the Mathematica %List function.
class List: public Mathematica::FunctionBase {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<List> Ptr;

	//! @brief Construct empty list.
	List();

	/**
	 * @brief Construct list from a vector of arguments.
	 * @param args [in] the vector of arguments
	 */
	List(const std::vector<Mathematica::Expression::Ptr>& args);

	/**
	 * @brief Construct list from a list of arguments.
	 * @param args [in] the list of arguments
	 */
	List(const std::list<rw::common::Ptr<const Mathematica::Expression> >& args);

#if __cplusplus >= 201103L
	/**
	 * @brief Construct new list from variable number of arguments.
	 * @param expressions [in] the expressions.
	 * @note Only available for C++11
	 */
	template<typename... Exp>
	List(const Exp&... expressions):
		Mathematica::FunctionBase("List")
	{
		std::list<Mathematica::AutoExpression> exps;
		Mathematica::Expression::toList(exps, expressions...);
		_data.reserve(exps.size());
		for(const Mathematica::AutoExpression& exp : exps) {
			_data.push_back(exp.expression());
		}
	}
#endif

	//! @brief Destructor.
	virtual ~List();

	//! @copydoc Mathematica::FunctionBase::getArguments
	virtual std::list<rw::common::Ptr<const Mathematica::Expression> > getArguments() const;

	//! @copydoc Mathematica::Expression::clone
	virtual Mathematica::Expression::Ptr clone() const;

	/**
	 * @brief Get expression at some location in the list.
	 * @param i [in] the index.
	 * @return the expression.
	 */
	Mathematica::Expression::Ptr operator[](std::size_t i);

	/**
	 * @brief Get expression at some location in the list.
	 * @param i [in] the index.
	 * @return the expression.
	 */
	rw::common::Ptr<const Mathematica::Expression> operator[](std::size_t i) const;

	/**
	 * @brief Add an expression to list.
	 * @param expression [in] the expression to add.
	 * @return a reference to the list for chaining.
	 */
	List& add(Mathematica::Expression::Ptr expression);

	/**
	 * @brief Add an expression to list.
	 * @param expression [in] the expression to add.
	 * @return a reference to the list for chaining.
	 */
	List& add(const Mathematica::AutoExpression& expression);

	/**
	 * @brief Overwrite a specific expression in list.
	 * @param i [in] the index.
	 * @param expression [in] the expression.
	 */
	void set(std::size_t i, Mathematica::Expression::Ptr expression);

	/**
	 * @brief Parse a list from a generic expression.
	 * @param exp [in] the expression to parse.
	 * @return a new list.
	 */
	static List::Ptr fromExpression(const Mathematica::Expression& exp);

	/**
	 * @brief Convert a RobWork vector to a Mathematica list.
	 * @param vector [in] the vector.
	 * @return the list expression.
	 */
	template<typename T>
	static List::Ptr toList(const T& vector) {
		const List::Ptr list = rw::common::ownedPtr(new List());
		for (std::size_t i = 0; i < vector.size(); i++) {
			list->add(vector[i]);
		}
		return list;
	}

private:
	std::vector<Mathematica::Expression::Ptr> _data;
};
//! @}
} /* namespace mathematica */
} /* namespace rwlibs */
#endif /* RWLIBS_MATHEMATICA_LIST_HPP_ */
