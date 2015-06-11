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

#ifndef RWLIBS_MATHEMATICA_TOEXPRESSION_HPP_
#define RWLIBS_MATHEMATICA_TOEXPRESSION_HPP_

/**
 * @file ToExpression.hpp
 *
 * \copydoc rwlibs::mathematica::ToExpression
 */

#include "Mathematica.hpp"

namespace rwlibs {
namespace mathematica {
//! @addtogroup mathematica

//! @{
//! @brief Representation of the Mathematica %ToExpression function.
class ToExpression: public Mathematica::FunctionBase {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<ToExpression> Ptr;

	/**
	 * @brief Get expression from a string.
	 * @param expression [in] the expression as a string.
	 */
	ToExpression(const char* expression);

	/**
	 * @brief Get expression from a string.
	 * @param expression [in] the expression as a string.
	 */
	ToExpression(const std::string& expression);

	/**
	 * @brief Get expression from a string.
	 * @param expression [in] the expression as a Mathematica string.
	 */
	ToExpression(const Mathematica::String& expression);

	//! @brief Destructor.
	virtual ~ToExpression();

	//! @copydoc Mathematica::FunctionBase::getArguments
	std::list<rw::common::Ptr<const Mathematica::Expression> > getArguments() const;

	//! @copydoc Mathematica::Expression::clone
	Mathematica::Expression::Ptr clone() const;

private:
	rw::common::Ptr<Mathematica::String> _expression;
};
//! @}
} /* namespace mathematica */
} /* namespace rwlibs */
#endif /* RWLIBS_MATHEMATICA_TOEXPRESSION_HPP_ */
