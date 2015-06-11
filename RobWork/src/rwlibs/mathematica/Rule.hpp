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

#ifndef RWLIBS_MATHEMATICA_RULE_HPP_
#define RWLIBS_MATHEMATICA_RULE_HPP_

/**
 * @file Rule.hpp
 *
 * \copydoc rwlibs::mathematica::Rule
 */

#include "Mathematica.hpp"

namespace rw { namespace common { class PropertyMap; } }
namespace rw { namespace common { class PropertyBase; } }

namespace rwlibs {
namespace mathematica {
//! @addtogroup mathematica

//! @{
//! @brief Representation of a Mathematica %Rule.
class Rule: public Mathematica::FunctionBase {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<Rule> Ptr;

	/**
	 * @brief Construct a new Rule expression.
	 * @param name [in] the name of the rule.
	 * @param value [in] the value of the rule.
	 */
	Rule(const Mathematica::Symbol& name, const Mathematica::AutoExpression& value);

	/**
	 * @brief Construct a Mathematica Rule based on RobWork property.
	 * @param property [in] the property.
	 */
	Rule(const rw::common::PropertyBase& property);

	//! @brief Destructor.
	virtual ~Rule();

	//! @copydoc Mathematica::FunctionBase::getArguments
	std::list<rw::common::Ptr<const Mathematica::Expression> > getArguments() const;

	//! @copydoc Mathematica::Expression::clone
	Mathematica::Expression::Ptr clone() const;

	/**
	 * @brief Get the symbol name of the rule.
	 * @return the name of the rule.
	 */
	std::string getId() const;

	/**
	 * @brief Set the value of the rule.
	 * @param value [in] new value.
	 */
	void setValue(const Mathematica::Expression& value);

	/**
	 * @brief Convert a PropertyMap to a list of Mathematica rules.
	 * @param options [in] the map with options.
	 * @return a list of Rule expressions.
	 * @throws rw::common::Exception if a property not supported.
	 */
	static std::list<Rule::Ptr> toRules(const rw::common::PropertyMap& options);

	/**
	 * @brief Construct PropertyMap from existing expression.
	 * @param rules [in] the expressions to parse as Rules.
	 * @return the PropertyMap.
	 * @throws rw::common::Exception if parsing fails.
	 */
	static rw::common::Ptr<rw::common::PropertyMap> toPropertyMap(const std::list<rw::common::Ptr<const Mathematica::Expression> >& rules);

	/**
	 * @brief Convert a Mathematica rule expression to a Property.
	 * @param rule [in] the rule expression.
	 * @return the property.
	 */
	static rw::common::Ptr<rw::common::PropertyBase> toProperty(const Mathematica::Expression& rule);

private:
	Mathematica::Symbol::Ptr _name;
	Mathematica::Expression::Ptr _value;
};
//! @}
} /* namespace mathematica */
} /* namespace rwlibs */
#endif /* RWLIBS_MATHEMATICA_RULE_HPP_ */
