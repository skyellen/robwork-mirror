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

#ifndef RWLIBS_MATHEMATICA_IMAGE_HPP_
#define RWLIBS_MATHEMATICA_IMAGE_HPP_

/**
 * @file rwlibs/mathematica/Image.hpp
 *
 * \copydoc rwlibs::mathematica::Image
 */

#include "Mathematica.hpp"

namespace rw { namespace common { class PropertyMap; } }
namespace rw { namespace sensor { class Image; } }

namespace rwlibs {
namespace mathematica {

class Rule;

//! @addtogroup mathematica

//! @{
//! @brief Representation of the Mathematica %Image function.
class Image: public Mathematica::FunctionBase {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<Image> Ptr;

	/**
	 * @brief Construct a Image expression from a RobWork image.
	 * @param image [in] the image.
	 */
	Image(rw::common::Ptr<const rw::sensor::Image> image);

	/**
	 * @brief Construct a Image expression.
	 * @param data [in] the data expression (e.g. a variable name, table/list expression, or data array).
	 */
	Image(const Mathematica::Expression& data);

	/**
	 * @brief Construct a Image expression.
	 * @param data [in] the data expression (e.g. a variable name, table/list expression, or data array).
	 * @param options [in] the options to use (such as ColorSpace, ImageSize etc.)
	 */
	Image(const Mathematica::Expression& data, const rw::common::PropertyMap& options);

	//! @brief Destructor.
	virtual ~Image();

	//! @copydoc Mathematica::FunctionBase::getArguments
	virtual std::list<rw::common::Ptr<const Mathematica::Expression> > getArguments() const;

	//! @copydoc Mathematica::Expression::clone
	virtual Mathematica::Expression::Ptr clone() const;

	/**
	 * @brief Set ImageSize option.
	 * @param width [in] the width.
	 * @param height [in] the height.
	 */
	void setImageSize(std::size_t width, std::size_t height);

	/**
	 * @brief Construct RobWork Image from Mathematica Image expression.
	 * @param expression [in] the expression to parse.
	 * @return the parsed Image expression.
	 * @throws rw::common::Exception if parsing fails.
	 */
	static rw::common::Ptr<rw::sensor::Image> toRobWorkImage(const Mathematica::Expression& expression);

private:
	Image();

private:
	Mathematica::Expression::Ptr _data;
	Mathematica::String::Ptr _type;
	std::list<rw::common::Ptr<Rule> > _rules;
};
//! @}
} /* namespace mathematica */
} /* namespace rwlibs */
#endif /* RWLIBS_MATHEMATICA_IMAGE_HPP_ */
