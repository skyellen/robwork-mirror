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

#ifndef RWLIBS_MATHEMATICA_LISTPLOT_HPP_
#define RWLIBS_MATHEMATICA_LISTPLOT_HPP_

/**
 * @file ListPlot.hpp
 *
 * \copydoc rwlibs::mathematica::ListPlot
 */

#include "Mathematica.hpp"

#if __cplusplus >= 201103L
#include <initializer_list>
#include "List.hpp"
#include "Rule.hpp"
#endif

namespace rw { namespace common { class PropertyMap; } }

namespace rwlibs {
namespace mathematica {

class Rule;

//! @addtogroup mathematica

//! @{
/**
 * @brief Representation of the Mathematica %ListPlot function.
 */
class ListPlot: public Mathematica::FunctionBase {
public:
	//! @brief Smart pointer type.
	typedef rw::common::Ptr<ListPlot> Ptr;

	/**
	 * @brief Construct a ListPlot expression.
	 * @param data [in] the data expression (e.g. a variable name, table/list expression, or data array).
	 */
	ListPlot(const Mathematica::Expression& data);

	/**
	 * @brief Construct a ListPlot expression.
	 * @param data [in] the data expression (e.g. a variable name, table/list expression, or data array).
	 * @param options [in] the options to use (such as PlotRange, AxesLabels etc.)
	 */
	ListPlot(const Mathematica::Expression& data, const std::list<rw::common::Ptr<Rule> >& options);

	/**
	 * @brief Construct a ListPlot expression.
	 * @param x [in] a list of x-values.
	 * @param y [in] a list of y-values.
	 * @param options [in] (optional) the options to use (such as PlotRange, AxesLabels etc.)
	 */
	ListPlot(const std::vector<double>& x, const std::vector<double>& y, const std::list<rw::common::Ptr<Rule> >& options = std::list<rw::common::Ptr<Rule> >());

#if __cplusplus >= 201103L
	/**
	 * @brief Construct a ListPlot expression.
	 * @param data [in] the data expression (e.g. a variable name, table/list expression, or data array).
	 * @param options [in] the options to use (such as PlotRange, AxesLabels etc.)
	 * @note Only available for C++-11
	 */
	template<typename... Option>
	ListPlot(const std::initializer_list<std::initializer_list<double> >& data, const Option&... options):
	Mathematica::FunctionBase("ListPlot")
	{
		List::Ptr list = rw::common::ownedPtr(new List());
		for(const std::initializer_list<double>& val : data) {
			List::Ptr inner = rw::common::ownedPtr(new List());
			for(const double v : val) {
				inner->add(v);
			}
			list->add(inner);
		}
		_data = list;
		toList<Rule>(_options,options...);
	}
#endif

	//! @brief Destructor.
	virtual ~ListPlot();

	//! @copydoc Mathematica::FunctionBase::getArguments
	virtual std::list<rw::common::Ptr<const Mathematica::Expression> > getArguments() const;

	//! @copydoc Mathematica::Expression::clone
	virtual Mathematica::Expression::Ptr clone() const;

	/**
	 * @brief Set an option.
	 * @param name [in] the name of the option.
	 * @param value [in] the value to set.
	 */
	void option(const std::string& name, const Mathematica::Expression& value);

	/**
	 * @brief Set the image size option.
	 * @param width [in] the width.
	 * @param height [in] the height.
	 */
	void setImageSize(int width, int height);

	/**
	 * @brief Construct ListPlot from existing expression.
	 * @param expression [in] the expression to parse as ListPlot.
	 * @return the parsed ListPlot expression.
	 * @throws rw::common::Exception if parsing fails.
	 */
	static ListPlot fromExpression(const Mathematica::Expression& expression);

private:
	ListPlot();

	Mathematica::Expression::Ptr _data;
	std::list<rw::common::Ptr<Rule> > _options;
};
//! @}
} /* namespace mathematica */
} /* namespace rwlibs */
#endif /* RWLIBS_MATHEMATICA_LISTPLOT_HPP_ */
