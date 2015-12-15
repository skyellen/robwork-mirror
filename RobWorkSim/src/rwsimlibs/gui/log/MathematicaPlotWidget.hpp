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

#ifndef RWSIMLIBS_GUI_MATHEMATICAPLOTWIDGET_HPP_
#define RWSIMLIBS_GUI_MATHEMATICAPLOTWIDGET_HPP_

/**
 * @file MathematicaPlotWidget.hpp
 *
 * \copydoc rwsimlibs::gui::MathematicaPlotWidget
 */

#include <QLabel>

namespace rwsimlibs {
namespace gui {
//! @addtogroup rwsimlibs_gui

//! @{
//! @brief Widget for Mathematica plots.
class MathematicaPlotWidget: public QLabel {
public:
	/**
	 * @brief Construct new widget for a Mathematica plot.
	 * @param parent [in] (optional) the parent Qt widget. Ownership is shared by the caller and the parent widget if given.
	 */
	MathematicaPlotWidget(QWidget* parent = 0);

	//! @brief Destructor.
	virtual ~MathematicaPlotWidget();

	/**
	 * @brief Do a ListPlot with the given x- and y-values.
	 * @param x [in] the x-values.
	 * @param y [in] the y-values.
	 */
	void listPlot(const std::vector<double>& x, const std::vector<double>& y);

	/**
	 * @brief Resize plot if widget size is changed.
	 * @param event [in] the resize event.
	 */
	void resizeEvent(QResizeEvent* event);

private:
	void render();
	void setStyle() const;

private:
	struct Kernel;
	struct RenderInfo;
	Kernel* const _kernel;
	RenderInfo* const _render;
};
//! @}
} /* namespace gui */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_GUI_MATHEMATICAPLOTWIDGET_HPP_ */
