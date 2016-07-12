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

#ifndef RWSIMLIBS_GUI_SIMULATORSTATISTICSWIDGET_HPP_
#define RWSIMLIBS_GUI_SIMULATORSTATISTICSWIDGET_HPP_

/**
 * @file SimulatorStatisticsWidget.hpp
 *
 * \copydoc rwsimlibs::gui::SimulatorStatisticsWidget
 */

#include <rw/common/Ptr.hpp>
#include <QWidget>

namespace rwsim { namespace log { class SimulatorStatistics; } }

namespace Ui { class SimulatorStatisticsWidget; }

class QItemSelection;

namespace rwsimlibs {
namespace gui {
//! @addtogroup rwsimlibs_gui

//! @{
//! @brief Widget for visualisation of rwsim::log::SimulatorStatistics .
class SimulatorStatisticsWidget: public QWidget {
    Q_OBJECT
public:
	/**
	 * @brief Construct new widget for statistics.
	 * @param stats [in] the statistics.
	 * @param parent [in] (optional) the parent Qt widget. Ownership is shared by the caller and the parent widget if given.
	 */
	SimulatorStatisticsWidget(rw::common::Ptr<const rwsim::log::SimulatorStatistics> stats, QWidget* parent = 0);

	//! @brief Destructor.
	virtual ~SimulatorStatisticsWidget();

	/**
	 * @brief Get name of widget.
	 * @return the name.
	 */
	std::string getName() const;

private slots:
	void changed(const QItemSelection& newSelection, const QItemSelection& oldSelection);

private:
    Ui::SimulatorStatisticsWidget* const _ui;
    rw::common::Ptr<const rwsim::log::SimulatorStatistics> _stats;
    class MathematicaPlotWidget* const _mathematica;
};
//! @}
} /* namespace gui */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_GUI_SIMULATORSTATISTICSWIDGET_HPP_ */
