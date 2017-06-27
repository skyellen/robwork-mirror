/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSLIBS_ATASK_STRATEGYLIBRARYDIALOG_HPP_
#define RWSLIBS_ATASK_STRATEGYLIBRARYDIALOG_HPP_

/**
 * @file StrategyLibraryDialog.hpp
 *
 * \copydoc rwslibs::StrategyLibraryDialog
 */

#include <QDialog>

#include <rw/common/Ptr.hpp>

namespace rw { namespace models { class WorkCell; } }
namespace rw { namespace sensor { class FTSensor; } }
namespace rwlibs { namespace assembly { class AssemblyControlStrategy; } }
namespace rwlibs { namespace assembly { class AssemblyRegistry; } }

namespace Ui {
    class StrategyLibraryDialog;
}

namespace rwslibs {
//! @addtogroup rwslibs

//! @{
/**
 * @brief Dialog showing examples of the available assembly strategies.
 *
 * \image html assembly/StrategyLibraryDialog.png "The dialog with a few strategies available."
 *
 * When a workcell is set in the dialog, the configure buttons will be available for further configuration of a task in the TaskSetupDialog.
 * Notice that example animations are only shown when the strategy parameterizations have been explicitly specified in this dialog.
 */
class StrategyLibraryDialog: public QDialog {
	Q_OBJECT
public:
	/**
	 * @brief Construct new dialog.
	 * @param parent the Qt widget owning this dialog.
	 */
	StrategyLibraryDialog(QWidget* parent = 0);

	//! @brief Destructor.
	virtual ~StrategyLibraryDialog();

	/**
	 * @brief Set a workcell. This will allow configuration of a concrete assembly task for the given workcell, using the TaskSetupDialog.
	 * @param wc [in] the workcell.
	 */
	void setWorkCell(rw::common::Ptr<const rw::models::WorkCell> wc);

private slots:
	void step();
	void configure();

private:
	struct Cell;
	Cell makeCell(rwlibs::assembly::AssemblyControlStrategy* strategy) const;

	rw::sensor::FTSensor* const _dummyFTSensor;
	rw::common::Ptr<const rw::models::WorkCell> _wc;

	Ui::StrategyLibraryDialog* const _ui;
	rw::common::Ptr<rwlibs::assembly::AssemblyRegistry> _registry;
	std::vector<Cell> _cells;
	QTimer* const _timer;
};
//! @}
} /* namespace rwslibs */

#endif /* RWSLIBS_ATASK_STRATEGYLIBRARYDIALOG_HPP_ */
