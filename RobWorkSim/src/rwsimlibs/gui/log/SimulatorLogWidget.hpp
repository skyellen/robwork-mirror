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

#ifndef RWSIMLIBS_GUI_SIMULATORLOGWIDGET_HPP_
#define RWSIMLIBS_GUI_SIMULATORLOGWIDGET_HPP_

/**
 * @file SimulatorLogWidget.hpp
 *
 * \copydoc rwsimlibs::gui::SimulatorLogWidget
 */

#include <rw/common/Ptr.hpp>

#include <QWidget>
#include <QItemSelection>

namespace rw { namespace graphics { class GroupNode; } }
namespace rwlibs { namespace opengl { class Drawable; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace log { class SimulatorLog; } }
namespace rwsim { namespace log { class SimulatorLogEntry; } }
namespace rwsim { namespace log { class SimulatorLogScope; } }
namespace rws { class SceneOpenGLViewer; }

class GLViewRW;
namespace Ui { class SimulatorLogWidget; }

namespace rwsimlibs {
namespace gui {

class SimulatorLogModel;

//! @addtogroup rwsimlibs_gui

//! @{
//! @brief Widget for visualization of a simulation log.
class SimulatorLogWidget: public QWidget {
    Q_OBJECT
public:
	/**
	 * @brief Construct new widget for a simulation log.
	 * @param parent [in] (optional) the parent Qt widget. Ownership is shared by the caller and the parent widget if given.
	 */
	SimulatorLogWidget(QWidget* parent = 0);

	//! @brief Destructor.
	virtual ~SimulatorLogWidget();

	/**
	 * @brief Set the dynamic workcell for visualisation.
	 * @param dwc [in] the dynamic workcell.
	 */
	void setDWC(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

	/**
	 * @brief Set the simulation log to show.
	 * @param info [in/out] the simulation log (might be updated with statistics).
	 */
	void setLog(rw::common::Ptr<rwsim::log::SimulatorLogScope> info);

	/**
	 * @brief Try to find a specific simulation time.
	 * @param time [in] the time.
	 */
	void setSelectedTime(double time);

	//! @brief Re-read the log and update.
	void updateInfo();

public slots:
	//! @brief Update the graphical view.
	void updateOpenGLView();

private slots:
	void selectionChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection);
	void currentChanged(const QModelIndex& newSelection, const QModelIndex& oldSelection);
	void collapsed(const QModelIndex& index);
	void btnPressed();
	void tabCloseRequested(int index);

private:
    Ui::SimulatorLogWidget* const _ui;
    rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> _dwc;
    rws::SceneOpenGLViewer* const _glview;
    rw::common::Ptr<rwsim::log::SimulatorLogScope> _log;
    SimulatorLogModel* _model;

    rw::common::Ptr<rw::graphics::GroupNode> _root;

    std::map<const rwsim::log::SimulatorLog*, std::list<QWidget*> > _entryToWidgets;
};
//! @}
} /* namespace gui */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_GUI_SIMULATORLOGWIDGET_HPP_ */
