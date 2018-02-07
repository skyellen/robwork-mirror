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

#ifndef RWSLIBS_ATASK_TASKSETUPDIALOG_HPP_
#define RWSLIBS_ATASK_TASKSETUPDIALOG_HPP_

/**
 * @file TaskSetupDialog.hpp
 *
 * \copydoc rwslibs::TaskSetupDialog
 */

#include <QDialog>

#include <rw/common/Ptr.hpp>
#include <rw/math/Transform3D.hpp>

namespace rw { namespace common { class PropertyMap; } }
namespace rw { namespace models { class WorkCell; } }
namespace rwlibs { namespace assembly { class AssemblyControlStrategy; } }
namespace rws { class SceneOpenGLViewer; }

namespace Ui {
    class TaskSetupDialog;
}

class HelpAssistant;

namespace rwslibs {
//! @addtogroup rwslibs

//! @{
/**
 * @brief Dialog for configuration of an assembly task.
 *
 * \image html assembly/TaskSetupDialog.png "The dialog for setting up an assembly task. In this case for the rwlibs::assembly::SpiralStrategy ."
 *
 * A workcell with at least two objects is needed to set up an assembly task.
 * The approach and target views show the transforms between the objects at the beginning and end of the strategy.
 * The target transform is set under the task definition section, while the approach is depdendent on the strategy parameters, which are specific to the chosen strategy.
 * The insertion view shows an animation of the objects being assembled as specified by the strategy.
 * Notice that this is a kinematic simulation only. This means that the objects can penetrate, no dynamics is calculated, and the force/torque sensors will
 * appear as measuring zero force at all times.
 *
 * It is possible to load and save the final definition of the task. This can then later work as the input to a dynamic simulation of the assembly procedure.
 */
class TaskSetupDialog: public QDialog {
	Q_OBJECT
public:
	/**
	 * @brief Construct new dialog.
	 * @param parent the Qt owner of the dialog.
	 * @param wc [in] the workcell to set up assembly task for.
	 * @param strategy the assembly strategy to use.
	 */
	TaskSetupDialog(QWidget* parent, rw::common::Ptr<const rw::models::WorkCell> wc, rw::common::Ptr<rwlibs::assembly::AssemblyControlStrategy> strategy);

	//! @brief Destructor.
	virtual ~TaskSetupDialog();

private slots:
	void setFemaleObject(const QString &text);
	void setMaleObject(const QString &text);
	void rpyChanged(double value);
	void dialChanged(int value);
	void updateViews();
	void updatePositions();
	void stepInsertion();
	void loadTask();
	void saveTask();
	void showHelpStrategy();
	void showHelpParameterization();

private:
	void updateView(rws::SceneOpenGLViewer* widget);
	void updatePosition(rws::SceneOpenGLViewer* widget, const rw::math::Transform3D<>& fTm);

	struct InsertionSimulation;

	Ui::TaskSetupDialog* const _ui;
	const rw::common::Ptr<const rw::models::WorkCell> _wc;
	rw::common::Ptr<rwlibs::assembly::AssemblyControlStrategy> _strategy;
	QTimer* const _timer;
	rw::common::Ptr<rw::common::PropertyMap> _propertyMap;
	std::string _lastDir;
	InsertionSimulation* const _simulation;
	HelpAssistant* const _help;

};
//! @}
} /* namespace rwslibs */

#endif /* RWSLIBS_ATASK_TASKSETUPDIALOG_HPP_ */
