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

#ifndef RWSIMLIBS_GUI_CONTACTVELOCITIESWIDGET_HPP_
#define RWSIMLIBS_GUI_CONTACTVELOCITIESWIDGET_HPP_

/**
 * @file ContactVelocitiesWidget.hpp
 *
 * \copydoc rwsimlibs::gui::ContactVelocitiesWidget
 */

#include "SimulatorLogEntryWidget.hpp"

#include <rw/math/Vector3D.hpp>

namespace rwsim { namespace contacts { class Contact; } }
namespace rwsim { namespace log { class LogContactSet; } }
namespace rwsim { namespace log { class LogContactVelocities; } }

namespace Ui { class ContactVelocitiesWidget; }

class QItemSelection;

namespace rwsimlibs {
namespace gui {
//! @addtogroup rwsimlibs_gui

//! @{
//! @brief Graphical representation of the log entry rwsim::log::LogContactVelocities.
class ContactVelocitiesWidget: public SimulatorLogEntryWidget {
    Q_OBJECT
public:
	/**
	 * @brief Construct new widget for a log entry.
	 * @param entry [in] a contact velocities entry.
	 * @param parent [in] (optional) the parent Qt widget. Ownership is shared by the caller and the parent widget if given.
	 */
	ContactVelocitiesWidget(rw::common::Ptr<const rwsim::log::LogContactVelocities> entry, QWidget* parent = 0);

	//! @brief Destructor.
	virtual ~ContactVelocitiesWidget();

	//! @copydoc SimulatorLogEntryWidget::setDWC
	virtual void setDWC(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

	//! @copydoc SimulatorLogEntryWidget::setEntry
	virtual void setEntry(rw::common::Ptr<const rwsim::log::SimulatorLog> entry);

	//! @copydoc SimulatorLogEntryWidget::getEntry
	virtual rw::common::Ptr<const rwsim::log::SimulatorLog> getEntry() const;

	//! @copydoc SimulatorLogEntryWidget::updateEntryWidget
	virtual void updateEntryWidget();

	//! @copydoc SimulatorLogEntryWidget::showGraphics
	virtual void showGraphics(rw::common::Ptr<rw::graphics::GroupNode> root, rw::common::Ptr<rw::graphics::SceneGraph> graph);

	//! @copydoc SimulatorLogEntryWidget::getName
	virtual std::string getName() const;

	//! @copydoc SimulatorLogEntryWidget::Dispatcher
	class Dispatcher: public SimulatorLogEntryWidget::Dispatcher {
	public:
		//! @brief Constructor.
		Dispatcher();

		//! @brief Destructor.
		virtual ~Dispatcher();

		//! @copydoc SimulatorLogEntryWidget::Dispatcher::makeWidget
		SimulatorLogEntryWidget* makeWidget(rw::common::Ptr<const rwsim::log::SimulatorLog> entry, QWidget* parent = 0) const;

		//! @copydoc SimulatorLogEntryWidget::Dispatcher::accepts
		bool accepts(rw::common::Ptr<const rwsim::log::SimulatorLog> entry) const;
	};

private slots:
	void contactSetPairsChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection);
	void contactSetChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection);

private:
	QString toQString(const rw::math::Vector3D<>& vec);
	QString toQString(const rwsim::contacts::Contact& contact, const rw::math::Vector3D<>& velA, const rw::math::Vector3D<>& velB);

private:
    Ui::ContactVelocitiesWidget* const _ui;
    rw::common::Ptr<const rwsim::log::LogContactVelocities> _velocities;
    rw::common::Ptr<const rwsim::log::LogContactSet> _contactSet;
    rw::common::Ptr<rw::graphics::GroupNode> _root;
    rw::common::Ptr<rw::graphics::SceneGraph> _graph;
};
//! @}
} /* namespace gui */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_GUI_CONTACTVELOCITIESWIDGET_HPP_ */
