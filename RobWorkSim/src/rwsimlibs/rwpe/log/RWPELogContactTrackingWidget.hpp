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

#ifndef RWSIMLIBS_RWPE_LOG_RWPELOGCONTACTTRACKINGWIDGET_HPP_
#define RWSIMLIBS_RWPE_LOG_RWPELOGCONTACTTRACKINGWIDGET_HPP_

/**
 * @file RWPELogContactTrackingWidget.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPELogContactTrackingWidget
 */

#include <rwsimlibs/gui/log/SimulatorLogEntryWidget.hpp>

#include <QItemSelection>

namespace rwsim { namespace contacts { class Contact; } }

namespace Ui { class RWPELogContactTrackingWidget; }

namespace rwsimlibs {
namespace rwpe {

class RWPELogContactTracking;

//! @addtogroup INSERT_DOC_GROUP

//! @{
/**
 * @brief INSERT_SHORT_DESCRIPTION
 */
class RWPELogContactTrackingWidget: public gui::SimulatorLogEntryWidget {
    Q_OBJECT
public:
	RWPELogContactTrackingWidget(rw::common::Ptr<const RWPELogContactTracking> entry, QWidget* parent = 0);

	//! @brief Destructor.
	virtual ~RWPELogContactTrackingWidget();

	//! @copydoc InternalInfoEntryWidget::setDWC
	virtual void setDWC(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc);

	//! @copydoc InternalInfoEntryWidget::setEntry
	virtual void setEntry(rw::common::Ptr<const rwsim::log::SimulatorLog> entry);

	//! @copydoc InternalInfoEntryWidget::getEntry
	virtual rw::common::Ptr<const rwsim::log::SimulatorLog> getEntry() const;

	//! @copydoc InternalInfoEntryWidget::updateEntryWidget
	virtual void updateEntryWidget();

	//! @copydoc InternalInfoEntryWidget::showGraphics
	virtual void showGraphics(rw::common::Ptr<rw::graphics::GroupNode> root, rw::common::Ptr<rw::graphics::SceneGraph> graph);

	//! @copydoc InternalInfoEntryWidget::getName
	virtual std::string getName() const;

	//! @copydoc InternalInfoEntryWidget::Dispatcher
	class Dispatcher: public SimulatorLogEntryWidget::Dispatcher {
	public:
		//! @brief Constructor.
		Dispatcher();

		//! @brief Destructor.
		virtual ~Dispatcher();

		//! @copydoc InternalInfoEntryWidget::Dispatcher::makeWidget
		SimulatorLogEntryWidget* makeWidget(rw::common::Ptr<const rwsim::log::SimulatorLog> entry, QWidget* parent = 0) const;

		//! @copydoc InternalInfoEntryWidget::Dispatcher::accepts
		bool accepts(rw::common::Ptr<const rwsim::log::SimulatorLog> entry) const;
	};

private slots:
	void contactSetPairsChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection);
	void contactSetChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection);
	void checkboxChanged(int state);

private:
	QString toQString(const rw::math::Vector3D<>& vec);
	QString toQString(const rwsim::contacts::Contact& contact);

private:
    Ui::RWPELogContactTrackingWidget* const _ui;
    rw::common::Ptr<const RWPELogContactTracking> _tracking;
    rw::common::Ptr<rw::graphics::GroupNode> _root;
    rw::common::Ptr<rw::graphics::SceneGraph> _graph;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_LOG_RWPELOGCONTACTTRACKINGWIDGET_HPP_ */
