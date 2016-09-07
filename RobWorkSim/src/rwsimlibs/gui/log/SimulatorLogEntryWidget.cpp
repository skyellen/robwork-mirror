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

#include "SimulatorLogEntryWidget.hpp"
#include "BodyMotionWidget.hpp"
#include "CollisionResultWidget.hpp"
#include "ConstraintWidget.hpp"
#include "ContactSetWidget.hpp"
#include "ContactVelocitiesWidget.hpp"
#include "EquationSystemWidget.hpp"
#include "ForceTorqueWidget.hpp"
#include "LogMessageWidget.hpp"
#include "LogValuesWidget.hpp"

using namespace rw::common;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

SimulatorLogEntryWidget::SimulatorLogEntryWidget(QWidget* parent):
	QWidget(parent)
{
}

SimulatorLogEntryWidget::~SimulatorLogEntryWidget() {
}

SimulatorLogEntryWidget::Factory::Factory():
	ExtensionPoint<SimulatorLogEntryWidget::Dispatcher>("rwsimlibs.gui.InternalInfoEntryWidget", "InternalInfoEntryWidget extension point.")
{
}

std::list<SimulatorLogEntryWidget::Dispatcher::Ptr> SimulatorLogEntryWidget::Factory::getWidgetDispatchers(rw::common::Ptr<const SimulatorLog> entry) {
	std::list<SimulatorLogEntryWidget::Dispatcher::Ptr> res;

	SimulatorLogEntryWidget::Dispatcher::Ptr dispatcher;
	dispatcher = ownedPtr(new BodyMotionWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new CollisionResultWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new ConstraintWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new ForceTorqueWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new ContactSetWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new ContactVelocitiesWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new EquationSystemWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new LogValuesWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);
	dispatcher = ownedPtr(new LogMessageWidget::Dispatcher());
	if (dispatcher->accepts(entry))
		res.push_back(dispatcher);

	const SimulatorLogEntryWidget::Factory factory;
	const std::vector<Extension::Ptr> exts = factory.getExtensions();
	BOOST_FOREACH(const Extension::Ptr ext, exts){
		const SimulatorLogEntryWidget::Dispatcher::Ptr dispatcher = ext->getObject().cast<const SimulatorLogEntryWidget::Dispatcher>();
		if (!(dispatcher == NULL)) {
			if (dispatcher->accepts(entry))
				res.push_back(dispatcher);
		}
	}
	return res;
}

std::list<SimulatorLogEntryWidget*> SimulatorLogEntryWidget::Factory::makeWidgets(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) {
	std::list<SimulatorLogEntryWidget*> res;
	const std::list<SimulatorLogEntryWidget::Dispatcher::Ptr> dispatchers = getWidgetDispatchers(entry);
	BOOST_FOREACH(const SimulatorLogEntryWidget::Dispatcher::Ptr dispatcher, dispatchers) {
		res.push_back(dispatcher->makeWidget(entry,parent));
	}
	return res;
}
