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

#include "LogMessageWidget.hpp"

#include <QTextBrowser>
#include <QVBoxLayout>

#include "../../../rwsim/log/LogMessage.hpp"

using namespace rw::graphics;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

LogMessageWidget::LogMessageWidget(rw::common::Ptr<const LogMessage> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_message(entry),
	_text(new QTextBrowser())
{
	setLayout(new QVBoxLayout(this));
	layout()->addWidget(_text);
}

LogMessageWidget::~LogMessageWidget() {
}

void LogMessageWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
}

void LogMessageWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogMessage> msg = entry.cast<const LogMessage>();
	if (!(msg == NULL)) {
		_message = msg;
	} else {
		RW_THROW("LogMessageWidget (setEntry): invalid entry!");
	}
}

rw::common::Ptr<const SimulatorLog> LogMessageWidget::getEntry() const {
	if (!(_message == NULL))
		return _message;
	RW_THROW("LogMessageWidget (getEntry): both positions and velocities is zero!");
	return NULL;
}

void LogMessageWidget::updateEntryWidget() {
	std::stringstream str;
	str << _message->getDescription() << std::endl;
	str << std::endl;
	str << _message->getMessage();
	_text->setText(QString::fromStdString(str.str()));
}

void LogMessageWidget::showGraphics(rw::common::Ptr<GroupNode> root, rw::common::Ptr<SceneGraph> graph) {
}

std::string LogMessageWidget::getName() const {
	return "Message";
}

LogMessageWidget::Dispatcher::Dispatcher() {
}

LogMessageWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* LogMessageWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	rw::common::Ptr<const LogMessage> const msg = entry.cast<const LogMessage>();
	if (!(msg == NULL))
		return new LogMessageWidget(msg, parent);
	RW_THROW("LogMessageWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool LogMessageWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogMessage>() == NULL))
		return true;
	return false;
}
