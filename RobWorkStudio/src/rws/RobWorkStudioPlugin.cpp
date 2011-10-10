/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#include "RobWorkStudioPlugin.hpp"

#include "RobWorkStudio.hpp"

//#include <rw/common/Log.hpp>

using namespace rw;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::models;

using namespace rws;
//----------------------------------------------------------------------
// Virtual methods

RobWorkStudioPlugin::~RobWorkStudioPlugin()
{}

void RobWorkStudioPlugin::close()
{}

void RobWorkStudioPlugin::initialize() {
    _log = _studio->logPtr();
    Log::setLog( _log );
}

void RobWorkStudioPlugin::open(WorkCell* workcell) {

}

RobWorkStudioPlugin::RobWorkStudioPlugin(const QString& name, const QIcon& icon) :
    QDockWidget(name),
    _showAction(icon, name, this),
    _name(name),
    _log(NULL)
{
    connect(&_showAction, SIGNAL(triggered()), this, SLOT(showPlugin()));
    _log = Log::getInstance();
}


void RobWorkStudioPlugin::showPlugin()
{
    if (isVisible()) {
        setVisible(false);
    } else {
        this->show();
    }
}

void RobWorkStudioPlugin::setupMenu(QMenu* menu)
{
    menu->addAction(&_showAction);
}

void RobWorkStudioPlugin::setupToolBar(QToolBar* toolbar)
{
    toolbar->addAction(&_showAction);
}



QString RobWorkStudioPlugin::name() const
{
    return _name;
}

void RobWorkStudioPlugin::setRobWorkStudio(RobWorkStudio* studio)
{
    _studio = studio;
}

RobWorkStudio* RobWorkStudioPlugin::getRobWorkStudio() {
    return _studio;
}


void RobWorkStudioPlugin::setRobWorkInstance(RobWork::Ptr robwork) {
    RobWork::setInstance(robwork);
    _robwork = robwork;
}

RobWork::Ptr RobWorkStudioPlugin::getRobWorkInstance() {
    return _robwork;
}


rw::common::Log& RobWorkStudioPlugin::log(){
	return *_log;
}

void RobWorkStudioPlugin::setLog(rw::common::Log::Ptr log){
    _log = log;
    Log::setLog(_log);
}
