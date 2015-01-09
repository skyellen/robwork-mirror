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

const rw::kinematics::State& RobWorkStudioPlugin::getState(){
	return getRobWorkStudio()->getState();
}

void RobWorkStudioPlugin::setState(const rw::kinematics::State& s){
	return getRobWorkStudio()->setState(s);
}


RobWorkStudioPlugin::RobWorkStudioPlugin(const QString& name, const QIcon& icon) :
    QDockWidget(name),
    _showAction(icon, name, this),
    _name(name),
    _log(NULL)
{
    setObjectName(name);
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

boost::tuple<QWidget*, QAction*, int> RobWorkStudioPlugin::getAction(QWidget* widget, const std::string& actionName){
    QList<QAction*> list = widget->actions();
    for (int i = 0; i < list.size(); ++i) {
        //std::cout << list.at(i)->text().toStdString() << "==" <<  actionName << std::endl;
        if (list.at(i)->text().toStdString() == actionName){
            //std::cout << "Found File at position " << i << std::endl;
            return boost::make_tuple(widget,list.at(i), i);
        }
    }
    return boost::make_tuple(widget,(QAction*)NULL, -1);
}

boost::tuple<QWidget*, QMenu*,int> RobWorkStudioPlugin::getMenu(QWidget* widget, const std::string& menuName){
    boost::tuple<QWidget*, QAction*,int> res = getAction(widget, menuName);
    if( (res.get<1>()!=NULL) && (res.get<1>()->menu()!=NULL) ){
        return boost::make_tuple(widget, res.get<1>()->menu(), res.get<2>());
    }
    return boost::make_tuple(widget,(QMenu*)NULL, -1);
}

boost::tuple<QMenu*, QAction*,int> RobWorkStudioPlugin::getAction(QWidget* widget, const std::string& actionName, const std::string& actionName2){
    QWidget *wid; QMenu *pmenu; QAction* action; int index;
    boost::tie(wid, pmenu,index) = getMenu(widget,actionName);
    if(pmenu==NULL)
        return boost::make_tuple((QMenu*)NULL,(QAction*)NULL, -1);
    boost::tie(wid, action, index) = getAction(pmenu, actionName2);
    if(action==NULL)
        return boost::make_tuple((QMenu*)NULL, (QAction*)NULL, -1);
    return boost::make_tuple(pmenu, action, index);
}
