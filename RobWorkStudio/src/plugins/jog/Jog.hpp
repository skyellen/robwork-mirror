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

#ifndef JOGMODULE_H
#define JOGMODULE_H

#include <QTabWidget>
#include <QTextEdit>
#include <QSlider>
#include <QtGui>

#include <list>
#include <vector>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/use_robwork_namespace.hpp>

#include <rws/RobWorkStudioPlugin.hpp>
#include "DeviceTab.hpp"

class Jog: public RobWorkStudioPlugin
{
    Q_OBJECT
    Q_INTERFACES(RobWorkStudioPlugin)

public:
    Jog();

    virtual ~Jog();

    virtual void initialize();

    virtual void open(robwork::WorkCell* workcell);

    virtual void close();

protected:
//	virtual void stateChangedHandler(RobWorkStudioPlugin* sender);

    void showEvent ( QShowEvent * event );

private slots:
    void stateChangedSlot();



private:
    robwork::WorkCell* _workcell;
    robwork::State _state;
    rw::kinematics::Frame* _selectedFrame;

    std::vector<DeviceTab*> _deviceTabs;
    QTabWidget* _tabWidget;

    void stateChangedListener(const rw::kinematics::State& state);
    void keyListener(int key, Qt::KeyboardModifiers modifiers);
    double _keyStepSize;
    void frameSelectedListener(rw::kinematics::Frame* frame);
    //RobWorkStudio::StateChangedListener

    QIcon getIcon();
    void updateValues();
};

#endif //#ifndef JOGMODULE
