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

#include "Jog.hpp"

#include <iostream>
#include <QMessageBox>
#include <boost/bind.hpp>

#include <rws/RobWorkStudio.hpp>
#include <rws/Event.hpp>

#include <rw/math/RPY.hpp>

using namespace robwork;
using namespace rwlibs::drawable;
using namespace rws;

QIcon Jog::getIcon() {
    Q_INIT_RESOURCE(resources);
    return QIcon(":/jog.png");
}

Jog::Jog():
    RobWorkStudioPlugin("Jog", getIcon()),
    _workcell(0),
    _selectedFrame(0),
    _keyStepSize(0.1)
{

    // Construct layout and widget
    QWidget *widg = new QWidget(this);
    QVBoxLayout *lay = new QVBoxLayout(widg);
    widg->setLayout(lay);
    this->setWidget(widg);


    _tabWidget = new QTabWidget();
    lay->addWidget(_tabWidget); // own _tabWidget
}

Jog::~Jog()
{
    Q_CLEANUP_RESOURCE(resources);
}

void Jog::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(
        boost::bind(&Jog::stateChangedListener, this, _1), this);
    getRobWorkStudio()->keyEvent().add(
        boost::bind(&Jog::keyListener, this, _1, _2), this);
    getRobWorkStudio()->frameSelectedEvent().add(
        boost::bind(&Jog::frameSelectedListener, this, _1), this);
}

void Jog::open(WorkCell* workcell)
{
    typedef std::vector<Device*>::const_iterator MI;
    const std::vector<Device*>& devices = workcell->getDevices();

    close();
    _workcell = workcell;
    _selectedFrame = 0;
    if (workcell != NULL) {
        _state = getRobWorkStudio()->getState();
        int qs_pos = 0;
        for (MI it = devices.begin(); it != devices.end(); ++it, ++qs_pos) {
            DeviceTab* tab = new DeviceTab(*it, &_state);
            _tabWidget->addTab(tab, (**it).getName().c_str()); // own tab

            connect(tab, SIGNAL(updateSignal()), this, SLOT(stateChangedSlot()));

            _deviceTabs.push_back(tab);
        }
    } else {
        close();
    }
}

void Jog::close()
{
    _workcell = NULL;
    _deviceTabs.clear();
    while (_tabWidget->count() > 0) {
        QWidget* widget = _tabWidget->widget(0);
        _tabWidget->removeTab(0);
        delete widget;
    }
}


void Jog::updateValues() {
    typedef std::vector<DeviceTab*>::iterator I;
    for (I p = _deviceTabs.begin(); p != _deviceTabs.end(); ++p) {
        (**p).updateDisplayValues();
    }
}


void Jog::showEvent ( QShowEvent * event )
{
    _state = getRobWorkStudio()->getState();
    updateValues();
}

void Jog::stateChangedListener(const State& state)
{
    _state = state;
    if (isVisible()) {
        updateValues();
    }
}

void Jog::keyListener(int key, Qt::KeyboardModifiers modifiers)
{
    if (_selectedFrame == NULL) return;

    MovableFrame* movableFrame = dynamic_cast<MovableFrame*>(_selectedFrame);
    if (movableFrame != NULL) {
        Transform3D<> transform = movableFrame->getTransform(_state);
        Vector3D<> input;
        if (modifiers & Qt::ControlModifier) {
            switch (key) {
            case Qt::Key_7: input(0) = 1; break;
            case Qt::Key_9: input(0) = -1; break;
            case Qt::Key_4: input(1) = 1; break;
            case Qt::Key_6: input(1) = -1; break;
            case Qt::Key_1: input(2) = 1; break;
            case Qt::Key_3: input(2) = -1; break;
            case Qt::Key_Plus: _keyStepSize /= 0.8; break;
            case Qt::Key_Minus: _keyStepSize *= 0.8; break;
            }
        }
        input *= _keyStepSize;

        Transform3D<> tnew;
        if (modifiers & Qt::AltModifier) {
            tnew.R() = RPY<>(input(0), input(1), input(2)).toRotation3D();
        } else {
            tnew.P() = input;
        }
        movableFrame->setTransform(tnew*transform, _state);
        getRobWorkStudio()->setState(_state);
    }
}

void Jog::frameSelectedListener(rw::kinematics::Frame* frame) {
    _selectedFrame = frame;
}

void Jog::stateChangedSlot()
{
    getRobWorkStudio()->setState(_state);
}

#ifndef RW_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Jog, Jog)
#endif
