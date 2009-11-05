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
#include <rw/math/Constants.hpp>
#include <rw/kinematics/Kinematics.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rwlibs::drawable;




QIcon Jog::getIcon() {
  //  Q_INIT_RESOURCE(resources);
    return QIcon(":/jog.png");
}

Jog::Jog():
    RobWorkStudioPlugin("Jog", getIcon()),
    _workcell(0),
    _selectedFrame(0),
    _updating(false),
    _cartesianBounds(Q::zero(6), Q::zero(6))
{

    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);
    int row = 0;

    _cmbDevices = new QComboBox();

    pLayout->addWidget(_cmbDevices, row++, 0);
    connect(_cmbDevices, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbChanged ( int )));


    _tabWidget = new QTabWidget();
    pLayout->addWidget(_tabWidget, row++, 0); // own _tabWidget


    pLayout->setRowStretch(row, 1);


    for (int i = 0; i<3; i++) {
        _cartesianBounds.first(i) = -10;
        _cartesianBounds.second(i) = 10;
        _cartesianBounds.first(i+3) = -2*Pi;
        _cartesianBounds.second(i+3) = 2*Pi;
    }

    // Construct layout and widget
    /*QWidget *widg = new QWidget(this);
    QVBoxLayout *lay = new QVBoxLayout(widg);
    widg->setLayout(lay);
    this->setWidget(widg);
*/

}

Jog::~Jog()
{
 //   Q_CLEANUP_RESOURCE(resources);
}

void Jog::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(
    		boost::bind(&Jog::stateChangedListener, this, _1), this);

    getRobWorkStudio()->frameSelectedEvent().add(
    		boost::bind(&Jog::frameSelectedListener, this, _1), this);

}

void Jog::open(WorkCell* workcell)
{
    typedef std::vector<Device*>::const_iterator DevI;
    typedef std::vector<Frame*> FrameVector;

    const std::vector<Device*>& devices = workcell->getDevices();

    close();
    _workcell = workcell;
    _selectedFrame = 0;

    if (workcell != NULL) {
        //std::cout<<"Get State"<<std::endl;
        _state = getRobWorkStudio()->getState();
        int qs_pos = 0;
        for (DevI it = devices.begin(); it != devices.end(); ++it, ++qs_pos) {
            _items.push_back(std::make_pair((*it), (MovableFrame*)NULL));
            _frameToIndex[*((*it)->getBase())] = _items.size()-1;
            _cmbDevices->addItem((*it)->getName().c_str(), QVariant((int)(_items.size()-1)));
        }
        //std::cout<<"Device Initialized"<<std::endl;
        FrameVector frames = Kinematics::findAllFrames(workcell->getWorldFrame(), _state);
        for (FrameVector::iterator it = frames.begin(); it != frames.end(); ++it) {
            MovableFrame* frame = dynamic_cast<MovableFrame*>(*it);
            if (frame) {
                _items.push_back(std::make_pair((Device*)NULL, frame));
                _frameToIndex[*frame] = _items.size()-1;
                _cmbDevices->addItem(frame->getName().c_str(), QVariant((int)(_items.size()-1)));
            }
        }
    } else {
        close();
    }
}

void Jog::cmbChanged ( int index ) {
    if(index<0)
        return;

    std::string str = _cmbDevices->currentText().toStdString();
    int n = _cmbDevices->itemData(index).toInt();
    if (_items[n].first != NULL) {
        removeTabs();
        _selectedDevice = _items[n].first;
        _selectedFrame = NULL;
        constructTabs(_selectedDevice);
    } else if (_items[n].second != NULL) {
        removeTabs();
        _selectedDevice = NULL;
        _selectedFrame = _items[n].second;
        constructCartTab(_selectedFrame);
    }
}

void Jog::removeTabs() {
    while (_tabWidget->count() > 0) {
        QWidget* widget = _tabWidget->widget(0);
        _tabWidget->removeTab(0);
        delete widget;
    }
    _jointSliderWidget = NULL;
    _cartesianTab = NULL;

}

void Jog::constructCartTab(MovableFrame* frame) {
    _cartesianTab = new MovableFrameTab(_cartesianBounds, frame, _workcell, _state);
    //_cartesianTab->setup(_cartesianBounds, frame);
    _cartesianTab->updateValues(_state);
    _tabWidget->addTab(_cartesianTab, "Cartesian");
    connect(_cartesianTab,
            SIGNAL(stateChanged(const rw::kinematics::State&)),
            this,
            SLOT(stateChanged(const rw::kinematics::State&)));

}

void Jog::constructTabs(Device* device) {
    _jointSliderWidget = new JointSliderWidget();
    _jointSliderWidget->setup(device->getBounds(), device->getQ(_state));
    _tabWidget->addTab(_jointSliderWidget, "Joint");
    connect(_jointSliderWidget, SIGNAL(valueChanged(const rw::math::Q&)), this, SLOT(deviceConfigChanged(const rw::math::Q&)));

    SerialDevice* sdevice = dynamic_cast<SerialDevice*>(device);

    if (sdevice != NULL) {
        _cartesianDeviceTab = new CartesianDeviceTab(_cartesianBounds, sdevice, _workcell, _state);
        _tabWidget->addTab(_cartesianDeviceTab, "InvKin");
        connect(_cartesianDeviceTab,
                SIGNAL(stateChanged(const rw::kinematics::State&)),
                this,
                SLOT(stateChanged(const rw::kinematics::State&)));
    } else {
        _cartesianDeviceTab = NULL;
    }
    //SliderTab* cartTab = new SliderTab(device->getBounds(), device->getQ(_state));
    //_tabWidget->addTab(cartTab, "Cart");

}

void Jog::stateChanged(const rw::kinematics::State& state) {
    if (_updating)
        return;

    getRobWorkStudio()->setState(state);
}

void Jog::deviceConfigChanged(const Q& q) {
    if (_updating)
        return;

    if (_selectedDevice != NULL) {
        _selectedDevice->setQ(q, _state);
        getRobWorkStudio()->setState(_state);
    }
}

void Jog::frameConfigChanged(const rw::math::Transform3D<>& transform) {
    if (_updating)
        return;

    if (_selectedFrame != NULL) {
        _selectedFrame->setTransform(transform, _state);
        getRobWorkStudio()->setState(_state);
    }
}

void Jog::close()
{
    _workcell = NULL;
    _cmbDevices->clear();
    _items.clear();
    removeTabs();
    _frameToIndex.clear();
}


void Jog::updateValues() {
    _updating = true;
    if (_selectedDevice != NULL && _jointSliderWidget != NULL) {
        Q q = _selectedDevice->getQ(_state);
        _jointSliderWidget->updateValues(q);
        if (_cartesianDeviceTab != NULL) {
            _cartesianDeviceTab->updateValues(_state);
        }
    }
    if (_selectedFrame != NULL && _cartesianTab != NULL) {
        _cartesianTab->updateValues(_state);
    }
    _updating = false;
}


void Jog::showEvent ( QShowEvent * event )
{
    _state = getRobWorkStudio()->getState();
    updateValues();
}

void Jog::stateChangedListener(const State& state)
{
    _state = state;
    if (isVisible() && !_updating) {
        updateValues();
    }
}

void Jog::frameSelectedListener(Frame* frame) {
	if(frame==NULL)
		return;
	if(_frameToIndex.has(*frame)){
		_cmbDevices->setCurrentIndex(_frameToIndex[*frame]);
	}
}

#ifndef RW_STATIC_LINK_PLUGINS
Q_EXPORT_PLUGIN2(Jog, Jog)
#endif
