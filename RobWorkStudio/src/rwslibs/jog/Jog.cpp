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

#include <sstream>

#include <boost/bind.hpp>

#include <QComboBox>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QScrollArea>

#include <rws/RobWorkStudio.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/models/JointDevice.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/SphericalJoint.hpp>
#include <rw/models/UniversalJoint.hpp>
#include <rw/models/PrismaticSphericalJoint.hpp>
#include <rw/models/PrismaticUniversalJoint.hpp>
//#include <sandbox/models/BeamJoint.hpp>

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::graphics;

using namespace rws;

namespace {
    /*
     * This function takes string descriptions of the desired angular and translational units.
     * These units are then matched (case insensitive) to the known units and a correctly parsed
     * pair of unit descriptions are returned (default is <Radians, Meters>).
     */
    std::pair<std::string, std::string> formatUnitDescriptions(const std::string& angles,
                                                               const std::string& distances,
                                                               const std::map<std::string, double>& angleUnitConverters,
                                                               const std::map<std::string, double>& distanceUnitConverters) {
        // Default values if matches are not found
        std::pair<std::string, std::string> descs("Radians", "Meters");

        // Angles
        for(std::map<std::string, double>::const_iterator it = angleUnitConverters.begin(); it != angleUnitConverters.end(); ++it)
            if(QString::compare(QString(angles.c_str()), QString(it->first.c_str()), Qt::CaseInsensitive) == 0)
                descs.first = it->first;

        // Distances
        for(std::map<std::string, double>::const_iterator it = distanceUnitConverters.begin(); it != distanceUnitConverters.end(); ++it)
            if(QString::compare(QString(distances.c_str()), QString(it->first.c_str()), Qt::CaseInsensitive) == 0)
                descs.second = it->first;

        return descs;
    }

    void makeSliderUnitDataJoint(
    		const std::pair<std::string, std::string> descs,
			const std::map<std::string, double>& angleUnitConverters,
			const std::map<std::string, double>& distanceUnitConverters,
			Joint* const joint,
			std::vector<std::string>& desc,
			std::vector<double>& conv)
    {
		if(dynamic_cast<const rw::models::RevoluteJoint*>(joint)) { // Revolute joint
			// Insert angle converter
			desc.insert(desc.end(), descs.first);
			const double toUnit = angleUnitConverters.find(descs.first)->second;
			conv.insert(conv.end(), toUnit);
		} else if(dynamic_cast<const rw::models::PrismaticJoint*>(joint)) { // Prismatic joint
			// Insert distance converter
			desc.insert(desc.end(), descs.second);
			const double toUnit = distanceUnitConverters.find(descs.second)->second;
			conv.insert(conv.end(), toUnit);
		} else if(dynamic_cast<const rw::models::SphericalJoint*>(joint)) {
			// Insert distance converter
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			const double toUnit = angleUnitConverters.find(descs.first)->second;
			conv.insert(conv.end(), toUnit);
			conv.insert(conv.end(), toUnit);
			conv.insert(conv.end(), toUnit);
		} else if(dynamic_cast<const rw::models::PrismaticSphericalJoint*>(joint)) {
			// Insert distance converter
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			const double toUnitAngle = angleUnitConverters.find(descs.first)->second;
			conv.insert(conv.end(), toUnitAngle);
			conv.insert(conv.end(), toUnitAngle);
			conv.insert(conv.end(), toUnitAngle);
			const double toUnitLen = distanceUnitConverters.find(descs.second)->second;
			conv.insert(conv.end(), toUnitLen);
		} else if(dynamic_cast<const rw::models::UniversalJoint*>(joint)) {
			// Insert distance converter
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			const double toUnit = angleUnitConverters.find(descs.first)->second;
			conv.insert(conv.end(), toUnit);
			conv.insert(conv.end(), toUnit);
		} else if(dynamic_cast<const rw::models::PrismaticUniversalJoint*>(joint)) {
			// Insert distance converter
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			desc.insert(desc.end(), descs.second);
			const double toUnitAngle = angleUnitConverters.find(descs.first)->second;
			conv.insert(conv.end(), toUnitAngle);
			conv.insert(conv.end(), toUnitAngle);
			const double toUnitLen = distanceUnitConverters.find(descs.second)->second;
			conv.insert(conv.end(), toUnitLen);
		}
    }

    /*
     * Inputs are the desired angular/translational unit descriptions, a map of descriptions/converters and a device pointer.
     * First the descriptions are corrected (capital first letter) and then the unit data is created.
     * If a device is input, a vector of angular <descriptions, converters> are returned for each joint.
     * Else the first three DOF designate the translational part and the last three DOF the angular part.
     */
    std::pair<std::vector<std::string>, std::vector<double> > makeSliderUnitData(const std::string& angles,
                                                                                 const std::string& distances,
                                                                                 const std::map<std::string, double>& angleUnitConverters,
                                                                                 const std::map<std::string, double>& distanceUnitConverters,
																				 const rw::models::Device::Ptr selectedDevice = 0) {
        const std::pair<std::string, std::string> descs = formatUnitDescriptions(angles, distances, angleUnitConverters, distanceUnitConverters);
        std::vector<std::string> desc;
        std::vector<double> conv;
        if(selectedDevice) { // Device
            // TODO: this whole thing --------------------------------------------------
            // Cast to joint device to get joint info
			rw::models::JointDevice::Ptr jointDevice = selectedDevice.cast<rw::models::JointDevice>();
            if(jointDevice) {
              // Get joints
              const std::vector<rw::models::Joint*>& joints = jointDevice->getJoints();
              // Iterate through
              for(std::vector<rw::models::Joint*>::const_iterator it = joints.begin(); it != joints.end(); ++it) {
          		makeSliderUnitDataJoint(descs, angleUnitConverters, distanceUnitConverters, *it, desc, conv);
              }
            } else {
                // create unit converter that does nothing
                conv.push_back(1.0);
                desc.push_back("None");
            }
            // --------------------------------------------------
            /*
            desc.insert(desc.end(), selectedDevice->getDOF(), descs.first);
            const double toUnit = angleUnitConverters.find(descs.first)->second;
            conv.insert(conv.end(), selectedDevice->getDOF(), toUnit);
            */
        } else { // Cartesian
            desc.insert(desc.end(), 3, descs.first);
            desc.insert(desc.end(), 3, descs.second);
            const double toUnitDistance = distanceUnitConverters.find(descs.second)->second;
            conv.insert(conv.end(), 3, toUnitDistance);
            const double toUnitAngle = angleUnitConverters.find(descs.first)->second;
            conv.insert(conv.end(), 3, toUnitAngle);
        }
        // if there is no elements in these then it will fail later on
        RW_ASSERT(desc.size()>0);
        RW_ASSERT(conv.size()>0);
        return std::pair<std::vector<std::string>, std::vector<double> >(desc, conv);
    }

    std::pair<std::vector<std::string>, std::vector<double> > makeSliderUnitDataFull(const std::string& angles,
    		const std::string& distances,
			const std::map<std::string, double>& angleUnitConverters,
			const std::map<std::string, double>& distanceUnitConverters,
			const rw::models::ParallelDevice::Ptr pdevice)
	{
    	const std::pair<std::string, std::string> descs = formatUnitDescriptions(angles, distances, angleUnitConverters, distanceUnitConverters);
    	std::vector<std::string> desc;
    	std::vector<double> conv;

    	// Get joints
    	const std::vector<Joint*>& joints = pdevice->getAllJoints();
    	// Iterate through
    	for(std::vector<Joint*>::const_iterator it = joints.begin(); it != joints.end(); ++it) {
    		makeSliderUnitDataJoint(descs, angleUnitConverters, distanceUnitConverters, *it, desc, conv);
    	}

    	// if there is no elements in these then it will fail later on
    	RW_ASSERT(desc.size()>0);
    	RW_ASSERT(conv.size()>0);
    	return std::pair<std::vector<std::string>, std::vector<double> >(desc, conv);
    }

    /*
     * Update a unit combo box index by searching (case insensitive) for the current unit description set.
     */
    void updateUnitCB(QComboBox* cmbUnits, const std::string& desc) {
        int index = cmbUnits->findText(QObject::tr(desc.c_str()), static_cast<Qt::MatchFlags>(Qt::MatchFixedString));
        if(index == -1)
            index = 0;
        cmbUnits->setCurrentIndex(index);
    }
}

QIcon Jog::getIcon() {
  //  Q_INIT_RESOURCE(resources);
    return QIcon(":/jog.png");
}

Jog::Jog():
    RobWorkStudioPlugin("Jog", getIcon()),
    _workcell(0),
    _selectedDevice(0),
    _jointSliderWidget(0),
    _jointSliderWidgetFull(0),
    _selectedFrame(0),
    _cartesianTab(0),
    _cartesianDeviceTab(0),
    _updating(false),
    _cartesianBounds(Q::zero(6), Q::zero(6)),
    _rwsSettings(0)
{

    QScrollArea *widg = new QScrollArea(this);
    widg->setWidgetResizable(true);
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);

    int row = 0;

    _cmbDevices = new QComboBox();

    pLayout->addWidget(_cmbDevices, row++, 0);
    connect(_cmbDevices, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbChanged ( int )));

    _cmbAngleUnit = new QComboBox();
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(tr("Angle unit:")));
    hbox->addWidget(_cmbAngleUnit);
    pLayout->addLayout(hbox, row++, 0);

    _cmbDistanceUnit = new QComboBox();
    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(tr("Distance unit:")));
    hbox->addWidget(_cmbDistanceUnit);
    pLayout->addLayout(hbox, row++, 0);

    // TODO: Hard-coded map of known units and their converters (base: Radians/Meters).
    _angleUnitConverters.insert(std::pair<std::string, double>("Radians", 1.0));
    _angleUnitConverters.insert(std::pair<std::string, double>("Degrees", rw::math::Rad2Deg));
    _angleUnitConverters.insert(std::pair<std::string, double>("Grads", rw::math::Rad2Deg*4.0/3.6));
    _angleUnitConverters.insert(std::pair<std::string, double>("Turns", 0.5/rw::math::Pi));

    for(std::map<std::string, double>::const_iterator it = _angleUnitConverters.begin(); it != _angleUnitConverters.end(); ++it)
        _cmbAngleUnit->addItem(tr(it->first.c_str()));

    _distanceUnitConverters.insert(std::pair<std::string, double>("Meters", 1.0));
    _distanceUnitConverters.insert(std::pair<std::string, double>("Centimeters", 100.0));
    _distanceUnitConverters.insert(std::pair<std::string, double>("Millimeters", 1000.0));
    _distanceUnitConverters.insert(std::pair<std::string, double>("Inches", rw::math::Meter2Inch));

    for(std::map<std::string, double>::const_iterator it = _distanceUnitConverters.begin(); it != _distanceUnitConverters.end(); ++it)
        _cmbDistanceUnit->addItem(tr(it->first.c_str()));

    connect(_cmbAngleUnit, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbUnitChanged( int )));
    connect(_cmbDistanceUnit, SIGNAL(currentIndexChanged(int)), this, SLOT(cmbUnitChanged( int )));

    _tabWidget = new QTabWidget();
    pLayout->addWidget(_tabWidget, row++, 0); // own _tabWidget

    connect(_tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));

    pLayout->setRowStretch(row, 1);


    for (int i = 0; i<3; i++) {
        _cartesianBounds.first(i) = -5;
        _cartesianBounds.second(i) = 5;
        _cartesianBounds.first(i+3) = -2*Pi;
        _cartesianBounds.second(i+3) = 2*Pi;
    }

    widg->setWidget(base);

    //QHBoxLayout *formLayout = new QHBoxLayout;
    //formLayout->addWidget(widg);
    //this->setLayout(formLayout);
    this->setWidget(widg);

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
    		
    getRobWorkStudio()->genericEvent().add(
		boost::bind(&Jog::genericEventListener, this, _1), this);
}


void Jog::open(WorkCell* workcell)
{
	typedef std::vector<Device::Ptr>::const_iterator DevI;
    typedef std::vector<Frame*> FrameVector;

	const std::vector<Device::Ptr>& devices = workcell->getDevices();
    close();
    _workcell = workcell;
    
    _workcell->workCellChangedEvent().add(boost::bind(&Jog::workcellChangedListener, this, _1), this);
    
    _selectedFrame = 0;
    if (workcell) {
        //std::cout<<"Get State"<<std::endl;
        _state = getRobWorkStudio()->getState();
        int qs_pos = 0;
        for (DevI it = devices.begin(); it != devices.end(); ++it, ++qs_pos) {
            std::string name = (*it)->getName();
            _items.push_back(std::make_pair((*it), (MovableFrame*)NULL));
            _chosenTabs.push_back(0);
            _frameToIndex[*((*it)->getBase())] = (int)_items.size()-1;
            QVariant qvar((int)(_items.size()-1));
            _cmbDevices->addItem(name.c_str(), qvar);
        }
        //std::cout<<"Device Initialized"<<std::endl;
        FrameVector frames = Kinematics::findAllFrames(workcell->getWorldFrame(), _state);
        for (FrameVector::iterator it = frames.begin(); it != frames.end(); ++it) {
            MovableFrame* frame = dynamic_cast<MovableFrame*>(*it);
            if (frame) {
                _items.push_back(std::make_pair((Device*)NULL, frame));
                _chosenTabs.push_back(0);
                _frameToIndex[*frame] = (int)_items.size()-1;
                _cmbDevices->addItem(frame->getName().c_str(), QVariant((int)(_items.size()-1)));
            }
        }

    } else {
        close();
    }
    _rwsSettings = getRobWorkStudio()->getPropertyMap().getPtr<rw::common::PropertyMap>("RobWorkStudioSettings");
    if(_rwsSettings) {
        // Find the unit properties if there are any
        std::string unitDescAngle = _rwsSettings->get<std::string>("AngleUnit", "");
        std::string unitDescDistance = _rwsSettings->get<std::string>("DistanceUnit", "");
        // Update combo boxes
        if(unitDescAngle.size()) {
            updateUnitCB(_cmbAngleUnit, unitDescAngle);
            unitDescAngle = _cmbAngleUnit->currentText().toStdString();
        }
        if(unitDescDistance.size()) {
            updateUnitCB(_cmbDistanceUnit, unitDescDistance);
            unitDescDistance = _cmbDistanceUnit->currentText().toStdString();
        }

        updateUnit(unitDescAngle, unitDescDistance);
    }
}

void Jog::cmbChanged ( int index ) {
    if(index<0)
        return;
    std::string str = _cmbDevices->currentText().toStdString();
    int n = _cmbDevices->itemData(index).toInt();
    disconnect(_tabWidget, 0, 0, 0);
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
    _tabWidget->setCurrentIndex(_chosenTabs[index]);
    connect(_tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));
    updateUnit(_cmbAngleUnit->currentText().toStdString(), _cmbDistanceUnit->currentText().toStdString());
}

void Jog::tabChanged(int index) {
    _chosenTabs[_cmbDevices->currentIndex()] = (unsigned int)index;
}

void Jog::cmbUnitChanged( int index ) {
    updateUnit(_cmbAngleUnit->currentText().toStdString(), _cmbDistanceUnit->currentText().toStdString());
}

/*
 * Update all sliders as well as the property map. If no such property exists, "set" will create it.
 */
void Jog::updateUnit(const std::string& angles, const std::string& distances) {
    // Create the unit data: a set of <string, double> pairs containing label
    const std::pair<std::vector<std::string>, std::vector<double> > sliderUnitDataCartesian = makeSliderUnitData(angles, distances, _angleUnitConverters, _distanceUnitConverters, 0);
    if(_jointSliderWidget) {
        const std::pair<std::vector<std::string>, std::vector<double> > sliderUnitDataJoint = makeSliderUnitData(angles, distances, _angleUnitConverters, _distanceUnitConverters, _selectedDevice);
        _jointSliderWidget->setUnits(sliderUnitDataJoint.second, sliderUnitDataJoint.first);
    }
    if(_jointSliderWidgetFull) {
        const std::pair<std::vector<std::string>, std::vector<double> > sliderUnitDataJoint = makeSliderUnitDataFull(angles, distances, _angleUnitConverters, _distanceUnitConverters, _selectedDevice.cast<ParallelDevice>());
        _jointSliderWidgetFull->setUnits(sliderUnitDataJoint.second, sliderUnitDataJoint.first);
    }
    if(_cartesianTab) {
        _cartesianTab->setUnits(sliderUnitDataCartesian.second, sliderUnitDataCartesian.first);
    }
    if(_cartesianDeviceTab) {
        _cartesianDeviceTab->setUnits(sliderUnitDataCartesian.second, sliderUnitDataCartesian.first);
    }
    if(_rwsSettings) {
        _rwsSettings->set<std::string>("AngleUnit", sliderUnitDataCartesian.first.front());
        _rwsSettings->set<std::string>("DistanceUnit", sliderUnitDataCartesian.first.back());
    }
}

void Jog::removeTabs() {
    while (_tabWidget->count() > 0) {
        QWidget* widget = _tabWidget->widget(0);
        _tabWidget->removeTab(0);
        delete widget;
    }
    _jointSliderWidget = NULL;
    _jointSliderWidgetFull = NULL;
    _cartesianTab = NULL;
    _cartesianDeviceTab = NULL;
}

void Jog::constructCartTab(MovableFrame* frame) {
    // Construct Cartesian tab for frames
    _cartesianTab = new MovableFrameTab(_cartesianBounds, frame, _workcell, _state);
    _cartesianTab->updateValues(_state);
    _tabWidget->addTab(_cartesianTab, "Cartesian");
    connect(_cartesianTab,
            SIGNAL(stateChanged(const rw::kinematics::State&)),
            this,
            SLOT(stateChanged(const rw::kinematics::State&)));

}

void Jog::constructTabs(Device::Ptr device) {
    // Construct joint tab for device
    _jointSliderWidget = new JointSliderWidget();
    std::vector<std::string> titles(device->getDOF());
    for(unsigned int i = 0; i < device->getDOF(); ++i) {
      std::stringstream ss;
      ss << "q" << i;
      titles[i] = ss.str();
    }
    const bool enablers = !device.cast<ParallelDevice>().isNull();
    _jointSliderWidget->setup(titles, device->getBounds(), device->getQ(_state), enablers);

    QPushButton* btnPasteQ = new QPushButton("Paste", _jointSliderWidget);
    QPushButton* btnCopyQ = new QPushButton("Copy", _jointSliderWidget);
    QHBoxLayout* btnlayout = new QHBoxLayout();
    btnlayout->addWidget(btnCopyQ);
    btnlayout->addWidget(btnPasteQ);
    connect(btnPasteQ, SIGNAL(clicked()), _jointSliderWidget, SLOT(paste()));
    connect(btnCopyQ, SIGNAL(clicked()), _jointSliderWidget, SLOT(copy()));
    
    QVBoxLayout* tablayout = new QVBoxLayout();
    tablayout->addLayout(btnlayout);
    tablayout->addWidget(_jointSliderWidget);

    QWidget* tabWidget = new QWidget();
    tabWidget->setLayout(tablayout);

    //_tabWidget->addTab(_jointSliderWidget, "Joint");
    _tabWidget->addTab(tabWidget, "Joint");
    connect(_jointSliderWidget, SIGNAL(valueChanged(const rw::math::Q&)), this, SLOT(deviceConfigChanged(const rw::math::Q&)));

    // Construct IK tab for serial and parallel devices only
	SerialDevice::Ptr sdevice = device.cast<SerialDevice>();
	const ParallelDevice::Ptr pdevice = device.cast<ParallelDevice>();
    if (sdevice != NULL) {
        _cartesianDeviceTab = new CartesianDeviceTab(_cartesianBounds, sdevice, _workcell, _state);
        _tabWidget->addTab(_cartesianDeviceTab, "InvKin");
        connect(_cartesianDeviceTab,
                SIGNAL(stateChanged(const rw::kinematics::State&)),
                this,
                SLOT(stateChanged(const rw::kinematics::State&)));
    } else if (!pdevice.isNull()) {
    	_jointSliderWidgetFull = new JointSliderWidget();
        std::vector<std::string> titles(pdevice->getFullDOF());
        for(unsigned int i = 0; i < pdevice->getFullDOF(); ++i) {
          std::stringstream ss;
          ss << "q" << i;
          titles[i] = ss.str();
        }
        _jointSliderWidgetFull->setup(titles, pdevice->getAllBounds(), pdevice->getFullQ(_state), false);

        QPushButton* btnPasteQ = new QPushButton("Paste", _jointSliderWidgetFull);
        QPushButton* btnCopyQ = new QPushButton("Copy", _jointSliderWidgetFull);
        QHBoxLayout* btnlayout = new QHBoxLayout();
        btnlayout->addWidget(btnCopyQ);
        btnlayout->addWidget(btnPasteQ);
        connect(btnPasteQ, SIGNAL(clicked()), _jointSliderWidgetFull, SLOT(paste()));
        connect(btnCopyQ, SIGNAL(clicked()), _jointSliderWidgetFull, SLOT(copy()));

        QVBoxLayout* tablayout = new QVBoxLayout();
        tablayout->addLayout(btnlayout);
        tablayout->addWidget(_jointSliderWidgetFull);

        QWidget* tabWidget = new QWidget();
        tabWidget->setLayout(tablayout);
        _tabWidget->addTab(tabWidget, "All Joints");
        connect(_jointSliderWidgetFull, SIGNAL(valueChanged(const rw::math::Q&)), this, SLOT(deviceConfigChangedFull(const rw::math::Q&)));

        _cartesianDeviceTab = new CartesianDeviceTab(_cartesianBounds, pdevice, _workcell, _state);
        _tabWidget->addTab(_cartesianDeviceTab, "InvKin");
        connect(_cartesianDeviceTab,
        		SIGNAL(stateChanged(const rw::kinematics::State&)),
				this,
				SLOT(stateChanged(const rw::kinematics::State&)));
    } else {
        _cartesianDeviceTab = NULL;
    }

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
    	if (const ParallelDevice::Ptr pdevice = _selectedDevice.cast<ParallelDevice>()) {
    		const std::vector<bool> enabled = _jointSliderWidget->enabledState();
    		pdevice->setQ(q, enabled, _state);
    	} else {
    		_selectedDevice->setQ(q, _state);
    	}
        getRobWorkStudio()->setState(_state);
    }
}

void Jog::deviceConfigChangedFull(const Q& q) {
    if (_updating)
        return;

    if (_selectedDevice != NULL) {
    	if (const ParallelDevice::Ptr pdevice = _selectedDevice.cast<ParallelDevice>()) {
    		pdevice->setFullQ(q, _state);
    	} else {
    		_selectedDevice->setQ(q, _state);
    	}
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
    disconnect(_tabWidget, 0, 0, 0);
    _workcell = NULL;
    _cmbDevices->clear();
    _items.clear();
    _chosenTabs.clear();
    removeTabs();
    _frameToIndex.clear();
}


void Jog::updateValues() {
    _updating = true;
    if (_selectedDevice != NULL) {
    	if (_jointSliderWidget != NULL) {
    		const Q q = _selectedDevice->getQ(_state);
    		_jointSliderWidget->updateValues(q);
    		if (_cartesianDeviceTab != NULL) {
    			_cartesianDeviceTab->updateValues(_state);
    		}
    	}
    	if (_jointSliderWidgetFull != NULL) {
    		const Q q = _selectedDevice.cast<ParallelDevice>()->getFullQ(_state);
    		_jointSliderWidgetFull->updateValues(q);
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

void Jog::update() {
	typedef std::vector<Device::Ptr>::const_iterator DevI;
    typedef std::vector<Frame*> FrameVector;

	const std::vector<Device::Ptr>& devices = _workcell->getDevices();
    disconnect(_tabWidget, 0, 0, 0);
    _cmbDevices->clear();
    _items.clear();
    _chosenTabs.clear();
    removeTabs();
    _frameToIndex.clear();
    WorkCell::Ptr workcell = _workcell;
    
    _selectedFrame = 0;
    if (workcell) {
        //std::cout<<"Get State"<<std::endl;
        _state = getRobWorkStudio()->getState();
        int qs_pos = 0;
        for (DevI it = devices.begin(); it != devices.end(); ++it, ++qs_pos) {
            std::string name = (*it)->getName();
            _items.push_back(std::make_pair((*it), (MovableFrame*)NULL));
            _chosenTabs.push_back(0);
            _frameToIndex[*((*it)->getBase())] = (int)_items.size()-1;
            QVariant qvar((int)(_items.size()-1));
            _cmbDevices->addItem(name.c_str(), qvar);
        }
        //std::cout<<"Device Initialized"<<std::endl;
        FrameVector frames = Kinematics::findAllFrames(workcell->getWorldFrame(), _state);
        for (FrameVector::iterator it = frames.begin(); it != frames.end(); ++it) {
            MovableFrame* frame = dynamic_cast<MovableFrame*>(*it);
            if (frame) {
                _items.push_back(std::make_pair((Device*)NULL, frame));
                _chosenTabs.push_back(0);
                _frameToIndex[*frame] = (int)_items.size()-1;
                _cmbDevices->addItem(frame->getName().c_str(), QVariant((int)(_items.size()-1)));
            }
        }

    } else {
        disconnect(_tabWidget, 0, 0, 0);
		_cmbDevices->clear();
		_items.clear();
		_chosenTabs.clear();
		removeTabs();
		_frameToIndex.clear();
    }
    _rwsSettings = getRobWorkStudio()->getPropertyMap().getPtr<rw::common::PropertyMap>("RobWorkStudioSettings");
    if(_rwsSettings) {
        // Find the unit properties if there are any
        std::string unitDescAngle = _rwsSettings->get<std::string>("AngleUnit", "");
        std::string unitDescDistance = _rwsSettings->get<std::string>("DistanceUnit", "");
        // Update combo boxes
        if(unitDescAngle.size()) {
            updateUnitCB(_cmbAngleUnit, unitDescAngle);
            unitDescAngle = _cmbAngleUnit->currentText().toStdString();
        }
        if(unitDescDistance.size()) {
            updateUnitCB(_cmbDistanceUnit, unitDescDistance);
            unitDescDistance = _cmbDistanceUnit->currentText().toStdString();
        }

        updateUnit(unitDescAngle, unitDescDistance);
    }
}

void Jog::workcellChangedListener(int){
    // we need to call the update slot, but since this is possibly from a non qt thread, we need to separate
    // it through Qt::queue
//     std::cout << "TreeView: WORKCELL CHANGED" << std::endl;

    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);

}


void Jog::frameSelectedListener(Frame* frame) {
	if(frame==NULL)
		return;
	if(_frameToIndex.has(*frame)){
		_cmbDevices->setCurrentIndex(_frameToIndex[*frame]);
	}
}

void Jog::genericEventListener(const std::string& event)
{
	if (event == "WorkcellUpdated") {
		log().info() << "WorkcellUpdated event" << std::endl;
		open(_workcell);
	}
}

#ifndef RWS_USE_STATIC_LINK_PLUGINS
#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN2(Jog, Jog)
#endif
#endif
