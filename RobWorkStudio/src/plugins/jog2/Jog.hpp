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
#include <rw/kinematics/MovableFrame.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>
#include <rw/use_robwork_namespace.hpp>

#include <rw/kinematics/FrameMap.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include "SliderTab.hpp"

/**
 * @brief The Jog plugin enables cartesean and joint level jogging of MovableFrames
 * and Devices. Reference frame can freely be choosen and the devices are jogged using
 * general inverse kinematic solvers
 */
class Jog: public RobWorkStudioPlugin
{
    Q_OBJECT
#ifndef RW_STATIC_LINK_PLUGINS
    Q_INTERFACES(RobWorkStudioPlugin)
#endif
public:
	/**
	 * @brief Constructor
	 */
	Jog();

	/**
	 * @brief Destructor
	 */
    virtual ~Jog();

	/**
	 * @copydoc RobWorkStudioPlugin::initialize
	 */
    virtual void initialize();

	/**
	 * @copydoc RobWorkStudioPlugin::open
	 */
    virtual void open(robwork::WorkCell* workcell);

	/**
	 * @copydoc RobWorkStudioPlugin::close
	 */
    virtual void close();

	/**
	 * @copydoc RobWorkStudioPlugin::frameSelectedListener
	 */
    void frameSelectedListener(rw::kinematics::Frame* frame);

protected:
    void showEvent ( QShowEvent * event );

private slots:
    void cmbChanged ( int index );

    void stateChanged(const rw::kinematics::State& state);

    void deviceConfigChanged(const rw::math::Q& q);
    void frameConfigChanged(const rw::math::Transform3D<>& transform);

private:

    robwork::WorkCell* _workcell;
    robwork::State _state;
    rw::models::Device* _selectedDevice;
    JointSliderWidget* _jointSliderWidget;

    rw::kinematics::MovableFrame* _selectedFrame;
    MovableFrameTab* _cartesianTab;
    CartesianDeviceTab* _cartesianDeviceTab;
    bool _updating;

    QComboBox* _cmbDevices;
    QTabWidget* _tabWidget;
    std::vector<std::pair<rw::models::Device*, rw::kinematics::MovableFrame*> > _items;

    std::pair<rw::math::Q, rw::math::Q> _cartesianBounds;
    //std::vector<JointTab*> _sliders;

    /*std::vector<DeviceTab*> _deviceTabs;
    QTabWidget* _tabWidget;
*/
    void removeTabs();
    void constructTabs(rw::models::Device* device);
    void constructCartTab(rw::kinematics::MovableFrame* device);

    void stateChangedListener(const rw::kinematics::State& state);
    void keyListener(int key, Qt::KeyboardModifiers modifiers);
    //RobWorkStudio::StateChangedListener

    QIcon getIcon();
    void updateValues();

    rw::kinematics::FrameMap<int> _frameToIndex;
};

#endif //#ifndef JOGMODULE
