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

#include <RobWorkStudioConfig.hpp>

#include <QObject>

#include <vector>

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include "SliderTab.hpp"

namespace rw { namespace kinematics { class MovableFrame; } }
namespace rw { namespace models { class Device; } }

class QTabWidget;

namespace rws {

/**
 * @brief The Jog plugin enables cartesean and joint level jogging of MovableFrames
 * and Devices. Reference frame can freely be choosen and the devices are jogged using
 * general inverse kinematic solvers
 */
class Jog: public RobWorkStudioPlugin
{
    Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
    Q_INTERFACES(rws::RobWorkStudioPlugin)
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
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
    virtual void open(rw::models::WorkCell* workcell);

	/**
	 * @copydoc RobWorkStudioPlugin::close
	 */
    virtual void close();

	/**
	 * @copydoc RobWorkStudioPlugin::frameSelectedListener
	 */
    void frameSelectedListener(rw::kinematics::Frame* frame);
    
    /**
     * @brief Listen for change in workcell.
     *
     * This method can be safely called from non-qt threads.
     *
     * @param notUsed [in] not used.
     */
    void workcellChangedListener(int notUsed);
    
    /**
     * @brief Listen for generic events:
     * - WorkcellUpdated event makes Jog plugin refresh GUI to represent new workcell configuration (i.e. new frames & devices).
     */
    void genericEventListener(const std::string& event);

protected:
    /**
     * @brief Update the widget when it is shown.
     * @param event [in] the event (not used).
     */
    void showEvent ( QShowEvent * event );

private slots:
    void cmbChanged ( int index );
    void cmbUnitChanged(int index);

    void tabChanged(int);
    
    void update();

    void stateChanged(const rw::kinematics::State& state);

    void deviceConfigChanged(const rw::math::Q& q);
    void deviceConfigChangedFull(const rw::math::Q& q);
    void frameConfigChanged(const rw::math::Transform3D<>& transform);

private:

    rw::models::WorkCell* _workcell;
    rw::kinematics::State _state;
	rw::common::Ptr<rw::models::Device> _selectedDevice;
    JointSliderWidget* _jointSliderWidget;
    JointSliderWidget* _jointSliderWidgetFull;

    rw::kinematics::MovableFrame* _selectedFrame;
    MovableFrameTab* _cartesianTab;
    CartesianDeviceTab* _cartesianDeviceTab;
    bool _updating;

    QComboBox* _cmbDevices;
    QTabWidget* _tabWidget;
	std::vector<std::pair<rw::common::Ptr<rw::models::Device>, rw::kinematics::MovableFrame*> > _items;
    std::vector<unsigned int> _chosenTabs;
    QComboBox *_cmbAngleUnit, *_cmbDistanceUnit;

    std::pair<rw::math::Q, rw::math::Q> _cartesianBounds;
    //std::vector<JointTab*> _sliders;

    /*std::vector<DeviceTab*> _deviceTabs;
    QTabWidget* _tabWidget;
*/
    void removeTabs();
	void constructTabs(rw::common::Ptr<rw::models::Device> device);
    void constructCartTab(rw::kinematics::MovableFrame* device);

    void stateChangedListener(const rw::kinematics::State& state);
    void keyListener(int key, Qt::KeyboardModifiers modifiers);
    //RobWorkStudio::StateChangedListener

    QIcon getIcon();
    void updateValues();

    void updateUnit(const std::string& angles, const std::string& distances);
    rw::common::PropertyMap* _rwsSettings;
    std::map<std::string, double> _angleUnitConverters, _distanceUnitConverters;

    rw::kinematics::FrameMap<int> _frameToIndex;
};

}

#endif //#ifndef JOGMODULE
