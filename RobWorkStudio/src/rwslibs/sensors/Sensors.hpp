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

#ifndef RW_STUDIO_SENSORS_MODULE_H
#define RW_STUDIO_SENSORS_MODULE_H

#include "SensorView.hpp"

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/Message.hpp>
#include <rw/trajectory/Path.hpp>

#include <rw/sensor/Scanner25D.hpp>
#include <rw/sensor/Scanner2D.hpp>
#include <rw/sensor/Scanner1D.hpp>
#include <rw/sensor/Camera.hpp>

#include <rwlibs/simulation/SimulatedSensor.hpp>

#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include "ui_SensorsPlugin.h"

namespace rws {

class Sensors : public RobWorkStudioPlugin, private Ui::SensorsPlugin
{
    Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
    Q_INTERFACES(rws::RobWorkStudioPlugin)
#endif

public:
    Sensors();

    virtual ~Sensors();

    void initialize();

    void open(rw::models::WorkCell* workcell);

    void close();

    //void setupToolBar(QToolBar* toolbar);
public slots:
    void updateSim();

private slots:
    void on_btnDisplay_clicked(bool checked);
    void on_spnUpdateTime_valueChanged(int value);

    void viewClosed(SensorView* view);
private:
    // This listens for changes to the state of RobWorkStudio.
    void stateChangedListener(const rw::kinematics::State& state);

private:
    QTimer *_timer;

    rw::kinematics::State _state;
    rw::models::WorkCell* _workcell;


    struct SensorSet {
    public:
        SensorSet(rwlibs::simulation::SimulatedSensor::Ptr sensor, SensorView::Ptr view):
            sensor(sensor), view(view)
        {}

        rwlibs::simulation::SimulatedSensor::Ptr sensor;
        SensorView::Ptr view;
    };

    std::vector<SensorSet> _sensors;

    //std::vector<SensorView*> _sensorViews;

    // the simulated sensors
    //std::vector<rwlibs::simulation::SimulatedSensor*> _simSensors;

    // sensors that we support in a kinematic simulation
    //rw::sensor::CameraPtr _camera;
    //rw::sensor::Scanner25D *_scanner25d;
    //rw::sensor::Scanner2D *_scanner2d;
    //rw::sensor::Scanner1D *_scanner1d;

    // need tactile switch sensor
    //rwlibs::simulation::FrameGrabberPtr _framegrabber;
    //rwlibs::simulation::FrameGrabber25DPtr _framegrabber25d;

    //std::vector<rwlibs::drawable::RenderPtr> _renders;

    // and somewhere to display our sensor results
    //rwlibs::drawable::RenderImagePtr _imgRender;
    //rwlibs::drawable::RenderScanPtr _scanRender;
    //rwlibs::drawable::RenderLinesPtr _cameraViewRender, _scanViewRender;

};

}

#endif
