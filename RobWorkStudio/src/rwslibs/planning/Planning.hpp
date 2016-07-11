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


#ifndef PLANNING_HPP
#define PLANNING_HPP

#define QT_NO_EMIT

#include <QObject>

#include <rw/trajectory/Path.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

namespace rw { namespace models { class Device; } }
namespace rw { namespace models { class WorkCell; } }

class QComboBox;
class QCheckBox;

namespace rws {


class Planning: public RobWorkStudioPlugin {
Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
#endif
public:
    Planning();

    virtual ~Planning();
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();

private slots:
    void setStart();
    void gotoStart();
    void setGoal();
    void gotoGoal();
    void setPlanAll(int state);
    void deviceChanged(int index);
    void plan();
    void optimize();
    void savePath();
    void loadPath();
    void performStatistics();

private:
    rw::models::WorkCell* _workcell;
	rw::common::Ptr<rw::models::Device> _device;
	rw::common::Ptr<rw::models::Device> _compositeDevice;

    QComboBox* _cmbDevices;
    QCheckBox* _planAllDev;

    std::vector<rw::math::Q> _starts;
    std::vector<rw::math::Q> _goals;

    QComboBox* _cmbPlanners;
    QComboBox* _cmbCollisionDetectors;
    QComboBox* _cmbOptimization;

    rw::trajectory::QPath _path;

    rw::common::Ptr<rw::models::Device> getDevice();

    void setAsTimedStatePath();

    std::string _previousOpenSaveDirectory;
};
}
#endif //#ifndef PLANNING_HPP
