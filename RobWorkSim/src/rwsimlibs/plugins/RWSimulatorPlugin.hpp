/************************************************************************
 * RobWorkStudio Version 0.1
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * This Software is developed using the Qt Open Source Edition, and is
 * therefore only available under the GNU General Public License (GPL).
 *
 * RobWorkStudio can be used, modified and redistributed freely.
 * RobWorkStudio is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWorkStudio relies on RobWork, which has a different
 * license. For more information goto your RobWork directory and read license.txt.
 ************************************************************************/

#ifndef RWSIM_RWSIMULATORPLUGIN_HPP
#define RWSIM_RWSIMULATORPLUGIN_HPP


#ifdef __WIN32
#include <windows.h>
#endif

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>

#include <rw/graphics/WorkCellScene.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

#include <rwsim/simulator/DynamicSimulator.hpp>
#include <rwsim/rwphysics/ConstantForceManipulator.hpp>

#include <rwsim/drawable/RenderContacts.hpp>
#include <rwsim/drawable/RenderGhost.hpp>

#include <rwlibs/control/JointController.hpp>

#include <rwsimlibs/gui/JointControlDialog.hpp>

#include <RobWorkStudioPlugin.hpp>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QObject>
#include <QtGui>
#include <QTimer>

class RWSimulatorPlugin: public rws::RobWorkStudioPlugin {
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "RWSimulatorPlugin.json")
#endif
public:
    RWSimulatorPlugin();

    virtual ~RWSimulatorPlugin();

    void open(rw::models::WorkCell* workcell);

    void close();

    void initialize();

    void genericEventListener(const std::string& event);

    void open(const std::string& file);
protected:
	virtual void stateChangedHandler(rws::RobWorkStudioPlugin* sender);

private slots:
    void open();
    void update();
    void startSimulation();
    void stopSimulation();
    void resetSimulation();
    void printContactGraph();
    void setSave();
    void updateCfgInfo();
    void openControlDialog();
    void openControlDialog1();
    void openControlDialog2();

private: // qt stuff
	QCheckBox *_checkBox,*_debugDrawBox,*_forceUpdateBox;
	QDoubleSpinBox *_dtBox;
	QComboBox* _engineBox;

private:
    std::string _previousOpenDirectory;
    rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dworkcell;
//    rw::kinematics::State *_state;
    rw::kinematics::State _jointState;
    rwsim::simulator::ConstantForceManipulator *_gravity;
    QTimer *_timer;

    rwsim::drawable::RenderContacts _miscForces;
    rwsim::drawable::RenderContacts _bodyForces;

    double _timeStep;
    double _nextTime;
    bool _save;
    JointControlDialog *_jointDialog,*_jointDialog1;
    rwsim::drawable::RenderGhost *_ghost;

    //std::vector<rwsim::dynamics::FixedDevice*> _fdevs;
    QLabel* _timeLabel;
    rw::trajectory::TimedStatePath _statePath;
    rwsim::simulator::DynamicSimulator::Ptr _simulator;


    //std::vector<rwlibs::control::JointController*> _controllers;
    rwlibs::opengl::Drawable *_dBtWorld;
};



#endif //#ifndef ROBOCATCHER_HPP
