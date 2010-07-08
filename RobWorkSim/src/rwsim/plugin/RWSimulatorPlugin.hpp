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

#include <rwlibs/drawable/WorkCellGLDrawer.hpp>

#include <rwsim/dynamics/DynamicWorkcell.hpp>

#include <rwsim/simulator/Simulator.hpp>
#include <rwsim/simulator/rwphysics/ConstantForceManipulator.hpp>

#include <rwsim/drawable/RenderContacts.hpp>
#include <rwsim/drawable/RenderGhost.hpp>

#include <rwlibs/control/JointController.hpp>

#include "JointControlDialog.hpp"

#include <RobWorkStudioPlugin.hpp>
#include <QObject>
#include <QtGui>
#include <QTimer>

class RWSimulatorPlugin: public rws::RobWorkStudioPlugin {
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
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
    rw::common::Ptr<rwsim::dynamics::DynamicWorkcell> _dworkcell;
//    rw::kinematics::State *_state;
    rw::kinematics::State _dState,_jointState;
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
    boost::shared_ptr<rwsim::simulator::Simulator> _simulator;


    std::vector<JointController*> _controllers;
    rwlibs::drawable::Drawable *_dBtWorld;
};



#endif //#ifndef ROBOCATCHER_HPP
