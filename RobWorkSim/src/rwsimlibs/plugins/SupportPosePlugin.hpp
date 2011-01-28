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

#ifndef SUPPORTPOSEPLUGIN_HPP
#define SUPPORTPOSEPLUGIN_HPP

#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/TrajectoryFactory.hpp>

#include <rwlibs/drawable/WorkCellGLDrawer.hpp>

#include <dynamics/DynamicWorkcell.hpp>

#include <dynamics/FixedDevice.hpp>
#include <simulator/Simulator.hpp>
#include <simulator/rwphysics/ConstantForceManipulator.hpp>

#include <drawable/RenderForce.hpp>
#include <drawable/RenderGhost.hpp>

#include <sandbox/control/JointController.hpp>

#include <simulator/ThreadSimulator.hpp>

#include <RobWorkStudioPlugin.hpp>
#include <QObject>
#include <QtGui>
#include <QTimer>

#include "RestingPoseDialog.hpp"


class SupportPosePlugin: public RobWorkStudioPlugin {
Q_OBJECT
Q_INTERFACES( RobWorkStudioPlugin )
public:
    SupportPosePlugin();

    virtual ~SupportPosePlugin();

    void open(rw::models::WorkCell* workcell);

    void close();

    void initialize();

    void genericEventListener(const std::string& event);

protected:
	virtual void stateChangedHandler(RobWorkStudioPlugin* sender);

private slots:
    void update();
    void startSimulation();
    void stopSimulation();
    void resetSimulation();
    void printContactGraph();
    void setSave();
    void updateCfgInfo();
    void saveState();
    void createSimulator();
    void destroySimulator();
    void startRestingPoseSim();
    void updateViewEvent();

private: // qt stuff
	QCheckBox *_checkBox,*_debugDrawBox,*_forceUpdateBox;
	QDoubleSpinBox *_dtBox;
	QComboBox* _engineBox;

private:
    std::string _previousOpenDirectory;
    boost::shared_ptr<dynamics::DynamicWorkCell> _dwc;
//    rw::kinematics::State *_state;
    rw::kinematics::State _dState,_jointState,_defState;
    ConstantForceManipulator *_gravity;
    QTimer *_timer;

    std::vector<RenderForce*> _miscForces;
    std::vector<RenderForce*> _bodyForces;

    double _timeStep;
    double _nextTime;
    bool _save;
    RenderGhost *_ghost;

    std::vector<dynamics::FixedDevice*> _fdevs;
    QLabel* _timeLabel, *_nrOfSimsLabel;
    rw::trajectory::TimedStatePath _statePath;

    ThreadSimulator *_simulator;

    std::vector<JointController*> _controllers;
    rwlibs::drawable::Drawable *_dBtWorld;

    std::vector<dynamics::RigidBody*> _bodies;

    std::vector<rw::kinematics::State> _supportPoses;

    double _simStartTime;
    int _nrOfSims;
    int _maxNrOfSims;

    RestingPoseDialog *_restPoseDialog;
};



#endif //#ifndef ROBOCATCHER_HPP
