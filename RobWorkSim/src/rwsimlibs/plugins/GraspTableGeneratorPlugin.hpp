/*
 * GraspTableGeneratorPlugin.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef GraspTableGeneratorPlugin_HPP_
#define GraspTableGeneratorPlugin_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include "ui_GraspTableGeneratorPlugin.h"

#include <rw/kinematics/State.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsim/control/PDController.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/graspplanning/GraspTable.hpp>
#include <rwsim/util/MovingAverage.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/util/RestingPoseGenerator.hpp>

#include <rwsim/util/GraspStrategy.hpp>
#include <rwsim/util/GraspPolicy.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>

#include <rwsimlibs/gui/ThreadSafeStack.hpp>


struct RestingConfig {
	RestingConfig(const rw::kinematics::State& state, const std::string& str):
		_state(state),_desc(str){}
	RestingConfig(){};
	rw::kinematics::State _state;
	std::string _desc;
};
/**
 * @brief a grphical interface for calculating resting configurations of
 * rigid bodies using rigid body physics simulation.
 *
 *
 */
class GraspTableGeneratorPlugin : public rws::RobWorkStudioPlugin, private Ui::GraspTableGeneratorPlugin
    {
        Q_OBJECT
		Q_INTERFACES( rws::RobWorkStudioPlugin )

    public:
        typedef std::vector<boost::numeric::ublas::matrix<float> > TactileSensorData;

        /**
         * @brief constructor
         */
        GraspTableGeneratorPlugin();

        /**
         * @brief destructor
         */
        virtual ~GraspTableGeneratorPlugin();

        /**
         * @brief starts the generation of the grasp table
         */
        void startTableGeneration();

        /**
         * @brief callback used for interfacing to ThreadSimulator
         */
        void stepCallBack(int i, const rw::kinematics::State& state);

        /**
         * @brief for state changes of RWS
         */
        void stateChangedListener(const rw::kinematics::State& state);

        /**
         * @brief we listen for events regarding opening and closing of dynamic
         * workcell
         */
        void genericEventListener(const std::string& event);

        ////// inherited from RobWorkStudioPlugin

        //! @copydoc RobWorkStudioPlugin::open
        void open(rw::models::WorkCell* workcell);

        //! @copydoc RobWorkStudioPlugin::close
        void close();

        //! @copydoc RobWorkStudioPlugin::initialize
        void initialize();

    private slots:
        void btnPressed();
        void changedEvent();

    private:
        /*
        void initializeStart();
        void updateStatus();

        bool isSimulationFinished( SimulatorPtr sim, const rw::kinematics::State& state );
        bool saveRestingState( int simidx, SimulatorPtr sim , const rw::kinematics::State& state );
         */

        void cleanup();
        void loadConfiguration(const std::string& filename);
        void saveConfiguration(const std::string& filename);
        void applyConfiguration();
        void readConfiguration();

    private:
        rw::common::PropertyMap _settings;

        struct CallBackFunctor {
            CallBackFunctor(int i,GraspTableGeneratorPlugin *parent):_i(i),_parent(parent){}

            void stepCallBack(const rw::kinematics::State& state){
                _parent->stepCallBack(_i, state);
            }

            int _i;
            GraspTableGeneratorPlugin *_parent;

        };

        std::vector<rwsim::util::RestingPoseGenerator*> _generators;

        Ui::GraspTableGeneratorPlugin _ui;

        rw::kinematics::State _defstate;
        rw::kinematics::State _state;
        QTimer *_timer;
        std::vector<rw::common::Ptr<rwsim::simulator::ThreadSimulator> > _simulators;
        std::vector<rw::kinematics::State> _initStates;
        std::vector<double> _simStartTimes;
        int _nrOfTests;
        double _totalSimTime;
        std::vector<rwsim::dynamics::RigidBody*> _bodies;

        long _startTime;

        std::vector<rw::kinematics::State> _startPoses;
        std::vector<rw::kinematics::State> _resultPoses;

        rw::kinematics::FrameMap<rwsim::dynamics::RigidBody*> _frameToBody;
        rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;

        rw::proximity::CollisionDetector *_colDect;
        double _lastTime,_lastBelowThresUpdate;
        rwsim::util::MovingAverage _avgSimTime;
        rwsim::util::MovingAverage _avgTime;

        std::vector<rwsim::control::PDControllerPtr> _controllers;
        std::vector<rw::math::Q> _preshapes;
        std::vector<rw::math::Q> _targetQ;
        rwsim::dynamics::RigidBody *_body;
        rwsim::dynamics::RigidDevice *_hand;
        rw::kinematics::MovableFrame *_handBase,*_object;

        rwsim::sensor::BodyContactSensorPtr _bodySensor;

        bool _exitHard;

        bool _graspNotStable;

        std::vector<bool> _fingersInContact;

        std::vector<std::vector< rw::math::Q > > _handconfigs;
        std::vector<std::vector< TactileSensorData > > _tactiledatas;

        std::vector<rw::common::Ptr<CallBackFunctor> > _functors;
        std::vector<double> _nextTimeUpdate;
        int _nrOfTestsOld;

        ThreadSafeStack<RestingConfig> _restingConfigs;

        //QSampler *_handQSampler;

        std::vector<int> _currentPreshapeIDX;
        rw::math::Q _target,_preshape;
        rw::math::Transform3D<> _objTransform;

        int _nrOfGraspsInGroup, _lastTableBackupCnt;
        int _tactileDataOnAllCnt;

        rwsim::util::GraspStrategy::Ptr _gstrategy;
        rwsim::util::GraspPolicy::Ptr _gpolicy;
        rwsim::simulator::DynamicSimulator::Ptr _simulator;

        rw::graspplanning::GraspTable *_gtable;
        std::string _configFile; // loadet on initialization
        rw::common::PropertyMap _config;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
