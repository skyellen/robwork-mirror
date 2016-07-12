/*
 * GraspRestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef GRASPRESTINGPOSEDIALOG_HPP_
#define GRASPRESTINGPOSEDIALOG_HPP_

#include <QObject>
#include <QDialog>

#include <rw/kinematics/State.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rw/kinematics/FrameMap.hpp>

#include <rw/graspplanning/GraspTable.hpp>
#include <rwsim/util/MovingAverage.hpp>

#include "ThreadSafeStack.hpp"

namespace rw { namespace proximity { class CollisionDetector; } }
namespace rwsim { namespace control { class PDController; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace dynamics { class RigidDevice; } }
namespace rwsim { namespace sensor { class BodyContactSensor; } }
namespace rwsim { namespace simulator { class DynamicSimulator; } }
namespace rwsim { namespace simulator { class ThreadSimulator; } }

namespace Ui {
    class GraspRestingPoseDialog;
}

class QTimer;

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
class GraspRestingPoseDialog : public QDialog
    {
        Q_OBJECT

    public:
        typedef std::vector<Eigen::MatrixXf> TactileSensorData;

        GraspRestingPoseDialog(const rw::kinematics::State& state,
                          rwsim::dynamics::DynamicWorkCell *dwc,
                          rw::proximity::CollisionDetector *detector,
                          QWidget *parent = 0);

        const rw::kinematics::State& getState(){ return _state; };

        std::vector<rwsim::dynamics::RigidBody::Ptr>& getBodies(){ return _bodies; };

        std::vector<rw::kinematics::State>& getStartPoses(){return _startPoses;};

        std::vector<rw::kinematics::State>& getRestingPoses(){return _resultPoses;};

        void setPreshapeStrategy(const std::string& str);

        void setSaveDir(const std::string& str);

        void setUniqueID(const std::string& id){
        	_id = id;
        }

        void startAuto();

        void stepCallBack(int i, const rw::kinematics::State& state);


    signals:
        /**
         * @brief The state of one simulation thread can be actively monitored
         * through this signal.
         * @param state
         */
        void stateChanged(const rw::kinematics::State& state);

        /**
         * @brief An event that is fired when a resting pose has been calculated.
         */
        void restingPoseEvent(const RestingConfig& restcfg);

    private slots:
        void btnPressed();
        void changedEvent();

    private:
    	void initializeStart();
        void updateStatus();

        /**
         * @brief calculates a random configuration of
         * all bodies
         * @param state
         */
        void calcRandomCfg(rw::kinematics::State& state);

        /**
         * @brief Calculate random configuration for \b bodies
         * @param bodies
         * @param state
         */
        void calcRandomCfg(std::vector<rwsim::dynamics::RigidBody::Ptr> &bodies,
                           rw::kinematics::State& state);

        /**
         * @brief calculates a collision free random configuration of
         * all bodies
         * @param state
         */
        void calcColFreeRandomCfg(rw::kinematics::State& state);

        bool isSimulationFinished( rw::common::Ptr<rwsim::simulator::DynamicSimulator> sim, const rw::kinematics::State& state );

        bool saveRestingState( int simidx, rw::common::Ptr<rwsim::simulator::DynamicSimulator> sim , const rw::kinematics::State& state );


    private:
        struct CallBackFunctor {
            CallBackFunctor(int i,GraspRestingPoseDialog *parent):_i(i),_parent(parent){}

            void stepCallBack(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state){
                _parent->stepCallBack(_i, state);
            }

            int _i;
            GraspRestingPoseDialog *_parent;

        };

        Ui::GraspRestingPoseDialog *_ui;
        rw::kinematics::State _defstate;
        rw::kinematics::State _state;
        QTimer *_timer;
        std::vector<rw::common::Ptr<rwsim::simulator::ThreadSimulator> > _simulators;
        std::vector<rw::kinematics::State> _initStates;
        std::vector<double> _simStartTimes;
        int _nrOfTests;
        double _totalSimTime;
        std::vector<rwsim::dynamics::RigidBody::Ptr> _bodies;

        long _startTime;
        std::string _id;
        std::vector<rw::kinematics::State> _startPoses;
        std::vector<rw::kinematics::State> _resultPoses;

        rw::kinematics::FrameMap<rwsim::dynamics::RigidBody::Ptr> _frameToBody;
        rwsim::dynamics::DynamicWorkCell *_dwc;
        rw::proximity::CollisionDetector *_colDect;
        double _lastTime,_lastBelowThresUpdate;
        rwsim::util::MovingAverage _avgSimTime;
        rwsim::util::MovingAverage _avgTime;

        std::vector<rw::common::Ptr<rwsim::control::PDController> > _controllers;
        std::vector<rw::math::Q> _preshapes;
        std::vector<rw::math::Q> _targetQ;
        rwsim::dynamics::RigidBody::Ptr _body;
        rw::common::Ptr<rwsim::dynamics::RigidDevice> _hand;
        rw::kinematics::MovableFrame *_handBase,*_object;

        rw::common::Ptr<rwsim::sensor::BodyContactSensor> _bodySensor;

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
        rw::math::Q _target,_preshape,_handForceLimitsDefault;
        rw::math::Transform3D<> _objTransform;

        rw::graspplanning::GraspTable _gtable;
        int _nrOfGraspsInGroup, _lastTableBackupCnt;
        int _tactileDataOnAllCnt;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
