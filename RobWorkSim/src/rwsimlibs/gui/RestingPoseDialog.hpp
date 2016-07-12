/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef RESTINGPOSEDIALOG_HPP_
#define RESTINGPOSEDIALOG_HPP_

#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/State.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/util/MovingAverage.hpp>

#include <QObject>
#include <QDialog>

namespace rw { namespace proximity { class CollisionDetector; } }
namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace simulator { class ThreadSimulator; } }

namespace Ui {
    class RestingPoseDialog;
}

class QTimer;

/**
 * @brief a grphical interface for calculating resting configurations of
 * rigid bodies using rigid body physics simulation.
 *
 *
 */
class RestingPoseDialog : public QDialog
    {
        Q_OBJECT

    public:
        RestingPoseDialog(const rw::kinematics::State& state,
                          rwsim::dynamics::DynamicWorkCell *dwc,
                          rw::proximity::CollisionDetector *detector,
                          QWidget *parent = 0);

        const rw::kinematics::State& getState(){ return _state; };

        std::vector<rwsim::dynamics::RigidBody::Ptr>& getBodies(){ return _bodies; };

        std::vector<rw::kinematics::State>& getStartPoses(){return _startPoses;};

        std::vector<rw::kinematics::State>& getRestingPoses(){return _resultPoses;};


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
        void restingPoseEvent(const rw::kinematics::State& state, const rw::kinematics::State&);

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


    private:
        Ui::RestingPoseDialog *_ui;
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

        std::vector<rw::kinematics::State> _startPoses;
        std::vector<rw::kinematics::State> _resultPoses;

        rw::kinematics::FrameMap<rwsim::dynamics::RigidBody::Ptr> _frameToBody;
        rwsim::dynamics::DynamicWorkCell *_dwc;
        rw::proximity::CollisionDetector *_colDect;
        double _lastTime,_lastBelowThresUpdate;
        rwsim::util::MovingAverage _avgSimTime;
        rwsim::util::MovingAverage _avgTime;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
