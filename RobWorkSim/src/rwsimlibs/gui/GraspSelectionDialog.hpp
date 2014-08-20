/*
 * GraspSelectionDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef GRASPSELECTIONDIALOG_HPP_
#define GRASPSELECTIONDIALOG_HPP_

#ifdef __WIN32
#include <windows.h>
#endif



#include <rw/kinematics/State.hpp>
#include <rw/graspplanning/GraspTable.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>

#include <rw/kinematics/FrameMap.hpp>

#include <rwsim/util/MovingAverage.hpp>

#include <rw/proximity/CollisionDetector.hpp>

#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>

#include <QObject>
#include <QtGui>
#include <QTimer>
#include <QDialog>

namespace Ui {
    class GraspSelectionDialog;
}

/**
 * @brief a grphical interface for calculating resting configurations of
 * rigid bodies using rigid body physics simulation.
 *
 *
 */
class GraspSelectionDialog : public QDialog
    {
        Q_OBJECT

    public:
        GraspSelectionDialog(const rw::kinematics::State& state,
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
        void restingPoseEvent(const rw::kinematics::State& state,const rw::kinematics::State&);

    private slots:
        void btnPressed();
        void changedEvent();

    private:
    	void initializeStart();
        void updateStatus();

        void setGraspState(rw::graspplanning::GraspTable::GraspData& data, rw::kinematics::State &state);

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
        Ui::GraspSelectionDialog *_ui;
        rw::kinematics::State _defstate;
        rw::kinematics::State _state;
        QTimer *_timer;
        std::vector<rw::common::Ptr<rwsim::simulator::ThreadSimulator> > _simulators;
        std::vector<rw::kinematics::State> _initStates;
        std::vector<double> _simStartTimes;
        int _nrOfTests;
        double _totalSimTime;
        std::vector<rwsim::dynamics::RigidBody::Ptr> _bodies;

        rw::models::Device *_dev;
        rw::kinematics::MovableFrame* _object, *_handBase;

        long _startTime;

        std::vector<rw::kinematics::State> _startPoses;
        std::vector<rw::kinematics::State> _resultPoses;

        rw::kinematics::FrameMap<rwsim::dynamics::RigidBody::Ptr> _frameToBody;
        rwsim::dynamics::DynamicWorkCell *_dwc;
        rw::proximity::CollisionDetector *_colDect;
        double _lastTime,_lastBelowThresUpdate;
        rwsim::util::MovingAverage _avgSimTime;
        rwsim::util::MovingAverage _avgTime;

        rw::common::Ptr<rw::graspplanning::GraspTable> _gtable;
        typedef rwlibs::algorithms::KDTreeQ<rw::graspplanning::GraspTable::GraspData*> SearchTree;
        SearchTree *_kdtree;
        std::vector<SearchTree::KDNode> _nodes;
};


#endif /* GraspSelectionDialog_HPP_ */
