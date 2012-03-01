/*
 * RestingPoseDialog.hpp
 *
 *  Created on: 04-12-2008
 *      Author: jimali
 */

#ifndef RWSIMPLUGIN_HPP_
#define RWSIMPLUGIN_HPP_

#ifdef __WIN32
#include <windows.h>
#endif

#include "ui_RWSimPlugin.h"

#include <rwsimlibs/gui/TactileSensorDialog.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/simulator/ThreadSimulator.hpp>
#include <rwsim/util/MovingAverage.hpp>
#include <rw/graspplanning/GraspTable.hpp>

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rwlibs/opengl/Drawable.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include <rwsimlibs/ode/ODESimulator.hpp>

#include <RobWorkSimConfig.hpp>
#ifdef RWSIM_HAVE_LUA
#include <rws/swig/LuaState.hpp>
#include <rwsimlibs/swig/Lua.hpp>
#include <rwsimlibs/swig/ScriptTypes.hpp>
#endif


#include <QObject>
#include <QtGui>
#include <QTimer>

struct UserContext {
	std::string _previousOpenDirectory;
};

/**
 * @brief A plugin for loading dynamic workcells and for doing simple
 * dynamics simulation using different physics engines.
 *
 *
 */
class RWSimPlugin : public rws::RobWorkStudioPlugin, private Ui::RWSimPlugin
    {
		Q_OBJECT
		Q_INTERFACES( rws::RobWorkStudioPlugin )
    public:
    	/**
    	 * @brief constructor
    	 */
        RWSimPlugin();

        /**
         * @brief destructor
         */
        virtual ~RWSimPlugin();

        /**
         * @copydoc RobWorkStudioPlugin::open
         */
        void open(rw::models::WorkCell* workcell);

        /**
         * @copydoc RobWorkStudioPlugin::close
         */
        void close();

        /**
         * @copydoc RobWorkStudioPlugin::initialize
         */
        void initialize();

        /**
         *
         */
        void stateChangedListener(const rw::kinematics::State& state);

        /**
         * @brief opens a dynamic workcell with filename \b file.
         *
         * @note if successfully openned a generic event with string id
         * 'DynamicWorkcellLoadet' will be emitted and the DynamicWorkCell will
         * be saved in the RobWorkStudio propertymap with the string id 'DynamicWorkcell'.
         */
        void openDwc(const std::string& file);

        void stepCallBack(const rw::kinematics::State& state);

        void setupMenu(QMenu* menu);

    signals:
        void updateView();
        //void updateDialog();
        void updateDialog(const rw::kinematics::State& state);

    private slots:
        void btnPressed();
        void changedEvent();
        void setRobWorkStudioState(const rw::kinematics::State& state);

    protected:
    	void updateStatus();

    	rw::common::PropertyMap& settings();

    private:


        #ifdef RWSIM_HAVE_LUA
    	rws::LuaState *_luastate;
        #endif


        Ui::RWSimPlugin _ui;

        QTimer *_timer;
        UserContext _context;
        rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
        rw::common::Ptr<rwsim::simulator::ThreadSimulator> _sim;

        rw::kinematics::State _state;

        //std::vector<rw::kinematics::State> _initStates;
        //std::vector<double> _simStartTimes;
        //int _nrOfTests;
        //double _totalSimTime;
        //std::vector<rwsim::dynamics::RigidBody*> _bodies;

        long _startTime;

        //std::vector<rw::kinematics::State> _startPoses;
        //std::vector<rw::kinematics::State> _resultPoses;

        //rw::kinematics::FrameMap<rwsim::dynamics::RigidBody*> _frameToBody;
        //rw::proximity::CollisionDetector *_colDect;
        //double _lastTime,_lastBelowThresUpdate;
        rw::trajectory::TimedStatePath _path;
        rwlibs::opengl::Drawable *_debugDrawable;
        rwsim::drawable::SimulatorDebugRender::Ptr _debugRender;

        bool _forceSceneUpdateHist, _openCalled;

        TactileSensorDialog *_tactileSensorDialog;

        QTimer *_timerShot;

        rw::graspplanning::GraspTable _gtable;

        QAction *_openAction,
                *_planarPoseDistAction,
                *_poseDistAction,
                *_graspSelectionAction,
                *_graspRestPoseAction,
                *_restPoseAction,
                *_poseAnalyserAction;

};


#endif /* RESTINGPOSEDIALOG_HPP_ */
