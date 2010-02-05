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

#include <dynamics/DynamicWorkcell.hpp>
#include <dynamics/RigidBody.hpp>
#include <dynamics/DynamicWorkcell.hpp>
#include <simulator/ThreadSimulator.hpp>
#include <util/MovingAverage.hpp>
#include "TactileSensorDialog.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/proximity/CollisionDetector.hpp>

#include <rwlibs/drawable/Drawable.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

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
class RWSimPlugin : public RobWorkStudioPlugin, private Ui::RWSimPlugin
    {
		Q_OBJECT
		Q_INTERFACES( RobWorkStudioPlugin )
    public:
    	/**
    	 * @brief constructor
    	 */
        RWSimPlugin();

        /**
         * @brief destructor
         */
        virtual ~RWSimPlugin(){};

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

    signals:
        void updateView();
        void updateDialog();

    private slots:
        void btnPressed();
        void changedEvent();

    protected:
    	void updateStatus();

    private:

        Ui::RWSimPlugin _ui;

        QTimer *_timer;
        UserContext _context;
        rw::common::Ptr<dynamics::DynamicWorkcell> _dwc;
        rw::common::Ptr<ThreadSimulator> _sim;

        rw::kinematics::State _state;

        std::vector<rw::kinematics::State> _initStates;
        std::vector<double> _simStartTimes;
        int _nrOfTests;
        double _totalSimTime;
        std::vector<dynamics::RigidBody*> _bodies;

        long _startTime;

        std::vector<rw::kinematics::State> _startPoses;
        std::vector<rw::kinematics::State> _resultPoses;

        rw::kinematics::FrameMap<dynamics::RigidBody*> _frameToBody;
        rw::proximity::CollisionDetector *_colDect;
        double _lastTime,_lastBelowThresUpdate;

        rwlibs::drawable::Drawable *_debugRender;

        bool _forceSceneUpdateHist;

        TactileSensorDialog *_tactileSensorDialog;

        QTimer *_timerShot;
};


#endif /* RESTINGPOSEDIALOG_HPP_ */
