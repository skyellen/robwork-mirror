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

#ifndef SIMUTILITYPLUGIN_HPP
#define SIMUTILITYPLUGIN_HPP

#ifdef __WIN32
#include <windows.h>
#endif


#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <dynamics/DynamicWorkcell.hpp>
#include <rw/common/Ptr.hpp>
#include <RobWorkStudioPlugin.hpp>


#include <QObject>
#include <QtGui>

class RestingPoseDialog;
class SupportPoseAnalyserDialog;

#include "GraspRestingPoseDialog.hpp"

/**
 * @brief This plugin defines a set of utilities that are useful for analysis
 * and simulation purposes, that are based on some form of dynamic simulation.
 */
class SimUtilityPlugin: public RobWorkStudioPlugin {
Q_OBJECT
Q_INTERFACES( RobWorkStudioPlugin )
public:
    /**
     * @brief constructor
     */
    SimUtilityPlugin();

    /**
     * @brief destructor
     */
    virtual ~SimUtilityPlugin();

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
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);



protected:
	virtual void stateChangedHandler(RobWorkStudioPlugin* sender);

private slots:
    /**
     * @brief
     */
    void updateViewEvent();
    void stateChangedEvent(const rw::kinematics::State& state);
    void btnPressed();
    void restConfigEvent(const RestingConfig& state);

private: // qt stuff
	QPushButton *_restPoseBtn, *_poseAnalyserBtn, *_toolEvalBtn;

private:
    rw::common::Ptr<dynamics::DynamicWorkcell> _dwc;
    RestingPoseDialog *_restPoseDialog;
    GraspRestingPoseDialog *_graspRestPoseDialog;
    SupportPoseAnalyserDialog *_poseAnalyserDialog;
    QTimer *_timer;
};



#endif //#ifndef ROBOCATCHER_HPP
