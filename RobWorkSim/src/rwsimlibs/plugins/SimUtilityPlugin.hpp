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

#include <rw/common/Ptr.hpp>
#include <RobWorkStudioPlugin.hpp>

#include <QObject>

namespace rwsim { namespace dynamics { class DynamicWorkCell; } }

struct RestingConfig;
class GraspRestingPoseDialog;
class GraspSelectionDialog;
class RestingPoseDialog;
class SupportPoseAnalyserDialog;

class QPushButton;

/**
 * @brief This plugin defines a set of utilities that are useful for analysis
 * and simulation purposes, that are based on some form of dynamic simulation.
 */
class SimUtilityPlugin: public rws::RobWorkStudioPlugin {
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "SimUtilityPlugin.json")
#endif
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
	QPushButton *_restPoseBtn, *_poseAnalyserBtn, *_toolEvalBtn, *_graspSelectBtn;

private:
    rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    RestingPoseDialog *_restPoseDialog;

    SupportPoseAnalyserDialog *_poseAnalyserDialog;
    GraspRestingPoseDialog *_graspRestPoseDialog;
    GraspSelectionDialog *_graspSelectionDialog;
    QTimer *_timer;
};



#endif //#ifndef ROBOCATCHER_HPP
