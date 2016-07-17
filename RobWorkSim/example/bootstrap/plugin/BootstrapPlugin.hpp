#ifndef BootstrapPlugin_HPP
#define BootstrapPlugin_HPP

#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include <rws/RobWorkStudioPlugin.hpp>
#include "ui_BootstrapPlugin.h"

#include <QObject>
//#include <rw/common/Timer.hpp>

namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace simulator { class DynamicSimulator; } }
namespace rwsim { namespace simulator { class ThreadSimulator; } }

class Brain;
class QTimer;

/**
 * @brief A plugin
 */
class BootstrapPlugin: public rws::RobWorkStudioPlugin, private Ui::BootstrapPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
	Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "BootstrapPlugin.json")
#endif
public:

    /**
     * @brief constructor
     */
	BootstrapPlugin();

    //! destructor
    virtual ~BootstrapPlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

    /**
     * @brief we listen for events regarding opening and closing of dynamic
     * workcell
     */
    void genericEventListener(const std::string& event);

    void makeSimulator();
    void step(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state);
    void startSimulation();

private slots:
    void btnPressed();
    void stateChangedListener(const rw::kinematics::State& state);

private:
    rw::models::WorkCell* _wc;
    rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> _dwc;
    rw::common::Ptr<rwsim::simulator::ThreadSimulator> _tsim;
    rw::common::Ptr<rwsim::simulator::DynamicSimulator> _sim;

    rw::common::Ptr<Brain> _brain;

    QTimer *_timer;
    //rw::common::Timer _wallTimer, _wallTotalTimer;
};

#endif /*BootstrapPlugin_HPP*/
