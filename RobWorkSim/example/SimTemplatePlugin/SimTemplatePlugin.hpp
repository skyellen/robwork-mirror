#ifndef SimTemplatePlugin_HPP
#define SimTemplatePlugin_HPP

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include "ui_SimTemplatePlugin.h"

#include <QObject>
#include <QtGui>
#include <QTimer>

/**
 * @brief A plugin
 */
class SimTemplatePlugin: public rws::RobWorkStudioPlugin, private Ui::SimTemplatePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:

    /**
     * @brief constructor
     */
    SimTemplatePlugin();

    //! destructor
    virtual ~SimTemplatePlugin();

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
    rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
    rwsim::simulator::ThreadSimulator::Ptr _tsim;
    rwsim::simulator::DynamicSimulator::Ptr _sim;
    rwsim::simulator::ODESimulator::Ptr _engine;

    QTimer *_timer;
    rw::common::Timer _wallTimer, _wallTotalTimer;
};

#endif
