#ifndef SensorOutputGui_HPP
#define SensorOutputGui_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>

#include "ui_SensorOutputGui.h"

class SensorOutputGui: public RobWorkStudioPlugin, private Ui::SensorOutputGui
{
Q_OBJECT
Q_INTERFACES( RobWorkStudioPlugin )
public:
	SensorOutputGui();
	virtual ~SensorOutputGui();


    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void clickEvent();

    void stateChangedListener(const rw::kinematics::State& state);

private:
	Ui::SensorOutputGui _ui;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
