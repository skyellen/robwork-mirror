#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>

#include "ui_SamplePlugin.h"

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
    SamplePlugin();
	virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
	void on_btnAddGeometry_clicked();
	void on_btnGrasp_clicked();
	void on_btnCheckCollision_clicked();
	void on_btnRegEx_clicked();

    void stateChangedListener(const rw::kinematics::State& state);

private:
	rw::kinematics::State _state;
	rw::models::Device::Ptr _dev;
	rw::models::WorkCell::Ptr _workcell;
	rw::proximity::CollisionDetector::Ptr _cd;
	rw::proximity::BasicFilterStrategy::Ptr _bp;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
