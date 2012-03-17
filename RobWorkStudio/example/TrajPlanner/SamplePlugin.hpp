#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/rw.hpp>
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

    void planL(rw::models::JointDevice* dev,
                             const rw::math::Transform3D<>& from,
                             const rw::math::Transform3D<>& to);

private slots:
    void btnPressed();

    void stateChangedListener(const rw::kinematics::State& state);

private:
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
