#include "SamplePlugin.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>


USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    // now connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

}

SamplePlugin::~SamplePlugin()
{
}

void SamplePlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
}

void SamplePlugin::open(WorkCell* workcell)
{
}

void SamplePlugin::close() {
}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if(obj==_btn0){
        log().info() << "Button 0\n";
    } else if(obj==_btn1){
        log().info() << "Button 1\n";
    } else if(obj==_spinBox){
        log().info() << "spin value:" << _spinBox->value() << "\n";
    }


}

void SamplePlugin::stateChangedListener(const State& state) {

}

void SamplePlugin::planL(rw::models::JointDevice* dev,
                         const rw::math::Transform3D<>& from,
                         const rw::math::Transform3D<>& to)
{
    // first we make a linear interpolator
    rw::trajectory::LinearInterpolator<Transform3D<> > posepath(from, to, 1.0);

}




Q_EXPORT_PLUGIN(SamplePlugin);
