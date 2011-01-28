#include "SensorOutputGui.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;

//using namespace rwhw;



SensorOutputGui::SensorOutputGui():
    RobWorkStudioPlugin("Sample", QIcon(""))
{
	setupUi(this);

}


SensorOutputGui::~SensorOutputGui()
{
}



void SensorOutputGui::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SensorOutputGui::stateChangedListener, this, _1), this);
}



void SensorOutputGui::open(WorkCell* workcell)
{
}

void SensorOutputGui::close() {
}

void SensorOutputGui::clickEvent() {
/*
	if (!_sampleDialog ) {
        _sampleDialog = new SampleGUI(this);

        // connect all your signals to the dialog here
        //connect(_sampleDialog,SIGNAL(updateView()),this,SLOT(updateViewEvent()) );
    }

    _sampleDialog->show();
    _sampleDialog->raise();
    _sampleDialog->activateWindow();
*/
}

void SensorOutputGui::stateChangedListener(const State& state) {

}




Q_EXPORT_PLUGIN(SensorOutputGui);
