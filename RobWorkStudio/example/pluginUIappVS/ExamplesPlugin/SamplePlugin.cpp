#include "SamplePlugin.hpp"

#include <QPushButton>

#include <RobWorkStudio.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <boost/regex.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::task;

using namespace rws;



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);


}  

SamplePlugin::~SamplePlugin()
{
}    
 
void SamplePlugin::initialize() {
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
}

#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <fstream>
using namespace rw::trajectory;

class B;
class A {
protected:
	friend B;
	void test() {}

private: 
	int t;

};

class B: public A {
	void test2(A& a, B& b) {
		a.test();
		b.test();
		test();
	}
};



void SamplePlugin::open(WorkCell* workcell)
{
	_workcell = workcell;
	  if (workcell == NULL)
		  return;
	  if (workcell->getDevices().size() == 0)
		  return;

	  _dev = workcell->getDevices().front();

	_bp = ownedPtr(new BasicFilterStrategy(workcell));
	_cd = ownedPtr(new CollisionDetector(workcell, ProximityStrategyYaobi::make(), _bp));	
} 

void SamplePlugin::close() { 
}   
    

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
	on_btnCheckCollision_clicked();
}


void SamplePlugin::on_btnRegEx_clicked() {
	try {
	const boost::regex e(".*.XML"); //This is the regular expression
	QString str = QInputDialog::getText(this, "", "");
	if (regex_match( str.toStdString(), e ))
	{
		std::cout << str.toStdString()
			<< " contains lowercase characters.\n";
	}
	else
	{
		std::cout << str.toStdString()
			<< " contains no lowercase characters.\n";
	}	
	} catch (std::exception& exp) {
		std::cout<<"Exception : "<<exp.what()<<std::endl;
	}
	catch (...) {
		std::cout<<"Unknown Exception"<<std::endl;
	}

}

void SamplePlugin::on_btnCheckCollision_clicked() {
	if (_cd->inCollision(_state)) {
		label->setText("Collision");
	} else {
		label->setText("No Collision");
	}
}
 

void SamplePlugin::on_btnAddGeometry_clicked() {

	Geometry::Ptr geo = GeometryFactory::load("d:\\temp\\Kr16WorkCell\\Geometry\\bottle.stl");
	std::cout<<"Geometry = "<<geo<<std::endl;
	Frame* obj = _workcell->findFrame("Bottle");
	if (obj == NULL) {
		QMessageBox::information(this, "", "No frame");
		return;
	}
		/*
	_cd->addGeometry(obj, geo);
	getRobWorkStudio()->getWorkCellScene()->addGeometry("MyGeo", geo, obj);*/

}

void SamplePlugin::on_btnGrasp_clicked() {
	std::cout<<"We should grasp now"<<std::endl;
}



Q_EXPORT_PLUGIN(SamplePlugin);
