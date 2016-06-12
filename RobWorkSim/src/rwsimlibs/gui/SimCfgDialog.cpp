#include "SimCfgDialog.hpp"

#include <iostream>

#include <boost/foreach.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <RobWorkSimConfig.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwsim/simulator/PhysicsEngineFactory.hpp>
#include <QTabWidget>

#include "ODESimCfgDialog.hpp"

#include "ui_SimCfgDialog.h"

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;

#define RW_DEBUGS( str ) std::cout << str  << std::endl;

SimCfgDialog::SimCfgDialog(rw::common::Ptr<DynamicSimulator> sim, QWidget *parent):
    QDialog(parent),
    _sim(sim)
{
    _ui = new Ui::SimCfgDialog();
    _ui->setupUi(this);

    connect(_ui->_applyBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_cancelBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

	std::vector<std::string> engineIDs =
	PhysicsEngineFactory::getEngineIDs();

	//std::string id = _sim->getID();
	//_tabPane->addItem(id);
#ifdef RWSIM_HAVE_ODE
	ODESimCfgDialog *dialog = new ODESimCfgDialog(sim, this);
	_ui->_tabPane->addTab(dialog, "ODE");
#endif
	//BOOST_FOREACH(const std::string& engineID, engineIDs){
		//_tabPane->addTab(this, engineID.c_str() );
		//_spaceMethodBox->addItem(engineID.c_str());
	//}

    //if(engineIDs.size()==0)
    //	_createBtn->setDisabled(true);
    //_cancelBtn->setDisabled(true);

}

void SimCfgDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _ui->_applyBtn ){

    } else if( obj == _ui->_cancelBtn ) {

    }
}

void SimCfgDialog::changedEvent(){
    //QObject *obj = sender();
    /*
    if( obj == _timer ){

    }
    */
}

