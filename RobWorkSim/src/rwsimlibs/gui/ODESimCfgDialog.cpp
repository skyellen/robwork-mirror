#include "ODESimCfgDialog.hpp"

#include <iostream>

#include <boost/foreach.hpp>

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>

#include <rwsim/dynamics/RigidBody.hpp>

#include <rwsim/simulator/PhysicsEngineFactory.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionDetector.hpp>

using namespace rwsim::dynamics;
using namespace rwsim::simulator;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;

#define RW_DEBUGS( str ) std::cout << str  << std::endl;

ODESimCfgDialog::ODESimCfgDialog(rw::common::Ptr<rwsim::simulator::DynamicSimulator> sim, QWidget *parent):
    QDialog(parent),
    _sim(sim)
{
    setupUi(this);
    updateValues();
    connect(_applyBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_cancelBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );


    connect(_objListWidget, SIGNAL(itemActivated(QListWidgetItem *)), this, SLOT(changedEvent()) );
    connect(_objListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(changedEvent()) );
    connect(_crThresSpin, SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
}


void ODESimCfgDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _applyBtn ){
    	applyChanges();
    } else if( obj == _cancelBtn ){

    } else  {

    }
}

void ODESimCfgDialog::changedEvent(){
    QObject *obj = sender();
	ODESimulator *sim = dynamic_cast<ODESimulator*>(_sim.get());
	if(!sim)
		RW_THROW("Not a ODE simulator!");

    if( obj == _objListWidget ){
    	int idx = _objListWidget->currentIndex().row();
    	ODEBody* body = sim->getODEBodies()[idx];
    	_selectedName->setText( body->getFrame()->getName().c_str() );
    	switch(body->getType()){
    	case(ODEBody::FIXED): _objectTypeName->setText( "FIXED" ); break;
    	case(ODEBody::RIGID):_objectTypeName->setText( "RIGID" ); break;
    	case(ODEBody::RIGIDJOINT): _objectTypeName->setText( "RIGIDJOINT" ); break;
    	case(ODEBody::KINJOINT): _objectTypeName->setText( "KINJOINT" ); break;
    	}

    	int matId = body->getMaterialID();
    	_materialBox->clear();
    	BOOST_FOREACH(const std::string& name, sim->getDynamicWorkCell()->getMaterialData().getMaterials() ){
			_materialBox->addItem(name.c_str());
    	}
    	_materialBox->setCurrentIndex(matId);
    	//const std::string& mname = sim->getDynamicWorkCell()->getMaterialData().getMaterialName();
    	_crThresSpin->setValue( body->getCRThres() );


    } else if( obj == _crThresSpin ){
    	int idx = _objListWidget->currentIndex().row();
    	ODEBody* body = sim->getODEBodies()[idx];
    	body->setCRTThres( _crThresSpin->value() );
    }

}

void ODESimCfgDialog::applyChanges(){
	ODESimulator *sim = dynamic_cast<ODESimulator*>(_sim.get());
	if(!sim)
		RW_THROW("Not a ODE simulator!");

	std::string stepMethod = _stepMethodBox->currentText().toStdString();
	int maxIter = _maxIterSpin->value();
	std::string space(_spaceMethodBox->currentText().toStdString());
	double cfm = _cfmSpin->value();
	double erp = _erpSpin->value();
	double colMargin = _marginSpin->value();
	std::string clusterAlg(_clusterAlgBox->currentText().toStdString());

	PropertyMap& map = _sim->getPropertyMap();
	map.set<int>("MaxIterations", maxIter);
	map.set<double>("WorldCFM", cfm);
	map.set<double>("WorldERP", erp);
	map.set<std::string>("SpaceType", space);
	map.set<std::string>("StepMethod", stepMethod);
	map.set<std::string>("ContactClusteringAlg", clusterAlg);

	sim->emitPropertyChanged();

}

void ODESimCfgDialog::updateValues(){
	ODESimulator *sim = dynamic_cast<ODESimulator*>(_sim.get());
	if(!sim)
		RW_THROW("Not a ODE simulator!");

	PropertyMap& map = sim->getPropertyMap();
	int maxIter = map.get<int>("MaxIterations", 20);
	std::string spaceTypeStr = map.get<std::string>("SpaceType", "QuadTree");
	std::string stepStr = map.get<std::string>("StepMethod", "WorldQuickStep");
	double worldCFM = map.get<double>("WorldCFM", 0.6);
	double worldERP = map.get<double>("WorldERP", 0.3);
	std::string clustAlgStr =  map.get<std::string>("ContactClusteringAlg", "Box");

	_maxIterSpin->setValue(maxIter);
	_cfmSpin->setValue(worldCFM);
	_erpSpin->setValue(worldERP);


	int i = _spaceMethodBox->findText( spaceTypeStr.c_str() );
	_spaceMethodBox->setCurrentIndex(i);
	i = _stepMethodBox->findText( stepStr.c_str() );
	_stepMethodBox->setCurrentIndex(i);
	i = _clusterAlgBox->findText( clustAlgStr.c_str() );
	_clusterAlgBox->setCurrentIndex(i);

	std::vector<ODEBody*>& bodies = sim->getODEBodies();
	_objListWidget->clear();
	BOOST_FOREACH(ODEBody* body, bodies){
		_objListWidget->addItem( QString( body->getFrame()->getName().c_str() ) );
	}

}
