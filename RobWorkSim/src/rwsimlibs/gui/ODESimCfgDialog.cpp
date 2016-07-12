#include "ODESimCfgDialog.hpp"

#include <boost/foreach.hpp>

#include <rwsimlibs/ode/ODESimulator.hpp>

#include <rwsim/simulator/DynamicSimulator.hpp>

#include "ui_ODESimCfgForm.h"

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
    _ui = new Ui::ODESimCfgForm();
    _ui->setupUi(this);
    updateValues();
    connect(_ui->_applyBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_cancelBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );


    connect(_ui->_objListWidget, SIGNAL(itemActivated(QListWidgetItem *)), this, SLOT(changedEvent()) );
    connect(_ui->_objListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(changedEvent()) );
    connect(_ui->_crThresSpin, SIGNAL(valueChanged(double)), this, SLOT(changedEvent()) );
}


void ODESimCfgDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _ui->_applyBtn ){
    	applyChanges();
    } else if( obj == _ui->_cancelBtn ){

    } else  {

    }
}

void ODESimCfgDialog::changedEvent(){
    QObject *obj = sender();
	ODESimulator *sim = dynamic_cast<ODESimulator*>(_sim.get());
	if(!sim)
		RW_THROW("Not a ODE simulator!");

    if( obj == _ui->_objListWidget ){
    	int idx = _ui->_objListWidget->currentIndex().row();
    	ODEBody* body = sim->getODEBodies()[idx];
    	_ui->_selectedName->setText( body->getFrame()->getName().c_str() );
    	switch(body->getType()){
    	case(ODEBody::FIXED): _ui->_objectTypeName->setText( "FIXED" ); break;
    	case(ODEBody::RIGID):_ui->_objectTypeName->setText( "RIGID" ); break;
    	//case(ODEBody::RIGIDJOINT): _objectTypeName->setText( "RIGIDJOINT" ); break;
    	//case(ODEBody::KINJOINT): _objectTypeName->setText( "KINJOINT" ); break;
    	case(ODEBody::KINEMATIC): _ui->_objectTypeName->setText( "KINEMATIC" ); break;
    	case(ODEBody::LINK): _ui->_objectTypeName->setText( "LINK" ); break;
    	default: break;
    	}

    	int matId = body->getMaterialID();
    	_ui->_materialBox->clear();
    	BOOST_FOREACH(const std::string& name, sim->getDynamicWorkCell()->getMaterialData().getMaterials() ){
    	    _ui->_materialBox->addItem(name.c_str());
    	}
    	_ui->_materialBox->setCurrentIndex(matId);
    	//const std::string& mname = sim->getDynamicWorkCell()->getMaterialData().getMaterialName();
    	_ui->_crThresSpin->setValue( body->getCRThres() );


    } else if( obj == _ui->_crThresSpin ){
    	int idx = _ui->_objListWidget->currentIndex().row();
    	ODEBody* body = sim->getODEBodies()[idx];
    	body->setCRTThres( _ui->_crThresSpin->value() );
    }

}

void ODESimCfgDialog::applyChanges(){
	ODESimulator *sim = dynamic_cast<ODESimulator*>(_sim.get());
	if(!sim)
		RW_THROW("Not a ODE simulator!");

	std::string stepMethod = _ui->_stepMethodBox->currentText().toStdString();
	int maxIter = _ui->_maxIterSpin->value();
	std::string space(_ui->_spaceMethodBox->currentText().toStdString());
	double cfm = _ui->_cfmSpin->value();
	double erp = _ui->_erpSpin->value();
	//double colMargin = _marginSpin->value();
	std::string clusterAlg(_ui->_clusterAlgBox->currentText().toStdString());

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

	_ui->_maxIterSpin->setValue(maxIter);
	_ui->_cfmSpin->setValue(worldCFM);
	_ui->_erpSpin->setValue(worldERP);


	int i = _ui->_spaceMethodBox->findText( spaceTypeStr.c_str() );
	_ui->_spaceMethodBox->setCurrentIndex(i);
	i = _ui->_stepMethodBox->findText( stepStr.c_str() );
	_ui->_stepMethodBox->setCurrentIndex(i);
	i = _ui->_clusterAlgBox->findText( clustAlgStr.c_str() );
	_ui->_clusterAlgBox->setCurrentIndex(i);

	std::vector<ODEBody*>& bodies = sim->getODEBodies();
	_ui->_objListWidget->clear();
	BOOST_FOREACH(ODEBody* body, bodies){
	    _ui->_objListWidget->addItem( QString( body->getFrame()->getName().c_str() ) );
	}

}
