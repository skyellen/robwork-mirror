#include "CreateEngineDialog.hpp"

#include <QMessageBox>

#include <boost/foreach.hpp>

#include <rwsim/simulator/PhysicsEngine.hpp>

#include "ui_CreateEngineDialog.h"

using namespace rwsim::dynamics;
using namespace rw::math;
using namespace rw::common;
using namespace rwsim::simulator;

CreateEngineDialog::CreateEngineDialog(Ptr<DynamicWorkCell> dwc, QWidget *parent):
    QDialog(parent),
    _dwc(dwc)
{
	RW_ASSERT( _dwc );
	_ui = new Ui::CreateEngineDialog();
    _ui->setupUi(this);

	std::vector<std::string> engineIDs =
		PhysicsEngine::Factory::getEngineIDs();
	BOOST_FOREACH(const std::string& engineID, engineIDs){
	    _ui->_spaceMethodBox->addItem(engineID.c_str());
	}

	connect(_ui->_createBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_cancelBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    if(engineIDs.size()==0)
        _ui->_createBtn->setDisabled(true);
    _ui->_cancelBtn->setDisabled(true);
}


void CreateEngineDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _ui->_createBtn ){
        std::string engineId = _ui->_spaceMethodBox->currentText().toStdString();
        try {
            PhysicsEngine::Ptr pengine = PhysicsEngine::Factory::makePhysicsEngine(engineId, _dwc);
            _sim = ownedPtr( new DynamicSimulator(_dwc, pengine) );
        } catch(...) {
            QMessageBox::information(this, "Creating Engine", "Error creating Physics Engine!");
            _sim = NULL;
            reject();
        }

        std::cout << "********** SIMULATOR CREATED " << std::endl;
        accept();
    } else if( obj == _ui->_cancelBtn ) {
    	_sim = NULL;
    	reject();
    }
}

void CreateEngineDialog::changedEvent(){
    //QObject *obj = sender();

    //if( obj == _timer ){}
}

