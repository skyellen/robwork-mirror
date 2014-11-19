#include "DesignDialog.hpp"

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <QtGui>
#include <QRadioButton>
#include "GripperXMLLoader.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



DesignDialog::DesignDialog(QWidget* parent, rw::models::Gripper::Ptr gripper, std::string wd) :
	QDialog(parent),
	_gripper(gripper),
	_changed(false),
	_wd(wd)
{
	if (!_gripper) _gripper = ownedPtr(new Gripper);
	
	ui.setupUi(this);
	
	connect(ui.okButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.applyButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.cancelButton, SIGNAL(clicked()), this, SLOT(close()));
	connect(ui.defaultButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.prismaticButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.cylindricalButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.nameEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.lengthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.widthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.depthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.chfDepthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.chfAngleEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.cutDepthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.cutAngleEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.cutRadiusEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.cutTiltEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.tcpEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.jawdistEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.openingEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.basedxEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.basedyEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(ui.basedzEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	
	_updateGUI();
}



void DesignDialog::guiEvent()
{
	QObject* obj = sender();
	
	if (obj == ui.okButton) {
		_gripper = ownedPtr(new Gripper);
		_updateGripper();
		close();
	}
	
	else if (obj == ui.applyButton) {
		_updateGripper();
	}
	
	else if (obj == ui.defaultButton) {
		_gripper = ownedPtr(new Gripper);
		_updateGUI();
	}
	
	else if (obj == ui.prismaticButton) {
		_updateGripper();
		_updateGUI();
	}
	
	else if (obj == ui.cylindricalButton) {
		_updateGripper();
		_updateGUI();
	}
	
	_changed = true;
}



void DesignDialog::_updateGripper()
{
	if (_gripper) {
		// update jaw geometry
		Q jawParams(11);
		jawParams(0) = (ui.cylindricalButton->isChecked() ? 1 : 0);
		jawParams(1) = ui.lengthEdit->text().toDouble();
		jawParams(2) = ui.widthEdit->text().toDouble();
		jawParams(3) = ui.depthEdit->text().toDouble();
		jawParams(4) = ui.chfDepthEdit->text().toDouble();
		jawParams(5) = Deg2Rad*ui.chfAngleEdit->text().toDouble();
		//jawParams(6) = ui.lengthEdit->text().toDouble() - ui.tcpEdit->text().toDouble();
		jawParams(6) = ui.tcpEdit->text().toDouble();
		jawParams(7) = ui.cutDepthEdit->text().toDouble();
		jawParams(8) = Deg2Rad*ui.cutAngleEdit->text().toDouble();
		jawParams(9) = ui.cutRadiusEdit->text().toDouble();
		jawParams(10) = Deg2Rad*ui.cutTiltEdit->text().toDouble();
		_gripper->setJawGeometry(jawParams);
		
		// update base geometry
		Q baseParams(3);
		baseParams(0) = ui.basedxEdit->text().toDouble();
		baseParams(1) = ui.basedyEdit->text().toDouble();
		baseParams(2) = ui.basedzEdit->text().toDouble();
		_gripper->setBaseGeometry(baseParams);
		
		// update results
		_gripper->getQuality().quality = ui.qualityEdit->text().toDouble();
		
		// update general parameters
		_gripper->setName(ui.nameEdit->text().toStdString());
		//_gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, ui.lengthEdit->text().toDouble() - ui.tcpEdit->text().toDouble())));
		_gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, ui.tcpEdit->text().toDouble())));
		_gripper->setForce(ui.forceEdit->text().toDouble());
		_gripper->setJawdist(ui.jawdistEdit->text().toDouble());
		_gripper->setOpening(ui.openingEdit->text().toDouble());
	}
}



void DesignDialog::_updateGUI()
{
	if (_gripper) {		
		// update jaw geometry area
		Q jawParams = _gripper->getJawParameters();
		if (jawParams.size() == 10 || jawParams.size() == 11) {
			cout << jawParams(0) << endl;
			if (jawParams(0) == 0) {
				ui.prismaticButton->setChecked(true);
			} else {
				ui.cylindricalButton->setChecked(true);
			}
			ui.lengthEdit->setText(QString::number(jawParams(1)));
			ui.widthEdit->setText(QString::number(jawParams(2)));
			ui.depthEdit->setText(QString::number(jawParams(3)));
			ui.chfDepthEdit->setText(QString::number(jawParams(4)));
			ui.chfAngleEdit->setText(QString::number(Rad2Deg*jawParams(5)));
			//ui.cutPosEdit->setText(QString::number(jawParams(1)-jawParams(6)));
			ui.cutDepthEdit->setText(QString::number(jawParams(7)));
			ui.cutAngleEdit->setText(QString::number(Rad2Deg*jawParams(8)));
			ui.cutRadiusEdit->setText(QString::number(jawParams(9)));
			
			if (jawParams.size() == 11) {
				ui.cutTiltEdit->setText(QString::number(Rad2Deg*jawParams(10)));
			} else {
				ui.cutTiltEdit->setText(QString::number(0.0));
			}
		} else {
			QMessageBox::warning(NULL, "DesignDialog", "Gripper uses jaw geometry from STL!");
		}
		
		// update base geometry area
		Q baseParams = _gripper->getBaseParameters();
		if (baseParams.size() == 3) {
			ui.basedxEdit->setText(QString::number(baseParams(0)));
			ui.basedyEdit->setText(QString::number(baseParams(1)));
			ui.basedzEdit->setText(QString::number(baseParams(2)));
		} else {
			QMessageBox::warning(NULL, "DesignDialog", "Gripper uses base geometry from STL!");
		}
		
		// update results area
		ui.experimentsEdit->setText(QString::number(_gripper->getQuality().nOfExperiments));
		ui.successesEdit->setText(QString::number(_gripper->getQuality().nOfSuccesses));
		ui.samplesEdit->setText(QString::number(_gripper->getQuality().nOfSamples));
		//_shapeEdit->setText(QString::number(_gripper->getQuality()->shape));
		ui.coverageEdit->setText(QString::number(_gripper->getQuality().coverage));
		ui.successEdit->setText(QString::number(_gripper->getQuality().success));
		ui.wrenchEdit->setText(QString::number(_gripper->getQuality().wrench));
		ui.topWrenchEdit->setText(QString::number(_gripper->getQuality().topwrench));
		ui.qualityEdit->setText(QString::number(_gripper->getQuality().quality));
		
		// update general parameters area
		ui.nameEdit->setText(QString::fromStdString(_gripper->getName()));
		//ui.tcpEdit->setText(QString::number(_gripper->getJawParameters()[1]-_gripper->getTCP().P()[2]));
		ui.tcpEdit->setText(QString::number(_gripper->getTCP().P()[2]));
		ui.forceEdit->setText(QString::number(_gripper->getForce()));
		ui.jawdistEdit->setText(QString::number(_gripper->getJawdist()));
		ui.openingEdit->setText(QString::number(_gripper->getOpening()));
	}
}
