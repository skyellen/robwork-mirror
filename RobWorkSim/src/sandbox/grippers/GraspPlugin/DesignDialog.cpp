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
	_wd(wd)
{
	if (!_gripper) _gripper = ownedPtr(new Gripper);
	
	_createGUI();
	_updateGUI();
}



void DesignDialog::guiEvent()
{
	QObject* obj = sender();
	
	if (obj == _okButton) {
		//_gripper = ownedPtr(new Gripper);
		_updateGripper();
		//cout << "ok pressed" << endl;
		close();
	}
	
	else if (obj == _applyButton) {
		//_gripper = ownedPtr(new Gripper);
		_updateGripper();
	}
	
	/*else if (obj == _loadButton) {
		QString filename = QFileDialog::getOpenFileName(this,
			"Open file", QString::fromStdString(_wd), tr("Gripper files (*.xml)"));
			
		if (filename.isEmpty()) return;
			
		_wd = QFileInfo(filename).path().toStdString();
			
		_gripper = GripperXMLLoader::load(filename.toStdString());
		_updateGUI();
		
		//cout << *_gripper->getQuality() << endl;
	}
	
	else if (obj == _saveButton) {
		QString filename = QFileDialog::getSaveFileName(this,
			"Save file", QString::fromStdString(_wd), tr("Gripper files (*.xml)"));
			
		if (filename.isEmpty()) return;
			
		_wd = QFileInfo(filename).path().toStdString();
		string name = QFileInfo(filename).fileName().toStdString();
			
		GripperXMLLoader::save(_gripper, _wd, name);
	}*/
	
	else if (obj == _defaultButton) {
		_gripper = ownedPtr(new Gripper);
		_updateGUI();
	}
	
	else if (obj == _prismaticButton) {
		_gripper->getGeometry()->setCutType(JawPrimitive::Prismatic);
		_updateGUI();
	}
	
	else if (obj == _cylindricalButton) {
		_gripper->getGeometry()->setCutType(JawPrimitive::Cylindrical);
		_updateGUI();
	}
	//_updateGUI();
}



void DesignDialog::_updateGripper()
{
	if (_gripper) {
		JawPrimitive::Ptr geo = ownedPtr(new JawPrimitive);
		_gripper->setGeometry(geo);
		//dynamic_cast<PrismaticCutJaw*>(_gripper->getGeometry().get());
		
		_gripper->setName(_nameEdit->text().toStdString());
		
		if (_cylindricalButton->isChecked()) {
			geo->setCutType(JawPrimitive::Cylindrical);
		} else {
			geo->setCutType(JawPrimitive::Prismatic);
		}
		
		geo->setLength(_lengthEdit->text().toDouble());
		geo->setWidth(_widthEdit->text().toDouble());
		geo->setDepth(_depthEdit->text().toDouble());
		geo->setChamferDepth(_chfDepthEdit->text().toDouble());
		geo->setChamferAngle(Deg2Rad*_chfAngleEdit->text().toDouble());
		geo->setCutPosition(_tcpPosEdit->text().toDouble());
		geo->setCutDepth(_cutDepthEdit->text().toDouble());
		
		if (_cylindricalButton->isChecked()) {
			geo->setCutRadius(_cutAngleEdit->text().toDouble());
		} else {
			geo->setCutAngle(Deg2Rad*_cutAngleEdit->text().toDouble());
		}
		
		_gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, _tcpPosEdit->text().toDouble())));
		
		_gripper->setForce(_forceEdit->text().toDouble());
		_gripper->setJawdist(_jawdistEdit->text().toDouble());
		_gripper->setOpening(_openingEdit->text().toDouble());
		
		//_gripper->updateGripper();
	}
}



void DesignDialog::_createGUI()
{
	// make all the elments
	QLabel* lengthLabel = new QLabel("Length");
	_lengthEdit = new QLineEdit("0.1");
	QLabel* widthLabel = new QLabel("Width");
	_widthEdit = new QLineEdit("0.025");
	QLabel* depthLabel = new QLabel("Depth");
	_depthEdit = new QLineEdit("0.02");
	QLabel* chfDepthLabel = new QLabel("Chf. depth");
	_chfDepthEdit = new QLineEdit("0");
	QLabel* chfAngleLabel = new QLabel("Chf. angle");
	_chfAngleEdit = new QLineEdit("0");
	QLabel* cutDepthLabel = new QLabel("Cut depth");
	_cutDepthEdit = new QLineEdit("0");
	QLabel* cutAngleLabel = new QLabel("Cut angle/radius");
	_cutAngleEdit = new QLineEdit("90");
	QLabel* tcpPosLabel = new QLabel("TCP off.");
	_tcpPosEdit = new QLineEdit("0.05");
	QLabel* forceLabel = new QLabel("Force");
	_forceEdit = new QLineEdit("50");
	QLabel* jawdistLabel = new QLabel("Jaw dist.");
	_jawdistEdit = new QLineEdit("0");
	QLabel* openingLabel = new QLabel("Opening");
	_openingEdit = new QLineEdit("0.05");
	QLabel* nameLabel = new QLabel("Name");
	_nameEdit = new QLineEdit("gripper");
	
	_loadButton = new QPushButton("Load");
	_saveButton = new QPushButton("Save");
	_okButton = new QPushButton("OK");
	_applyButton = new QPushButton("Apply");
	_cancelButton = new QPushButton("Cancel");
	_defaultButton = new QPushButton("Default");
	
	QGroupBox* typeBox = new QGroupBox("Cutout type");
	_prismaticButton = new QRadioButton("Prismatic");
	_cylindricalButton = new QRadioButton("Cylindrical");
	_prismaticButton->setChecked(true);
	
	// connect stuff
	connect(_okButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_applyButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	//connect(_loadButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	//connect(_saveButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_cancelButton, SIGNAL(clicked()), this, SLOT(close()));
	connect(_defaultButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_prismaticButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_cylindricalButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	
	// setup layouts
	QHBoxLayout* layout = new QHBoxLayout;
	QGridLayout* left = new QGridLayout;
	QVBoxLayout* right = new QVBoxLayout;
	QHBoxLayout* type = new QHBoxLayout;
	
	type->addWidget(_prismaticButton);
	type->addWidget(_cylindricalButton);
	type->addStretch(1);
	typeBox->setLayout(type);	
	
	int row = 0;
	left->addWidget(lengthLabel, row, 0);
	left->addWidget(_lengthEdit, row, 1);
	left->addWidget(cutAngleLabel, row, 2);
	left->addWidget(_cutAngleEdit, row++, 3);
	
	left->addWidget(widthLabel, row, 0);
	left->addWidget(_widthEdit, row, 1);
	left->addWidget(tcpPosLabel, row, 2);
	left->addWidget(_tcpPosEdit, row++, 3);
	
	left->addWidget(depthLabel, row, 0);
	left->addWidget(_depthEdit, row, 1);
	left->addWidget(forceLabel, row, 2);
	left->addWidget(_forceEdit, row++, 3);
	
	left->addWidget(chfDepthLabel, row, 0);
	left->addWidget(_chfDepthEdit, row, 1);
	left->addWidget(jawdistLabel, row, 2);
	left->addWidget(_jawdistEdit, row++, 3);
	
	left->addWidget(chfAngleLabel, row, 0);
	left->addWidget(_chfAngleEdit, row, 1);
	left->addWidget(openingLabel, row, 2);
	left->addWidget(_openingEdit, row++, 3);
	
	left->addWidget(cutDepthLabel, row, 0);
	left->addWidget(_cutDepthEdit, row, 1);
	left->addWidget(nameLabel, row, 2);
	left->addWidget(_nameEdit, row++, 3);
	
	left->addWidget(typeBox, row++, 0, 1, 4);
	
	left->addWidget(_defaultButton, row, 0);
	//left->addWidget(_loadButton, row, 2);
	//left->addWidget(_saveButton, row++, 3);
	
	right->addWidget(_okButton);
	right->addWidget(_applyButton);
	right->addWidget(_cancelButton);
	right->addStretch(1);
	
	layout->addLayout(left);
	layout->addLayout(right);
	setLayout(layout);
	
	_updateGUI();
}



void DesignDialog::_updateGUI()
{
	if (_gripper) {
		//PrismaticCutJaw::Ptr geo = dynamic_cast<PrismaticCutJaw*>(_gripper->getGeometry().get());
		JawPrimitive::Ptr geo = _gripper->getGeometry();
		
		_nameEdit->setText(QString::fromStdString(_gripper->getName()));
		_lengthEdit->setText(QString::number(geo->getLength()));
		_widthEdit->setText(QString::number(geo->getWidth()));
		_depthEdit->setText(QString::number(geo->getDepth()));
		_chfDepthEdit->setText(QString::number(geo->getChamferDepth()));
		_chfAngleEdit->setText(QString::number(Rad2Deg*geo->getChamferAngle()));
		_cutDepthEdit->setText(QString::number(geo->getCutDepth()));
		
		if (geo->getCutType() == JawPrimitive::Cylindrical) {
			_cutAngleEdit->setText(QString::number(geo->getCutRadius()));
			_cylindricalButton->setChecked(true);
		} else {
			_cutAngleEdit->setText(QString::number(Rad2Deg*geo->getCutAngle()));
			_prismaticButton->setChecked(true);
		}
		
		_tcpPosEdit->setText(QString::number(geo->getCutPosition()));
		_forceEdit->setText(QString::number(_gripper->getForce()));
		_jawdistEdit->setText(QString::number(_gripper->getJawdist()));
		_openingEdit->setText(QString::number(_gripper->getOpening()));
	}
}
