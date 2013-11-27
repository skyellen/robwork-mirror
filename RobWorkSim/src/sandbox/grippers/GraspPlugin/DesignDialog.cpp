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
	
	_createGUI();
	_updateGUI();
}



void DesignDialog::guiEvent()
{
	QObject* obj = sender();
	
	if (obj == _okButton) {
		_gripper = ownedPtr(new Gripper);
		_updateGripper();
		close();
	}
	
	else if (obj == _applyButton) {
		_updateGripper();
	}
	
	else if (obj == _defaultButton) {
		_gripper = ownedPtr(new Gripper);
		_updateGUI();
	}
	
	else if (obj == _prismaticButton) {
		_updateGripper();
		_updateGUI();
	}
	
	else if (obj == _cylindricalButton) {
		_updateGripper();
		_updateGUI();
	}
	
	_changed = true;
}



void DesignDialog::_updateGripper()
{
	if (_gripper) {
		// update jaw geometry
		Q jawParams(10);
		jawParams(0) = (_cylindricalButton->isChecked() ? 1 : 0);
		jawParams(1) = _lengthEdit->text().toDouble();
		jawParams(2) = _widthEdit->text().toDouble();
		jawParams(3) = _depthEdit->text().toDouble();
		jawParams(4) = _chfDepthEdit->text().toDouble();
		jawParams(5) = Deg2Rad*_chfAngleEdit->text().toDouble();
		jawParams(6) = _tcpPosEdit->text().toDouble();
		jawParams(7) = _cutDepthEdit->text().toDouble();
		jawParams(8) = Deg2Rad*_cutAngleEdit->text().toDouble();
		jawParams(9) = _cutRadiusEdit->text().toDouble();
		_gripper->setJawGeometry(jawParams);
		
		// update base geometry
		Q baseParams(3);
		baseParams(0) = _basedxEdit->text().toDouble();
		baseParams(1) = _basedyEdit->text().toDouble();
		baseParams(2) = _basedzEdit->text().toDouble();
		_gripper->setBaseGeometry(baseParams);
		
		// update results
		_gripper->getQuality()->quality = _qualityEdit->text().toDouble();
		
		// update general parameters
		_gripper->setName(_nameEdit->text().toStdString());
		_gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, _tcpPosEdit->text().toDouble())));
		_gripper->setForce(_forceEdit->text().toDouble());
		_gripper->setJawdist(_jawdistEdit->text().toDouble());
		_gripper->setOpening(_openingEdit->text().toDouble());
	}
}



void DesignDialog::_createGUI()
{
	int row = 0;
	
	// make all the elements
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
	QLabel* cutAngleLabel = new QLabel("Cut angle");
	_cutAngleEdit = new QLineEdit("90");
	QLabel* cutRadiusLabel = new QLabel("Cut radius");
	_cutRadiusEdit = new QLineEdit("0");
	QLabel* tcpPosLabel = new QLabel("TCP off.");
	_tcpPosEdit = new QLineEdit("0.075");
	QLabel* forceLabel = new QLabel("Force");
	_forceEdit = new QLineEdit("50");
	QLabel* jawdistLabel = new QLabel("Jaw dist.");
	_jawdistEdit = new QLineEdit("0");
	QLabel* openingLabel = new QLabel("Opening");
	_openingEdit = new QLineEdit("0.05");
	QLabel* nameLabel = new QLabel("Name");
	_nameEdit = new QLineEdit("gripper");
	QLabel* basedxLabel = new QLabel("dx");
	_basedxEdit = new QLineEdit("0.15");
	QLabel* basedyLabel = new QLabel("dy");
	_basedyEdit = new QLineEdit("0.1");
	QLabel* basedzLabel = new QLabel("dz");
	_basedzEdit = new QLineEdit("0.05");
	QLabel* experimentsLabel = new QLabel("N. of exp.");
	_experimentsEdit = new QLineEdit("0");
	_experimentsEdit->setDisabled(true);
	QLabel* successesLabel = new QLabel("N. of successes");
	_successesEdit = new QLineEdit("0");
	_successesEdit->setDisabled(true);
	QLabel* samplesLabel = new QLabel("N. of samples");
	_samplesEdit = new QLineEdit("0");
	_samplesEdit->setDisabled(true);
	QLabel* shapeLabel = new QLabel("Shape");
	_shapeEdit = new QLineEdit("0");
	_shapeEdit->setDisabled(true);
	QLabel* coverageLabel = new QLabel("Coverage");
	_coverageEdit = new QLineEdit("0");
	_coverageEdit->setDisabled(true);
	QLabel* successLabel = new QLabel("Success Ratio");
	_successEdit = new QLineEdit("0");
	_successEdit->setDisabled(true);
	QLabel* wrenchLabel = new QLabel("Wrench");
	_wrenchEdit = new QLineEdit("0");
	_wrenchEdit->setDisabled(true);
	QLabel* qualityLabel = new QLabel("QUALITY");
	_qualityEdit = new QLineEdit("0");
	
	_okButton = new QPushButton("OK");
	_applyButton = new QPushButton("Apply");
	_cancelButton = new QPushButton("Cancel");
	_defaultButton = new QPushButton("Default");
	
	QGroupBox* typeBox = new QGroupBox("Cutout type");
	QGroupBox* generalBox = new QGroupBox("General parameters");
	_prismaticButton = new QRadioButton("Prismatic");
	_cylindricalButton = new QRadioButton("Cylindrical");
	_prismaticButton->setChecked(true);
	
	// connect stuff
	connect(_okButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_applyButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_cancelButton, SIGNAL(clicked()), this, SLOT(close()));
	connect(_defaultButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_prismaticButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_cylindricalButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_nameEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_lengthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_widthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_depthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_chfDepthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_chfAngleEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_cutDepthEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_cutAngleEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_cutRadiusEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_tcpPosEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_jawdistEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_openingEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_basedxEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_basedyEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	connect(_basedzEdit, SIGNAL(editingFinished()), this, SLOT(guiEvent()));
	
	// setup layouts
	QHBoxLayout* layout = new QHBoxLayout;
	QVBoxLayout* left = new QVBoxLayout;
	QGridLayout* jawlayout = new QGridLayout;
	QGridLayout* baselayout = new QGridLayout;
	QGridLayout* reslayout = new QGridLayout;
	QVBoxLayout* btnlayout = new QVBoxLayout;
	QHBoxLayout* typelayout = new QHBoxLayout;
	QGridLayout* generallayout = new QGridLayout;
	
	typelayout->addWidget(_prismaticButton);
	typelayout->addWidget(_cylindricalButton);
	typelayout->addStretch(1);
	typeBox->setLayout(typelayout);
	
	row = 0;
	generallayout->addWidget(nameLabel, row, 0);
	generallayout->addWidget(_nameEdit, row++, 1, 1, 3);
	generallayout->addWidget(jawdistLabel, row, 0);
	generallayout->addWidget(_jawdistEdit, row, 1);
	generallayout->addWidget(tcpPosLabel, row, 2);
	generallayout->addWidget(_tcpPosEdit, row++, 3);
	generallayout->addWidget(openingLabel, row, 0);
	generallayout->addWidget(_openingEdit, row, 1);
	generallayout->addWidget(forceLabel, row, 2);
	generallayout->addWidget(_forceEdit, row++, 3);
	generalBox->setLayout(generallayout);
	
	row = 0;
	jawlayout->addWidget(lengthLabel, row, 0);
	jawlayout->addWidget(_lengthEdit, row, 1);
	jawlayout->addWidget(chfAngleLabel, row, 2);
	jawlayout->addWidget(_chfAngleEdit, row++, 3);
	jawlayout->addWidget(widthLabel, row, 0);
	jawlayout->addWidget(_widthEdit, row, 1);
	jawlayout->addWidget(cutDepthLabel, row, 2);
	jawlayout->addWidget(_cutDepthEdit, row++, 3);
	jawlayout->addWidget(depthLabel, row, 0);
	jawlayout->addWidget(_depthEdit, row, 1);
	jawlayout->addWidget(cutAngleLabel, row, 2);
	jawlayout->addWidget(_cutAngleEdit, row++, 3);
	jawlayout->addWidget(chfDepthLabel, row, 0);
	jawlayout->addWidget(_chfDepthEdit, row, 1);
	jawlayout->addWidget(cutRadiusLabel, row, 2);
	jawlayout->addWidget(_cutRadiusEdit, row++, 3);
	jawlayout->addWidget(typeBox, row++, 0, 1, 4);
	
	row = 0;
	baselayout->addWidget(basedxLabel, row, 0);
	baselayout->addWidget(_basedxEdit, row++, 1);
	baselayout->addWidget(basedyLabel, row, 0);
	baselayout->addWidget(_basedyEdit, row++, 1);
	baselayout->addWidget(basedzLabel, row, 0);
	baselayout->addWidget(_basedzEdit, row++, 1);
	
	row = 0;
	reslayout->addWidget(experimentsLabel, row, 0);
	reslayout->addWidget(_experimentsEdit, row, 1);
	reslayout->addWidget(coverageLabel, row, 2);
	reslayout->addWidget(_coverageEdit, row++, 3);
	reslayout->addWidget(successesLabel, row, 0);
	reslayout->addWidget(_successesEdit, row, 1);
	reslayout->addWidget(successLabel, row, 2);
	reslayout->addWidget(_successEdit, row++, 3);
	reslayout->addWidget(samplesLabel, row, 0);
	reslayout->addWidget(_samplesEdit, row, 1);
	reslayout->addWidget(wrenchLabel, row, 2);
	reslayout->addWidget(_wrenchEdit, row++, 3);
	//reslayout->addWidget(shapeLabel, row, 0);
	//reslayout->addWidget(_shapeEdit, row, 1);
	reslayout->addWidget(qualityLabel, row, 2);
	reslayout->addWidget(_qualityEdit, row++, 3);
	
	btnlayout->addWidget(_okButton);
	btnlayout->addWidget(_applyButton);
	btnlayout->addWidget(_cancelButton);
	btnlayout->addStretch(1);
	btnlayout->addWidget(_defaultButton);
	
	// construct tab widget
	QTabWidget* tabWidget = new QTabWidget;
	
	QWidget* jawTab = new QWidget;
	jawTab->setLayout(jawlayout);
	
	QWidget* baseTab = new QWidget;
	baseTab->setLayout(baselayout);
	
	QWidget* resTab = new QWidget;
	resTab->setLayout(reslayout);
	
	tabWidget->addTab(jawTab, "Jaws");
	tabWidget->addTab(baseTab, "Base");
	tabWidget->addTab(resTab, "Results");
	
	left->addWidget(tabWidget);
	left->addWidget(generalBox);
	
	// add layouts to dialog
	layout->addLayout(left);
	layout->addLayout(btnlayout);
	setLayout(layout);
	
	_updateGUI();
}



void DesignDialog::_updateGUI()
{
	if (_gripper) {		
		// update jaw geometry area
		Q jawParams = _gripper->getJawParameters();
		if (jawParams.size() == 10) {
			cout << jawParams(0) << endl;
			if (jawParams(0) == 0) {
				_prismaticButton->setChecked(true);
			} else {
				_cylindricalButton->setChecked(true);
			}
			_lengthEdit->setText(QString::number(jawParams(1)));
			_widthEdit->setText(QString::number(jawParams(2)));
			_depthEdit->setText(QString::number(jawParams(3)));
			_chfDepthEdit->setText(QString::number(jawParams(4)));
			_chfAngleEdit->setText(QString::number(Rad2Deg*jawParams(5)));
			_cutDepthEdit->setText(QString::number(jawParams(7)));
			_cutAngleEdit->setText(QString::number(Rad2Deg*jawParams(8)));
			_cutRadiusEdit->setText(QString::number(jawParams(9)));
		} else {
			QMessageBox::warning(NULL, "DesignDialog", "Gripper uses jaw geometry from STL!");
		}
		
		// update base geometry area
		Q baseParams = _gripper->getBaseParameters();
		if (baseParams.size() == 3) {
			_basedxEdit->setText(QString::number(baseParams(0)));
			_basedyEdit->setText(QString::number(baseParams(1)));
			_basedzEdit->setText(QString::number(baseParams(2)));
		} else {
			QMessageBox::warning(NULL, "DesignDialog", "Gripper uses base geometry from STL!");
		}
		
		// update results area
		_experimentsEdit->setText(QString::number(_gripper->getQuality()->nOfExperiments));
		_successesEdit->setText(QString::number(_gripper->getQuality()->nOfSuccesses));
		_samplesEdit->setText(QString::number(_gripper->getQuality()->nOfSamples));
		//_shapeEdit->setText(QString::number(_gripper->getQuality()->shape));
		_coverageEdit->setText(QString::number(_gripper->getQuality()->coverage));
		_successEdit->setText(QString::number(_gripper->getQuality()->success));
		_wrenchEdit->setText(QString::number(_gripper->getQuality()->wrench));
		_qualityEdit->setText(QString::number(_gripper->getQuality()->quality));
		
		// update general parameters area
		_nameEdit->setText(QString::fromStdString(_gripper->getName()));
		_tcpPosEdit->setText(QString::number(_gripper->getTCP().P()[2]));
		_forceEdit->setText(QString::number(_gripper->getForce()));
		_jawdistEdit->setText(QString::number(_gripper->getJawdist()));
		_openingEdit->setText(QString::number(_gripper->getOpening()));
	}
}
