#include "TaskDialog.hpp"

#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <QtGui>
#include <QRadioButton>
#include <QComboBox>



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;



TaskDialog::TaskDialog(QWidget* parent, TaskDescription::Ptr td, std::string wd) :
	QDialog(parent),
	_td(td),
	_changed(false),
	_wd(wd)
{
	createGUI();
	updateGUI();
}



void TaskDialog::guiEvent()
{
	QObject* obj = sender();
	
	if (obj == _okButton) {
		updateTaskDescription();
		close();
	}
	
	else if (obj == _applyButton) {
		updateTaskDescription();
	}
}



void TaskDialog::guiEvent(int index)
{
	QObject* obj = sender();
	
	if (obj == _gripperCombo) {
		// we have to find proper frames and update GUI
		
	}
}



void TaskDialog::createGUI()
{
	int row = 0;
	
	/* create dialog elements */
	_okButton = new QPushButton("Ok");
	_applyButton = new QPushButton("Apply");
	_cancelButton = new QPushButton("Cancel");
	
	QGroupBox* targetBox = new QGroupBox("Target");
	_targetCombo = new QComboBox;
	
	QGroupBox* gripperBox = new QGroupBox("Gripper");
	QLabel* gripperLabel = new QLabel("Device");
	_gripperCombo = new QComboBox;
	
	QGroupBox* resultBox = new QGroupBox("Evaluation parameters");
	QLabel* baseLabel = new QLabel("Baseline");
	QLabel* weightLabel = new QLabel("Weights");
	QLabel* shapeLabel = new QLabel("Shape");
	QLabel* coverageLabel = new QLabel("Coverage");
	QLabel* successLabel = new QLabel("Success");
	QLabel* wrenchLabel = new QLabel("Wrench");
	_baseShapeEdit = new QLineEdit("0"); _baseShapeEdit->setDisabled(true);
	_baseCoverageEdit = new QLineEdit("1");
	_baseSuccessEdit = new QLineEdit("1");
	_baseWrenchEdit = new QLineEdit("1");
	_weightShapeEdit = new QLineEdit("0"); _weightShapeEdit->setDisabled(true);
	_weightCoverageEdit = new QLineEdit("1");
	_weightSuccessEdit = new QLineEdit("1");
	_weightWrenchEdit = new QLineEdit("1");
	
	/* connect elements */
	connect(_okButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_applyButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(_cancelButton, SIGNAL(clicked()), this, SLOT(close()));
	
	/* create layouts */
	QGridLayout* layout = new QGridLayout;
	QHBoxLayout* buttonLayout = new QHBoxLayout;
	QGridLayout* targetLayout = new QGridLayout;
	QGridLayout* gripperLayout = new QGridLayout;
	QGridLayout* resultLayout = new QGridLayout;
	
	/* add widgets to proper layouts */
	buttonLayout->addWidget(_okButton);
	buttonLayout->addWidget(_applyButton);
	buttonLayout->addWidget(_cancelButton);
	buttonLayout->addStretch(1);
	
	row = 0;
	targetLayout->addWidget(_targetCombo, row++, 0);
	
	row = 0;
	gripperLayout->addWidget(gripperLabel, row, 0);
	gripperLayout->addWidget(_gripperCombo, row++, 1);
	
	row = 0;
	resultLayout->addWidget(shapeLabel, row, 1);
	resultLayout->addWidget(coverageLabel, row, 2);
	resultLayout->addWidget(successLabel, row, 3);
	resultLayout->addWidget(wrenchLabel, row++, 4);
	resultLayout->addWidget(baseLabel, row, 0);
	resultLayout->addWidget(_baseShapeEdit, row, 1);
	resultLayout->addWidget(_baseCoverageEdit, row, 2);
	resultLayout->addWidget(_baseSuccessEdit, row, 3);
	resultLayout->addWidget(_baseWrenchEdit, row++, 4);
	resultLayout->addWidget(weightLabel, row, 0);
	resultLayout->addWidget(_weightShapeEdit, row, 1);
	resultLayout->addWidget(_weightCoverageEdit, row, 2);
	resultLayout->addWidget(_weightSuccessEdit, row, 3);
	resultLayout->addWidget(_weightWrenchEdit, row++, 4);
	
	row = 0;
	layout->addWidget(targetBox, row++, 0);
	layout->addWidget(gripperBox, row++, 0);
	layout->addWidget(resultBox, row++, 0, 1, 2);
	layout->addLayout(buttonLayout, row++, 0);
	
	/* set layouts */
	targetBox->setLayout(targetLayout);
	gripperBox->setLayout(gripperLayout);
	resultBox->setLayout(resultLayout);
	setLayout(layout);
}



void TaskDialog::updateGUI()
{
	// fill the target box
	_targetCombo->clear();
	BOOST_FOREACH (Object::Ptr obj, _td->getWorkCell()->getObjects()) {
		_targetCombo->addItem(QString::fromStdString(obj->getName()));
	}
	_targetCombo->setCurrentIndex(_targetCombo->findText(QString::fromStdString(_td->getTargetObject()->getName())));
	
	// fill the gripper box
	_gripperCombo->clear();
	BOOST_FOREACH (Device::Ptr dev, _td->getWorkCell()->getDevices()) {
		_gripperCombo->addItem(QString::fromStdString(dev->getName()));
	}
	_gripperCombo->setCurrentIndex(_gripperCombo->findText(QString::fromStdString(_td->getGripperDevice()->getName())));
	
	// update the result section
	_baseShapeEdit->setText(QString::number(_td->getBaseline().shape));
	_baseCoverageEdit->setText(QString::number(_td->getBaseline().coverage));
	_baseSuccessEdit->setText(QString::number(_td->getBaseline().success));
	_baseWrenchEdit->setText(QString::number(_td->getBaseline().wrench));
	_weightShapeEdit->setText(QString::number(_td->getWeights().shape));
	_weightCoverageEdit->setText(QString::number(_td->getWeights().coverage));
	_weightSuccessEdit->setText(QString::number(_td->getWeights().success));
	_weightWrenchEdit->setText(QString::number(_td->getWeights().wrench));
}



void TaskDialog::updateTaskDescription()
{
	// set new target
	_td->setTarget(_targetCombo->currentText().toStdString());
	
	// update baseline & weights
	TaskDescription::Qualities& base = _td->getBaseline();
	base.shape = _baseShapeEdit->text().toDouble();
	base.coverage = _baseCoverageEdit->text().toDouble();
	base.success = _baseSuccessEdit->text().toDouble();
	base.wrench = _baseWrenchEdit->text().toDouble();
	
	TaskDescription::Qualities& w = _td->getWeights();
	w.shape = _weightShapeEdit->text().toDouble();
	w.coverage = _weightCoverageEdit->text().toDouble();
	w.success = _weightSuccessEdit->text().toDouble();
	w.wrench = _weightWrenchEdit->text().toDouble();
}
