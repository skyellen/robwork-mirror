/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "TaskSetupDialog.hpp"

#include "ui_TaskSetupDialog.h"

#include <rw/graphics/GroupNode.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Object.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/sensor/FTSensor.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rwlibs/assembly/AssemblyControlResponse.hpp>
#include <rwlibs/assembly/AssemblyControlStrategy.hpp>
#include <rwlibs/assembly/AssemblyParameterization.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/AssemblyTask.hpp>
#include <rwlibs/opengl/DrawableFactory.hpp>
#include <rws/HelpAssistant.hpp>
#include <rws/SceneOpenGLViewer.hpp>

#include <QDateTime>
#include <QDesktopServices>
#include <QFileDialog>
#include <QTimer>

#include <iostream>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using rw::sensor::FTSensor;
using namespace rwlibs::assembly;
using rwlibs::opengl::DrawableFactory;
using rws::SceneOpenGLViewer;
using namespace rwslibs;

#define DT 100 // in milliseconds

namespace {
void setupViewWidget(SceneOpenGLViewer* widget) {
	// Use view take up the maximum possible size
	widget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

	// Make world node and hide pivot point
	const GroupNode::Ptr world = ownedPtr(new GroupNode("World"));
	widget->setWorldNode(world);
	widget->getPivotDrawable()->setVisible(false);

	// Zoom in
	Transform3D<> Tcam = widget->getCameraController()->getTransform();
	Tcam.P() /= 40.;
	widget->getCameraController()->setTransform(Tcam);

	// Do not draw background
    const Property<bool>::Ptr bgProp = widget->getPropertyMap().findProperty<bool>("DrawBackGround");
    bgProp->setValue(false);
    bgProp->changedEvent().fire(bgProp.get());

    // Update
    widget->updateView();
}

class DummyFTSensor: public FTSensor {
public:
	DummyFTSensor(): FTSensor("DummyFTSensor","") {}
	~DummyFTSensor() {}
	void acquire() {}
	double getMaxForce() { return 100; }
	double getMaxTorque() { return 100; }
	Vector3D<> getForce() { return Vector3D<>::zero(); }
	Vector3D<> getTorque() { return Vector3D<>::zero(); }
};
}

struct TaskSetupDialog::InsertionSimulation {
	InsertionSimulation(): time(0), _dummyFTSensor(new DummyFTSensor()) {}
	~InsertionSimulation() { delete _dummyFTSensor; }
	AssemblyParameterization::Ptr parameters;
	AssemblyControlStrategy::ControlState::Ptr cstate;
	AssemblyControlResponse::Ptr response;
	double time;
	rw::sensor::FTSensor* const _dummyFTSensor;

	void reset() {
		cstate = NULL;
		response = NULL;
		time = 0;
	}
};

TaskSetupDialog::TaskSetupDialog(QWidget* parent, rw::common::Ptr<const WorkCell> wc, AssemblyControlStrategy::Ptr strategy):
	QDialog(parent),
	_ui(new Ui::TaskSetupDialog()),
	_wc(wc),
	_strategy(strategy),
	_timer(new QTimer()),
	_simulation(new InsertionSimulation()),
	_help(new HelpAssistant())
{
	_ui->setupUi(this);
	setWindowTitle("Assembly Task Configuration - " + QString::fromStdString(strategy->getID()));
	//_ui->docLink->setText("<a href=\"http://www.robwork.dk/apidoc/nightly/rw/classrwlibs_1_1assembly_1_1AssemblyParameterization.html\">Parameterization documentation</a> - <a href=\"http://www.robwork.dk/apidoc/nightly/rw/classrwlibs_1_1assembly_1_1AssemblyControlStrategy.html\">Strategy documentation</a>");
	//_ui->docLink->setOpenExternalLinks(true);

	setupViewWidget(_ui->approach);
	setupViewWidget(_ui->insertion);
	setupViewWidget(_ui->target);

	const PropertyMap::Ptr emptyMap = ownedPtr(new PropertyMap());
	const AssemblyParameterization::Ptr parameters = strategy->createParameterization(emptyMap);
	_propertyMap = parameters->toPropertyMap();
	_ui->parameters->setPropertyMap(_propertyMap);
	connect(_ui->parameters, SIGNAL(propertyChanged(const std::string&)), this, SLOT(updatePositions()));

	if (!wc.isNull()) {
		const std::vector<Object::Ptr> objects = wc->getObjects();
		_ui->male->addItem("");
		_ui->female->addItem("");
		for (std::size_t i = 0; i < objects.size(); i++) {
			_ui->male->addItem(QString::fromStdString(objects[i]->getName()));
			_ui->female->addItem(QString::fromStdString(objects[i]->getName()));
		}
		connect(_ui->female, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setFemaleObject(const QString&)));
		connect(_ui->male, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setMaleObject(const QString&)));
	}
	connect(_ui->maleTCP, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(updateViews()));
	connect(_ui->femaleTCP, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(updateViews()));
	connect(_ui->x, SIGNAL(valueChanged(double)), this, SLOT(updatePositions()));
	connect(_ui->y, SIGNAL(valueChanged(double)), this, SLOT(updatePositions()));
	connect(_ui->z, SIGNAL(valueChanged(double)), this, SLOT(updatePositions()));
	connect(_ui->R, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	connect(_ui->P, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	connect(_ui->Y, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	connect(_ui->fixGroup, SIGNAL(buttonClicked(int)), this, SLOT(updatePositions()));
	connect(_ui->Rdial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	connect(_ui->Pdial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	connect(_ui->Ydial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));

	connect(_ui->loadBtn, SIGNAL(pressed()), this, SLOT(loadTask()));
	connect(_ui->saveBtn, SIGNAL(pressed()), this, SLOT(saveTask()));
	connect(_ui->docStrat, SIGNAL(pressed()), this, SLOT(showHelpStrategy()));
	connect(_ui->docPar, SIGNAL(pressed()), this, SLOT(showHelpParameterization()));

	connect(_timer, SIGNAL(timeout()), this, SLOT(stepInsertion()));
}

TaskSetupDialog::~TaskSetupDialog() {
	delete _timer;
	delete _simulation;
	delete _help;
	delete _ui;
}

void TaskSetupDialog::setFemaleObject(const QString &text) {
	// Rebuild the male list with the chosen female object removed
	disconnect(_ui->male, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setMaleObject(const QString&)));
	const QString cur = _ui->male->currentText();
	_ui->male->clear();
	const std::vector<Object::Ptr> objects = _wc->getObjects();
	_ui->male->addItem("");
	for (std::size_t i = 0; i < objects.size(); i++) {
		if (text.size() == 0 || text.toStdString() != objects[i]->getName()) {
			_ui->male->addItem(QString::fromStdString(objects[i]->getName()));
		}
	}
	for (int i = 0; i < _ui->male->count(); i++) {
		if (_ui->male->itemText(i) == cur) {
			_ui->male->setCurrentIndex(i);
			break;
		}
	}
	connect(_ui->male, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setMaleObject(const QString&)));
	// Build TCP list
	_ui->femaleTCP->clear();
	const Object::Ptr object = _wc->findObject(text.toStdString());
	if (!object.isNull()) {
		_ui->femaleTCP->addItem("");
		const Frame* const base = object->getBase();
		Frame::const_iterator_pair children = base->getChildren();
		for (Frame::const_iterator it = children.first; it != children.second; it++) {
			if (!Kinematics::isDAF(&*it) && Kinematics::isFixedFrame(&*it)) {
				_ui->femaleTCP->addItem(QString::fromStdString(it->getName()));
			}
		}
	}
	updateViews();
}

void TaskSetupDialog::setMaleObject(const QString &text) {
	// Rebuild the female list with the chosen male object removed
	disconnect(_ui->female, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setFemaleObject(const QString&)));
	const QString cur = _ui->female->currentText();
	_ui->female->clear();
	const std::vector<Object::Ptr> objects = _wc->getObjects();
	_ui->female->addItem("");
	for (std::size_t i = 0; i < objects.size(); i++) {
		if (text.size() == 0 || text.toStdString() != objects[i]->getName()) {
			_ui->female->addItem(QString::fromStdString(objects[i]->getName()));
		}
	}
	for (int i = 0; i < _ui->female->count(); i++) {
		if (_ui->female->itemText(i) == cur) {
			_ui->female->setCurrentIndex(i);
			break;
		}
	}
	connect(_ui->female, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(setFemaleObject(const QString&)));
	// Build TCP list
	_ui->maleTCP->clear();
	const Object::Ptr object = _wc->findObject(text.toStdString());
	if (!object.isNull()) {
		_ui->maleTCP->addItem("");
		const Frame* const base = object->getBase();
		Frame::const_iterator_pair children = base->getChildren();
		for (Frame::const_iterator it = children.first; it != children.second; it++) {
			if (!Kinematics::isDAF(&*it) && Kinematics::isFixedFrame(&*it)) {
				_ui->maleTCP->addItem(QString::fromStdString(it->getName()));
			}
		}
	}
	updateViews();
}

void TaskSetupDialog::rpyChanged(double value) {
	disconnect(_ui->Rdial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	disconnect(_ui->Pdial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	disconnect(_ui->Ydial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	_ui->Rdial->setValue(static_cast<int>(Math::round(_ui->R->value()*Rad2Deg)));
	_ui->Pdial->setValue(static_cast<int>(Math::round(_ui->P->value()*Rad2Deg)));
	_ui->Ydial->setValue(static_cast<int>(Math::round(_ui->Y->value()*Rad2Deg)));
	connect(_ui->Rdial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	connect(_ui->Pdial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	connect(_ui->Ydial, SIGNAL(valueChanged(int)), this, SLOT(dialChanged(int)));
	updatePositions();
}

void TaskSetupDialog::dialChanged(int value) {
	disconnect(_ui->R, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	disconnect(_ui->P, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	disconnect(_ui->Y, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	_ui->R->setValue(static_cast<double>(_ui->Rdial->value())*Deg2Rad);
	_ui->P->setValue(static_cast<double>(_ui->Pdial->value())*Deg2Rad);
	_ui->Y->setValue(static_cast<double>(_ui->Ydial->value())*Deg2Rad);
	connect(_ui->R, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	connect(_ui->P, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	connect(_ui->Y, SIGNAL(valueChanged(double)), this, SLOT(rpyChanged(double)));
	updatePositions();
}

void TaskSetupDialog::updateViews() {
	_ui->saveBtn->setEnabled(false);
	updateView(_ui->target);
	updateView(_ui->insertion);
	updateView(_ui->approach);
	updatePositions();

	const Object::Ptr male = _wc->findObject(_ui->male->currentText().toStdString());
	const Object::Ptr female = _wc->findObject(_ui->female->currentText().toStdString());
	if (!male.isNull() && !female.isNull())
		_ui->saveBtn->setEnabled(true);
}

void TaskSetupDialog::updatePositions() {
	const Transform3D<> ftmtcpTarget(Vector3D<>(_ui->x->value(),_ui->y->value(),_ui->z->value()),RPY<>(_ui->R->value(),_ui->P->value(),_ui->Y->value()));
	updatePosition(_ui->target,ftmtcpTarget);

	_ui->errorLabel->setText("");
	try {
		const AssemblyParameterization::Ptr parameters = _strategy->createParameterization(_propertyMap);
		RW_ASSERT(!parameters.isNull());
		const Transform3D<> approach = _strategy->getApproach(parameters);
		updatePosition(_ui->approach,approach);

		_simulation->time = 0;
		_simulation->cstate = _strategy->createState();
		_simulation->parameters = parameters;

		_timer->start(DT);
	} catch(const Exception& e) {
		_ui->errorLabel->setText(QString::fromStdString(e.getMessage().getText()));
	}
}

void TaskSetupDialog::stepInsertion() {
	if (!_simulation->response.isNull()) {
		if (_simulation->response->done) {
			_simulation->reset();
			_simulation->cstate = _strategy->createState();
		}
	}
	AssemblyState::Ptr real = ownedPtr(new AssemblyState());
	AssemblyState::Ptr assumed = ownedPtr(new AssemblyState());
	if (!_simulation->response.isNull()) {
		if (_simulation->response->type == AssemblyControlResponse::POSITION_TRAJECTORY) {
			real->femaleTmale = _simulation->response->worldTendTrajectory->x(_simulation->time);
		}
	}
	if (_simulation->cstate.isNull())
		_simulation->cstate = _strategy->createState();
	State state;
	AssemblyControlResponse::Ptr response = NULL;
	try {
		response = _strategy->update(_simulation->parameters,real,assumed,_simulation->cstate,state,_simulation->_dummyFTSensor,_simulation->time);
	} catch (const Exception& e) {
		_timer->stop();
		_ui->errorLabel->setText(QString::fromStdString(e.getMessage().getText()));
		return;
	}
	if (response != NULL) {
		_simulation->response = response;
	}
	if (!_simulation->response.isNull()) {
		if (_simulation->response->type == AssemblyControlResponse::POSITION_TRAJECTORY) {
			updatePosition(_ui->insertion,real->femaleTmale);
		}
	}
	_simulation->time += static_cast<double>(DT)/1000.;
}

void TaskSetupDialog::loadTask() {
    //std::string prevDir = settings().get<std::string>("LastDir","");
    //std::string filename = getRobWorkStudio()->getPropertyMap().get<PropertyMap>("cmdline").get<std::string>("AssemblyTask", "");
    //std::string taskFile = filename;

    //QString selectedFilter;
    //if(filename=="")
    //	filename = prevDir;
    //const QString dir(filename.c_str());

    const QString file = QFileDialog::getOpenFileName(
    		this,
    		"Open Assembly Task file", // Title
    		QString::fromStdString(_lastDir), // Directory
    		" \nRW Assembly Task files ( *.tasks.assembly.xml )"
    		" \nRW Assembly files ( *.assembly.xml )"
    		"All supported ( *.xml )"
    		" \n All ( *.* )"/*,
    		&selectedFilter*/);
    if(file.size() == 0)
    	return;

    const std::vector<AssemblyTask::Ptr> tasks = AssemblyTask::load(file.toStdString());
    if (tasks.size() == 0) {
        _ui->errorLabel->setText("No tasks defined in file!");
        return;
    }
    const AssemblyTask::Ptr task = tasks[0];

    bool found;
    std::string error;

    // Find female frame
    found = false;
	for (int i = 0; i < _ui->female->count(); i++) {
		if (_ui->female->itemText(i).toStdString() == task->femaleID) {
			_ui->female->setCurrentIndex(i);
			found = true;
			break;
		}
	}
	if (!found) {
		if (_ui->female->count() > 0)
		_ui->female->setCurrentIndex(0);
	    if (error == "")
	    	error = "Could not find the female object specified in task.";
	}

	// Find male frame
	found = false;
	for (int i = 0; i < _ui->male->count(); i++) {
		if (_ui->male->itemText(i).toStdString() == task->maleID) {
			_ui->male->setCurrentIndex(i);
			found = true;
			break;
		}
	}
	if (!found) {
		if (_ui->male->count() > 0)
		_ui->male->setCurrentIndex(0);
	    if (error == "")
	    	error = "Could not find the male object specified in task.";
	}

	// Find female tcp frame
    found = false;
	for (int i = 0; i < _ui->femaleTCP->count(); i++) {
		if (_ui->femaleTCP->itemText(i).toStdString() == task->femaleTCP) {
			_ui->femaleTCP->setCurrentIndex(i);
			found = true;
			break;
		}
	}
	if (!found) {
		if (_ui->femaleTCP->count() > 0)
		_ui->femaleTCP->setCurrentIndex(0);
	    if (error == "")
	    	error = "Could not find the female TCP frame specified in task.";
	}

	// Find male tcp frame
    found = false;
	for (int i = 0; i < _ui->maleTCP->count(); i++) {
		if (_ui->maleTCP->itemText(i).toStdString() == task->maleTCP) {
			_ui->maleTCP->setCurrentIndex(i);
			found = true;
			break;
		}
	}
	if (!found) {
		if (_ui->maleTCP->count() > 0)
		_ui->maleTCP->setCurrentIndex(0);
	    if (error == "")
	    	error = "Could not find the male TCP frame specified in task.";
	}

	// Set target values
	_ui->x->setValue(task->femaleTmaleTarget.P()[0]);
	_ui->y->setValue(task->femaleTmaleTarget.P()[1]);
	_ui->z->setValue(task->femaleTmaleTarget.P()[2]);
	const RPY<> rpy(task->femaleTmaleTarget.R());
	_ui->R->setValue(rpy[0]);
	_ui->P->setValue(rpy[1]);
	_ui->Y->setValue(rpy[2]);

	// Check strategy
	if (error == "") {
		if (task->strategy.isNull()) {
			error = "Could not find the strategy specified in task.";
		} else if (task->strategy->getID() != _strategy->getID()) {
			error = "The strategy given in task does not match the chosen strategy.";
		}
	}

	// Get parameters
	if (task->parameters.isNull()) {
		if (error == "")
			error = "Did not retrieve parameters from given task.";
	} else {
		_propertyMap = task->parameters->toPropertyMap();
		_ui->parameters->setPropertyMap(_propertyMap);
	}

    _ui->errorLabel->setText(QString::fromStdString(error));

    //settings().set<std::string>("LastDir",StringUtil::getDirectoryName(taskFile));
    //selectTask(1);
}

void TaskSetupDialog::saveTask() {
    const QString file = QFileDialog::getSaveFileName(this, "Save Assembly Task file", QString::fromStdString(_lastDir), "RW Assembly Task files ( *.tasks.assembly.xml )");
    if(file.size() == 0)
    	return;

	AssemblyTask::Ptr task = ownedPtr(new AssemblyTask());
	const QDateTime now = QDateTime::currentDateTime();
	const Transform3D<> target(Vector3D<>(_ui->x->value(),_ui->y->value(),_ui->z->value()),RPY<>(_ui->R->value(),_ui->P->value(),_ui->Y->value()));

	task->date = now.toString("yyyy-MM-dd HH:mm:ss").toStdString();
	task->generator = "ATaskVisPlugin TaskSetupDialog";
	task->femaleID = _ui->female->currentText().toStdString();
	task->femaleTCP = _ui->femaleTCP->currentText().toStdString();
	task->maleID = _ui->male->currentText().toStdString();
	task->maleTCP = _ui->maleTCP->currentText().toStdString();
	task->parameters = _strategy->createParameterization(_propertyMap);
	task->strategy = _strategy;
	task->femaleTmaleTarget = target;
	task->workcellName = _wc->getName();

	AssemblyTask::saveRWTask(task,file.toStdString()); // choose from dialog...
}

void TaskSetupDialog::updateView(SceneOpenGLViewer* widget) {
	const GroupNode::Ptr world = widget->getWorldNode();
	if (world->hasChild("Male"))
		world->removeChild("Male");
	if (world->hasChild("Female"))
		world->removeChild("Female");

	if (_wc.isNull())
		return;

	const Object::Ptr male = _wc->findObject(_ui->male->currentText().toStdString());
	const Object::Ptr female = _wc->findObject(_ui->female->currentText().toStdString());

	const State defState = _wc->getDefaultState();

	if (!female.isNull()) {
		GroupNode::Ptr group = ownedPtr(new GroupNode("Female"));
		const std::string tcpName = _ui->femaleTCP->currentText().toStdString();
		const Frame* const tcpFrame = _wc->findFrame(tcpName);
		Transform3D<> tcpTbase = Transform3D<>::identity();
		if (tcpFrame != NULL)
			tcpTbase = Kinematics::frameTframe(tcpFrame,female->getBase(),defState);

		const std::vector<Model3D::Ptr> models = female->getModels();
		for (std::size_t i = 0; i < models.size(); i++) {
			const DrawableNode::Ptr drawable = widget->getScene()->makeDrawable(models[i]->getName(),models[i]);
			drawable->setTransform(tcpTbase);
			drawable->setVisible(true);
			GroupNode::addChild(drawable,group);
		}
		GroupNode::addChild(group,world);

		const DrawableNode::Ptr base = widget->getScene()->makeDrawableFrameAxis("Base",0.03);
		base->setTransform(tcpTbase);
		GroupNode::addChild(base,group);

		base->setVisible(true);

		if (tcpName != "") {
			const DrawableNode::Ptr tcp = widget->getScene()->makeDrawableFrameAxis("TCP",0.03);
			GroupNode::addChild(tcp,group);
			tcp->setVisible(true);
		}
	}
	if (!male.isNull()) {
		GroupNode::Ptr group = ownedPtr(new GroupNode("Male"));
		const std::string tcpName = _ui->maleTCP->currentText().toStdString();
		const Frame* const tcpFrame = _wc->findFrame(tcpName);
		Transform3D<> tcpTbase = Transform3D<>::identity();
		if (tcpFrame != NULL)
			tcpTbase = Kinematics::frameTframe(tcpFrame,male->getBase(),defState);

		const std::vector<Model3D::Ptr> models = male->getModels();
		for (std::size_t i = 0; i < models.size(); i++) {
			const DrawableNode::Ptr drawable = widget->getScene()->makeDrawable(models[i]->getName(),models[i]);
			drawable->setTransform(tcpTbase);
			drawable->setVisible(true);
			GroupNode::addChild(drawable,group);
		}
		GroupNode::addChild(group,world);

		const DrawableNode::Ptr base = widget->getScene()->makeDrawableFrameAxis("Base",0.03);
		base->setTransform(tcpTbase);
		GroupNode::addChild(base,group);

		base->setVisible(true);

		if (tcpName != "") {
			const DrawableNode::Ptr tcp = widget->getScene()->makeDrawableFrameAxis("TCP",0.03);
			GroupNode::addChild(tcp,group);
			tcp->setVisible(true);
		}
	}
}

void TaskSetupDialog::updatePosition(SceneOpenGLViewer* widget, const Transform3D<>& fTm) {
	const GroupNode::Ptr world = widget->getWorldNode();
	GroupNode::Ptr female = NULL;
	GroupNode::Ptr male = NULL;
	const std::list<SceneNode::Ptr> nodes = world->_childNodes;
	for (std::list<SceneNode::Ptr>::const_iterator it = world->_childNodes.begin(); it != world->_childNodes.end(); it++) {
		if ((*it)->getName() == "Female")
			female = it->cast<GroupNode>();
		else if ((*it)->getName() == "Male")
			male = it->cast<GroupNode>();
	}
	if (!female.isNull()) {
		if (_ui->fixFemale->isChecked())
			female->setTransform(Transform3D<>::identity());
		else
			female->setTransform(inverse(fTm));
	}
	if (!male.isNull()) {
		if (_ui->fixMale->isChecked())
			male->setTransform(Transform3D<>::identity());
		else
			male->setTransform(fTm);
	}
    widget->updateView();
}

void TaskSetupDialog::showHelpStrategy() {
    QStringList filepaths;
    filepaths.append( QCoreApplication::applicationDirPath() );

	if (_help->showDocumentation(filepaths)) {
		_help->gotoURL("qthelp://robworkproject/doc/classrwlibs_1_1assembly_1_1PiHStrategy.html#details");
	} else {
		QDesktopServices::openUrl(QUrl("file:///home/tnt/Code/RobWorkTrunk/apidocs/html/classrwlibs_1_1assembly_1_1PiHStrategy.html#details"));
	}
}

void TaskSetupDialog::showHelpParameterization() {
    QStringList filepaths;
    filepaths.append( QCoreApplication::applicationDirPath() );

	if (_help->showDocumentation(filepaths)) {
		_help->gotoURL("qthelp://robworkproject/doc/classrwlibs_1_1assembly_1_1PiHParameterization.html#details");
	} else {
		QDesktopServices::openUrl(QUrl("file:///home/tnt/Code/RobWorkTrunk/apidocs/html/classrwlibs_1_1assembly_1_1PiHParameterization.html#details"));
	}
}
