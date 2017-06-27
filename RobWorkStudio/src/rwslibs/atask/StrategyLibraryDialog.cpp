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

#include "StrategyLibraryDialog.hpp"
#include "TaskSetupDialog.hpp"

#include "ui_StrategyLibraryDialog.h"

#include <rw/models/WorkCell.hpp>
#include <rw/sensor/FTSensor.hpp>
#include <rwlibs/assembly/AssemblyControlResponse.hpp>
#include <rwlibs/assembly/AssemblyControlStrategy.hpp>
#include <rwlibs/assembly/AssemblyRegistry.hpp>
#include <rwlibs/assembly/AssemblyState.hpp>
#include <rwlibs/assembly/PiHParameterization.hpp>
#include <rwlibs/assembly/PiHStrategy.hpp>
#include <rwlibs/assembly/SpiralParameterization.hpp>
#include <rwlibs/assembly/SpiralStrategy.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/opengl/DrawableFactory.hpp>
#include <rws/SceneOpenGLViewer.hpp>

#include <QPushButton>
#include <QTimer>

using namespace rw::common;
using namespace rw::graphics;
using rw::kinematics::State;
using namespace rw::math;
using rw::models::WorkCell;
using rw::sensor::FTSensor;
using namespace rwlibs::assembly;
using namespace rwlibs::opengl;
using rws::SceneOpenGLViewer;
using namespace rwslibs;

#define DT 100 // in milliseconds

namespace {
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

struct StrategyLibraryDialog::Cell {
	Cell(): widget(NULL), configBtn(NULL), viewWidget(NULL), time(0) {}
	QWidget* widget;
	QPushButton* configBtn;
	AssemblyParameterization::Ptr parameters;
	AssemblyControlStrategy::Ptr strategy;
	AssemblyControlStrategy::ControlState::Ptr cstate;
	AssemblyControlResponse::Ptr response;
	SceneOpenGLViewer* viewWidget;
	GroupNode::Ptr female;
	GroupNode::Ptr male;
	double time;

	void reset() {
		cstate = strategy->createState();
		response = NULL;
		time = 0;
	}
};

StrategyLibraryDialog::StrategyLibraryDialog(QWidget* parent):
	QDialog(parent),
	_dummyFTSensor(new DummyFTSensor()),
	_ui(new Ui::StrategyLibraryDialog()),
	_registry(ownedPtr(new AssemblyRegistry())),
	_timer(new QTimer(this))
{
	_ui->setupUi(this);

	const std::vector<std::string> strategies = _registry->getStrategies();
	_cells.resize(strategies.size());

	_ui->tableWidget->setColumnCount(static_cast<int>(strategies.size()));
	_ui->tableWidget->setRowCount(1);
	for (std::size_t i = 0; i < strategies.size(); i++) {
		const AssemblyControlStrategy::Ptr strategy = _registry->getStrategy(strategies[i]);
		RW_ASSERT(!strategy.isNull());
		_cells[i] = makeCell(strategy.get());
		_cells[i].strategy = strategy;
		_ui->tableWidget->setCellWidget(0,static_cast<int>(i),_cells[i].widget);
		_ui->tableWidget->setColumnWidth(static_cast<int>(i),200);
	}
	_ui->tableWidget->setRowHeight(0,200);

	connect(_timer, SIGNAL(timeout()), this, SLOT(step()));
	_timer->start(DT);

	setWorkCell(NULL);
}

StrategyLibraryDialog::~StrategyLibraryDialog() {
	delete _dummyFTSensor;
	delete _ui;
}

void StrategyLibraryDialog::setWorkCell(rw::common::Ptr<const WorkCell> wc) {
	_wc = NULL;
	bool enableConfig = false;
	if (wc.isNull()) {
		_ui->status->setText("Please load a WorkCell first to be able to proceed with configuration of a strategy.");
	} else if (wc->getObjects().size() <= 1) {
		_ui->status->setText("Please load a WorkCell with at least two objects to be able to proceed with configuration of a strategy.");
	} else {
		_ui->status->setText("Press configure to proceed with configuration of a specific strategy for the current workcell.");
		_wc = wc;
		enableConfig = true;
	}
	for (std::size_t i = 0; i < _cells.size(); i++) {
		_cells[i].configBtn->setVisible(enableConfig);
	}
}

void StrategyLibraryDialog::step() {
	for (std::size_t i = 0; i < _cells.size(); i++) {
		Cell& cell = _cells[i];
		if (cell.viewWidget == NULL)
			continue;
		if (!cell.response.isNull()) {
			if (cell.response->done)
				continue;
		}
		AssemblyState::Ptr real = ownedPtr(new AssemblyState());
		// real->femaleTmale // for done checks
		if (cell.cstate.isNull())
			cell.cstate = cell.strategy->createState();
		State state;
		const AssemblyControlResponse::Ptr response = cell.strategy->update(cell.parameters,real,real,cell.cstate,state,_dummyFTSensor,cell.time);
		if (response != NULL) {
			cell.response = response;
		}
		if (!cell.response.isNull()) {
			if (cell.response->type == AssemblyControlResponse::POSITION_TRAJECTORY) {
				const Transform3D<> T = cell.response->worldTendTrajectory->x(cell.time);
				cell.male->setTransform(T);
				cell.viewWidget->updateView();
			}
			if (cell.response->done)
				cell.reset();
		}
		cell.time += static_cast<double>(DT)/1000.;
	}
}

void StrategyLibraryDialog::configure() {
	QVariant stratID = sender()->property("AssemblyStrategyID");
	if (stratID.isValid()) {
		const AssemblyControlStrategy::Ptr strategy = _registry->getStrategy(stratID.toString().toStdString());
		if (strategy.isNull())
			return;
		QDialog* const dialog = new TaskSetupDialog(this,_wc,strategy);
		dialog->show();
		dialog->raise();
		dialog->activateWindow();
	}
}

StrategyLibraryDialog::Cell StrategyLibraryDialog::makeCell(AssemblyControlStrategy* const strategy) const {
	Cell cell;
	cell.widget = new QWidget(_ui->tableWidget);
	QVBoxLayout* const layout = new QVBoxLayout(cell.widget);
	cell.widget->setLayout(layout);

	QLabel* const idLabel = new QLabel("<b>"+QString::fromStdString(strategy->getID())+"</b>");
	idLabel->setToolTip(QString::fromStdString(strategy->getDescription()));
	layout->addWidget(idLabel);
	if (dynamic_cast<SpiralStrategy*>(strategy)) {
		const double r = 0.00015;
		const double n = 3.0; // number of rotations
		const double length_peg = 0.035;
		const double length_push = 0.0007;
		const double length_start_traj = 0.01; // initial z coordinate of peg tcp
		const double depth_in_hole_for_success = 0.1;
		const double speed = 0.025;
		const double d_path = 0.6;
		const double maxAllowedForce = 100000000;
		cell.parameters = ownedPtr(new SpiralParameterization(r, n, length_peg, length_push, length_start_traj, depth_in_hole_for_success, speed, d_path, maxAllowedForce));

	} else if (dynamic_cast<PiHStrategy*>(strategy)) {
		const double holeRadius = 0.01;
		const double pegRadius = 0.009;
		const double theta = 0.7;
		const double phi = 0;
		const double distX = 0.02;
		const double distY = 0;
		const double distTContact = 0;
		const double x0 = 0;
		cell.parameters = ownedPtr(new PiHParameterization(holeRadius, pegRadius, theta, phi, distX, distY, distTContact, x0));
	}

	if (!cell.parameters.isNull()) {
		const Transform3D<> mtcpTm(Vector3D<>::z()*0.035);
		const Transform3D<> fTmtcp = strategy->getApproach(cell.parameters);

		cell.viewWidget = new SceneOpenGLViewer(cell.widget);
		cell.viewWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

		//const GroupNode::Ptr world = widget->getWorldNode();
		const GroupNode::Ptr world = ownedPtr(new GroupNode("World"));
		cell.viewWidget->setWorldNode(world);
		cell.viewWidget->getPivotDrawable()->setVisible(false);
		const Drawable::Ptr female = DrawableFactory::constructFromGeometry("#Tube 0.01 0.001 0.04 8", "Female", true); // #Tube radius thickness height divisions
		const Drawable::Ptr male = DrawableFactory::constructFromGeometry("#Cylinder 0.009 0.035 8", "Female", true); // #Cylinder radius height divisions
		RW_ASSERT(!female.isNull());
		RW_ASSERT(!male.isNull());

		cell.female = ownedPtr(new GroupNode("Female"));
		cell.male = ownedPtr(new GroupNode("Male"));
		GroupNode::addChild(cell.female,world);
		GroupNode::addChild(cell.male,world);

		const DrawableNode::Ptr femaleTCP = cell.viewWidget->getScene()->makeDrawableFrameAxis("FemaleTCP",0.03);
		const DrawableNode::Ptr maleTCP = cell.viewWidget->getScene()->makeDrawableFrameAxis("MaleTCP",0.03);
		const DrawableNode::Ptr maleBase = cell.viewWidget->getScene()->makeDrawableFrameAxis("MaleBase",0.03);

		female->setTransform(Transform3D<>(-Vector3D<>::z()*0.04/2.));
		male->setTransform(mtcpTm*Transform3D<>(-Vector3D<>::z()*0.035/2.));
		maleBase->setTransform(mtcpTm);
		cell.female->setTransform(Transform3D<>::identity());
		cell.male->setTransform(fTmtcp);

		GroupNode::addChild(female,cell.female);
		GroupNode::addChild(male,cell.male);
		GroupNode::addChild(femaleTCP,cell.female);
		GroupNode::addChild(maleTCP,cell.male);
		GroupNode::addChild(maleBase,cell.male);
		female->setVisible(true);
		male->setVisible(true);
		femaleTCP->setVisible(true);
		maleTCP->setVisible(true);
		maleBase->setVisible(true);

		// Zoom in
		Transform3D<> Tcam = cell.viewWidget->getCameraController()->getTransform();
		Tcam.P() /= 40.;
		cell.viewWidget->getCameraController()->setTransform(Tcam);

		// Do not draw background
		const Property<bool>::Ptr bgProp = cell.viewWidget->getPropertyMap().findProperty<bool>("DrawBackGround");
		bgProp->setValue(false);
		bgProp->changedEvent().fire(bgProp.get());

		cell.viewWidget->updateView();
		layout->addWidget(cell.viewWidget);
	}

	cell.configBtn = new QPushButton("Configure");
	cell.configBtn->setProperty("AssemblyStrategyID",QString::fromStdString(strategy->getID()));
	layout->addWidget(cell.configBtn);
	connect(cell.configBtn, SIGNAL(pressed()), this, SLOT(configure()));
    return cell;
}
