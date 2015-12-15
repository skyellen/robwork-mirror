/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "BodyMotionWidget.hpp"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/opengl/RenderVelocity.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/LogPositions.hpp>
#include <rwsim/log/LogVelocities.hpp>

#include "ui_BodyMotionWidget.h"

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

BodyMotionWidget::BodyMotionWidget(rw::common::Ptr<const LogPositions> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::BodyMotionWidget()),
	_dwc(NULL),
	_positions(entry),
	_velocities(NULL)
{
	_ui->setupUi(this);

	connect(_ui->_motionBodiesTable->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(motionBodiesChanged(const QItemSelection &, const QItemSelection &)));
}

BodyMotionWidget::BodyMotionWidget(rw::common::Ptr<const LogVelocities> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::BodyMotionWidget()),
	_dwc(NULL),
	_positions(entry->getPositions()),
	_velocities(entry)
{
	_ui->setupUi(this);

	connect(_ui->_motionBodiesTable->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(motionBodiesChanged(const QItemSelection &, const QItemSelection &)));
}

BodyMotionWidget::~BodyMotionWidget() {
	if (_root != NULL) {
		_root->removeChild(_positionGroup);
		_root->removeChild(_velocityGroup);
	}
}

void BodyMotionWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
	_dwc = dwc;
}

void BodyMotionWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogPositions> pos = entry.cast<const LogPositions>();
	const rw::common::Ptr<const LogVelocities> vel = entry.cast<const LogVelocities>();
	if (!(pos == NULL)) {
		_positions = pos;
	} else if (!(vel == NULL)) {
		_velocities = vel;
		_positions = vel->getPositions();
	} else {
		RW_THROW("BodyMotionWidget (setEntry): invalid entry!");
	}
}

rw::common::Ptr<const SimulatorLog> BodyMotionWidget::getEntry() const {
	if (!(_velocities == NULL))
		return _velocities;
	else if (!(_positions == NULL))
		return _positions;
	RW_THROW("BodyMotionWidget (getEntry): both positions and velocities is zero!");
	return NULL;
}

void BodyMotionWidget::updateEntryWidget() {
	_ui->_motionBodiesTable->setColumnCount(7);
	QStringList headerLabels;
	headerLabels.push_back("Body");
	if (!(_velocities == NULL)) {
		headerLabels.push_back("Vel X");
		headerLabels.push_back("Vel Y");
		headerLabels.push_back("Vel Z");
		headerLabels.push_back("Avel X");
		headerLabels.push_back("Avel Y");
		headerLabels.push_back("Avel Z");
		_ui->_motionBodiesTable->setHorizontalHeaderLabels(headerLabels);
		const std::size_t n = _velocities->size();
		_ui->_motionDescription->setText(QString::fromStdString(_velocities->getDescription()));
		_ui->_motionBodies->setText(QString::number(n));
		_ui->_motionBodiesTable->setRowCount(n);
		int row = 0;
		_ui->_motionBodiesTable->setSortingEnabled(false);
		const std::map<std::string, VelocityScrew6D<> >& velMap = _velocities->getVelocities();
		std::map<std::string, VelocityScrew6D<> >::const_iterator it;
		for (it = velMap.begin(); it != velMap.end(); it++) {
			const std::string& body = it->first;
			const VelocityScrew6D<>& vel = it->second;
			_ui->_motionBodiesTable->setItem(row,0,new QTableWidgetItem(QString::fromStdString(body)));
			for (std::size_t i = 0; i < 6; i++)
				_ui->_motionBodiesTable->setItem(row,i+1,new QTableWidgetItem(QString::number(vel[i],'d',4)));
			row++;
		}
		_ui->_motionBodiesTable->setSortingEnabled(true);
		if (n > 0)
			_ui->_motionBodiesTable->setRangeSelected(QTableWidgetSelectionRange(0,0,n-1,6),true);
	} else if (!(_positions == NULL)) {
		headerLabels.push_back("Pos X");
		headerLabels.push_back("Pos Y");
		headerLabels.push_back("Pos Z");
		headerLabels.push_back("Rot X");
		headerLabels.push_back("Rot Y");
		headerLabels.push_back("Rot Z");
		_ui->_motionBodiesTable->setHorizontalHeaderLabels(headerLabels);
		const std::size_t n = _positions->size();
		_ui->_motionDescription->setText(QString::fromStdString(_positions->getDescription()));
		_ui->_motionBodies->setText(QString::number(n));
		_ui->_motionBodiesTable->setRowCount(n);
		int row = 0;
		_ui->_motionBodiesTable->setSortingEnabled(false);
		const std::map<std::string, Transform3D<> >& posMap = _positions->getPositions();
		std::map<std::string, Transform3D<> >::const_iterator it;
		for (it = posMap.begin(); it != posMap.end(); it++) {
			const std::string& body = it->first;
			const Transform3D<>& pos = it->second;
			_ui->_motionBodiesTable->setItem(row,0,new QTableWidgetItem(QString::fromStdString(body)));
			for (std::size_t i = 0; i < 3; i++)
				_ui->_motionBodiesTable->setItem(row,i+1,new QTableWidgetItem(QString::number(pos.P()[i],'d',4)));
			const EAA<> rot(pos.R());
			for (std::size_t i = 0; i < 3; i++)
				_ui->_motionBodiesTable->setItem(row,i+4,new QTableWidgetItem(QString::number(rot[i],'d',4)));
			row++;
		}
		_ui->_motionBodiesTable->setSortingEnabled(true);
		if (n > 0)
			_ui->_motionBodiesTable->setRangeSelected(QTableWidgetSelectionRange(0,0,n-1,6),true);
	}
}

void BodyMotionWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL) {
		_root->removeChild(_positionGroup);
		_root->removeChild(_velocityGroup);
	}
	_root = root;
	_graph = graph;
}

std::string BodyMotionWidget::getName() const {
	if (!(_velocities == NULL))
		return "Velocity";
	else if (!(_positions == NULL))
		return "Position";
	return "Motion";
}

void BodyMotionWidget::motionBodiesChanged(const QItemSelection& selection, const QItemSelection& deselection) {
	if (_root == NULL || _dwc == NULL || _positions == NULL)
		return;
	if (_velocities == NULL) {
		if (_positionGroup == NULL) {
			_positionGroup = ownedPtr(new GroupNode("Positions"));
			GroupNode::addChild(_positionGroup,_root);
		}
		// Remove deselected bodies
		foreach (QModelIndex index, deselection.indexes()) {
			if (index.column() > 0)
				continue;
			const std::string name = index.data().toString().toStdString();
			const Body::Ptr body = _dwc->findBody(name);
			_positionGroup->removeChild(body->getName());
		}
		// Add selected bodies
		foreach (QModelIndex index, selection.indexes()) {
			if (index.column() > 0)
				continue;
			const std::string name = index.data().toString().toStdString();
			const Body::Ptr body = _dwc->findBody(name);
			const GroupNode::Ptr bodyGroup = ownedPtr(new GroupNode(body->getName()));
			const std::vector<Model3D::Ptr>& models = body->getObject()->getModels();
			BOOST_FOREACH(const Model3D::Ptr model, models) {
				const DrawableNode::Ptr node = _graph->makeDrawable(model->getName(),model,DrawableNode::Physical);
				const Transform3D<>& t3d = _positions->getPosition(name);
				node->setTransform(t3d);
				GroupNode::addChild(node,bodyGroup);
				node->setVisible(true);
			}
			GroupNode::addChild(bodyGroup,_positionGroup);
		}
	} else {
		if (_velocityGroup == NULL) {
			_velocityGroup = ownedPtr(new GroupNode("Velocities"));
			GroupNode::addChild(_velocityGroup,_root);
		}
		// Remove deselected bodies
		foreach (QModelIndex index, deselection.indexes()) {
			if (index.column() > 0)
				continue;
			const std::string name = index.data().toString().toStdString();
			const Body::Ptr body = _dwc->findBody(name);
			_velocityGroup->removeChild(body->getName());
		}
		// Add selected bodies
		foreach (QModelIndex index, selection.indexes()) {
			if (index.column() > 0)
				continue;
			const std::string name = index.data().toString().toStdString();
			const Body::Ptr body = _dwc->findBody(name);
			const GroupNode::Ptr bodyGroup = ownedPtr(new GroupNode(body->getName()));
			BOOST_FOREACH(const Geometry::Ptr geo, body->getGeometry()) {
				const RenderVelocity::Ptr render = ownedPtr(new RenderVelocity());
				render->setVelocity(_velocities->getVelocity(name));
				render->setScaleAngular(0.1/(Pi/2));
				//render->setScaleLinear(0.1/(Pi/2));
				render->setScaleLinear(1.0); // spring test
				//render->setScaleAngular(0.1/(4*Pi)); // rotation test
				DrawableNode::Ptr node = ownedPtr(new Drawable(render,"VelocityRender",DrawableNode::Virtual));
				const Transform3D<> t3d = _positions->getPosition(name);
				const Vector3D<>& com = body->getInfo().masscenter;
				node->setTransform(Transform3D<>(t3d.P()+t3d.R()*com));
				GroupNode::addChild(node,bodyGroup);
				node->setVisible(true);
			}
			GroupNode::addChild(bodyGroup,_velocityGroup);
		}
	}
	emit graphicsUpdated();
}

BodyMotionWidget::Dispatcher::Dispatcher() {
}

BodyMotionWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* BodyMotionWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	rw::common::Ptr<const LogPositions> const pos = entry.cast<const LogPositions>();
	rw::common::Ptr<const LogVelocities> const vel = entry.cast<const LogVelocities>();
	if (!(pos == NULL))
		return new BodyMotionWidget(pos, parent);
	else if (!(vel == NULL))
		return new BodyMotionWidget(vel, parent);
	RW_THROW("BodyMotionWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool BodyMotionWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogPositions>() == NULL))
		return true;
	else if (!(entry.cast<const LogVelocities>() == NULL))
		return true;
	return false;
}
