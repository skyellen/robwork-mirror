/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ConstraintWidget.hpp"
#include "ui_ConstraintWidget.h"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderFrame.hpp>
#include <rwsim/log/LogConstraints.hpp>
#include <rwsim/log/LogPositions.hpp>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

ConstraintWidget::ConstraintWidget(rw::common::Ptr<const LogConstraints> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::ConstraintWidget()),
	_constraints(entry)
{
	_ui->setupUi(this);
	connect(_ui->_constraintTable->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(constraintSetChanged(const QItemSelection &, const QItemSelection &)));

	QStringList headerLabels;
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Type");
	_ui->_constraintTable->setColumnCount(3);
	_ui->_constraintTable->setHorizontalHeaderLabels(headerLabels);
}

ConstraintWidget::~ConstraintWidget() {
	if (_root != NULL) {
		_root->removeChild("Constraints");
	}
}

void ConstraintWidget::setDWC(rw::common::Ptr<const rwsim::dynamics::DynamicWorkCell> dwc) {
}

void ConstraintWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogConstraints> set = entry.cast<const LogConstraints>();
	if (!(set == NULL))
		_constraints = set;
	else
		RW_THROW("ConstraintWidget (setEntry): invalid entry!");
}

rw::common::Ptr<const SimulatorLog> ConstraintWidget::getEntry() const {
	return _constraints;
}

void ConstraintWidget::updateEntryWidget() {
	_ui->_constraintsDescription->setText(QString::fromStdString(_constraints->getDescription()));
	_ui->_constraints->setText(QString::number(_constraints->size()));

	_ui->_constraintTable->clearSelection();
	_ui->_constraintTable->setRowCount(_constraints->size());
	_ui->_constraintTable->setSortingEnabled(false);
	int row = 0;
	BOOST_FOREACH(const LogConstraints::Constraint& c, _constraints->getConstraints()) {
		const std::string& nameA = c.frameA;
		const std::string& nameB = c.frameB;
		const QString qnameA = QString::fromStdString(nameA);
		const QString qnameB = QString::fromStdString(nameB);
		const QString hover = "Bodies: " + qnameA + " - " + qnameB + "<br/>"
				+ "Points: " + toQString(c.posA) + " - " + toQString(c.posB);
		// Note: setItem takes ownership of the QTableWidgetItems
		QTableWidgetItem* itemA = new QTableWidgetItem(qnameA);
		QTableWidgetItem* itemB = new QTableWidgetItem(qnameB);
		QTableWidgetItem* itemC = new QTableWidgetItem(QString::fromStdString(c.type));
		itemA->setData(Qt::ToolTipRole,hover);
		itemB->setData(Qt::ToolTipRole,hover);
		itemC->setData(Qt::ToolTipRole,hover);
		itemA->setData(Qt::UserRole,QVariant::fromValue(row));
		_ui->_constraintTable->setItem(row,0,itemA);
		_ui->_constraintTable->setItem(row,1,itemB);
		_ui->_constraintTable->setItem(row,2,itemC);
		row++;
	}
	_ui->_constraintTable->setSortingEnabled(true);
	if (_constraints->size() > 0)
		_ui->_constraintTable->setRangeSelected(QTableWidgetSelectionRange(0,0,_constraints->size()-1,2),true);
}

void ConstraintWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL)
		_root->removeChild("Constraints");
	_root = root;
	_graph = graph;
}

std::string ConstraintWidget::getName() const {
	return "Constraints";
}

ConstraintWidget::Dispatcher::Dispatcher() {
}

ConstraintWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* ConstraintWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	const rw::common::Ptr<const LogConstraints> set = entry.cast<const LogConstraints>();
	if (!(set == NULL))
		return new ConstraintWidget(set, parent);
	RW_THROW("ConstraintWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool ConstraintWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogConstraints>() == NULL))
		return true;
	return false;
}

void ConstraintWidget::constraintSetChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection) {
	const QModelIndexList indexes = _ui->_constraintTable->selectionModel()->selectedIndexes();
	std::vector<LogConstraints::Constraint> show;
	foreach (QModelIndex index, indexes) {
		if (index.column() > 0)
			continue;
		const std::size_t i = index.data(Qt::UserRole).toUInt();
		const LogConstraints::Constraint& c = _constraints->getConstraints()[i];
		show.push_back(c);
	}
	_root->removeChild("Constraints");
	GroupNode::Ptr constraintGroup = ownedPtr(new GroupNode("Constraints"));
	const RenderFrame::Ptr render = ownedPtr(new RenderFrame(0.1));
	const DrawableNode::Ptr drawable = _graph->makeDrawable("Frame",render,DrawableNode::Physical);
	for (std::size_t i = 0; i < show.size(); i++) {
		LogConstraints::Constraint& con = show[i];
		std::stringstream str;
		str << "Constraint" << i;
		GroupNode::Ptr g1 = ownedPtr(new GroupNode(str.str()+"LinParent"));
		GroupNode::Ptr g2 = ownedPtr(new GroupNode(str.str()+"LinChild"));
		GroupNode::Ptr g3 = ownedPtr(new GroupNode(str.str()+"AngParent"));
		GroupNode::Ptr g4 = ownedPtr(new GroupNode(str.str()+"AngChild"));
		g1->setTransform(Transform3D<>(con.posA,con.rotAlin));
		g2->setTransform(Transform3D<>(con.posB,con.rotBlin));
		g3->setTransform(Transform3D<>(con.posA,con.rotAang));
		g4->setTransform(Transform3D<>(con.posB,con.rotBang));
		GroupNode::addChild(drawable,g1);
		GroupNode::addChild(drawable,g2);
		GroupNode::addChild(drawable,g3);
		GroupNode::addChild(drawable,g4);
		GroupNode::addChild(g1, constraintGroup);
		GroupNode::addChild(g2, constraintGroup);
		GroupNode::addChild(g3, constraintGroup);
		GroupNode::addChild(g4, constraintGroup);
	}
	drawable->setVisible(true);
	GroupNode::addChild(constraintGroup, _root);

	emit graphicsUpdated();
}

QString ConstraintWidget::toQString(const Vector3D<>& vec) {
	std::stringstream str;
	str << vec;
	return QString::fromStdString(str.str());
}
