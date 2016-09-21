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

#include "ForceTorqueWidget.hpp"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderVelocity.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/LogForceTorque.hpp>
#include "ui_ForceTorqueWidget.h"

#include <QItemSelection>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

ForceTorqueWidget::ForceTorqueWidget(rw::common::Ptr<const LogForceTorque> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::ForceTorqueWidget()),
	_forces(entry)
{
	_ui->setupUi(this);
	connect(_ui->_pairs->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(pairsChanged(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_table->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(contactSetChanged(const QItemSelection &, const QItemSelection &)));

	QStringList headerLabels;
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Number of Values");
	_ui->_pairs->setHorizontalHeaderLabels(headerLabels);

	_ui->_table->setColumnCount(6);
	headerLabels.clear();
	headerLabels.push_back("First");
	headerLabels.push_back("Force");
	headerLabels.push_back("Torque");
	headerLabels.push_back("Second");
	headerLabels.push_back("Force");
	headerLabels.push_back("Torque");
	_ui->_table->setHorizontalHeaderLabels(headerLabels);
}

ForceTorqueWidget::~ForceTorqueWidget() {
	if (_root != NULL) {
		_root->removeChild("Wrenches");
	}
}

void ForceTorqueWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
}

void ForceTorqueWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogForceTorque> set = entry.cast<const LogForceTorque>();
	if (!(set == NULL)) {
		_forces = set;
	} else {
		RW_THROW("ForceTorqueWidget (setEntry): invalid entry!");
	}
}

rw::common::Ptr<const SimulatorLog> ForceTorqueWidget::getEntry() const {
	return _forces;
}

void ForceTorqueWidget::updateEntryWidget() {
	_ui->_description->setText(QString::fromStdString(_forces->getDescription()));
	_ui->_values->setText(QString::number(_forces->sizeLinkedEntry()));
	typedef std::pair<std::string, std::string> FramePair;
	std::set<FramePair> pairs;
	for (int i = 0; i < _forces->sizeLinkedEntry(); i++) {
		const std::string& nameA = _forces->getNameA(i);
		const std::string& nameB = _forces->getNameB(i);
		if (nameA < nameB)
			pairs.insert(std::make_pair(nameA,nameB));
		else
			pairs.insert(std::make_pair(nameB,nameA));
	}
	const int nrOfPairs = static_cast<int>(pairs.size());
	if (pairs.size() > static_cast<std::size_t>(nrOfPairs))
		RW_THROW("There are too many entries for the widget to handle!");
	_ui->_pairs->setRowCount(nrOfPairs);
	int row = 0;
	_ui->_pairs->setSortingEnabled(false);
	BOOST_FOREACH(const FramePair& pair, pairs) {
		// Count how many contacts there are for this pair
		int contacts = 0;
		for (int i = 0; i < _forces->sizeLinkedEntry(); i++) {
			if (_forces->getNameA(i) == pair.first && _forces->getNameB(i) == pair.second)
				contacts++;
			else if (_forces->getNameA(i) == pair.second && _forces->getNameB(i) == pair.first)
				contacts++;
		}
		// Note: setItem takes ownership of the QTableWidgetItems
		_ui->_pairs->setItem(row,0,new QTableWidgetItem(QString::fromStdString(pair.first)));
		_ui->_pairs->setItem(row,1,new QTableWidgetItem(QString::fromStdString(pair.second)));
		_ui->_pairs->setItem(row,2,new QTableWidgetItem(QString::number(contacts)));
		row++;
	}
	_ui->_pairs->setSortingEnabled(true);
	if (nrOfPairs > 0)
		_ui->_pairs->setRangeSelected(QTableWidgetSelectionRange(0,0,nrOfPairs-1,2),true);
}

void ForceTorqueWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL)
		_root->removeChild("Bodies");
	_root = root;
	_graph = graph;
}

std::string ForceTorqueWidget::getName() const {
	return "Forces and Torques";
}

void ForceTorqueWidget::pairsChanged(const QItemSelection&, const QItemSelection&) {
	typedef std::pair<std::string, std::string> NamePair;
	const QModelIndexList indexes = _ui->_pairs->selectionModel()->selectedIndexes();
	std::list<NamePair> names;
	foreach (QModelIndex index, indexes) {
		if (index.column() != 0)
			continue;
		const std::string nameA = _ui->_pairs->item(index.row(),0)->data(Qt::DisplayRole).toString().toStdString();
		const std::string nameB = _ui->_pairs->item(index.row(),1)->data(Qt::DisplayRole).toString().toStdString();
		names.push_back(NamePair(nameA,nameB));
	}
	std::vector<std::size_t> contactsToShow;
	for (int i = 0; i < _forces->sizeLinkedEntry(); i++) {
		bool show = false;
		BOOST_FOREACH(const NamePair& name, names) {
			const std::string& nameA = _forces->getNameA(i);
			const std::string& nameB = _forces->getNameB(i);
			if (nameA == name.first && nameB == name.second)
				show = true;
			else if (nameB == name.first && nameA == name.second)
				show = true;
			if (show)
				break;
		}
		if (show)
			contactsToShow.push_back(i);
	}
	const int nrOfContactsToShow = static_cast<int>(contactsToShow.size());
	if (contactsToShow.size() > static_cast<std::size_t>(nrOfContactsToShow))
		RW_THROW("There are too many entries for the widget to handle!");
	_ui->_table->clearSelection();
	_ui->_table->setRowCount(nrOfContactsToShow);
	int row = 0;
	_ui->_table->setSortingEnabled(false);
	BOOST_FOREACH(const std::size_t i, contactsToShow) {
		const Wrench6D<> ftA = _forces->getWrenchBodyA(i);
		const Wrench6D<> ftB = _forces->getWrenchBodyB(i);
		const std::string& nameA = _forces->getNameA(i);
		const std::string& nameB = _forces->getNameB(i);
		const QString hover = toQString(nameA, nameB, ftA, ftB);
		// Note: setItem takes ownership of the QTableWidgetItems
		QTableWidgetItem* itemA;
		QTableWidgetItem* itemB;
		QTableWidgetItem* itemAf = new QTableWidgetItem(QString::number(ftA.force().norm2()));
		QTableWidgetItem* itemAt = new QTableWidgetItem(QString::number(ftA.torque().norm2()));
		QTableWidgetItem* itemBf = new QTableWidgetItem(QString::number(ftB.force().norm2()));
		QTableWidgetItem* itemBt = new QTableWidgetItem(QString::number(ftB.torque().norm2()));
		if (nameA < nameB) {
			itemA = new QTableWidgetItem(QString::fromStdString(nameA));
			itemB = new QTableWidgetItem(QString::fromStdString(nameB));
		} else {
			itemA = new QTableWidgetItem(QString::fromStdString(nameB));
			itemB = new QTableWidgetItem(QString::fromStdString(nameA));
		}
		itemA->setData(Qt::ToolTipRole,hover);
		itemB->setData(Qt::ToolTipRole,hover);
		itemAf->setData(Qt::ToolTipRole,hover);
		itemAt->setData(Qt::ToolTipRole,hover);
		itemBf->setData(Qt::ToolTipRole,hover);
		itemBt->setData(Qt::ToolTipRole,hover);
		itemA->setData(Qt::UserRole,QVariant::fromValue(i));
		_ui->_table->setItem(row,0,itemA);
		_ui->_table->setItem(row,1,itemAf);
		_ui->_table->setItem(row,2,itemAt);
		_ui->_table->setItem(row,3,itemB);
		_ui->_table->setItem(row,4,itemBf);
		_ui->_table->setItem(row,5,itemBt);
		row++;
	}
	_ui->_table->setSortingEnabled(true);
	if (nrOfContactsToShow > 0)
		_ui->_table->setRangeSelected(QTableWidgetSelectionRange(0,0,nrOfContactsToShow-1,5),true);
}

void ForceTorqueWidget::contactSetChanged(const QItemSelection&, const QItemSelection&) {
	const QModelIndexList indexes = _ui->_table->selectionModel()->selectedIndexes();
	_root->removeChild("Wrenches");
	GroupNode::Ptr wrenchGroup = ownedPtr(new GroupNode("Wrenches"));
	foreach (QModelIndex index, indexes) {
		if (index.column() > 0)
			continue;
		const std::size_t i = index.data(Qt::UserRole).toUInt();
		//contacts.push_back(_contactSet->contacts[i]);
		const RenderVelocity::Ptr renderA = ownedPtr(new RenderVelocity());
		const RenderVelocity::Ptr renderB = ownedPtr(new RenderVelocity());
		const Wrench6D<> ftA = _forces->getWrenchBodyA(i);
		const Wrench6D<> ftB = _forces->getWrenchBodyB(i);
		renderA->setVelocity(VelocityScrew6D<>(ftA.force(),EAA<>(ftA.torque())));
		renderB->setVelocity(VelocityScrew6D<>(ftB.force(),EAA<>(ftB.torque())));
		renderA->setScales(0.1f,0.1f);
		renderB->setScales(0.1f,0.1f);
		const DrawableNode::Ptr drawableA = _graph->makeDrawable("WrenchA",renderA,DrawableNode::Physical);
		const DrawableNode::Ptr drawableB = _graph->makeDrawable("WrenchB",renderB,DrawableNode::Physical);
		drawableA->setTransform(Transform3D<>(_forces->getPositionA(i)));
		drawableB->setTransform(Transform3D<>(_forces->getPositionB(i)));
		GroupNode::addChild(drawableA,wrenchGroup);
		GroupNode::addChild(drawableB,wrenchGroup);
		drawableA->setVisible(true);
		drawableB->setVisible(true);
	}
	GroupNode::addChild(wrenchGroup, _root);
	emit graphicsUpdated();
}

QString ForceTorqueWidget::toQString(const Vector3D<>& vec) {
	std::stringstream str;
	str << vec;
	return QString::fromStdString(str.str());
}

QString ForceTorqueWidget::toQString(const std::string& nameA, const std::string& nameB, const Wrench6D<>& ftA, const Wrench6D<>& ftB) {
	const QString qnameA = QString::fromStdString(nameA);
	const QString qnameB = QString::fromStdString(nameB);
	return qnameA + ": force " + toQString(ftA.force()) + " torque " + toQString(ftA.torque()) + "<br/>"
			+ qnameB + ": force " + toQString(ftB.force()) + " torque " + toQString(ftB.torque());
}

ForceTorqueWidget::Dispatcher::Dispatcher() {
}

ForceTorqueWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* ForceTorqueWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	const rw::common::Ptr<const LogForceTorque> forces = entry.cast<const LogForceTorque>();
	if (!(forces == NULL))
		return new ForceTorqueWidget(forces, parent);
	RW_THROW("ForceTorqueWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool ForceTorqueWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogForceTorque>() == NULL))
		return true;
	return false;
}
