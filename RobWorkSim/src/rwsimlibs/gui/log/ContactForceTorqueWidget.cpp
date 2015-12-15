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

#include "ContactForceTorqueWidget.hpp"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderVelocity.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include "../../../rwsim/log/LogContactForceTorque.hpp"
#include "../../../rwsim/log/LogContactSet.hpp"
#include "ui_ContactForceTorqueWidget.h"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

ContactForceTorqueWidget::ContactForceTorqueWidget(rw::common::Ptr<const LogContactForceTorque> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::ContactForceTorqueWidget()),
	_forces(entry),
	_contactSet(entry->getContacts())
{
	_ui->setupUi(this);
	connect(_ui->_contactBodyPairs->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(contactSetPairsChanged(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_contactTable->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(contactSetChanged(const QItemSelection &, const QItemSelection &)));

	QStringList headerLabels;
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Contacts");
	_ui->_contactBodyPairs->setHorizontalHeaderLabels(headerLabels);

	_ui->_contactTable->setColumnCount(6);
	headerLabels.clear();
	headerLabels.push_back("First");
	headerLabels.push_back("Force");
	headerLabels.push_back("Torque");
	headerLabels.push_back("Second");
	headerLabels.push_back("Force");
	headerLabels.push_back("Torque");
	_ui->_contactTable->setHorizontalHeaderLabels(headerLabels);
}

ContactForceTorqueWidget::~ContactForceTorqueWidget() {
	if (_root != NULL) {
		_root->removeChild("ContactForces");
	}
}

void ContactForceTorqueWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
}

void ContactForceTorqueWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogContactForceTorque> set = entry.cast<const LogContactForceTorque>();
	if (!(set == NULL)) {
		_forces = set;
		_contactSet = _forces->getContacts();
	} else {
		RW_THROW("ContactForceTorqueWidget (setEntry): invalid entry!");
	}
}

rw::common::Ptr<const SimulatorLog> ContactForceTorqueWidget::getEntry() const {
	return _contactSet;
}

void ContactForceTorqueWidget::updateEntryWidget() {
	_ui->_contactDescription->setText(QString::fromStdString(_forces->getDescription()));
	_ui->_contacts->setText(QString::number(_contactSet->size()));
	typedef std::pair<std::string, std::string> FramePair;
	std::set<FramePair> pairs;
	BOOST_FOREACH(const Contact& c, _contactSet->getContacts()) {
		const std::string& nameA = c.getNameA();
		const std::string& nameB = c.getNameB();
		if (nameA < nameB)
			pairs.insert(std::make_pair(nameA,nameB));
		else
			pairs.insert(std::make_pair(nameB,nameA));
	}
	_ui->_contactBodyPairs->setRowCount(pairs.size());
	int row = 0;
	_ui->_contactBodyPairs->setSortingEnabled(false);
	BOOST_FOREACH(const FramePair& pair, pairs) {
		// Count how many contacts there are for this pair
		int contacts = 0;
		BOOST_FOREACH(const Contact& c, _contactSet->getContacts()) {
			if (c.getNameA() == pair.first && c.getNameB() == pair.second)
				contacts++;
			else if (c.getNameA() == pair.second && c.getNameB() == pair.first)
				contacts++;
		}
		// Note: setItem takes ownership of the QTableWidgetItems
		_ui->_contactBodyPairs->setItem(row,0,new QTableWidgetItem(QString::fromStdString(pair.first)));
		_ui->_contactBodyPairs->setItem(row,1,new QTableWidgetItem(QString::fromStdString(pair.second)));
		_ui->_contactBodyPairs->setItem(row,2,new QTableWidgetItem(QString::number(contacts)));
		row++;
	}
	_ui->_contactBodyPairs->setSortingEnabled(true);
	if (pairs.size() > 0)
		_ui->_contactBodyPairs->setRangeSelected(QTableWidgetSelectionRange(0,0,pairs.size()-1,2),true);
}

void ContactForceTorqueWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL)
		_root->removeChild("Bodies");
	_root = root;
	_graph = graph;
}

std::string ContactForceTorqueWidget::getName() const {
	return "Contact Forces";
}

void ContactForceTorqueWidget::contactSetPairsChanged(const QItemSelection&, const QItemSelection&) {
	typedef std::pair<std::string, std::string> NamePair;
	const QModelIndexList indexes = _ui->_contactBodyPairs->selectionModel()->selectedIndexes();
	std::list<NamePair> names;
	foreach (QModelIndex index, indexes) {
		if (index.column() != 0)
			continue;
		const std::string nameA = _ui->_contactBodyPairs->item(index.row(),0)->data(Qt::DisplayRole).toString().toStdString();
		const std::string nameB = _ui->_contactBodyPairs->item(index.row(),1)->data(Qt::DisplayRole).toString().toStdString();
		names.push_back(NamePair(nameA,nameB));
	}
	std::vector<std::size_t> contactsToShow;
	for (std::size_t i = 0; i < _contactSet->size(); i++) {
		const Contact& c = _contactSet->getContact(i);
		bool show = false;
		BOOST_FOREACH(const NamePair& name, names) {
			const std::string& nameA = c.getNameA();
			const std::string& nameB = c.getNameB();
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
	_ui->_contactTable->clearSelection();
	_ui->_contactTable->setRowCount(contactsToShow.size());
	int row = 0;
	_ui->_contactTable->setSortingEnabled(false);
	BOOST_FOREACH(const std::size_t i, contactsToShow) {
		const Contact& c = _contactSet->getContact(i);
		const Wrench6D<> ftA = _forces->getWrenchBodyA(i);
		const Wrench6D<> ftB = _forces->getWrenchBodyB(i);
		const std::string& nameA = c.getNameA();
		const std::string& nameB = c.getNameB();
		const QString hover = toQString(c, ftA, ftB);
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
		_ui->_contactTable->setItem(row,0,itemA);
		_ui->_contactTable->setItem(row,1,itemAf);
		_ui->_contactTable->setItem(row,2,itemAt);
		_ui->_contactTable->setItem(row,3,itemB);
		_ui->_contactTable->setItem(row,4,itemBf);
		_ui->_contactTable->setItem(row,5,itemBt);
		row++;
	}
	_ui->_contactTable->setSortingEnabled(true);
	if (contactsToShow.size() > 0)
		_ui->_contactTable->setRangeSelected(QTableWidgetSelectionRange(0,0,contactsToShow.size()-1,5),true);
}

void ContactForceTorqueWidget::contactSetChanged(const QItemSelection&, const QItemSelection&) {
	const QModelIndexList indexes = _ui->_contactTable->selectionModel()->selectedIndexes();
	_root->removeChild("ContactForces");
	GroupNode::Ptr contactGroup = ownedPtr(new GroupNode("ContactForces"));
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
		renderA->setScales(0.1,0.1);
		renderB->setScales(0.1,0.1);
		const DrawableNode::Ptr drawableA = _graph->makeDrawable("ContactWrenchA",renderA,DrawableNode::Physical);
		const DrawableNode::Ptr drawableB = _graph->makeDrawable("ContactWrenchB",renderB,DrawableNode::Physical);
		drawableA->setTransform(Transform3D<>(_forces->getContacts()->getContact(i).getPointA()));
		drawableB->setTransform(Transform3D<>(_forces->getContacts()->getContact(i).getPointB()));
		GroupNode::addChild(drawableA,contactGroup);
		GroupNode::addChild(drawableB,contactGroup);
		drawableA->setVisible(true);
		drawableB->setVisible(true);
	}
	GroupNode::addChild(contactGroup, _root);
	emit graphicsUpdated();
}

QString ContactForceTorqueWidget::toQString(const Vector3D<>& vec) {
	std::stringstream str;
	str << vec;
	return QString::fromStdString(str.str());
}

QString ContactForceTorqueWidget::toQString(const Contact& contact, const Wrench6D<>& ftA, const Wrench6D<>& ftB) {
	const QString nameA = QString::fromStdString(contact.getNameA());
	const QString nameB = QString::fromStdString(contact.getNameB());
	return nameA + ": force " + toQString(ftA.force()) + " torque " + toQString(ftA.torque()) + "<br/>"
			+ nameB + ": force " + toQString(ftB.force()) + " torque " + toQString(ftB.torque());
}

ContactForceTorqueWidget::Dispatcher::Dispatcher() {
}

ContactForceTorqueWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* ContactForceTorqueWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	const rw::common::Ptr<const LogContactForceTorque> forces = entry.cast<const LogContactForceTorque>();
	if (!(forces == NULL))
		return new ContactForceTorqueWidget(forces, parent);
	RW_THROW("ContactForceTorqueWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool ContactForceTorqueWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogContactForceTorque>() == NULL))
		return true;
	return false;
}
