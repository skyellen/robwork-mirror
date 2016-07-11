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

#include "ContactVelocitiesWidget.hpp"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderVelocity.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/LogContactSet.hpp>
#include <rwsim/log/LogContactVelocities.hpp>

#include "ui_ContactVelocitiesWidget.h"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

ContactVelocitiesWidget::ContactVelocitiesWidget(rw::common::Ptr<const LogContactVelocities> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::ContactVelocitiesWidget()),
	_velocities(entry),
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

	_ui->_contactTable->setColumnCount(8);
	headerLabels.clear();
	headerLabels.push_back("First");
	headerLabels.push_back("x");
	headerLabels.push_back("y");
	headerLabels.push_back("z");
	headerLabels.push_back("Second");
	headerLabels.push_back("x");
	headerLabels.push_back("y");
	headerLabels.push_back("z");
	_ui->_contactTable->setHorizontalHeaderLabels(headerLabels);
}

ContactVelocitiesWidget::~ContactVelocitiesWidget() {
	if (_root != NULL) {
		_root->removeChild("ContactVelocities");
	}
}

void ContactVelocitiesWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
}

void ContactVelocitiesWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogContactVelocities> set = entry.cast<const LogContactVelocities>();
	if (!(set == NULL)) {
		_velocities = set;
		_contactSet = _velocities->getContacts();
	} else {
		RW_THROW("ContactVelocitiesWidget (setEntry): invalid entry!");
	}
}

rw::common::Ptr<const SimulatorLog> ContactVelocitiesWidget::getEntry() const {
	return _contactSet;
}

void ContactVelocitiesWidget::updateEntryWidget() {
	_ui->_contactDescription->setText(QString::fromStdString(_velocities->getDescription()));
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

void ContactVelocitiesWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL)
		_root->removeChild("Bodies");
	_root = root;
	_graph = graph;
}

std::string ContactVelocitiesWidget::getName() const {
	return "Contact Velocities";
}

void ContactVelocitiesWidget::contactSetPairsChanged(const QItemSelection&, const QItemSelection&) {
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
		const Vector3D<> velA = _velocities->getVelocityBodyA(i);
		const Vector3D<> velB = _velocities->getVelocityBodyB(i);
		const std::string& nameA = c.getNameA();
		const std::string& nameB = c.getNameB();
		const QString hover = toQString(c, velA, velB);
		// Note: setItem takes ownership of the QTableWidgetItems
		QTableWidgetItem* itemA;
		QTableWidgetItem* itemB;
		QTableWidgetItem* itemAx = new QTableWidgetItem(QString::number(velA[0]));
		QTableWidgetItem* itemAy = new QTableWidgetItem(QString::number(velA[1]));
		QTableWidgetItem* itemAz = new QTableWidgetItem(QString::number(velA[2]));
		QTableWidgetItem* itemBx = new QTableWidgetItem(QString::number(velB[0]));
		QTableWidgetItem* itemBy = new QTableWidgetItem(QString::number(velB[1]));
		QTableWidgetItem* itemBz = new QTableWidgetItem(QString::number(velB[2]));
		if (nameA < nameB) {
			itemA = new QTableWidgetItem(QString::fromStdString(nameA));
			itemB = new QTableWidgetItem(QString::fromStdString(nameB));
		} else {
			itemA = new QTableWidgetItem(QString::fromStdString(nameB));
			itemB = new QTableWidgetItem(QString::fromStdString(nameA));
		}
		itemA->setData(Qt::ToolTipRole,hover);
		itemB->setData(Qt::ToolTipRole,hover);
		itemAx->setData(Qt::ToolTipRole,hover);
		itemAy->setData(Qt::ToolTipRole,hover);
		itemAz->setData(Qt::ToolTipRole,hover);
		itemBx->setData(Qt::ToolTipRole,hover);
		itemBy->setData(Qt::ToolTipRole,hover);
		itemBz->setData(Qt::ToolTipRole,hover);
		itemA->setData(Qt::UserRole,QVariant::fromValue(i));
		_ui->_contactTable->setItem(row,0,itemA);
		_ui->_contactTable->setItem(row,1,itemAx);
		_ui->_contactTable->setItem(row,2,itemAy);
		_ui->_contactTable->setItem(row,3,itemAz);
		_ui->_contactTable->setItem(row,4,itemB);
		_ui->_contactTable->setItem(row,5,itemBx);
		_ui->_contactTable->setItem(row,6,itemBy);
		_ui->_contactTable->setItem(row,7,itemBz);
		row++;
	}
	_ui->_contactTable->setSortingEnabled(true);
	if (contactsToShow.size() > 0)
		_ui->_contactTable->setRangeSelected(QTableWidgetSelectionRange(0,0,contactsToShow.size()-1,7),true);
}

void ContactVelocitiesWidget::contactSetChanged(const QItemSelection&, const QItemSelection&) {
	const QModelIndexList indexes = _ui->_contactTable->selectionModel()->selectedIndexes();
	_root->removeChild("ContactVelocities");
	GroupNode::Ptr contactGroup = ownedPtr(new GroupNode("ContactVelocities"));
	foreach (QModelIndex index, indexes) {
		if (index.column() > 0)
			continue;
		const std::size_t i = index.data(Qt::UserRole).toUInt();
		//contacts.push_back(_contactSet->contacts[i]);
		const RenderVelocity::Ptr renderA = ownedPtr(new RenderVelocity());
		const RenderVelocity::Ptr renderB = ownedPtr(new RenderVelocity());
		renderA->setVelocity(VelocityScrew6D<>(_velocities->getVelocityBodyA(i),EAA<>()));
		renderB->setVelocity(VelocityScrew6D<>(_velocities->getVelocityBodyB(i),EAA<>()));
		const DrawableNode::Ptr drawableA = _graph->makeDrawable("ContactVelocityA",renderA,DrawableNode::Physical);
		const DrawableNode::Ptr drawableB = _graph->makeDrawable("ContactVelocityB",renderB,DrawableNode::Physical);
		drawableA->setTransform(Transform3D<>(_velocities->getContacts()->getContact(i).getPointA()));
		drawableB->setTransform(Transform3D<>(_velocities->getContacts()->getContact(i).getPointB()));
		GroupNode::addChild(drawableA,contactGroup);
		GroupNode::addChild(drawableB,contactGroup);
		drawableA->setVisible(true);
		drawableB->setVisible(true);
	}
	GroupNode::addChild(contactGroup, _root);
	emit graphicsUpdated();
}

QString ContactVelocitiesWidget::toQString(const Vector3D<>& vec) {
	std::stringstream str;
	str << vec;
	return QString::fromStdString(str.str());
}

QString ContactVelocitiesWidget::toQString(const Contact& contact, const Vector3D<>& velA, const Vector3D<>& velB) {
	const QString nameA = QString::fromStdString(contact.getNameA());
	const QString nameB = QString::fromStdString(contact.getNameB());
	return nameA + ": " + toQString(velA) + "<br/>"
			+ nameB + ": " + toQString(velB);
}

ContactVelocitiesWidget::Dispatcher::Dispatcher() {
}

ContactVelocitiesWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* ContactVelocitiesWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	const rw::common::Ptr<const LogContactVelocities> velocities = entry.cast<const LogContactVelocities>();
	if (!(velocities == NULL))
		return new ContactVelocitiesWidget(velocities, parent);
	RW_THROW("ContactVelocitiesWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool ContactVelocitiesWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogContactVelocities>() == NULL))
		return true;
	return false;
}
