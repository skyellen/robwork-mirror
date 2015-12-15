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

#include "RWPELogContactTracking.hpp"
#include "RWPELogContactTrackingWidget.hpp"
#include "ui_RWPELogContactTrackingWidget.h"

#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderLines.hpp>
#include <rwlibs/opengl/RenderPointCloud.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rwlibs::opengl;
using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;
using namespace rwsimlibs::rwpe;

RWPELogContactTrackingWidget::RWPELogContactTrackingWidget(rw::common::Ptr<const RWPELogContactTracking> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::RWPELogContactTrackingWidget()),
	_tracking(entry)
{
	_ui->setupUi(this);
	connect(_ui->_contactBodyPairs->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(contactSetPairsChanged(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_contactTable->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(contactSetChanged(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_before, SIGNAL(stateChanged(int)), this, SLOT(checkboxChanged(int)));
	connect(_ui->_after, SIGNAL(stateChanged(int)), this, SLOT(checkboxChanged(int)));
	connect(_ui->_added, SIGNAL(stateChanged(int)), this, SLOT(checkboxChanged(int)));
	connect(_ui->_gone, SIGNAL(stateChanged(int)), this, SLOT(checkboxChanged(int)));

	QStringList headerLabels;
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	_ui->_contactBodyPairs->setHorizontalHeaderLabels(headerLabels);

	_ui->_contactTable->setColumnCount(3);
	headerLabels.clear();
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Type");
	_ui->_contactTable->setHorizontalHeaderLabels(headerLabels);
}

RWPELogContactTrackingWidget::~RWPELogContactTrackingWidget() {
	if (_root != NULL) {
		_root->removeChild("RWPEContactTracking");
	}
}

void RWPELogContactTrackingWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
}

void RWPELogContactTrackingWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const RWPELogContactTracking> tracking = entry.cast<const RWPELogContactTracking>();
	if (!tracking.isNull())
		_tracking = tracking;
	else
		RW_THROW("RWPELogContactTrackingWidget (setEntry): invalid entry!");
}

rw::common::Ptr<const SimulatorLog> RWPELogContactTrackingWidget::getEntry() const {
	return _tracking;
}

void RWPELogContactTrackingWidget::updateEntryWidget() {
	_ui->_contactDescription->setText(QString::fromStdString(_tracking->getDescription()));
	_ui->_contacts->setText("Before: " + QString::number(_tracking->before.size()) + ", After: " + QString::number(_tracking->after.size()) + ", Added: " + QString::number(_tracking->added.size()) + ", Gone: " + QString::number(_tracking->gone.size()));
	typedef std::pair<std::string, std::string> FramePair;
	std::set<FramePair> pairs;
	if (_ui->_before->isChecked()) {
		BOOST_FOREACH(const Contact& c, _tracking->before) {
			const std::string& nameA = c.getNameA();
			const std::string& nameB = c.getNameB();
			if (nameA < nameB)
				pairs.insert(std::make_pair(nameA,nameB));
			else
				pairs.insert(std::make_pair(nameB,nameA));
		}
	}
	if (_ui->_after->isChecked()) {
		BOOST_FOREACH(const Contact& c, _tracking->after) {
			const std::string& nameA = c.getNameA();
			const std::string& nameB = c.getNameB();
			if (nameA < nameB)
				pairs.insert(std::make_pair(nameA,nameB));
			else
				pairs.insert(std::make_pair(nameB,nameA));
		}
	}
	if (_ui->_added->isChecked()) {
		BOOST_FOREACH(const Contact& c, _tracking->added) {
			const std::string& nameA = c.getNameA();
			const std::string& nameB = c.getNameB();
			if (nameA < nameB)
				pairs.insert(std::make_pair(nameA,nameB));
			else
				pairs.insert(std::make_pair(nameB,nameA));
		}
	}
	if (_ui->_gone->isChecked()) {
		BOOST_FOREACH(const Contact& c, _tracking->gone) {
			const std::string& nameA = c.getNameA();
			const std::string& nameB = c.getNameB();
			if (nameA < nameB)
				pairs.insert(std::make_pair(nameA,nameB));
			else
				pairs.insert(std::make_pair(nameB,nameA));
		}
	}
	_ui->_contactBodyPairs->clearSelection();
	_ui->_contactBodyPairs->setRowCount(pairs.size());
	int row = 0;
	_ui->_contactBodyPairs->setSortingEnabled(false);
	BOOST_FOREACH(const FramePair& pair, pairs) {
		// Note: setItem takes ownership of the QTableWidgetItems
		_ui->_contactBodyPairs->setItem(row,0,new QTableWidgetItem(QString::fromStdString(pair.first)));
		_ui->_contactBodyPairs->setItem(row,1,new QTableWidgetItem(QString::fromStdString(pair.second)));
		row++;
	}
	_ui->_contactBodyPairs->setSortingEnabled(true);
	if (pairs.size() > 0)
		_ui->_contactBodyPairs->setRangeSelected(QTableWidgetSelectionRange(0,0,pairs.size()-1,1),true);
}

void RWPELogContactTrackingWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL)
		_root->removeChild("RWPEContactTracking");
	_root = root;
	_graph = graph;
}

std::string RWPELogContactTrackingWidget::getName() const {
	return "RWPE Contact Tracking";
}

void RWPELogContactTrackingWidget::checkboxChanged(int state) {
	updateEntryWidget();
}

void RWPELogContactTrackingWidget::contactSetPairsChanged(const QItemSelection&, const QItemSelection&) {
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
	std::vector<std::vector<std::size_t> > contactsToShow(4);
	std::vector<std::vector<Contact> > contacts(4);
	contacts[0] = _tracking->before;
	contacts[1] = _tracking->after;
	contacts[2] = _tracking->added;
	contacts[3] = _tracking->gone;
	for (std::size_t g = 0; g < 4; g++) {
		if (g < 2 && (!_ui->_before->isChecked() && !_ui->_after->isChecked()))
			continue;
		else if (g == 2 && !_ui->_added->isChecked())
			continue;
		else if (g == 3 && !_ui->_gone->isChecked())
			continue;
		for (std::size_t i = 0; i < contacts[g].size(); i++) {
			const Contact& c = contacts[g][i];
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
				contactsToShow[g].push_back(i);
		}
	}
	RW_ASSERT(contactsToShow[0].size() == contactsToShow[1].size());
	_ui->_contactTable->clearSelection();
	_ui->_contactTable->setRowCount(contactsToShow[0].size()+contactsToShow[2].size()+contactsToShow[3].size());
	int row = 0;
	_ui->_contactTable->setSortingEnabled(false);
	for (std::size_t g = 0; g < 4; g++) {
		BOOST_FOREACH(const std::size_t i, contactsToShow[g]) {
			const Contact& c = contacts[g][i];
			const std::string& nameA = c.getNameA();
			const std::string& nameB = c.getNameB();
			const QString hover = toQString(c);
			// Note: setItem takes ownership of the QTableWidgetItems
			QTableWidgetItem* itemA;
			QTableWidgetItem* itemB;
			QTableWidgetItem* itemC;
			if (nameA < nameB) {
				itemA = new QTableWidgetItem(QString::fromStdString(nameA));
				itemB = new QTableWidgetItem(QString::fromStdString(nameB));
			} else {
				itemA = new QTableWidgetItem(QString::fromStdString(nameB));
				itemB = new QTableWidgetItem(QString::fromStdString(nameA));
			}
			if (g < 2)
				itemC = new QTableWidgetItem("Before+After");
			else if (g == 2)
				itemC = new QTableWidgetItem("Added");
			else
				itemC = new QTableWidgetItem("Gone");
			if (g < 2) {
				QLinearGradient grad(0,0,0,20);
				grad.setSpread(QLinearGradient::PadSpread);
				if (_ui->_before->isEnabled())
					grad.setColorAt(0,QColor(Qt::red));
				else
					grad.setColorAt(0,QColor(Qt::green));
				if (_ui->_after->isEnabled())
					grad.setColorAt(1.0,QColor(Qt::green));
				else
					grad.setColorAt(1.0,QColor(Qt::red));
				QBrush background(grad);
				itemA->setData(Qt::BackgroundRole,background);
				itemB->setData(Qt::BackgroundRole,background);
				itemC->setData(Qt::BackgroundRole,background);
			} else if (g == 2) {
				itemA->setData(Qt::ForegroundRole,Qt::blue);
				itemB->setData(Qt::ForegroundRole,Qt::blue);
				itemC->setData(Qt::ForegroundRole,Qt::blue);
			} else {
				//itemA->setData(Qt::BackgroundRole,Qt::white);
				//itemB->setData(Qt::BackgroundRole,Qt::white);
				//itemC->setData(Qt::BackgroundRole,Qt::white);
			}
			itemA->setData(Qt::ToolTipRole,hover);
			itemB->setData(Qt::ToolTipRole,hover);
			itemC->setData(Qt::ToolTipRole,hover);
			itemA->setData(Qt::UserRole,QVariant::fromValue(g));
			itemB->setData(Qt::UserRole,QVariant::fromValue(i));
			_ui->_contactTable->setItem(row,0,itemA);
			_ui->_contactTable->setItem(row,1,itemB);
			_ui->_contactTable->setItem(row,2,itemC);
			row++;
		}
	}
	_ui->_contactTable->setSortingEnabled(true);
	if (contactsToShow.size() > 0)
		_ui->_contactTable->setRangeSelected(QTableWidgetSelectionRange(0,0,contactsToShow[0].size()+contactsToShow[2].size()+contactsToShow[3].size()-1,2),true);
}

void RWPELogContactTrackingWidget::contactSetChanged(const QItemSelection&, const QItemSelection&) {
	const QModelIndexList indexes = _ui->_contactTable->selectionModel()->selectedIndexes();
	std::vector<std::vector<Contact> > contactSets(4);
	contactSets[0] = _tracking->before;
	contactSets[1] = _tracking->after;
	contactSets[2] = _tracking->added;
	contactSets[3] = _tracking->gone;
	std::map<std::size_t,std::pair<std::size_t,std::size_t> > selMap;
	std::vector<std::vector<Contact> > contacts(4);
	foreach (QModelIndex index, indexes) {
		if (index.column() == 0) {
			const std::size_t g = index.data(Qt::UserRole).toUInt();
			selMap[index.row()].first = g;
		} else if (index.column() == 1) {
			const std::size_t i = index.data(Qt::UserRole).toUInt();
			selMap[index.row()].second = i;
		}
	}
	std::map<std::size_t,std::pair<std::size_t,std::size_t> >::const_iterator it;
	for (it = selMap.begin(); it != selMap.end(); it++) {
		const std::size_t g = it->second.first;
		const std::size_t i = it->second.second;
		contacts[g].push_back(contactSets[g][i]);
		if (g == 0)
			contacts[1].push_back(contactSets[1][i]);
	}
	_root->removeChild("RWPEContactTracking");
	GroupNode::Ptr contactGroup = ownedPtr(new GroupNode("RWPEContactTracking"));
	for (std::size_t g = 0; g < 4; g++) {
		if (contacts[g].size() == 0)
			continue;
		if (g == 0 && !_ui->_before->isChecked())
			continue;
		else if (g == 1 && !_ui->_after->isChecked())
			continue;
		const RenderPointCloud::Ptr render = ownedPtr(new RenderPointCloud());
		const RenderLines::Ptr renderLines = ownedPtr(new RenderLines());
		BOOST_FOREACH(const Contact& c, contacts[g]) {
			render->addPoint(c.getPointA());
			render->addPoint(c.getPointB());
			renderLines->addLine(c.getPointA(),c.getPointB());
		}
		render->setPointSize(10);
		render->setColor(g==0?1:0,g==1?1:0,g==2?1:0,1);
		std::stringstream name;
		name << "Group" << g;
		const DrawableNode::Ptr drawable = _graph->makeDrawable(name.str(),render,DrawableNode::Physical);
		name << "Lines";
		const DrawableNode::Ptr drawableLines = _graph->makeDrawable(name.str(),renderLines,DrawableNode::Physical);
		GroupNode::addChild(drawable,contactGroup);
		GroupNode::addChild(drawableLines,contactGroup);
		drawable->setVisible(true);
		drawableLines->setVisible(true);
	}
	if (contacts[0].size() > 0 && _ui->_before->isChecked() && _ui->_after->isChecked()) {
		const RenderLines::Ptr renderLines = ownedPtr(new RenderLines());
		for (std::size_t i = 0; i < contacts[0].size(); i++) {
			const Vector3D<> p1 = (contacts[0][i].getPointA()+contacts[0][i].getPointB())/2.;
			const Vector3D<> p2 = (contacts[1][i].getPointA()+contacts[1][i].getPointB())/2.;
			renderLines->addLine(p1,p2);
		}
		const DrawableNode::Ptr drawableLines = _graph->makeDrawable("LinesTracked",renderLines,DrawableNode::Physical);
		GroupNode::addChild(drawableLines,contactGroup);
		drawableLines->setVisible(true);
	}
	GroupNode::addChild(contactGroup, _root);
	emit graphicsUpdated();
}

QString RWPELogContactTrackingWidget::toQString(const Vector3D<>& vec) {
	std::stringstream str;
	str << vec;
	return QString::fromStdString(str.str());
}

QString RWPELogContactTrackingWidget::toQString(const Contact& contact) {
	const QString nameA = QString::fromStdString(contact.getNameA());
	const QString nameB = QString::fromStdString(contact.getNameB());
	return "Bodies: " + nameA + " - " + nameB + "<br/>"
			+ "Points: " + toQString(contact.getPointA()) + " - " + toQString(contact.getPointB()) + "<br/>"
			+ "Normal: " + toQString(contact.getNormal()) + "<br/>"
			+ "Depth: " + QString::number(contact.getDepth());
}

RWPELogContactTrackingWidget::Dispatcher::Dispatcher() {
}

RWPELogContactTrackingWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* RWPELogContactTrackingWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	const rw::common::Ptr<const RWPELogContactTracking> tracking = entry.cast<const RWPELogContactTracking>();
	if (!tracking.isNull())
		return new RWPELogContactTrackingWidget(tracking, parent);
	RW_THROW("RWPELogContactTrackingWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool RWPELogContactTrackingWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!entry.cast<const RWPELogContactTracking>().isNull())
		return true;
	return false;
}
