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

#include "ContactTableWidget.hpp"

#include <rw/graphics/SceneGraph.hpp>
#include <rwsim/contacts/Contact.hpp>
#include <rwsim/contacts/RenderContacts.hpp>

#include <boost/foreach.hpp>

#include <QHeaderView>

QT_BEGIN_NAMESPACE

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::math;
using namespace rwsim::contacts;

ContactTableWidget::ContactTableWidget(QWidget* parent):
	QTableWidget(parent)
{
	setColumnCount(3);
	QStringList headerLabels;
	headerLabels.clear();
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Depth");
	setHorizontalHeaderLabels(headerLabels);
	setEditTriggers(QAbstractItemView::NoEditTriggers);
	setSelectionBehavior(QAbstractItemView::SelectRows);
	verticalHeader()->setVisible(false);
	verticalHeader()->setDefaultSectionSize(20);
	horizontalHeader()->setSortIndicatorShown(true);
	horizontalHeader()->setStretchLastSection(true);

	connect(selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(contactSetChanged(const QItemSelection &, const QItemSelection &)));
}

ContactTableWidget::~ContactTableWidget() {
}

void ContactTableWidget::setContacts(const std::vector<Contact>& contacts) {
	_contacts = contacts;
	clearSelection();
	setRowCount(_contacts.size());
	setSortingEnabled(false);
	int row = 0;
	BOOST_FOREACH(const Contact& c, _contacts) {
		const std::string& nameA = c.getNameA();
		const std::string& nameB = c.getNameB();
		const QString hover = toQString(c);
		// Note: setItem takes ownership of the QTableWidgetItems
		QTableWidgetItem* itemA;
		QTableWidgetItem* itemB;
		QTableWidgetItem* itemC = new QTableWidgetItem();
		if (nameA < nameB) {
			itemA = new QTableWidgetItem(QString::fromStdString(nameA));
			itemB = new QTableWidgetItem(QString::fromStdString(nameB));
		} else {
			itemA = new QTableWidgetItem(QString::fromStdString(nameB));
			itemB = new QTableWidgetItem(QString::fromStdString(nameA));
		}
		itemC->setData(Qt::DisplayRole,c.getDepth());
		itemA->setData(Qt::ToolTipRole,hover);
		itemB->setData(Qt::ToolTipRole,hover);
		itemC->setData(Qt::ToolTipRole,hover);
		if (c.getDepth() > 0) {
			itemA->setData(Qt::ForegroundRole, QColor(Qt::red));
			itemB->setData(Qt::ForegroundRole, QColor(Qt::red));
			itemC->setData(Qt::ForegroundRole, QColor(Qt::red));
		} else {
			itemA->setData(Qt::ForegroundRole, QColor(Qt::green));
			itemB->setData(Qt::ForegroundRole, QColor(Qt::green));
			itemC->setData(Qt::ForegroundRole, QColor(Qt::green));
		}
		itemA->setData(Qt::UserRole,QVariant::fromValue(row));
		setItem(row,0,itemA);
		setItem(row,1,itemB);
		setItem(row,2,itemC);
		row++;
	}
	setSortingEnabled(true);
}

void ContactTableWidget::showGraphics(rw::common::Ptr<GroupNode> root, rw::common::Ptr<SceneGraph> graph) {
	_root = root;
	_graph = graph;
}


void ContactTableWidget::selectAll() {
	if (_contacts.size() > 0)
		setRangeSelected(QTableWidgetSelectionRange(0,0,_contacts.size()-1,2),true);
}

void ContactTableWidget::clearContents() {
	QTableWidget::clearContents();
	setRowCount(0);
	if (!_root.isNull()) {
		_root->removeChild("ContactsPenetration");
		_root->removeChild("ContactsNoPenetration");
	}
}

void ContactTableWidget::contactSetChanged(const QItemSelection&, const QItemSelection&) {
	if (_root.isNull())
		return;
	const QModelIndexList indexes = selectionModel()->selectedIndexes();
	std::vector<Contact> contactsPen;
	std::vector<Contact> contactsNon;
	foreach (QModelIndex index, indexes) {
		if (index.column() > 0)
			continue;
		const std::size_t i = index.data(Qt::UserRole).toUInt();
		const Contact& c = _contacts[i];
		if (c.getDepth() > 0)
			contactsPen.push_back(c);
		else
			contactsNon.push_back(c);
	}
	_root->removeChild("ContactsPenetration");
	_root->removeChild("ContactsNoPenetration");
	const RenderContacts::Ptr renderPen = ownedPtr(new RenderContacts());
	const RenderContacts::Ptr renderNon = ownedPtr(new RenderContacts());
	renderPen->setColorPoints(1,0,0);
	renderNon->setColorPoints(0,1,0);
	renderPen->setContacts(contactsPen);
	renderNon->setContacts(contactsNon);
	const DrawableNode::Ptr drawablePen = _graph->makeDrawable("ContactsPenetration",renderPen,DrawableNode::Physical);
	const DrawableNode::Ptr drawableNon = _graph->makeDrawable("ContactsNoPenetration",renderNon,DrawableNode::Physical);
	GroupNode::addChild(drawablePen,_root);
	GroupNode::addChild(drawableNon,_root);
	drawablePen->setVisible(true);
	drawableNon->setVisible(true);

	emit graphicsUpdated();
}

QString ContactTableWidget::toQString(const Vector3D<>& vec) {
	std::stringstream str;
	str << vec;
	return QString::fromStdString(str.str());
}

QString ContactTableWidget::toQString(const Contact& contact) {
	const QString nameA = QString::fromStdString(contact.getNameA());
	const QString nameB = QString::fromStdString(contact.getNameB());
	return "Bodies: " + nameA + " - " + nameB + "<br/>"
			+ "Points: " + toQString(contact.getPointA()) + " - " + toQString(contact.getPointB()) + "<br/>"
			+ "Normal: " + toQString(contact.getNormal()) + "<br/>"
			+ "Depth: " + QString::number(contact.getDepth());
}

QT_END_NAMESPACE
