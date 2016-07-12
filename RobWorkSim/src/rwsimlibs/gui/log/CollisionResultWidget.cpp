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

#include "CollisionResultWidget.hpp"

#include "ui_CollisionResultWidget.h"

#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/graphics/GroupNode.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rwlibs/opengl/RenderGeometry.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/log/LogCollisionResult.hpp>
#include <rwsim/log/LogPositions.hpp>

#include <QItemSelection>

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::proximity;
using namespace rwlibs::opengl;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;

CollisionResultWidget::CollisionResultWidget(rw::common::Ptr<const LogCollisionResult> entry, QWidget* parent):
	SimulatorLogEntryWidget(parent),
	_ui(new Ui::CollisionResultWidget()),
	_positions(entry->getPositions()),
	_result(entry)
{
	_ui->setupUi(this);
	connect(_ui->_framePairTable->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(framePairsChanged(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_collisionPairs->selectionModel(),
			SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(collisionPairsChanged(const QItemSelection &, const QItemSelection &)));

	QStringList headerLabels;
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Collisions");
	headerLabels.push_back("Bounding Volume Tests");
	headerLabels.push_back("Primitive Tests");
	_ui->_framePairTable->setHorizontalHeaderLabels(headerLabels);

	_ui->_collisionPairs->setColumnCount(3);
	headerLabels.clear();
	headerLabels.push_back("First");
	headerLabels.push_back("Second");
	headerLabels.push_back("Primitive Pairs");
	_ui->_collisionPairs->setHorizontalHeaderLabels(headerLabels);
}

CollisionResultWidget::~CollisionResultWidget() {
	if (_root != NULL) {
		_root->removeChild("Collisions");
	}
}

void CollisionResultWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
	_dwc = dwc;
}

void CollisionResultWidget::setEntry(rw::common::Ptr<const SimulatorLog> entry) {
	const rw::common::Ptr<const LogCollisionResult> set = entry.cast<const LogCollisionResult>();
	if (!(set == NULL))
		_result = set;
	else
		RW_THROW("CollisionResultWidget (setEntry): invalid entry!");
}

rw::common::Ptr<const SimulatorLog> CollisionResultWidget::getEntry() const {
	return _result;
}

void CollisionResultWidget::updateEntryWidget() {
	const std::vector<LogCollisionResult::ResultInfo>& results = _result->getResults();
	_ui->_description->setText(QString::fromStdString(_result->getDescription()));
	int bvTests = 0;
	int primTests = 0;
	BOOST_FOREACH(const LogCollisionResult::ResultInfo& info, results) {
		bvTests += info.result._nrBVTests;
		primTests += info.result._nrPrimTests;
	}
	_ui->_bvTests->setText(QString::number(bvTests));
	_ui->_primTests->setText(QString::number(primTests));

	_ui->_framePairTable->setRowCount(results.size());
	_ui->_framePairTable->setSortingEnabled(false);

	int row = 0;
	BOOST_FOREACH(const LogCollisionResult::ResultInfo& info, results) {
		const std::string nameA = (info.frameA == "") ? "Unknown" : info.frameA;
		const std::string nameB = (info.frameB == "") ? "Unknown" : info.frameB;
		// Note: setItem takes ownership of the QTableWidgetItems
		if (nameA < nameB) {
			_ui->_framePairTable->setItem(row,0,new QTableWidgetItem(QString::fromStdString(nameA)));
			_ui->_framePairTable->setItem(row,1,new QTableWidgetItem(QString::fromStdString(nameB)));
		} else {
			_ui->_framePairTable->setItem(row,0,new QTableWidgetItem(QString::fromStdString(nameB)));
			_ui->_framePairTable->setItem(row,1,new QTableWidgetItem(QString::fromStdString(nameA)));
		}
		_ui->_framePairTable->setItem(row,2,new QTableWidgetItem(QString::number(info.result._collisionPairs.size())));
		_ui->_framePairTable->setItem(row,3,new QTableWidgetItem(QString::number(info.result._nrBVTests)));
		_ui->_framePairTable->setItem(row,4,new QTableWidgetItem(QString::number(info.result._nrPrimTests)));
		row++;
	}
	_ui->_framePairTable->setSortingEnabled(true);
	if (results.size() > 0)
		_ui->_framePairTable->setRangeSelected(QTableWidgetSelectionRange(0,0,results.size()-1,4),true);
}

void CollisionResultWidget::showGraphics(GroupNode::Ptr root, SceneGraph::Ptr graph) {
	if (root == NULL && _root != NULL)
		_root->removeChild("Collisions");
	_root = root;
	_graph = graph;
}

std::string CollisionResultWidget::getName() const {
	return "Collision Result";
}

void CollisionResultWidget::framePairsChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection) {
	const std::vector<LogCollisionResult::ResultInfo>& results = _result->getResults();
	const QModelIndexList indexes = _ui->_framePairTable->selectionModel()->selectedIndexes();
	int row = 0;
	_ui->_collisionPairs->setRowCount(0);
	_ui->_collisionPairs->setSortingEnabled(false);
	foreach (const QModelIndex& index, indexes) {
		if (index.column() > 0)
			continue;
		const LogCollisionResult::ResultInfo& info = results[(std::size_t)index.row()];
		_ui->_collisionPairs->setRowCount(_ui->_collisionPairs->rowCount()+info.result._collisionPairs.size());
		for (std::size_t i = 0; i < info.result._collisionPairs.size(); i++) {
			const std::string geoA = (info.geoNamesA[i] == "") ? "Unknown" : info.geoNamesA[i];
			const std::string geoB = (info.geoNamesB[i] == "") ? "Unknown" : info.geoNamesB[i];
			const CollisionStrategy::Result::CollisionPair& pair = info.result._collisionPairs[i];
			QTableWidgetItem* itemA;
			QTableWidgetItem* itemB;
			if (geoA < geoB) {
				itemA = new QTableWidgetItem(QString::fromStdString(geoA));
				itemB = new QTableWidgetItem(QString::fromStdString(geoB));
			} else {
				itemA = new QTableWidgetItem(QString::fromStdString(geoB));
				itemB = new QTableWidgetItem(QString::fromStdString(geoA));
			}
			itemA->setData(Qt::UserRole, index.row());
			itemA->setData(Qt::UserRole+1, (int)i);
			_ui->_collisionPairs->setItem(row,0,itemA);
			_ui->_collisionPairs->setItem(row,1,itemB);
			_ui->_collisionPairs->setItem(row,2,new QTableWidgetItem(QString::number(pair.size)));
			row++;
		}
	}
	_ui->_collisionPairs->setSortingEnabled(true);
	if (_ui->_collisionPairs->rowCount() > 0)
		_ui->_collisionPairs->setRangeSelected(QTableWidgetSelectionRange(0,0,_ui->_collisionPairs->rowCount()-1,_ui->_collisionPairs->columnCount()-1),true);
}

void CollisionResultWidget::collisionPairsChanged(const QItemSelection& newSelection, const QItemSelection& oldSelection) {
	if (_root == NULL || _dwc == NULL || _positions == NULL)
		return;

	const std::vector<LogCollisionResult::ResultInfo>& results = _result->getResults();

	PlainTriMesh<>::Ptr mesh = ownedPtr(new PlainTriMesh<>());

	const QModelIndexList indexes = _ui->_collisionPairs->selectionModel()->selectedIndexes();
	foreach(const QModelIndex& index, indexes) {
		if (index.column() > 0)
			continue;
		const std::size_t resultId = index.data(Qt::UserRole).toUInt();
		const std::size_t collPairId = index.data(Qt::UserRole+1).toUInt();

		const LogCollisionResult::ResultInfo& result = results[resultId];
		const CollisionStrategy::Result::CollisionPair& collPair = result.result._collisionPairs[collPairId];

		const int startId = collPair.startIdx;
		const int size = collPair.size;
		const int geoIdxA = collPair.geoIdxA;
		const int geoIdxB = collPair.geoIdxB;

		BOOST_FOREACH(const Object::Ptr object, _dwc->getWorkcell()->getObjects()) {
			BOOST_FOREACH(const Geometry::Ptr geo, object->getGeometry()) {
				if (geo->getFrame()->getName() == result.frameA && geo->getId() == result.geoNamesA[geoIdxA]) {
					const TriMesh::Ptr data = geo->getGeometryData()->getTriMesh();
					for (int i = startId; i < startId+size; i++) {
						const std::pair<int,int>& primIds = result.result._geomPrimIds[i];
						Transform3D<> T;
						if (_positions->has(result.frameA))
							T = _positions->getPosition(result.frameA);
						T = T*geo->getTransform();
						const Triangle<> tri = data->getTriangle(primIds.first).transform(T);
						mesh->add(tri);
					}
				} else if (geo->getFrame()->getName() == result.frameB && geo->getId() == result.geoNamesB[geoIdxB]) {
					const TriMesh::Ptr data = geo->getGeometryData()->getTriMesh();
					for (int i = startId; i < startId+size; i++) {
						const std::pair<int,int>& primIds = result.result._geomPrimIds[i];
						Transform3D<> T;
						if (_positions->has(result.frameB))
							T = _positions->getPosition(result.frameB);
						T = T*geo->getTransform();
						const Triangle<> tri = data->getTriangle(primIds.second).transform(T);
						mesh->add(tri);
					}
				}
			}
		}
	}

	_root->removeChild("Collisions");
	GroupNode::Ptr contactGroup = ownedPtr(new GroupNode("Collisions"));
	const RenderGeometry::Ptr render = ownedPtr(new RenderGeometry(mesh));
	render->setColor(0,1,0);
	const DrawableNode::Ptr drawable = _graph->makeDrawable("Collisions",render,DrawableNode::Virtual);
	GroupNode::addChild(drawable,contactGroup);
	drawable->setVisible(true);

	GroupNode::addChild(contactGroup, _root);
	drawable->setVisible(true);

	emit graphicsUpdated();
}

CollisionResultWidget::Dispatcher::Dispatcher() {
}

CollisionResultWidget::Dispatcher::~Dispatcher() {
}

SimulatorLogEntryWidget* CollisionResultWidget::Dispatcher::makeWidget(rw::common::Ptr<const SimulatorLog> entry, QWidget* parent) const {
	const rw::common::Ptr<const LogCollisionResult> tentry = entry.cast<const LogCollisionResult>();
	if (!(tentry == NULL))
		return new CollisionResultWidget(tentry, parent);
	RW_THROW("CollisionResultWidget::Dispatcher (makeWidget): invalid entry!");
	return NULL;
}

bool CollisionResultWidget::Dispatcher::accepts(rw::common::Ptr<const SimulatorLog> entry) const {
	if (!(entry.cast<const LogCollisionResult>() == NULL))
		return true;
	return false;
}
