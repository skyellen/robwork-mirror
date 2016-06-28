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

#include "SimulatorLogWidget.hpp"
#include "SimulatorLogModel.hpp"
#include "SimulatorLogEntryWidget.hpp"
#include "SimulatorStatisticsWidget.hpp"

#include <rw/geometry/Line.hpp>

#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rws/SceneOpenGLViewer.hpp>
#include <RobWorkStudioConfig.hpp>

#include "ui_SimulatorLogWidget.h"

#include <boost/filesystem/path.hpp>
#include <rwsim/log/SimulatorLog.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/log/SimulatorLogEntry.hpp>
#include <rwsim/log/SimulatorStatistics.hpp>

#include <iostream>
#include <queue>
#include "../../../rwsim/log/LogStep.hpp"

using namespace rw::common;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::math;
using namespace rwsim::dynamics;
using namespace rwsim::log;
using namespace rwsimlibs::gui;
using namespace rws;

SimulatorLogWidget::SimulatorLogWidget(QWidget* parent):
	QWidget(parent),
	_ui(new Ui::SimulatorLogWidget()),
	_dwc(NULL),
	_glview(new SceneOpenGLViewer(_ui->_viewFrame)),
	_log(NULL),
	_model(new SimulatorLogModel(this)),
	_root(ownedPtr(new GroupNode("World")))
{
	_ui->setupUi(this);

    QGridLayout* lay = new QGridLayout(_ui->_viewFrame);
    _ui->_viewFrame->setLayout(lay);
    lay->addWidget(_glview);
    _glview->setWorldNode(_root);

    // No background
    _glview->getPropertyMap().set<bool>("DrawBackGround", false);
    const Property<bool>::Ptr bgProp = _glview->getPropertyMap().findProperty<bool>("DrawBackGround");
    bgProp->setValue(false);
    bgProp->changedEvent().fire(bgProp.get());

    // Make floor grid
    const SceneGraph::Ptr scene = _glview->getScene();
    const DrawableGeometryNode::Ptr grid = scene->makeDrawable("FloorGrid",Line::makeGrid(10,10,0.5,0.5),DrawableNode::Virtual);
    grid->setColor(Vector3D<>(0.8f, 0.8f, 0.8f));
    GroupNode::addChild(grid,_root);
    grid->setVisible(true);

    _glview->updateView();

	// Left Tree Selector
	_ui->_tree->setModel(_model);
#if RWS_USE_QT5
	_ui->_tree->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
#else
	_ui->_tree->header()->setResizeMode(QHeaderView::ResizeToContents);
#endif
	connect(_ui->_tree->selectionModel(), SIGNAL(selectionChanged (const QItemSelection &, const QItemSelection &)),
			this, SLOT(selectionChanged(const QItemSelection &, const QItemSelection &)));
	connect(_ui->_tree->selectionModel(), SIGNAL(currentRowChanged (const QModelIndex &, const QModelIndex &)),
			this, SLOT(currentChanged(const QModelIndex &, const QModelIndex &)));
	connect(_ui->_tree, SIGNAL(collapsed(const QModelIndex&)),
			this, SLOT(collapsed(const QModelIndex&)));

	// Tabs
	_ui->_pageStack->clear();
	_ui->_pageStack->addTab(_ui->_noPage,"Instructions");
	_ui->_pageStack->setCurrentWidget(_ui->_noPage);

	connect(_ui->_pageStack, SIGNAL(tabCloseRequested(int)), this, SLOT(tabCloseRequested(int)));
}

SimulatorLogWidget::~SimulatorLogWidget() {
	delete _glview;
	delete _model;
}

void SimulatorLogWidget::setDWC(rw::common::Ptr<const DynamicWorkCell> dwc) {
	_ui->_tree->selectionModel()->clearSelection();
	_dwc = dwc;
}

void SimulatorLogWidget::setLog(rw::common::Ptr<SimulatorLogScope> log) {
	_ui->_tree->selectionModel()->clearSelection();
	_log = log;
	_model->setRoot(_log);
}

void SimulatorLogWidget::setSelectedTime(double time) {
	if (isVisible()) {
		int row = 0;
		BOOST_FOREACH(SimulatorLog::Ptr const entry, _log->getChildren()) {
			const LogStep::Ptr step = entry.cast<LogStep>();
			if (step != NULL) {
				if (time >= step->timeBegin() && time < step->timeEnd())
					break;
			}
			row++;
		}
		if (row >= (int)_log->children())
			row = (int)_log->children() - 1;
		_ui->_tree->selectionModel()->select(_model->index(row,0,QModelIndex()),QItemSelectionModel::ClearAndSelect | QItemSelectionModel::Current | QItemSelectionModel::Rows);
		_ui->_tree->scrollTo(_model->index(row,0,QModelIndex()));
	}
}

void SimulatorLogWidget::updateInfo() {
	_model->update();
}

#include <rwlibs/opengl/DrawableGeometry.hpp>
#include <rwlibs/opengl/RenderGeometry.hpp>
using namespace rwlibs::opengl;
void SimulatorLogWidget::updateOpenGLView() {
	const CameraController::Ptr controller = _glview->getCameraController();
	std::list<DrawableGeometry::Ptr> nodes;
	std::queue<GroupNode::Ptr> groups;
	groups.push(_root);
	while (groups.size() > 0) {
		const GroupNode::Ptr group = groups.front();
		groups.pop();
		BOOST_FOREACH(const SceneNode::Ptr node, group->_childNodes) {
			const GroupNode::Ptr groupNode = node.cast<GroupNode>();
			if (!groupNode.isNull()) {
				groups.push(groupNode);
			} else {
				const DrawableGeometry::Ptr realNode = node.cast<DrawableGeometry>();
				if (!realNode.isNull()) {
					//if (realNode->getType() == DrawableNode::Physical)
						nodes.push_back(realNode);
				}
			}
		}
	}
	std::list<RenderGeometry::Ptr> renders;
	BOOST_FOREACH(const DrawableGeometry::Ptr node, nodes) {
		BOOST_FOREACH(const Render::Ptr render, node->getRenders()) {
			if (const RenderGeometry::Ptr rg = render.cast<RenderGeometry>())
				renders.push_back(rg);
		}
	}
	const ProjectionMatrix proj = _glview->getViewCamera()->getProjectionMatrix();
	/*BOOST_FOREACH(const RenderGeometry::Ptr render, renders) {
		const Geometry::Ptr geo = render->getGeometry();
		const TriMesh::Ptr mesh = geo->getGeometryData()->getTriMesh();
		for (std::size_t i = 0; i < mesh->getSize(); i++) {
			Triangle<> tri;
			mesh->getTriangle(i,tri);
			const Vector3D<>& v1 = tri.getVertex(0);
			const Vector3D<>& v2 = tri.getVertex(1);
			const Vector3D<>& v3 = tri.getVertex(2);
		}
	}*/
    //Transform3D<> T = controller->getTransform();
    //Vector3D<> translateVector(0, 0, event->delta()/(240.0) );
    //T.P() -= T.R()*translateVector;
    //controller->setTransform(T);
	_glview->updateView();
}

void SimulatorLogWidget::selectionChanged(const QItemSelection& selected, const QItemSelection& deselected) {
	_ui->_pageStack->removeTab(_ui->_pageStack->indexOf(_ui->_noPage));
	// If deselected entries has an associated widget, this is deleted.
	foreach(const QModelIndex& index, deselected.indexes()) {
		const SimulatorLog* const entry = static_cast<const SimulatorLog*>(index.internalPointer());
		if (entry == NULL)
			continue;
		std::map<const SimulatorLog*, std::list<QWidget*> >::iterator it = _entryToWidgets.find(entry);
		if (it != _entryToWidgets.end()) {
			BOOST_FOREACH(QWidget* const widget, it->second) {
				_ui->_pageStack->removeTab(_ui->_pageStack->indexOf(widget));
				delete widget;
			}
			_entryToWidgets.erase(it);
		}
		// Remove color from dependent entries
		if (const SimulatorLogEntry* const leaf = dynamic_cast<const SimulatorLogEntry*>(entry)) {
			BOOST_FOREACH(const SimulatorLogEntry::Ptr dep, leaf->getLinkedEntries()) {
				QModelIndex search = index;
				bool found = false;
				while (search.isValid() && !found) {
					search = _ui->_tree->indexAbove(search);
					if (search.isValid()) {
						const SimulatorLog* const searchEntry = static_cast<const SimulatorLog*>(search.internalPointer());
						if (searchEntry == dep.get())
							found = true;
					}
				}
				if (found) {
					_model->setData(search, QVariant(), Qt::BackgroundColorRole);
					_ui->_tree->update(search);
					_ui->_tree->update(_model->index(search.row(),search.column()+1,search.parent()));
				}
			}
		}
	}
	bool added = false;
	// Now create new widgets for the selected entries
	foreach(const QModelIndex& index, selected.indexes()) {
		if (index.column() != 0)
			continue;
		SimulatorLog* const entry = static_cast<SimulatorLog*>(index.internalPointer());
		if (entry == NULL)
			continue;
		RW_ASSERT(_entryToWidgets.find(entry) == _entryToWidgets.end());
		SimulatorLogScope* const scope = dynamic_cast<SimulatorLogScope*>(entry);
		if (scope) {
			const rw::common::Ptr<const SimulatorStatistics> stats = scope->getStatistics();
			if (!stats.isNull()) {
				if (stats->getSeries().size() > 0) {
					SimulatorStatisticsWidget* const widget = new SimulatorStatisticsWidget(stats,_ui->_pageStack);
					_ui->_pageStack->addTab(widget,QString::fromStdString(widget->getName()));
					_entryToWidgets[entry].push_back(widget);
					added = true;
				}
			}
		} else {
			const std::list<SimulatorLogEntryWidget*> widgets = SimulatorLogEntryWidget::Factory::makeWidgets(entry,_ui->_pageStack);
			BOOST_FOREACH(SimulatorLogEntryWidget* const widget, widgets) {
				widget->setDWC(_dwc);
				widget->showGraphics(_root,_glview->getScene());
				widget->updateEntryWidget();
				_ui->_pageStack->addTab(widget,QString::fromStdString(widget->getName()));
				_entryToWidgets[entry].push_back(widget);
				connect(widget, SIGNAL(graphicsUpdated()), this, SLOT(updateOpenGLView()));
				added = true;
			}
		}
/*
		const PhysicsEngineInternalInfo::LeafEntry* const leaf = dynamic_cast<const PhysicsEngineInternalInfo::LeafEntry*>(entry);
		const PhysicsEngineInternalInfo::CompositeEntry* const composite = dynamic_cast<const PhysicsEngineInternalInfo::CompositeEntry*>(entry);
		if (leaf != NULL) {
			_ui->_lines->setText(QString::number(leaf->line));
			const PhysicsEngineInternalInfo::ContactSet* const typed = dynamic_cast<const PhysicsEngineInternalInfo::ContactSet*>(leaf);
			if (typed != NULL) {
				ContactSetWidget* const widget = new ContactSetWidget(_dwc,_ui->_pageStack);
				widget->setContactSet(typed);
				widget->showGraphics(_root,_glview->getScene());
				widget->updateContactSetWidget();
				_ui->_pageStack->addTab(widget,"Contact Set");
				_entryToWidget[entry] = widget;
				return;
			}
			const PhysicsEngineInternalInfo::Positions* const pos = dynamic_cast<const PhysicsEngineInternalInfo::Positions*>(leaf);
			const PhysicsEngineInternalInfo::Velocities* const vel = dynamic_cast<const PhysicsEngineInternalInfo::Velocities*>(leaf);
			if (pos != NULL) {
				BodyMotionWidget* const widget = new BodyMotionWidget(_dwc,_ui->_pageStack);
				widget->setPositions(pos);
				widget->showGraphics(_root,_glview->getScene());
				widget->updateBodyMotionWidget();
				_ui->_pageStack->addTab(widget,"Body Positions");
				_entryToWidget[entry] = widget;
				return;
			}
			if (vel != NULL) {
				BodyMotionWidget* const widget = new BodyMotionWidget(_dwc,_ui->_pageStack);
				widget->setVelocities(vel);
				widget->showGraphics(_root,_glview->getScene());
				widget->updateBodyMotionWidget();
				_ui->_pageStack->addTab(widget,"Body Velocities");
				_entryToWidget[entry] = widget;
				return;
			}
		} else if (composite != NULL) {
			_ui->_lines->setText(QString::number(composite->line.first) + " to " + QString::number(composite->line.second));
		}*/
	}
	if (added)
		_ui->_pageStack->setCurrentIndex(_ui->_pageStack->count()-1);
	// Color dependent entries
	foreach(const QModelIndex& index, _ui->_tree->selectionModel()->selectedIndexes()) {
		const SimulatorLog* const entry = static_cast<const SimulatorLog*>(index.internalPointer());
		if (const SimulatorLogEntry* const leaf = dynamic_cast<const SimulatorLogEntry*>(entry)) {
			BOOST_FOREACH(const SimulatorLogEntry::Ptr dep, leaf->getLinkedEntries()) {
				QModelIndex search = index;
				bool found = false;
				while (search.isValid() && !found) {
					search = _ui->_tree->indexAbove(search);
					if (search.isValid()) {
						const SimulatorLog* const searchEntry = static_cast<const SimulatorLog*>(search.internalPointer());
						if (searchEntry == dep.get())
							found = true;
					}
				}
				if (found) {
					_model->setData(search, QColor(240,240,255), Qt::BackgroundColorRole);
					_ui->_tree->update(search);
					_ui->_tree->update(_model->index(search.row(),search.column()+1,search.parent()));
				}
			}
		}
	}

	if (_entryToWidgets.size() == 0) {
		bool showNoPage = true;
		const rw::common::Ptr<const SimulatorStatistics> stats = _log->getStatistics();
		if (!stats.isNull()) {
			if (stats->getSeries().size() > 0) {
				SimulatorStatisticsWidget* const widget = new SimulatorStatisticsWidget(stats,_ui->_pageStack);
				_ui->_pageStack->addTab(widget,QString::fromStdString(widget->getName()));
				_entryToWidgets[_log.get()].push_back(widget);
				showNoPage = false;
			}
		}
		if (showNoPage) {
			_ui->_pageStack->addTab(_ui->_noPage,"Instructions");
			_ui->_pageStack->setCurrentWidget(_ui->_noPage);
		}
	}

	updateOpenGLView();
}

void SimulatorLogWidget::currentChanged(const QModelIndex& newSelection, const QModelIndex& oldSelection) {
	bool failed = true;
	if (newSelection.isValid()) {
		SimulatorLog* const entry = static_cast<SimulatorLog*>(newSelection.internalPointer());
		if (entry == NULL)
			return;
		RW_ASSERT(_entryToWidgets.find(entry) == _entryToWidgets.end());
		boost::filesystem::path p(entry->getFilename());
		_ui->_file->setText(p.filename().string().c_str());
		_ui->_file->setToolTip(QString::fromStdString(entry->getFilename()));

		const SimulatorLogScope* const scope = dynamic_cast<const SimulatorLogScope*>(entry);
		const SimulatorLogEntry* const leaf = dynamic_cast<const SimulatorLogEntry*>(entry);
		if (leaf)
			_ui->_lines->setText(QString::number(leaf->line()));
		else
			_ui->_lines->setText(QString::number(scope->lineBegin()) + " to " + QString::number(scope->lineEnd()));

		failed = false;
	}
	if (failed) {
		_ui->_file->setText("N/A");
		_ui->_file->setToolTip("");
		_ui->_lines->setText("N/A");
	}
}

void SimulatorLogWidget::collapsed(const QModelIndex& index) {
	const SimulatorLogScope* const scope = static_cast<const SimulatorLogScope*>(index.internalPointer());
	QItemSelection deselection;
	foreach(const QModelIndex& selIndex, _ui->_tree->selectionModel()->selectedIndexes()) {
		const SimulatorLog* const selectedEntry = static_cast<const SimulatorLog*>(selIndex.internalPointer());
		SimulatorLogScope::Ptr selScope = selectedEntry->getParent();
		while (!selScope.isNull()) {
			if (selScope.get() == scope) {
				deselection.append(QItemSelectionRange(selIndex));
				break;
			}
			selScope = selScope->getParent();
		}
	}
	_ui->_tree->selectionModel()->select(deselection,QItemSelectionModel::Deselect | QItemSelectionModel::Rows);
	// Remove color from subitems
	std::queue<QModelIndex> scopeIndices;
	scopeIndices.push(index);
	while (scopeIndices.size() > 0) {
		const QModelIndex& scopeIndex = scopeIndices.front();
		for (int i = 0; i < _model->rowCount(scopeIndex); i++) {
			const QModelIndex child = _model->index(i,0,scopeIndex);
			_model->setData(child, QVariant(), Qt::BackgroundColorRole);
			if (_model->rowCount(child) > 0)
				scopeIndices.push(child);
		}
		scopeIndices.pop();
	}
}

void SimulatorLogWidget::btnPressed() {
    //const QObject* const obj = sender();
    //if(obj == _ui->_contactRenderBtn){
    //}
}

void SimulatorLogWidget::tabCloseRequested(int index) {
	const QWidget* const widget = _ui->_pageStack->widget(index);
	const SimulatorLogEntryWidget* const ewidget = dynamic_cast<const SimulatorLogEntryWidget*>(widget);
	if (ewidget == NULL)
		return;
	const rw::common::Ptr<const SimulatorLog> entry = ewidget->getEntry();
	if (entry == NULL)
		return;
	QItemSelection deselection;
	foreach(const QModelIndex& index, _ui->_tree->selectionModel()->selectedIndexes()) {
		const SimulatorLog* const selectedEntry = static_cast<const SimulatorLog*>(index.internalPointer());
		if (entry.get() == selectedEntry) {
			deselection.append(QItemSelectionRange(index));
			break;
		}
	}
	_ui->_tree->selectionModel()->select(deselection,QItemSelectionModel::Deselect | QItemSelectionModel::Rows);
}
