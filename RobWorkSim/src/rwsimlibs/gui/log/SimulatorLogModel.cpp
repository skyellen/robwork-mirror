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

#include "SimulatorLogModel.hpp"

#include <rwsim/log/LogStep.hpp>
#include <rwsim/log/SimulatorLog.hpp>
#include <rwsim/log/SimulatorLogScope.hpp>
#include <rwsim/log/SimulatorLogEntry.hpp>

using namespace rwsim::log;
using namespace rwsimlibs::gui;

SimulatorLogModel::SimulatorLogModel(QObject *parent):
	QAbstractItemModel(parent),
	_root(NULL)
{
}

SimulatorLogModel::~SimulatorLogModel() {
}

void SimulatorLogModel::setRoot(rw::common::Ptr<const SimulatorLog> root) {
	_root = root;
	beginResetModel();
}

void SimulatorLogModel::compare(rw::common::Ptr<const SimulatorLog> info) {
	compare(_root,info);
}

void SimulatorLogModel::compare(rw::common::Ptr<const SimulatorLog> a, rw::common::Ptr<const SimulatorLog> b) {
	const rw::common::Ptr<const SimulatorLogScope> aScope = a.cast<const SimulatorLogScope>();
	const rw::common::Ptr<const SimulatorLogScope> bScope = b.cast<const SimulatorLogScope>();
	if (*a != *b)
		_bgColor[a.get()] = QColor(255,0,0);
	if (aScope.isNull())
		return;
	if (!bScope.isNull()) {
		const std::vector<SimulatorLog::Ptr> aChildren = aScope->getChildren();
		const std::vector<SimulatorLog::Ptr> bChildren = bScope->getChildren();
		for (std::size_t i = 0; i < std::min(aChildren.size(),bChildren.size()); i++) {
			compare(aChildren[i],bChildren[i]);
		}
		if (aChildren.size() > bChildren.size()) {
			for (std::size_t i = bChildren.size(); i < aChildren.size(); i++) {
				compareFailSubTree(aChildren[i]);
			}
		}
	} else {
		compareFailSubTree(aScope);
	}
}

void SimulatorLogModel::compareFailSubTree(rw::common::Ptr<const SimulatorLog> a) {
	_bgColor[a.get()] = QColor(255,0,0);
	const rw::common::Ptr<const SimulatorLogScope> aScope = a.cast<const SimulatorLogScope>();
	if (!aScope.isNull()) {
		const std::vector<SimulatorLog::Ptr> aChildren = aScope->getChildren();
		BOOST_FOREACH(const SimulatorLog::Ptr child, aChildren) {
			compareFailSubTree(child);
		}
	}
}

int SimulatorLogModel::rowCount(const QModelIndex &parent) const {
	if (parent.column() > 0) {
		return 0;
	}
	const SimulatorLog* const parentNode = nodeFromIndex(parent);
	if (!parentNode) {
		return 0;
	}
	const int nrOfChildren = static_cast<int>(parentNode->children());
	if (parentNode->children() > static_cast<std::size_t>(nrOfChildren))
		RW_THROW("There are too many children for the log model to handle!");
	return nrOfChildren;
}

int SimulatorLogModel::columnCount(const QModelIndex &parent) const {
	return 2;
}

QVariant SimulatorLogModel::data(const QModelIndex &index, int role) const {
	const SimulatorLog* const node = nodeFromIndex(index);
	if (!node)
		return QVariant();
	if (role == Qt::DisplayRole) {
		if (index.column() == 0) {
			return QString::fromStdString(node->getDescription());
		} else if (index.column() == 1) {
			return QString::number(node->children());
		}
	} else if (role == Qt::ToolTipRole) {
		return QString::fromStdString(node->getDescription());
	} else if (role == Qt::BackgroundColorRole) {
		std::map<const SimulatorLog*, QColor>::const_iterator it = _bgColor.find(node);
		if (it != _bgColor.end())
			return it->second;
		else
			return QVariant();
	}
	return QVariant();
}

const SimulatorLog* SimulatorLogModel::nodeFromIndex(const QModelIndex &index) const {
	if (_root == NULL)
		return NULL;
	if (index.isValid()) {
		return static_cast<const SimulatorLog*>(index.internalPointer());
	} else {
		return _root.get();
	}
}

QModelIndex SimulatorLogModel::index(int row, int column, const QModelIndex &parent) const {
	if (row < 0 || column < 0)
		return QModelIndex();
	const SimulatorLog* const parentNode = nodeFromIndex(parent);
	if (parentNode == NULL)
		return QModelIndex();
	const SimulatorLogScope* const parentComposite = dynamic_cast<const SimulatorLogScope*>(parentNode);
	if (parentComposite == NULL)
		return QModelIndex();

	RW_ASSERT((std::size_t)row < parentComposite->children());
	const SimulatorLog::Ptr childNode = parentComposite->getChild((std::size_t)row);
	if (childNode == NULL)
		return QModelIndex();
	return createIndex(row, column, childNode.get());

}

QModelIndex SimulatorLogModel::parent(const QModelIndex &child) const {
	const SimulatorLog* const node = nodeFromIndex(child);
	if (!node) {
		return QModelIndex();
	}
	const SimulatorLogScope* parentNode = node->getParent();
	if (parentNode == NULL) {
		return QModelIndex();
	}
	const SimulatorLogScope* grandparentNode = parentNode->getParent();
	if (grandparentNode == NULL) {
		return QModelIndex();
	}

	const std::size_t row = grandparentNode->indexOf(parentNode);
	if (row > grandparentNode->children()) {
		return QModelIndex();
	}
	return createIndex((int)row, 0, (void*)parentNode);
}

void SimulatorLogModel::update() {
	beginResetModel();
}

bool SimulatorLogModel::setData(const QModelIndex& index, const QVariant& value, int role) {
	if (role != Qt::BackgroundColorRole || !index.isValid())
		return false;
	const SimulatorLog* const entry = static_cast<const SimulatorLog*>(index.internalPointer());
	if (value.isValid()) {
		_bgColor[entry] = value.value<QColor>();
	} else {
		//_bgColor.erase(entry);
	}
	return true;
}

QVariant SimulatorLogModel::headerData(int section, Qt::Orientation orientation, int role) const {
	if (role == Qt::TextAlignmentRole) {
		return Qt::AlignCenter + Qt::AlignVCenter;
	} else if (role == Qt::DisplayRole) {
		if (orientation != Qt::Horizontal)
			return QVariant();
		if (section == 0)
			return "Entry Name";
		else if (section == 1)
			return "Description";
		else
			return "";
	} else {
		return QVariant();
	}
}
