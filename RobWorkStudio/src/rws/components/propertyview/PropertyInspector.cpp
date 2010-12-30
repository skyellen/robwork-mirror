/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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


#include "PropertyInspector.hpp"

#include <QMessageBox>
#include <QHeaderView>

using namespace rw::common;


PropertyInspector::PropertyInspector(QWidget* parent):
    QTableWidget(0, 2, parent),
    _propertyMap(NULL),
    _updating(false)
{
	connect(this, SIGNAL(itemChanged(QTableWidgetItem*)), this, SLOT(propertyChanged(QTableWidgetItem*)));
	QStringList headers;
	headers<<"Identifier";
	headers<<"Value";
	setHorizontalHeaderLabels(headers);
	horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
	verticalHeader()->hide();
}

PropertyInspector::~PropertyInspector() {

}


void PropertyInspector::setPropertyMap(rw::common::PropertyMap* propertyMap) {
	_propertyMap = propertyMap;
    updateTable();
}


void PropertyInspector::updateTable() {
	_updating = true;
    if (_propertyMap == NULL)  {
    	clear();
        QStringList headers;
        headers<<"Identifier";
        headers<<"Value";
        setHorizontalHeaderLabels(headers);
        horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
    	_updating = false;
        return;
    }

    setRowCount(_propertyMap->size());

    PropertyList properties;
    properties.insert(
        properties.end(),
        _propertyMap->getProperties().first,
        _propertyMap->getProperties().second);

    size_t index = 0;
    for (PropertyList::iterator it = properties.begin(); it != properties.end(); ++it) {
        std::string identifier = (*it)->getIdentifier();
        if (identifier.at(0) == '#') {
        	removeRow(index);
        	continue;
        }

        QTableWidgetItem* idItem = new QTableWidgetItem();
        idItem->setText(identifier.c_str());
        idItem->setFlags(0);
        idItem->setBackgroundColor(palette().color(QPalette::Window));
        idItem->setForeground(palette().color(QPalette::Foreground));
        setItem(index, 0, idItem);

        QTableWidgetItem* item = new QTableWidgetItem((*it)->getType().getId());
        switch ((*it)->getType().getId()) {
        case PropertyType::String:
            item->setText(_propertyMap->get<std::string>(identifier).c_str());
            break;
        case PropertyType::Float:
            item->setText(QString::number(_propertyMap->get<float>(identifier)));
            break;
        case PropertyType::Double:
            item->setText(QString::number(_propertyMap->get<double>(identifier)));
            break;
        case PropertyType::Int:
        	item->setText(QString::number(_propertyMap->get<int>(identifier)));
        	break;
        case PropertyType::Bool:
        	item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
        	if (_propertyMap->get<bool>(identifier)) {
        		item->setCheckState(Qt::Checked);
        		item->setText("True");
        	}
        	else {
        		item->setCheckState(Qt::Unchecked);
        		item->setText("False");
        	}
        	break;
        default:
        	item->setFlags(0);
        	item->setText("-");
        }
        setItem(index, 1, item);

        index++;

    }

    resizeColumnsToContents();
    int clm1Width = columnWidth(0);
    int clm2Width = columnWidth(1);

    int totWidth = maximumViewportSize().width();
    if (totWidth-clm1Width > clm2Width)
        setColumnWidth(1, totWidth-clm1Width);

    _updating = false;
}


void PropertyInspector::propertyChanged(QTableWidgetItem* item) {
	if (_updating)
		return;

	int row = item->row();
	std::string identifier = this->item(row, 0)->text().toStdString();
	bool ok = false;
	switch (item->type()) {
	case PropertyType::String: {
		std::string val = item->text().toStdString();
		_propertyMap->set<std::string>(identifier, val);
		break;
	}
	case PropertyType::Float: {
		float val = item->text().toFloat(&ok);
		if (ok)
			_propertyMap->set<float>(identifier, val);
		else {
			QMessageBox::information(NULL, "Invalid type", "\""+item->text()+"\" is not a valid float", QMessageBox::Ok);
			item->setText(QString::number(_propertyMap->get<float>(identifier)));
		}
		break;
	}
	case PropertyType::Double: {
		double val = item->text().toDouble(&ok);
		if (ok)
			_propertyMap->set<double>(identifier, val);
		else {
			QMessageBox::information(NULL, "Invalid type", "\""+item->text()+"\" is not a valid double", QMessageBox::Ok);
			item->setText(QString::number(_propertyMap->get<double>(identifier)));
		}
		break;
	}
	case PropertyType::Int:{
		int val = item->text().toInt(&ok);
		if (ok)
			_propertyMap->set<int>(identifier, val);
		else {
			QMessageBox::information(NULL, "Invalid type", "\""+item->text()+"\" is not a valid integer", QMessageBox::Ok);
			item->setText(QString::number(_propertyMap->get<int>(identifier)));
		}
		break;
	}
	case PropertyType::Bool:
		if (item->checkState() == Qt::Checked) {
			_propertyMap->set<bool>(identifier, true);
			item->setText("True");
		} else {
			_propertyMap->set<bool>(identifier, false);
			item->setText("False");
		}
		break;
	}
	propertyChanged(identifier);
}


