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



#ifndef PROPERTYVIEWEDITOR_HPP_
#define PROPERTYVIEWEDITOR_HPP_


//#include "ui_PropertyViewEditor.h"
#include <QtTreePropertyBrowser>
#include <QtVariantEditorFactory>
#include <QtVariantPropertyManager>
#include <rw/common/Log.hpp>
#include <rw/common/PropertyMap.hpp>

#include <QMainWindow>
#include <QCloseEvent>
#include <QSettings>

class PropertyViewEditor : public QtTreePropertyBrowser {
Q_OBJECT
public:

    PropertyViewEditor(QWidget *parent);

    virtual ~PropertyViewEditor();

    /**
     * @brief set the propertymap and update it.
     * @param map
     */
    void setPropertyMap(rw::common::PropertyMap::Ptr map){
        _map = map;
        update();
    }
 
    /**
     * @brief updates the propertyviewer with its propertymap
     */
    void update();

signals:
    /**
     * @brief if a property is changed then its identifier is signalled.
     */
    void propertyChanged(const std::string& identifier);

private slots:
    //TODO find the right arguments
    // void propertyChanged(QTableWidgetItem* item);
private slots:
    void slotValueChanged(QtProperty *property, const QVariant &value);
    void slotPropertyDestroyed(QtProperty *property);

private:
	QtProperty* update(rw::common::PropertyMap::Ptr map, std::string propname);

private:
    QtVariantEditorFactory *_variantFactory;
    QtVariantPropertyManager *_variantManager;
    rw::common::PropertyMap::Ptr _map;
	std::map<QtProperty*, rw::common::PropertyBase::Ptr > _qtPropToRwProp;
    std::map<QtProperty*, rw::common::Ptr< rw::common::PropertyMap > > _qtPropToRwPropMap;
};

#endif /* PROPERTYVIEWDIALOG_HPP_ */
