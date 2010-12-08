/*
 * PropertyViewDialog.hpp
 *
 *  Created on: 01/09/2010
 *      Author: jimali
 */

#ifndef PROPERTYVIEWEDITOR_HPP_
#define PROPERTYVIEWEDITOR_HPP_


#include "ui_PropertyViewEditor.h"
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
    QtProperty* update(rw::common::PropertyMap *map, std::string propname);

private:
    QtVariantEditorFactory *_variantFactory;
    QtVariantPropertyManager *_variantManager;
    rw::common::PropertyMap::Ptr _map;
	std::map<QtProperty*, rw::common::PropertyBase::Ptr > _qtPropToRwProp;
    std::map<QtProperty*, rw::common::Ptr< rw::common::PropertyMap > > _qtPropToRwPropMap;
};

#endif /* PROPERTYVIEWDIALOG_HPP_ */
