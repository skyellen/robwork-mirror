/*
 * PropertyEditorWidget.hpp
 *
 *  Created on: 23/09/2010
 *      Author: jimali
 */

#ifndef PROPERTYEDITORWIDGET_HPP_
#define PROPERTYEDITORWIDGET_HPP_

class PropertyEditorWidget : public QTreeView {
    Q_OBJECT
public:

    typedef rw::common::Property* (*UserTypeCB)(const std::string& name, QObject* propertyObject, Property* parent);

    PropertyEditorWidget(QWidget* parent = 0);

    virtual ~PropertyEditorWidget(){};

    //! add a user defined property to this widget
    void addObject(QObject* propertyObject);
};

#endif /* PROPERTYEDITORWIDGET_HPP_ */
