/*
 * PropertyViewDialog.hpp
 *
 *  Created on: 01/09/2010
 *      Author: jimali
 */

#ifndef PROPERTYVIEWEDITOR_HPP_
#define PROPERTYVIEWEDITOR_HPP_


#include "ui_PropertyViewEditor.h"


class PropertyViewEditor : public QMainWindow, private Ui::PropertyViewEditor {
public:

    PropertyViewEditor(rw::common::LogPtr output, QWidget *parent);

private:
    PropertyInspector *_inspector;


};

#endif /* PROPERTYVIEWDIALOG_HPP_ */
