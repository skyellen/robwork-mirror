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

#ifndef PROPERTYVIEWDIALOG_HPP_
#define PROPERTYVIEWDIALOG_HPP_

#include <rw/common/PropertyMap.hpp>

#include <QDialog>

class Ui_PropertyViewDialog;

//! @brief Qt dialog for showing properties in a PropertyMap.
class PropertyViewDialog : public QDialog {
Q_OBJECT
public:
	/**
	 * Make new dialog.
	 * @param map [in] the property map to show.
	 * @param parent [in] the parent widget that owns the dialog.
	 */
	PropertyViewDialog(rw::common::PropertyMap::Ptr map, QWidget *parent);

private slots:
	void acceptPressed();
	void rejectPressed();
private:
	class Ui_PropertyViewDialog *ui;
	rw::common::PropertyMap::Ptr _pOriginalProperties;
	rw::common::PropertyMap _workingCopy;
};

#endif /* PROPERTYVIEWDIALOG_HPP_ */
