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


#ifndef RWSTUDIO_COMPONENTS_PROPERTYINSPECTOR_HPP
#define RWSTUDIO_COMPONENTS_PROPERTYINSPECTOR_HPP

#include <rw/common/PropertyMap.hpp>
#include <rw/common/PropertyBase.hpp>

#include <QTableWidget>

/**
 * @brief a widget which display the content of a propertymap in a QTableWidget.
 *
 * Only a specified set of types can be edited.
 */
class PropertyInspector: public QTableWidget {
Q_OBJECT
public:
    //! @brief constructor
    PropertyInspector(QWidget* parent = NULL);
    virtual ~PropertyInspector();

    /**
     * @brief set the propertymap to display/edit
     * @param propertyMap
     */
    void setPropertyMap(rw::common::PropertyMap* propertyMap);

private slots:
	//TODO find the right arguments
    void propertyChanged(QTableWidgetItem* item);

signals:
	void propertyChanged(const std::string& identifier);

private:
	typedef std::vector<rw::common::PropertyBase::Ptr> PropertyList;
	void updateTable();

private:
    rw::common::PropertyMap* _propertyMap;
	bool _updating;

};


#endif //#ifndef RWSTUDIO_COMPONENTS_PROPERTYINSPECTOR_HPP
