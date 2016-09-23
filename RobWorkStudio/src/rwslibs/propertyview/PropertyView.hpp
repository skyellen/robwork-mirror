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


#ifndef PROPERTYVIEW_HPP
#define PROPERTYVIEW_HPP

#include <string>

#include <rws/RobWorkStudioPlugin.hpp>

namespace rw { namespace kinematics { class Frame; } }

class PropertyViewEditor;

class QComboBox;

namespace rws {

//! @brief Plugin for editing properties.
class PropertyView: public RobWorkStudioPlugin {
Q_OBJECT
#ifndef RWS_USE_STATIC_LINK_PLUGINS
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
#endif
public:
	//! @brief Constructor.
    PropertyView();

	//! @brief Destructor.
    virtual ~PropertyView();

    //! @copydoc RobWorkStudioPlugin::initialize
    virtual void initialize();

    //! @copydoc RobWorkStudioPlugin::open
    virtual void open(rw::models::WorkCell* workcell);

    //! @copydoc RobWorkStudioPlugin::close
    virtual void close();

//protected:
	//void frameSelectedHandler(rw::kinematics::Frame* frame, RobWorkStudioPlugin* sender);
private slots:
    //void frameChanged(rw::kinematics::Frame* frame);

	void frameChanged(const QString& item);

	void propertyChanged(const std::string& identifier);

private:
    void frameSelectedListener(rw::kinematics::Frame* frame);

	void addFrame(const rw::kinematics::Frame* frame);

	bool _updating;

	rw::kinematics::State _state;
	rw::models::WorkCell* _workcell;

	QComboBox* _cmbFrames;
	PropertyViewEditor* _inspector;
};

}

#endif //#ifndef PROPERTYVIEW_HPP
