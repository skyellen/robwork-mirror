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



#pragma once

/* standard includes */
#include <QObject>
#include <QtGui>
#include <QTimer>

/* RobWork includes */
#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

/* ROS includes */
//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "geometry_msgs/TransformStamped.h"

/* local includes */
#include "ui_PointerPlugin.h"



namespace rws {



/**
 * @brief The Pointer plugin introduces a movable frame that serves as a 3D pointer in the RWS environment. The plugin utilizes event system of
 * the RobWork Studio to inform all listening plugins of a new pointer position, and whether an user induced action has occured at the position.
 * The plugin also listens for requests to change pointer position and updates the information accordingly, resending the event.
 * It is possible to jog the pointer in several modes: global, tool view, and in the current view coordinates.
 */
class PointerPlugin : public rws::RobWorkStudioPlugin
{
	Q_OBJECT
	Q_INTERFACES(rws::RobWorkStudioPlugin)
	
	public:
		/**
		 * @brief Constructor
		 */
		PointerPlugin();
		
		/**
		 * @brief Destructor
		 */
		virtual ~PointerPlugin();

		/**
		 * @copydoc RobWorkStudioPlugin::open
		 */
		virtual void open(rw::models::WorkCell* workcell);
		
		/**
		 * @copydoc RobWorkStudioPlugin::close
		 */
		virtual void close();
		
		/**
		 * @copydoc RobWorkStudioPlugin::initialize
		 */
		virtual void initialize();
		
		/* RW event handlers */
		/**
		 * @brief Listen for keyboard events
		 */
		void keyEventListener(int key, Qt::KeyboardModifiers modifier);
		
		/**
		 * @brief Listen for generic events
		 * - ChangePointerPosition event makes the plugin update the pointer\s position according to Transform3D stored in NewPointerPosition property in RWS
		 */
		void genericEventListener(const std::string& event);
		
		/* ROS callbacks */
		//void handlePositionMessage(const geometry_msgs::TransformStamped::ConstPtr& message);
		
	private slots:
		/**
		 * @brief Updates the RWS scene with new pointer position and emits PointerPositionChanged event
		 */
		void update();
		
		void guiEvent();
		void guiEvent(int);
		
		/**
		 * @brief Emits PointerAction1 event
		 */
		virtual void action1();
		
		/**
		 * @brief Emits PointerAction2 event
		 */
		virtual void action2();
		
	private:
		/* defines */
		enum MovingModes { WorldMode, ViewMode, ToolMode, RotationMode };
		
		/* ROS stuff */
		//ros::NodeHandle* _rosNode;
		//ros::Subscriber _rosSubPosition;
		//ros::Publisher _rosPubPosition;
		//ros::Publisher _sender;
		
		/* GUI */
		//QTimer* _timer;
		Ui::PointerPluginWidget ui;
		
		/* RobWork stuff */
		rw::models::WorkCell::Ptr _wc;
		rw::kinematics::State _state;
		rw::kinematics::MovableFrame::Ptr _pointerFrame;
		
		MovingModes _mode;
		bool _showPointer;
		bool _enableKeyboard;
		bool _lockView;
		double _velocityExponent;
		double _keyVelocityMultiplier;
		double _keyAngularVelocityMultiplier;
		rw::math::Transform3D<> _pose;
		rw::math::Transform3D<> _savedPose;
		//rw::math::Vector3D<> _position;
		//rw::math::RPY<> _rpy;
};



} // end rws namespace
