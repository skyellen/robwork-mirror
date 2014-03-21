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



#include "PointerPlugin.hpp"

#include <string>
#include <rws/RobWorkStudio.hpp>



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rws;



PointerPlugin::PointerPlugin() :
	RobWorkStudioPlugin("PointerPlugin", QIcon(":/pointer_icon.png")),
	_wc(NULL),
	_pointerFrame(NULL),
	_mode(WorldMode),
	_showPointer(false),
	_enableKeyboard(false),
	_lockView(false),
	_velocityExponent(1.0),
	_keyVelocityMultiplier(0.01),
	_keyAngularVelocityMultiplier(5.0*Deg2Rad)
{
	/* initialize ros */
	//int argc = 0;
	//ros::init(argc, NULL, "PointerPlugin");
	//_rosNode = new ros::NodeHandle("~");
	
	//_rosSubPosition = _rosNode->subscribe("/PointerPlugin/position_in", 1000, &PointerPlugin::handlePositionMessage, this);
	//_rosPubPosition = _rosNode->advertise<geometry_msgs::TransformStamped>("/PointerPlugin/position_out", 1000);
	//_receiver = _rosnode->subscribe("/RosPlugin/input", 1000, &RosPlugin::handleInput, this);
	//_sender = _rosnode->advertise<std_msgs::String>("/RosPlugin/output", 1000);
	
	/* setup GUI */
	ui.setupUi(this);
	
	connect(ui.originButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.saveButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.restoreButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.worldButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.viewButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.toolButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.rotationButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.showBox, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.keyboardBox, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.lockBox, SIGNAL(clicked()), this, SLOT(guiEvent()));
	connect(ui.speedSlider, SIGNAL(valueChanged(int)), this, SLOT(guiEvent(int)));
	
	//connect(ui.sendButton, SIGNAL(clicked()), this, SLOT(guiEvent()));
	
	/* initialize timer */
	//_timer = new QTimer(this);
    //connect(_timer, SIGNAL(timeout()), this, SLOT(update()));
    //_timer->start();
}



PointerPlugin::~PointerPlugin()
{
	//ros::shutdown();
}



void PointerPlugin::initialize()
{
	// set up a keyboard event handler
	getRobWorkStudio()->keyEvent().add(boost::bind(&PointerPlugin::keyEventListener, this, _1, _2), this);
	
	// set up a generic event handler
	getRobWorkStudio()->genericEvent().add(boost::bind(&PointerPlugin::genericEventListener, this, _1), this);
	
	// event is triggered by:
	// getRobWorkStudio()->genericEvent().fire("EVENT NAME");
	// to add data to property map:
	//getRobWorkStudio()->getPropertyMap().add<DynamicWorkCell::Ptr>(
    //        "DynamicWorkcell",
    //        "A workcell with dynamic description",
    //        _dwc );
}



void PointerPlugin::keyEventListener(int key, Qt::KeyboardModifiers modifier)
{
	// see if we should use kayboard at all...
	if (!_enableKeyboard) return;
	
	// let's get the RWS's view transform so we can walk in this frame...
	Transform3D<> viewT = getRobWorkStudio()->getViewTransform();
	Rotation3D<> viewR = viewT.R();
	
	// let's prepare direction vectors according to mode
	Vector3D<> forward, right, up;
	
	if (_mode == WorldMode) {
		forward = Vector3D<>(1.0, 0.0, 0.0);
		right = Vector3D<>(0.0, 1.0, 0.0);
		up = Vector3D<>(0.0, 0.0, 1.0);
	}
	else if (_mode == ViewMode) {
		forward = viewR * Vector3D<>(0.0, 1.0, 0.0);
		right = viewR * Vector3D<>(1.0, 0.0, 0.0);
		up = viewR * Vector3D<>(0.0, 0.0, -1.0);
	}
	else if (_mode == ToolMode) {
		forward = _pose.R() * Vector3D<>(1.0, 0.0, 0.0);
		right = _pose.R() * Vector3D<>(0.0, 1.0, 0.0);
		up = _pose.R() * Vector3D<>(0.0, 0.0, 1.0);
	}
	
	// scale vectors according to multiplier
	forward *= _keyVelocityMultiplier;
	right *= _keyVelocityMultiplier;
	up *= _keyVelocityMultiplier;
	
	bool change = false;
	
	switch (key) {
		case 'W':
			if (_mode == RotationMode || modifier & Qt::ShiftModifier) {
				_pose.R() = _pose.R() * RPY<>(0.0, -_keyAngularVelocityMultiplier, 0.0).toRotation3D();
			} else {
				_pose.P() += forward;
			}
			
			change = true;
			break;
			
		case 'S':
			if (_mode == RotationMode || modifier & Qt::ShiftModifier) {
				_pose.R() = _pose.R() * RPY<>(0.0, _keyAngularVelocityMultiplier, 0.0).toRotation3D();
			} else {
				_pose.P() += -forward;
			}
			
			change = true;
			break;
			
		case 'A':
			if (_mode == RotationMode || modifier & Qt::ShiftModifier) {
				_pose.R() = _pose.R() * RPY<>(0.0, 0.0, _keyAngularVelocityMultiplier).toRotation3D();
			} else {
				_pose.P() += -right;
			}
			
			change = true;
			break;
			
		case 'D':
			if (_mode == RotationMode || modifier & Qt::ShiftModifier) {
				_pose.R() = _pose.R() * RPY<>(0.0, 0.0, -_keyAngularVelocityMultiplier).toRotation3D();
			} else {
				_pose.P() += right;
			}
			
			change = true;
			break;
			
		case 'E':
			if (_mode == RotationMode || modifier & Qt::ShiftModifier) {
				_pose.R() = _pose.R() * RPY<>(_keyAngularVelocityMultiplier, 0.0, 0.0).toRotation3D();
			} else {
				_pose.P() += up;
			}
			
			change = true;
			break;
			
		case 'Q':
			if (_mode == RotationMode || modifier & Qt::ShiftModifier) {
				_pose.R() = _pose.R() * RPY<>(-_keyAngularVelocityMultiplier, 0.0, 0.0).toRotation3D();
			} else {
				_pose.P() += -up;
			}
			
			change = true;
			break;
			
		default:
			break;
	}
	
	if (change) {
		update();
	}
}



void PointerPlugin::genericEventListener(const std::string& event)
{
	// handle the position change request event
	if (event == "ChangePointerPosition") {
		_pose = getRobWorkStudio()->getPropertyMap().get<Transform3D<> >("NewPointerPosition", Transform3D<>());
		
		update();
	}
}



void PointerPlugin::open(rw::models::WorkCell* wc)
{
	_wc = wc;
	
	if (_wc) {
		// let's see if there's a pointer frame already defined
		//_pointerFrame = _wc->findFrame<MovableFrame>("Pointer");
		
		// if there isn't, we have to add it to the workcell
		//if (!_pointerFrame) {
		_pointerFrame = new MovableFrame("__Pointer");
		_wc->addFrame(_pointerFrame.get());
		//}
		
		_state = _wc->getDefaultState();
		getRobWorkStudio()->setState(_state);
		
		update();
	}
}



void PointerPlugin::close()
{
	_wc = NULL;
	_pointerFrame = NULL;
}



void PointerPlugin::guiEvent()
{
	QObject* obj = sender();
	
	if (obj == ui.originButton) {
		_pose = Transform3D<>();
		update();
	}
	
	else if (obj == ui.saveButton) {
		_savedPose = _pose;
	}
	
	else if (obj == ui.restoreButton) {
		_pose = _savedPose;
		update();
	}
	
	else if (obj == ui.worldButton) {
		_mode = WorldMode;
	}
	
	else if (obj == ui.viewButton) {
		_mode = ViewMode;
	}
	
	else if (obj == ui.toolButton) {
		_mode = ToolMode;
		update();
	}
	
	else if (obj == ui.rotationButton) {
		_mode = RotationMode;
		update();
	}
	
	else if (obj == ui.action1Button) {
		action1();
	}
	
	else if (obj == ui.action2Button) {
		action2();
	}
	
	else if (obj == ui.showBox) {
		_showPointer = ui.showBox->isChecked();
		update();
	}
	
	else if (obj == ui.keyboardBox) {
		_enableKeyboard = ui.keyboardBox->isChecked();
	}
	
	else if (obj == ui.lockBox) {
		_lockView = ui.lockBox->isChecked();
		update();
	}
}



void PointerPlugin::guiEvent(int value)
{
	QObject* obj = sender();
	
	if (obj == ui.speedSlider) {
		_velocityExponent = value;
		_keyVelocityMultiplier = 0.01 * pow(1.2, _velocityExponent);
		_keyAngularVelocityMultiplier = 3.0 * Deg2Rad * pow(1.2, _velocityExponent);
	}
}



void PointerPlugin::update()
{
	//ros::spinOnce();
	// make sure the pointer frame is set to visible
	WorkCellScene::Ptr scene = getRobWorkStudio()->getWorkCellScene();
	if (_pointerFrame) scene->setFrameAxisVisible(_showPointer, _pointerFrame.get());
	getRobWorkStudio()->updateAndRepaint();
	
	// update view
	if (_lockView && (_mode == ToolMode || _mode == RotationMode)) {
		// let's set screen to a tool view
		Transform3D<> viewT = getRobWorkStudio()->getViewTransform();
		viewT = _pose;
		
		viewT = viewT * Transform3D<>(RPY<>(90.0*Deg2Rad, 0.0, 180.0*Deg2Rad).toRotation3D());
		viewT.P() +=  viewT.R() * Vector3D<>(0.0, 0.0, 2.5);
		
		getRobWorkStudio()->setViewTransform(viewT);
		//getRobWorkStudio()->updateAndRepaint();
	}
	
	// redraw pointer
	_state = getRobWorkStudio()->getState();
	if (_pointerFrame) _pointerFrame->setTransform(_pose, _state);
	getRobWorkStudio()->setState(_state);
	
	// let other plugins know we changed pointer's position
	getRobWorkStudio()->getPropertyMap().add<Transform3D<> >("PointerPosition", "Position of the pointer frame", _pose);
	getRobWorkStudio()->genericEvent().fire("PointerPositionChanged");
	
	// update info in GUI
	ui.xEdit->setText(QString::number(_pose.P()(0)));
	ui.yEdit->setText(QString::number(_pose.P()(1)));
	ui.zEdit->setText(QString::number(_pose.P()(2)));
	
	RPY<> rpy(_pose.R());
	ui.rollEdit->setText(QString::number(rpy(0)));
	ui.pitchEdit->setText(QString::number(rpy(1)));
	ui.yawEdit->setText(QString::number(rpy(2)));
}



void PointerPlugin::action1()
{
	// let other plugins know we do something
	getRobWorkStudio()->genericEvent().fire("PointerAction1");
}



void PointerPlugin::action2()
{
	// let other plugins know we do something
	getRobWorkStudio()->genericEvent().fire("PointerAction2");
}



//id PointerPlugin::handlePositionMessage(const geometry_msgs::TransformStamped::ConstPtr& message)
//{
	//cout << message->transform.translation.x << endl;
	// we've received a new position for our pointer... set accordingly
	//_position = Vector3D<>(message->transform.translation.x, message->transform.translation.y, message->transform.translation.z);
	//Quaternion<> quat(message->transform.rotation.x, message->transform.rotation.y, message->transform.rotation.z, message->transform.rotation.w);
	//_rpy = RPY<>(quat.toRotation3D());
//}



Q_EXPORT_PLUGIN(PointerPlugin);
