#ifndef FALCONPLUGIN_HPP
#define FALCONPLUGIN_HPP

#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/math/Transform3D.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/rwsim.hpp>
#include <rwsimlibs/ode/ODESimulator.hpp>
#include <rwsim/control/SerialDeviceController.hpp>
#include <rwsim/control/PDController.hpp>
#include "ui_FalconPlugin.h"
#include <iostream>
#include <string>
#include <vector>
#include <QTimer>
#include "../src/FalconInterface.hpp"
#include "SimulationTrajectory.hpp"



class FalconPlugin: public rws::RobWorkStudioPlugin, private Ui::FalconPlugin
{
	Q_OBJECT
	Q_INTERFACES(rws::RobWorkStudioPlugin)
	
	public: // constants
		/// How much has the Falcon move from neutral position to record movement (in m)
		static const double deadZoneRadius = 0.01;
		
		/// Coefficient for angular movement velocity when using keys
		static const double angularVel = 0.05;
		
		/// Coefficient for speed when moving the viewpoint
		//static const double viewVel = 5.0;
		
		/// Modes of teleoperation
		enum FalconMode { WorldMode, ToolMode };
		
		/// Modes of keyboard
		enum InterfaceMode { ViewMode, RotationMode };
	
	public: // constructors
		/// Constructor
		FalconPlugin();
		
		/// Destructor
		virtual ~FalconPlugin();
		
	public: // methods
		/// Invoked whenever the new WC is opened
		virtual void open(rw::models::WorkCell* workcell);

		/// Invoked whenever the WC is closed
		virtual void close();
		
		/// Used to initialize plugin
		virtual void initialize();
		
		/// Used to listen for opening DWC event
		void genericEventListener(const std::string& event);
		
		/// Used to listen for key presses
		void keyEventListener(int key, Qt::KeyboardModifiers modifier);
		
		/// This is used as a step callback for the simulator
		void step(rwsim::simulator::ThreadSimulator* sim, const rw::kinematics::State& state);
		
		/// Used to prepare GUI
		void setupGui();
		
	private slots:
		void timerEvent();
		void guiEvent();
		void startSimulation();
		void stopSimulation();
		void startRecording();
	
	protected: // data
		QTimer* _timer;
		rwhw::FalconInterface::Ptr _falcon;
		rw::models::WorkCell::Ptr _wc;
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
		rw::kinematics::State _state;
		std::vector<rwlibs::simulation::SimulatedController::Ptr> _controllers;
		
		// assorted simulators
		rwsim::simulator::ThreadSimulator::Ptr _tsim;
		rwsim::simulator::DynamicSimulator::Ptr _sim;
		rwsim::simulator::ODESimulator::Ptr _engine;
		
		// controllers, devices & targets
		rw::models::Device::Ptr _dev; // robot
		rw::kinematics::Frame* _tcpFrame; // robot TCP
		rwsim::control::SerialDeviceController::Ptr _robotController;
		rwsim::control::PDController::Ptr _gripperController;
		std::vector<rwsim::dynamics::Body::Ptr> _bodies; // bodies in the scene
		
		FalconMode _mode; // mode of teleoperation
		InterfaceMode _wsad; // mode for WSAD keys
		bool _stickyMode;
		bool _moveJoints;
		
		bool _recordingEnabled;
		SimulationTrajectory::Ptr _trajectory; // recording of the simulation
		std::string _trajectoryFilename;
		
		rw::kinematics::MovableFrame::Ptr _pointerFrame; // pointer frame
		double _r, _p, _y;
		rw::math::Transform3D<> _target;
		rw::math::RPY<> _rpy;
		bool _grasping;
};

#endif // end of the include guard
