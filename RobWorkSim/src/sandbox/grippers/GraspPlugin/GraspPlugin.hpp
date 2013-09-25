#pragma once

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwsim/rwsim.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwsim/control/BodyController.hpp>
#include <rwsim/simulator/GraspTaskSimulator.hpp>
#include <rws/propertyview/PropertyViewEditor.hpp>
#include <rw/common/Timer.hpp>
#include <QObject>
#include <QtGui>
#include <QTimer>
#include "TaskGenerator.hpp"
#include "Gripper.hpp"
#include "GripperTaskSimulator.hpp"
#include "TaskDescription.hpp"



/**
 * @brief a plugin for testing grippers
 */
class GraspPlugin : public rws::RobWorkStudioPlugin
{
	Q_OBJECT
	Q_INTERFACES(rws::RobWorkStudioPlugin)
	
	public:
		//! @brief constructor
		GraspPlugin();

		//! @brief destructor
		virtual ~GraspPlugin();

		//! @copydoc rws::RobWorkStudioPlugin::open(rw::models::WorkCell* workcell)
		virtual void open(rw::models::WorkCell* workcell);

		//! @copydoc rws::RobWorkStudioPlugin::close()
		virtual void close();

		//! @copydoc rws::RobWorkStudioPlugin::initialize()
		virtual void initialize();

		/**
		 * @brief we listen for events regarding opening and closing of dynamic
		 * workcell
		 * @param event [in] the event id
		 */
		void genericEventListener(const std::string& event);

		//! @brief starts grasping simulation
		void startSimulation();

	private slots:
		//! @brief updates RWS state according to the simulation
		void updateSim();
		
		//! @brief GUI event handler
		void guiEvent();
		
		//! @brief Design event
		void designEvent();

	private:
		// methods
		//! @brief loads tasks into the simulator
		void setCurrentTask(rwlibs::task::GraspTask::Ptr task);
		
		//! @brief sets up open and closed gripper pose and approach vector
		rwlibs::task::GraspTask::Ptr generateTasks(int nTasks);
		
		//! @brief Plan tasks automatically
		void planTasks();
		
		//! @brief loads stl files for jaw geometry
		void loadGeometry(std::string directory);
		
		//! @brief shows tasks in RWS window
		void showTasks(rwlibs::task::GraspTask::Ptr tasks);
		
		//! @brief sets up the GUI
		void setupGUI();
		
		/**
		 * @brief Calculates gripper quality after the simulation is finished.
		 * So far only success ratio is taken into account.
		 */
		double calculateQuality(rwlibs::task::GraspTask::Ptr tasks, rwlibs::task::GraspTask::Ptr allTasks);
		
		/**
		 * @brief Calculates coverage.
		 */
		double calculateCoverage(rwlibs::task::GraspTask::Ptr tasks, rwlibs::task::GraspTask::Ptr allTasks, rw::math::Q diff);
		
		/**
		 * @brief Helper - filter & count targets using distance metric
		 */
		int filterAndCount(rwlibs::task::GraspTask::Ptr tasks, rw::math::Q diff) const;
		
		//! @brief Experimental - add new gripper device to the workcell
		void addGripper(rw::math::Transform3D<> transform);
		
		//! @brief Place for testing stuff out
		void test();

		// parameters
		rw::models::WorkCell* _wc; // workcell
		rw::models::TreeDevice::Ptr _dev; // gripper device
		rwsim::dynamics::RigidDevice::Ptr _ddev; // dynamic gripper device
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc; // dynamic workcell
		rwsim::simulator::GripperTaskSimulator::Ptr _graspSim; // simulator
		rw::kinematics::State _initState; // workcell initial state

		rw::graphics::Render::Ptr _render; // used to render targets
		QTimer *_timer; // used to update RWS view periodically
		
		// gripper
		rw::models::Gripper::Ptr _gripper;

		// flags
		bool _slowMotion;
		bool _showTasks;
		bool _silentMode;
		
		// grasp parameters
		rw::math::Transform3D<> _wTapproach; // approach and retract for grasping
		rw::math::Transform3D<> _wTtarget; // target for grasping
		
		int _nOfTargetsToGen;
		TaskGenerator::Ptr _generator;
		rwlibs::task::GraspTask::Ptr _tasks; // grasp tasks planned or loaded from file
		rwlibs::task::GraspTask::Ptr _samples; // all samples
		
		TaskDescription::Ptr _td;
		
		/* GUI */
		// setup panel
		QGroupBox* _setupBox;
		QPushButton* _editSetupButton;
		QPushButton* _loadSetupButton;
		QPushButton* _saveSetupButton;
		
		// geometry panel
		QGroupBox* _geometryBox;
		QPushButton* _designButton;
		QCheckBox* _autoPlanCheck;
		QPushButton* _loadGripperButton;
		QPushButton* _saveGripperButton;
		
		// manual grasp tasks generation
		QGroupBox* _manualBox;
		QPushButton* _initialButton;
		QPushButton* _approachButton;
		QPushButton* _targetButton;
		QPushButton* _generateButton;
		QLineEdit* _nManualEdit;
		
		// auto grasp tasks generation
		QGroupBox* _autoBox;
		QPushButton* _planButton;
		QLineEdit* _nAutoEdit;
		
		// simulation panel
		QGroupBox* _simBox;
		QPushButton* _loadTaskButton;
		QPushButton* _saveTaskButton;
		QCheckBox* _showCheck;
		QProgressBar* _progressBar;
		QPushButton* _startButton;
		QPushButton* _stopButton;
		QCheckBox* _slowCheck;
		QCheckBox* _silentCheck;
		
		QPushButton* _testButton;
		
		// working directory
		std::string _wd;
		
		double _interferenceLimit;
		double _wrenchLimit;
};
