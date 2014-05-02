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
#include "ui_GraspPlugin.h"



/**
 * @brief A plugin for testing grippers.
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
		
		/// listens for key presses
		void keyEventListener(int key, Qt::KeyboardModifiers modifier);

		//! @brief starts grasping simulation
		void startSimulation();

	private slots:
		//! @brief updates RWS state according to the simulation
		void updateSim();
		
		//! @brief GUI event handler
		void guiEvent();
		
		/// Another GUI event handler
		void guiEvent(int i);
		
		//! @brief Design event
		void designEvent();
		
		/// Setup editing event
		void setupEvent();
		
		/// Adds new hint for planning grasps
		void addHint();
		
		/// Removes all taught grasps
		void clearHints();

	private:
		// methods		
		/// Sets up open and closed gripper pose and approach vector.
		rwlibs::task::GraspTask::Ptr generateTasks(int nTasks);
		
		/// Plan tasks automatically.
		void planTasks();
		
		/// Generate a perturbed set of tasks.
		void perturbTasks();
		
		/// Shows tasks in RWS window.
		void showTasks();
		
		/// Sets up the GUI.
		void setupGUI();
		
		/// Loads gripper from XML file.
		void loadGripper(const std::string& filename);
		
		/// Updates gripper device in the workcell according to chosen gripper.
		void updateGripper();
		
		/// Place for testing stuff out
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
		
		// grippers
		rw::models::Gripper::Ptr _gripper;
		std::vector<rw::models::Gripper::Ptr> _gripperList;

		// flags
		bool _slowMotion;
		bool _showTasks;
		bool _showSamples;
		bool _showSuccesses;
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
		Ui::evaluationWidget ui;
		
		// setup panel
		/*QGroupBox* _setupBox;
		QPushButton* _editSetupButton;
		QPushButton* _loadSetupButton;
		QPushButton* _saveSetupButton;
		QPushButton* _addHintButton;
		QPushButton* _clearHintsButton;
		
		// geometry panel
		QGroupBox* _geometryBox;
		QPushButton* _designButton;
		QCheckBox* _autoPlanCheck;
		QPushButton* _loadGripperButton;
		QPushButton* _saveGripperButton;
		QComboBox* _gripperCombo;
		QPushButton* _clearButton;
		
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
		QCheckBox* _samplesCheck;
		QCheckBox* _successCheck;
		QProgressBar* _progressBar;
		QPushButton* _startButton;
		QPushButton* _stopButton;
		QCheckBox* _slowCheck;
		QCheckBox* _silentCheck;
		
		QPushButton* _testButton;*/
		
		// working directory
		std::string _wd;
		
		double _interferenceLimit;
		double _wrenchLimit;
};
