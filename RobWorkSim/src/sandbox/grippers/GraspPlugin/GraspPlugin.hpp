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

	private:
		// methods
		//! @brief loads tasks into the simulator
		void setCurrentTask(rwlibs::task::GraspTask::Ptr task);
		
		//! @brief sets up open and closed gripper pose and approach vector
		rwlibs::task::GraspTask::Ptr generateTasks(int nTasks);
		
		//! @brief loads stl files for jaw geometry
		void loadGeometry(std::string directory);
		
		//! @brief sets up the GUI
		void setupGUI();

		// parameters
		rw::models::WorkCell* _wc; // workcell
		rwsim::dynamics::DynamicWorkCell::Ptr _dwc; // dynamic workcell
		rw::proximity::CollisionDetector::Ptr _detector; // collision detector
		rwsim::simulator::GraspTaskSimulator::Ptr _graspSim; // simulator
		rw::kinematics::State _initState; // workcell initial state

		QTimer *_timer; // used to update RWS view periodically

		// flags
		bool _slowMotion;
		
		// grasp parameters
		rw::math::Transform3D<> _wTapproach; // approach and retract for grasping
		rw::math::Transform3D<> _wTtarget; // target for grasping
		
		int _nOfTargetsToGen;
		TaskGenerator::Ptr _generator;
		rwlibs::task::GraspTask::Ptr _tasks; // grasp task loaded from file
		
		/* GUI */
		// geometry panel
		QGroupBox* _geometryBox;
		QPushButton* _designButton;
		QPushButton* _loadGeoButton;
		QPushButton* _saveGeoButton;
		
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
		QProgressBar* _progressBar;
		QPushButton* _startButton;
		QPushButton* _stopButton;
		QCheckBox* _slowCheck;
};
