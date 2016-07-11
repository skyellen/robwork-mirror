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


#ifndef RWS_ROBWORKSTUDIO_HPP
#define RWS_ROBWORKSTUDIO_HPP

#define QT_NO_EMIT

#ifdef __WIN32
#include <windows.h>
#endif

#include <vector>

#include <QMainWindow>

#include <rw/models/WorkCell.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/common/Log.hpp>
#include <rw/common/Event.hpp>

#include "RWStudioView3D.hpp"

#include <boost/function.hpp>
#include <boost/any.hpp>

class QCloseEvent;
class QDragEnterEvent;
class QDragDropEvent;
class QDragMoveEvent;
class QSettings;
class QToolBar;

class PropertyViewEditor;
class HelpAssistant;

namespace rw { class RobWork; }

namespace rws {

	class AboutBox;
	class RobWorkStudioPlugin;

    /** @addtogroup rws
        @{ */

	/**
	 * @brief main robwork studio class
	 */
	class RobWorkStudio: public QMainWindow
	{
		Q_OBJECT
	public:

		/**
		   @brief RobWorkStudio object with a number of plugins loaded elsewhere.
		*/
		RobWorkStudio(const rw::common::PropertyMap& map);

		/**
		 * @brief destructor
		 * @return
		 */
		~RobWorkStudio();

		/**
		 * @brief Opens either a workcell file, device file or a CAD file. Supported formats
		 * are STL, STLA, STLB, 3DS, AC, AC3D, TRI, OBJ, WU, WC, DEV, XML
		 * @param filename [in] name of file
		 */
		void openFile(const std::string& filename);

		/**
		 * @brief returns the property map of this instance of robwork studio
		 * @return propertymap
		 */		
		rw::common::PropertyMap& getPropertyMap(){			
			return _propMap;			
		}
		

		/**
		 * @brief sets the workcell of robwork studio. If another workcell is currently
		 * loaded it will be closed.
		 * @param workcell [in] the workcell
		 */		
		void setWorkcell(rw::models::WorkCell::Ptr workcell);
		void setWorkCell(rw::models::WorkCell::Ptr workcell){ setWorkcell(workcell); }

		void postWorkCell(rw::models::WorkCell::Ptr workcell);
		void postOpenWorkCell(const std::string& string);
		/**
		 * @brief Returns the workcell opened in RobWorkStudio
		 */
		rw::models::WorkCell::Ptr getWorkcell();
		rw::models::WorkCell::Ptr getWorkCell(){ return getWorkcell(); }

		/**
		 * @brief Returns the collision detector used in the user interface
		 *
		 * The method provides a pointer to the collision checker used in the
		 * user interface.
		 *
		 * @return CollisionDetector
		 */				
		rw::proximity::CollisionDetector::Ptr getCollisionDetector() {			
			return _detector;
		}
		

		/**
		 * @brief Returns the component controlling the drawing
		 *
		 * Through the WorkCellGLDrawer the user can control the geometry
		 * associated to frames.
		 */
		rw::graphics::WorkCellScene::Ptr getWorkCellScene(){
		    return _view->getWorkCellScene(); 
		}

		

		/**
		 * @brief Returns reference to the common TimedStatePath
		 *
		 * The TimedStatePath can be used to represent a trajectory represented
		 * by full workcell states
		 *
		 * @return Reference to TimedStatePath
		 */		
		const rw::trajectory::TimedStatePath& getTimedStatePath() {			
			return _timedStatePath;			
		}
		

		/**
		 * @brief Sets the common TimedStatePath
		 *
		 * Use the common TimedStatePath to set at trajectory, which plugins, e.g. PlayBack
		 * should have access to.
		 *
		 * @param path [in] The new TimedStatePath
		 */		
		void setTimedStatePath(const rw::trajectory::TimedStatePath& path);
		void postTimedStatePath(const rw::trajectory::TimedStatePath& path);
		
		void postExit();

		/**
		 * @brief Sets the current state of for RobWorkStudio
		 *
		 * Sets the current state and calls all StateChanged listeners
		 *
		 * @param state [in] The new state
		 */		
		void setState(const rw::kinematics::State& state);
		void postState(const rw::kinematics::State& state);
		void postUpdateAndRepaint();
		void postSaveViewGL(const std::string& str);
		void postGenericEvent(const std::string& id);
		void postGenericAnyEvent(const std::string& id, boost::any);

		/**
		 * @brief Returns the current state
		 *
		 * Returns the current state of the workcell
		 * @return Current state
		 */		
		const rw::kinematics::State& getState() { return _state; }

		/**
		 * @brief the log of RobWorkStudio.
		 */		
		rw::common::Log& log();

		rw::common::Log::Ptr logPtr();

		bool event(QEvent *event);

		///////////////////////////////
		//Listener Interface
		///////////////////////////////

		/**
		 * @brief Defines a state changed listener.
		 *
		 * Listeners to this event is called when a change of the state occurs.
		 *
		 * StateChangedListener describes the signature of a callback method.
		 *
		 * Example usage in a plugin:
		 * \code
		 * void MyPlugin::initialize()
		 * {
		 *     getRobWorkStudio()->stateChangedEvent().add(
		 *         boost::bind(&MyPlugin::stateChangedListener, this, _1), this);
		 * }
		 *
		 * void MyPlugin::stateChangedListener(const State& state)
		 * {
		 * ...
		 * }
		 * \endcode
		 */		
		typedef boost::function<void(const rw::kinematics::State&)> StateChangedListener;
		
		/**
		 * @brief Defines event for state changes.
		 *
		 * This event is fired when setState is called, or when a user (plugin)
		 * manually calls fire.
		 */
		typedef rw::common::Event<StateChangedListener, const rw::kinematics::State&> StateChangedEvent;

		/**
		 * @brief Returns StateChangeEvent needed for subscribing and firing the event.
		 * @return Reference to the StateChangedEvent
		 */		
		StateChangedEvent& stateChangedEvent() { return _stateChangedEvent; }

		/**
		 * @brief Defines a frame selected listener.
		 *
		 * Listeners to this event is calls when a frame is selected.
		 *
		 * FrameSelectedListener describes the signature of the callback method.
		 *
		 * Example usage in a plugin, see RobWorkStudio::StateChangedListener
		 */
		typedef boost::function<void(rw::kinematics::Frame*)> FrameSelectedListener;

		/**
		 * @brief Defines event for frame selection
		 *
		 * The selected rw::kinematics::Frame* is provided as an argument
		 */
		typedef rw::common::Event<FrameSelectedListener, rw::kinematics::Frame*> FrameSelectedEvent;

		/**
		 * @brief Returns FrameSelectedEvent needed for subscription and firing of events
		 * @return Reference to the FrameSelectedEvent
		 */		
		FrameSelectedEvent& frameSelectedEvent() { return _frameSelectedEvent; }

		/**
		 * @brief Defines a generic event listener.
		 *
		 * Listeners to this event is called when someone fires a generic event.
		 *
		 * GenericEventListener describes the signature of a callback method, used for generic
		 * (user defined) event containing a string message.
		 *
		 * Example usage in a plugin, see RobWorkStudio::StateChangedListener
		 */
		typedef boost::function<void(const std::string&)> GenericEventListener;

		/**
		 * @brief Defines event for generic user events
		 */
		typedef rw::common::Event<GenericEventListener, const std::string&> GenericEvent;

		/**
		 * @brief Returns GenericEvent needed for subscription and firing of events
		 * @return Reference to the GenericEvent
		 */
		GenericEvent& genericEvent() { return _genericEvent; }

        typedef boost::function<void(const std::string&, boost::any)> GenericAnyEventListener;

        /**
         * @brief Defines event for generic user events
         */
        typedef rw::common::Event<GenericAnyEventListener, const std::string&, boost::any> GenericAnyEvent;

        /**
         * @brief a generic event like GenericEvent but with the possibility of attaching
         * any data to the event message.
         * @return Reference to the GenericAnyEvent
         */
        GenericAnyEvent& genericAnyEvent() { return _genericAnyEvent; }

        /**
         * @brief this will block until an anyevent with id \b id is emitted or timeout
         * is reached.
         * @param id [in] string id of the event that is expected
         * @param timeout [in] timeout in seconds
         * @return
         */
        boost::any waitForAnyEvent(const std::string& id, double timeout = -1.0);

		/**
		 * @brief Defines a key pressed event listener.
		 *
		 * Listeners are called when someone fires keyEvent. By default the View component
		 * fires out all key events it receives from QT.
		 *
		 * KeyEventListener describes the signature of a callback method.
		 *
		 * Example usage in a plugin:
		 * \code
		 * void MyPlugin::initialize() {
		 *     getRobWorkStudio()->keyEvent().add(
		 *         boost::bind(&MyPlugin::keyListener, this, _1, _2), this);
		 * }
		 *
		 * void MyPlugin::keyListener(int key, Qt::KeyboardModifiers modifiers) {
		 * ...
		 * }
		 * \endcode
		 */
		typedef boost::function<void(int, Qt::KeyboardModifiers)> KeyEventListener;

		/**
		 * @brief Defines event for key pressed events
		 */
		typedef rw::common::Event<KeyEventListener, int, Qt::KeyboardModifiers> KeyEvent;

		/**
		 * @brief Returns KeyEvent needed for subscription and firing of events
		 * @return Reference to the KeyEvent
		 */		
		KeyEvent& keyEvent() {			
			return _keyEvent;
		}
		

		/**
		 * @brief Defines a Mouse pressed event
		 * Listeners are called when someone fires a MousePressed event.
		 * MousePressedEventListener defines the signature of a callback method.
		 * Example usage in a plugin: See RobWorkStudio::StateChangedListener
		 */
		typedef boost::function<void(QMouseEvent*)> MousePressedEventListener;

		/**
		 * @brief Defines event for mouse pressed events
		 */
		typedef rw::common::Event<MousePressedEventListener, QMouseEvent*> MousePressedEvent;

		/**
		 * @brief Returns MousePressedEvent needed for subscription and firing of events
		 * @return Reference to the MousePressedEvent
		 */		
		MousePressedEvent& mousePressedEvent() {
			return _mousePressedEvent;
		}
		

		/**
		 * @brief Defines a StateTrajectory changed event
		 * Listeners are called when someone fires a stateTrajectoryChanged event.
		 * StateTrajectoryListener defines the signature of a callback method.
		 * Example usage in a plugin: See RobWorkStudio::StateChangedListener
		 */
		typedef boost::function<void(const rw::trajectory::TimedStatePath&)> StateTrajectoryChangedListener;

		/**
		 * @brief Defines event for key pressed events
		 */
		typedef rw::common::Event<StateTrajectoryChangedListener, const rw::trajectory::TimedStatePath&>  StateTrajectoryChangedEvent;

		/**
		 * @brief Returns stateTrajectoryChangedEvent needed for subscription and firing of event
		 * @return Reference to the stateTrajectoryChangedEvent
		 */		
		StateTrajectoryChangedEvent& stateTrajectoryChangedEvent() {
			return _stateTrajectoryChangedEvent;
		}
		


		/**
		 * @brief Defines a listener for position change events
		 *
		 * These listeners are called when user has selected a point in the 3D view.
		 *
		 * To send an event double clicking left mouse button on the point of interest while pressing shift.
		 *
		 * Example use in a plugin: See RobWorkStudio::StateChangedListener
		 */
		typedef boost::function<void(const rw::math::Vector3D<>&)> PositionSelectedListener;

		/**
		 * @brief Defines event for PositionChanged.
		 */
		typedef rw::common::Event<PositionSelectedListener, const rw::math::Vector3D<>&> PositionSelectedEvent;

		/**
		 * @brief Returns PositionChangedEvent object needed for subscription to and firing of event
		 * @return Reference to the PositionSelectedEvent
		 */		
		PositionSelectedEvent& positionSelectedEvent() {
			return _positionSelectedEvent;
		}




		/**
		 * @brief Saves the current opengl view
		 *
		 * The filename should end with either ".png", ".jpg" or ".bmp" to specify the format
		 * used to save the file.
		 */
		void saveViewGL(const QString& filename);

		/**
		 * @brief a method for updating the opengl graphics output
		 */
		
		void updateAndRepaint() {
			//update();
			_view->update();
		}
		

		/**
		 * @brief Returns the instance of the ViewGL class
		 */		
		RWStudioView3D::Ptr getView() { return _view; }
		
		/**
		 * @brief Returns the current view transform.
		 * 
		 * Convenience function.
		 */
		rw::math::Transform3D<> getViewTransform() { return getView()->getSceneViewer()->getTransform(); }
		
		/**
		 * @brief Sets current view transform.
		 * 
		 * Convenience function. Performs update & repaint as well.
		 */
		void setViewTransform(rw::math::Transform3D<> nviewT3D) {
			getView()->getSceneViewer()->setTransform(nviewT3D);
			updateAndRepaint();
		}
		
		void keyPressEvent(QKeyEvent *e);

		/**
		 * @return Returns the about box for RobWorkStudio
		 *
		 * Plugins can add their own tab to the about box.
		 */						
		AboutBox* getAboutBox() { return _aboutBox; };
			
		
		void propertyChangedListener(rw::common::PropertyBase* base);

        void addPlugin(RobWorkStudioPlugin* plugin,
                       bool visible,
                       Qt::DockWidgetArea area = Qt::LeftDockWidgetArea);

        void loadSettingsSetupPlugins(const std::string& file);

        rw::common::PropertyMap& getSettings(){ return *_settingsMap; }

	private:
		// all events are defined here
		StateChangedEvent _stateChangedEvent;
		FrameSelectedEvent _frameSelectedEvent;
		GenericEvent _genericEvent;
		GenericAnyEvent _genericAnyEvent;
		KeyEvent _keyEvent;
		MousePressedEvent _mousePressedEvent;
		StateTrajectoryChangedEvent _stateTrajectoryChangedEvent;
		PositionSelectedEvent _positionSelectedEvent;

	public slots:
	    void setTStatePath(rw::trajectory::TimedStatePath path);

	private slots:
		void newWorkCell();
		void open();
        void setCheckAction();
		void closeWorkCell();
		void showSolidTriggered();
		void showWireTriggered();
		void showBothTriggered();
		void showPropertyEditor();

		void updateHandler();
		void updateViewHandler();

		void dragMoveEvent(QDragMoveEvent *event);
		void dragEnterEvent(QDragEnterEvent* event);
		void dropEvent(QDropEvent* event);
		void showDocumentation();

		void showAboutBox();
		void printCollisions();
		void loadPlugin();
	protected:
		//! Close Event inherited from QT
		void closeEvent( QCloseEvent * e );

	private:
	    void updateLastFiles();

		void setupFileActions();
		void setupToolActions();
		void setupViewGL();
		void setupPluginsMenu();
		void setupHelpMenu();

		void createPlugins();

		void setupPlugin(const QString& pathname, const QString& filename, bool visible, int dock);
		void setupPlugins(QSettings& settings);

		void openDrawable(const QString& filename);
		void openWorkCellFile(const QString& filename);

		rw::common::Ptr<rw::RobWork> _robwork;

		RWStudioView3D* _view;
		AboutBox* _aboutBox;
		QMenuBar * _winMenuBar;
		rw::models::WorkCell::Ptr _workcell;
		rw::kinematics::State _state;
		rw::proximity::CollisionDetector::Ptr _detector;
		
		std::vector<RobWorkStudioPlugin*> _plugins;

		QMenu* _pluginsMenu, *_fileMenu, *_viewMenu, *_toolMenu;
		QToolBar* _pluginsToolBar;

		PropertyViewEditor *_propEditor;
		
		bool _inStateUpdate;

		rw::trajectory::TimedStatePath _timedStatePath;

		rw::common::PropertyMap _propMap;
		rw::common::PropertyMap *_settingsMap;
		std::vector<std::pair<QAction*,std::string> > _lastFilesActions;
		HelpAssistant *_assistant;
	private:
		void openAllPlugins();
		void closeAllPlugins();

		// These all forward to the plugin and catch any exceptions.
		void openPlugin(RobWorkStudioPlugin& plugin);
		void closePlugin(RobWorkStudioPlugin& plugin);
		//void sendStateUpdate(RobWorkStudioPlugin& plugin);

	private:
		RobWorkStudio(const RobWorkStudio&);
		RobWorkStudio& operator=(const RobWorkStudio&);
	};

	 //! @}

}

#endif
