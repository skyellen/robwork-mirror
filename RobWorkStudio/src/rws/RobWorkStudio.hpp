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


#ifndef ROBWORKSTUDIO_HPP
#define ROBWORKSTUDIO_HPP

#define QT_NO_EMIT

#ifdef __WIN32
#include <windows.h>
#endif

#include <vector>
#include <memory>

#include <QMainWindow>
#include <QCloseEvent>
#include <QSettings>

#include <rwlibs/drawable/Drawable.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/models/WorkCell.hpp>

#include <rw/common/Log.hpp>

#include "RobWorkStudioPlugin.hpp"
#include "Event.hpp"
#include "ViewGL.hpp"

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>


class ViewGL;
class QDragEnterEvent;
class QDragDropEvent;

/**
 * @brief main robwork studio class
 */
class RobWorkStudio: public QMainWindow
{
    Q_OBJECT
public:

    /**
       @brief A tuple of (plugin, visible, dockingArea).
    */
    struct PluginSetup {
        PluginSetup(
            RobWorkStudioPlugin* plugin,
            bool visible,
            Qt::DockWidgetArea area)
            :
            plugin(plugin),
            visible(visible),
            area(area)
        {}

        RobWorkStudioPlugin* plugin;
        bool visible;
        Qt::DockWidgetArea area;
    };

    /**
       @brief RobWorkStudio object with a number of plugins loaded elsewhere.
    */
    RobWorkStudio(const std::vector<PluginSetup>& plugins,
                  const rw::common::PropertyMap& map,
                  const std::string& inifile);

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
     * loadet it will be closed.
     * @param workcell [in] the workcell
     */
    void setWorkcell(rw::models::WorkCellPtr workcell);

    /**
     * @brief Returns the collision detector used in the user interface
     *
     * The method provides a pointer to the collision checker used in the
     * user interface.
     *
     * @return CollisionDetector
     */
    rw::proximity::CollisionDetector* getCollisionDetector() {
        return _detector.get();
    }

    /**
     * @brief Returns the component controlling the drawing
     *
     * Through the WorkCellGLDrawer the user can control the geometry
     * associated to frames.
     */
    rwlibs::drawable::WorkCellGLDrawer* getWorkCellGLDrawer() {
        return &_workcellGLDrawer;
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

    /**
     * @brief Sets the current state of for RobWorkStudio
     *
     * Sets the current state and calls all StateChanged listeners
     *
     * @param state [in] The new state
     */
    void setState(const rw::kinematics::State& state);

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
    typedef Event<StateChangedListener, StateChangedListener> StateChangedEvent;

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
    typedef Event<FrameSelectedListener, FrameSelectedListener> FrameSelectedEvent;

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
    typedef Event<GenericEventListener, GenericEventListener> GenericEvent;

    /**
     * @brief Returns GenericEvent needed for subscription and firing of events
     * @return Reference to the GenericEvent
     */
    GenericEvent& genericEvent() { return _genericEvent; }

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
    typedef Event<KeyEventListener, KeyEventListener> KeyEvent;

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
    typedef Event<MousePressedEventListener, MousePressedEventListener> MousePressedEvent;

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
    typedef Event<StateTrajectoryChangedListener, StateTrajectoryChangedListener>  StateTrajectoryChangedEvent;

    /**
     * @brief Returns KeyEvent needed for subscription and firing of events
     * @return Reference to the KeyEvent
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
    typedef boost::function<void(const rw::math::Vector3D<>& position)> PositionSelectedListener;
    /**
     * @brief Defines event for PositionChanged.
     */
    typedef Event<PositionSelectedListener, PositionSelectedListener> PositionSelectedEvent;

    /**
     * @brief Returns PositionChangedEvent object needed for subscription to and firing of event
     * @return REference to the PositionSelectedEvent
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
        update();
        _view->updateGL();
    }

    /**
     * @brief Returns the instance of the ViewGL class
     */
    ViewGL* getView() { return _view; }


private:
    StateChangedEvent _stateChangedEvent;

    void fireStateChangedEvent(const rw::kinematics::State& state) {
        BOOST_FOREACH(
            const StateChangedEvent::Listener& listener,
            stateChangedEvent().getListeners()) {
            listener.callback(state);
        }
    }

    FrameSelectedEvent _frameSelectedEvent;

    void fireFrameSelectedEvent(rw::kinematics::Frame* frame) {
        BOOST_FOREACH(
            const FrameSelectedEvent::Listener& listener,
            frameSelectedEvent().getListeners()) {
            listener.callback(frame);
        }
    }

    GenericEvent _genericEvent;
    void fireGenericEvent(const std::string& msg) {
        BOOST_FOREACH(
            const GenericEvent::Listener& listener,
            genericEvent().getListeners()) {
            listener.callback(msg);
        }
    }

    KeyEvent _keyEvent;
    void fireKeyEvent(int key, Qt::KeyboardModifiers modifiers) {
        BOOST_FOREACH(
            const KeyEvent::Listener& listener,
            keyEvent().getListeners()) {
            listener.callback(key, modifiers);
        }
    }

    MousePressedEvent _mousePressedEvent;
    void fireMousePressedEvent(QMouseEvent* qmouseEvent)
    {
        BOOST_FOREACH(const MousePressedEvent::Listener& listener, mousePressedEvent().getListeners())
        {
            listener.callback(qmouseEvent);
        }
    }


    StateTrajectoryChangedEvent _stateTrajectoryChangedEvent;
    void fireStateTrajectoryChangedEvent(const rw::trajectory::TimedStatePath& trajectory);


    PositionSelectedEvent _positionSelectedEvent;
    void firePositionSelectedEvent(const rw::math::Vector3D<>& position) {
        BOOST_FOREACH(
            const PositionSelectedEvent::Listener& listener,
            positionSelectedEvent().getListeners()) {
            listener.callback(position);
        }

    }

private slots:
    void newWorkCell();
    void open();
    void close();
    void showSolidTriggered();
    void showWireTriggered();
    void showBothTriggered();

    void updateHandler();
    void updateViewHandler();

    void sendAllMessages(
        std::string plugin,
        std::string id,
        rw::common::Message msg);

    void dragEnterEvent(QDragEnterEvent* event);
    void dropEvent(QDropEvent* event);

protected:
    void closeEvent( QCloseEvent * e );

private:

    void connectEmitMessage(RobWorkStudioPlugin* plugin);

    void addPlugin(
        RobWorkStudioPlugin* plugin,
        bool visible,
        Qt::DockWidgetArea area = Qt::LeftDockWidgetArea);

    void setupFileActions();
    void setupViewGL();

    void createPlugins();
    QSettings::Status loadSettingsSetupPlugins(const std::string& file);
    void setupPlugins(QSettings& settings);

    void openDrawable(const QString& filename);
    void openWorkCellFile(const QString& filename);
    ViewGL* _view;

    rw::models::WorkCellPtr _workcell;
    rw::kinematics::State _state;
    rw::proximity::CollisionDetectorPtr _detector;

    std::vector<RobWorkStudioPlugin*> _plugins;
    std::vector<rwlibs::drawable::Drawable*> _drawables;
    QMenu* _pluginsMenu;
    QMenu* _fileMenu;
    QMenu* _viewMenu;
    QToolBar* _pluginsToolBar;
    QToolBar* _viewToolBar;
    Convert _converter;

    //std::string _previousOpenDirectory;



    rwlibs::drawable::WorkCellGLDrawer _workcellGLDrawer;

    bool _inStateUpdate;

    rw::trajectory::TimedStatePath _timedStatePath;

    rw::common::PropertyMap _propMap;

    rw::common::PropertyMap *_settingsMap;

private:
    void openAllPlugins();
    void closeAllPlugins();

    // These all forward to the plugin and catch any exceptions.
    void openPlugin(RobWorkStudioPlugin& plugin);
    void closePlugin(RobWorkStudioPlugin& plugin);
    void sendStateUpdate(RobWorkStudioPlugin& plugin);
    void sendMessage(
        RobWorkStudioPlugin& plugin,
        const std::string& pluginName,
        const std::string& id,
        const rw::common::Message& msg);

private:
    RobWorkStudio(const RobWorkStudio&);
    RobWorkStudio& operator=(const RobWorkStudio&);
};

#endif
