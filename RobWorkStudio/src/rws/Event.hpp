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


#ifndef EVENT_HPP
#define EVENT_HPP


#include "RobWorkStudioPlugin.hpp"

#include <list>
#include <boost/foreach.hpp>
#include <boost/function_equal.hpp>


/**
 * @brief Event is used for managing subscribtions and firing of events.
 *
 * Event is used for managing subscribtions and firing of events. The signature of the
 * callback method and the fire method is termined through the template arguments.
 *
 * CallBackMethod, defines the signature of the callback method needed for subscribing to
 * the event.
 *
 * FireEventMethod, defines the interface for firing events.
 *
 * Example of usage in RobWorkStudio:
 * \code
 * In: RobWorkStudio.hpp
 * typedef boost::function<void(const rw::kinematics::State&)> StateChangedListener;
 * typedef Event<StateChangedListener, StateChangedListener> StateChangedEvent;
 *
 * StateChangedEvent& stateChangedEvent() {
 *     return _stateChangedEvent;
 * }
 *
 * void fireStateChangedEvent(const rw::kinematics::State& state) {
 *     BOOST_FOREACH(const StateChangedEvent::Listener& listener, stateChangedEvent().getListeners()) {
 *         listener.callback(state);
 *     }
 * }
 *
 * In: RobWorkStudio.cpp
 *
 * RobWorkStudio::RobWorkStudio(...):
 * _stateChangedEvent(boost::bind(&RobWorkStudio::fireStateChangedEvent, this, _1)),
 * ...
 * {
 * ...
 * }
 *
 * \endcode
 *
 */
template <class CallBackMethod, typename FireEventMethod>
class Event
{
public:
    /**
     * @brief Constructor for Event
     *
     * Constructs an event with the specified fire method.
     */
    Event(FireEventMethod fireMethod) {
        fire = fireMethod;
	}

	/**
	 * @brief Descructor.
	 */
	//virtual ~Event() {};

	/**
	 * @brief Adds a listener to the event
	 *
	 * Adds \b callback as a listener to the event. The optional \b obj and \b
	 * id are stored with \b callback to enable removing listeners. It is
	 * recommended to set \b obj as the object on which the callback is defined.
	 *
	 * Direct comparison of boost::function pointers does not work on all
	 * platform. It is thus necessary to provide the user with the optional \b
	 * id to enable removing a specific callback.
     *
	 * Typical usage
	 * \code
	 * void MyPlugin::frameSelectedListener(rw::kinematics::Frame* frame) {
	 *     ...
	 * }
	 *
	 * void MyPlugin::initialize() {
	 *     getRobWorkStudio()->frameSelectedEvent().add(
     *         boost::bind(&MyPlugin::frameSelectedListener, this, _1), this);
	 * }
	 * \endcode
	 *
	 *
	 * @param callback [in] The callback function
     *
	 * @param obj [in] Pointer to object associated with the listener (only used
	 * when removing listeners)
     *
	 * @param id [in] Id associated with the callback (only used for removing a
	 * specific listener)
	 */
	void add(CallBackMethod callback, const void* obj = NULL, int id = 0)
    {
	    _listeners.push_back(Listener(callback, obj, id));
	}

    /**
     * @brief Removes all callback method from a given obj
     *
     * All callbacks associated with \b obj are removed. Typical use will be to remove
     * all callbacks to an object before it is destroyed.
     *
     * @param obj [in] Object for which to remove listeners
     */
	void remove(void* obj)
    {
	    typename std::list<Listener>::iterator it = _listeners.begin();
	    while (it != _listeners.end()) {
	        if ((*it).obj == obj) {
	            it = _listeners.erase(it);
	        } else {
	            ++it;
	        }
	    }
	}

	/**
	 * @brief Removes all callback methods associated with the \b obj and \b id.
	 *
	 * @param obj [in] Object associated with the callback
	 * @param id [in] Id of the callback
	 */
    void remove(void* obj, int id)
    {
        typename std::list<Listener>::iterator it = _listeners.begin();
        while (it != _listeners.end()) {
            if ((*it).obj == obj && (*it).id == id) {
                it = _listeners.erase(it);
            } else {
                ++it;
            }
        }
    }

	/**
	 * @brief Fires the event
	 *
	 * The signature of the \b fire method depends on the FireEventMethod template argument.
	 */
	FireEventMethod fire;

	/**
	 * @brief Structure for data associated to a listener
	 */
	struct Listener
    {
	    /**
	     * @brief The callback method
	     */
	    CallBackMethod callback;

	    /**
	     * @brief The object associated with the callback
	     */
	    const void* obj;

	    /**
	     * @brief The id associated with the callback
	     */
	    int id;

	    /**
	     * @brief Constructs Listener data struct
	     */
	    Listener(CallBackMethod callback, const void* obj, int id):
	        callback(callback),
	        obj(obj),
	        id(id)
        {}
	};

	//! iterator of event listeners
	typedef typename std::list<Listener>::const_iterator ConstListenerIterator;

	/**
	 * @brief Returns list of listeners to the event
	 *
	 * @return List of listeners
	 */
	std::pair<ConstListenerIterator, ConstListenerIterator> getListeners() {
	    return std::make_pair(_listeners.begin(), _listeners.end());
	}

private:
    std::list<Listener> _listeners;
};

#endif /*EVENT_HPP*/
