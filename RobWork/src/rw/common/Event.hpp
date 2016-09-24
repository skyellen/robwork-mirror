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


#ifndef RW_COMMON_EVENT_HPP
#define RW_COMMON_EVENT_HPP


#include <list>
#include <boost/foreach.hpp>
#include <boost/function_equal.hpp>

namespace rw {
namespace common {

    class _n1{};
    class _n2{};
    class _n3{};
    class _n4{};
    class _n5{};
    class _n6{};
    class _n7{};
    class _n8{};

    template<class CallBackMethod, class T1=_n1, class T2=_n1, class T3=_n1, class T4=_n1, class T5=_n1> struct FireFunctor;

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
    template <class CallBackMethod, class T1=_n1, class T2=_n1, class T3=_n1, class T4=_n1>
    class Event
    {
    public:
        /**
         * @brief constructor
         */
        Event():fire(this){}

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

		/**
		 * @brief Get the list of listeners for this event.
		 * @return list of listeners.
		 */
		std::list<Listener>& getListenerList(){ return _listeners; }

	private:
		std::list<Listener> _listeners;

	public:
        /**
         * @brief Fires the event
         *
         * The signature of the \b fire method depends on the FireEventMethod template argument.
         */
        FireFunctor<CallBackMethod,T1,T2,T3,T4,_n1> fire;

	};

	// these empty classes are used in partial specialization of the firefunctor


    //template<class CallBackMethod> class FireFunctor<CallBackMethod,_n1,_n2,_n3,_n4> : public FireFunctor0<CallBackMethod> {};


    //! FireFunctor with 0 arguments
    template<class CallBackMethod, class T1, class T2, class T3, class T4, class T5> struct FireFunctor {
    public:
    	/**
    	 * @brief Constructor.
    	 * @param event [in] the event.
    	 */
        FireFunctor(Event<CallBackMethod>* event):_event(event){}
        //! @brief Functor operator for invoking the callback methods on all listeners.
        void operator()(){
            BOOST_FOREACH(typename Event<CallBackMethod>::Listener& listener, _event->getListenerList()) { listener.callback( ); } }
    private:
        Event<CallBackMethod> *_event;
    };

    //! FireFunctor with 1 arguments
    template<class CallBackMethod, class T1> struct FireFunctor<CallBackMethod, T1,_n1,_n1,_n1, _n1> {
    	//! @brief Type of the event.
        typedef Event<CallBackMethod, T1> EventType;
    	/**
    	 * @brief Constructor.
    	 * @param event [in] the event.
    	 */
        FireFunctor(EventType* event):_event(event){}
        //! @brief Functor operator for invoking the callback methods on all listeners.
        void operator()(T1 t1){
            BOOST_FOREACH(typename EventType::Listener& listener, _event->getListenerList()) { listener.callback( t1 ); } }
    private:
        EventType *_event;
    };

    //! FireFunctor with 2 arguments
    template<class CallBackMethod, class T1, class T2> struct FireFunctor<CallBackMethod, T1, T2,_n1,_n1, _n1> {
    	//! @brief Type of the event.
        typedef Event<CallBackMethod, T1, T2> EventType;
    	/**
    	 * @brief Constructor.
    	 * @param event [in] the event.
    	 */
        FireFunctor(EventType* event):_event(event){}
        //! @brief Functor operator for invoking the callback methods on all listeners.
        void operator()(T1 t1, T2 t2){
            BOOST_FOREACH(typename EventType::Listener& listener, _event->getListenerList()) { listener.callback( t1, t2 ); } }
    private:
        EventType *_event;
    };

    //! FireFunctor with 3 arguments
    template<class CallBackMethod, class T1, class T2, class T3> struct FireFunctor<CallBackMethod, T1, T2, T3,_n1, _n1> {
    	//! @brief Type of the event.
        typedef Event<CallBackMethod, T1, T2, T3> EventType;
    	/**
    	 * @brief Constructor.
    	 * @param event [in] the event.
    	 */
        FireFunctor(EventType* event):_event(event){}
        //! @brief Functor operator for invoking the callback methods on all listeners.
        void operator()(T1 t1, T2 t2, T3 t3){
            BOOST_FOREACH(typename EventType::Listener& listener, _event->getListenerList()) { listener.callback( t1, t2, t3 ); } }
    private:
        EventType *_event;
    };

    //! FireFunctor with 4 arguments
    template<class CallBackMethod, class T1, class T2, class T3, class T4> struct FireFunctor<CallBackMethod, T1, T2, T3, T4, _n1> {
    	//! @brief Type of the event.
        typedef Event<CallBackMethod, T1, T2, T3, T4> EventType;
    	/**
    	 * @brief Constructor.
    	 * @param event [in] the event.
    	 */
        FireFunctor(EventType* event):_event(event){}
        //! @brief Functor operator for invoking the callback methods on all listeners.
        void operator()(T1 t1, T2 t2, T3 t3, T4 t4){
            BOOST_FOREACH(typename EventType::Listener& listener, _event->getListenerList()) { listener.callback( t1, t2, t3, t4 ); } }
    private:
        EventType *_event;
    };

}
}

#endif /*EVENT_HPP*/
