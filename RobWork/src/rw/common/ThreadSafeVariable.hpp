/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RW_COMMON_THREADSAFEVARIABLE_HPP_
#define RW_COMMON_THREADSAFEVARIABLE_HPP_

/**
 * @file ThreadSafeVariable.hpp
 *
 * \copydoc rw::common::ThreadSafeVariable
 */

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/mutex.hpp>

namespace rw {
namespace common {
//! @addtogroup common

//! @{
/**
 * @brief A thread safe protected variable.
 *
 * This is very useful for making simple thread-safe variables, but also for synchronization between threads.
 */
template <typename T>
class ThreadSafeVariable {
public:
	/**
	 * @brief Create new protected variable.
	 * @param var [in] the initial value.
	 */
	ThreadSafeVariable(const T var) {
		_changed = false;
		_waiting = 0;
		_var = var;
	}

	//! @brief Destructor
	virtual ~ThreadSafeVariable() {}

	/**
	 * @brief Get the value.
	 * @return the value.
	 */
	T getVariable() {
		boost::shared_lock<boost::shared_mutex> lock2(_mutex);
		return _var;
	}

	/**
	 * @brief Change the value.
	 *
	 * @note If some are still waiting for the last update of the value (by using the waitForUpdate() function),
	 * this function will block until they have received the previous update first.
	 *
	 * @param var [in] the new value.
	 */
	void setVariable(const T var) {
		bool notifyChange = false;
		{
			boost::mutex::scoped_lock lock1(_changedMutex);
			while(_changed)
				_waitingCond.wait(lock1);
			boost::unique_lock<boost::shared_mutex> lock2(_mutex);
			if (_waiting > 0) {
				_changed = true;
				notifyChange = true;
			}
			_var = var;
		}
		if (notifyChange)
			_changeCond.notify_all();
	}

	/**
	 * @brief Wait for a change of the value (blocking).
	 * @param previous [in] the previous value to compare with.
	 *
	 * @note The type, T, should implement the operator== function for comparison.
	 *
	 * @return the new value that is not equal to the previous value.
	 */
	T waitForUpdate(T previous) {
		bool noMoreWaiting = false;
		{
			boost::mutex::scoped_lock lock1(_changedMutex);
			{
				boost::unique_lock<boost::shared_mutex> lock2(_mutex);
				if (!(_var == previous)) return _var;
			}
			while(!_changed) {
				_waiting++;
				_changeCond.wait(lock1);
				_waiting--;
			}
			boost::unique_lock<boost::shared_mutex> lock2(_mutex);
			if (_waiting == 0) {
				_changed = false;
				noMoreWaiting = true;
			}
		}
		if (noMoreWaiting)
			_waitingCond.notify_all();
		return _var;
	}

	/**
	 * @brief Use the () operator to access the value.
	 * @return the value.
	 */
	T operator()() {
		return getVariable();
	}

	/**
	 * @brief Set the value using the assignment operator (same as using setVariable()).
	 * @param var [in] the new value.
	 */
	void operator=(const T var) {
		setVariable(var);
	}

private:
	// Lock 1
	boost::mutex _changedMutex;
	boost::condition_variable _changeCond;
	boost::condition_variable _waitingCond;
	bool _changed;
	unsigned int _waiting;

	// Lock 2
	boost::shared_mutex _mutex;
	T _var;
};
//! @}
} /* namespace common */
} /* namespace rw */
#endif /* RW_COMMON_THREADSAFEVARIABLE_HPP_ */
