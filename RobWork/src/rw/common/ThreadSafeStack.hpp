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

#ifndef RW_COMMON_THREADSAFESTACK_HPP_
#define RW_COMMON_THREADSAFESTACK_HPP_

#include <stack>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>


namespace rw { namespace common {

/**
 * @brief Concurrent queue of WorkPiles
 *
 */
template <class T>
class ThreadSafeStack {
public:
	ThreadSafeStack() : _size(0) {}

	/**
	 * @brief Check if stack is empty.
	 * @return true if empty.
	 */
	inline bool empty() {
		//boost::mutex::scoped_lock lock(_mutex);
		return _stack.empty();
	}

	/**
	 * @brief Push a new element to the stack.
	 * @param wp [in] the element to add to stack.
	 */
	inline void push(T wp) {
        boost::mutex::scoped_lock lock(_mutex);
        _stack.push(wp);
		_size++;
        //lock.unlock();
        _cond.notify_one();
	}

	/**
	 * @brief Pop element from stack, if there is any.
	 * @param wp [out] the element.
	 * @return true.
	 */
	inline bool try_pop(T *wp) {
        boost::mutex::scoped_lock lock(_mutex);
		if (_stack.empty())
			return false;

		*wp = _stack.top();
		_stack.pop();
		_size--;
/*
		std::stack<T> tmpQ = _stack;
		while(!tmpQ.empty()){
	        T val = tmpQ.top();
	        tmpQ.pop();
	        RW_ASSERT( val!=*wp );
		}
*/
		return true;
	}

	/**
	 * @brief Pop element from stack. If empty, wait for an element to be pushed.
	 * @param wp [out] the element.
	 * @return true.
	 */
	inline bool pop(T *wp) {
        boost::mutex::scoped_lock lock(_mutex);
        while(_stack.empty())
        {
            _cond.wait(lock);
        }
        *wp = _stack.top();
        _stack.pop();
		_size--;
/*
		std::stack<T> tmpQ = _stack;
		while(!tmpQ.empty()){
	        T val = tmpQ.top();
	        tmpQ.pop();
	        RW_ASSERT( val!=*wp );
		}
*/
		return true;
	}

	/**
	 * @brief Check if given value is in stack.
	 * @param value [in] the value to look for.
	 * @return true if found, false otherwise.
	 */
	bool has(T value){
		boost::mutex::scoped_lock lock(_mutex);
		std::stack<T> tmpQ = _stack;
		while(!tmpQ.empty()){
	        T val = tmpQ.top();
	        tmpQ.pop();
	        if(value==val)
	        	return true;
		}
		return false;
	}

	/**
	 * @brief Get size of stack.
	 * @return the current size.
	 */
	inline size_t size() { return _size; }

private:
	std::stack<T> _stack;

	mutable boost::mutex _mutex;
	boost::condition _cond;

	size_t _size;
};

}}


#endif
