/**
 *       @file  WorkPileQueue.hpp
 *      @brief  Class implementing a concurrent queue
 *
 * Currently not very concurrent!
 *
 *     @author  Andreas Rune Fugl (arf), arf@mmmi.sdu.dk
 *
 *   @internal
 *     Created  2009-02-23
 *   Copyright  Copyright (c) 2009, University of Southern Denmark
 *
 * =====================================================================================
 */


#ifndef THREADSTACK_HPP_
#define THREADSTACK_HPP_

#include <stack>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

using namespace rw::common;

/**
 * @brief Concurrent queue of WorkPiles
 *
 */
template <class T>
class ThreadSafeStack {
public:
	ThreadSafeStack() : _size(0) {};

	inline bool empty() {
		//boost::mutex::scoped_lock lock(_mutex);
		return _stack.empty();
	};

	inline void push(T wp) {
        boost::mutex::scoped_lock lock(_mutex);
        _stack.push(wp);
		_size++;
        //lock.unlock();
        _cond.notify_one();
	};

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
	};

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

	inline size_t size() { return _size; };

private:
	std::stack<T> _stack;

	mutable boost::mutex _mutex;
	boost::condition _cond;

	size_t _size;
};


#endif
