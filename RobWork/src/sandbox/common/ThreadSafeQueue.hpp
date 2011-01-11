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


#ifndef THREADQUEUE_HPP_
#define THREADQUEUE_HPP_

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <rw/common/TimerUtil.hpp>

using namespace rw::common;

/**
 * @brief Concurrent queue of WorkPiles
 *
 */
template <class T>
class ThreadSafeQueue {
public:
	ThreadSafeQueue() : _size(0) {};

	inline bool empty() {
		boost::mutex::scoped_lock lock(_mutex);
		return _queue.empty();
	};

	inline void push(T wp) {
        boost::mutex::scoped_lock lock(_mutex);
        _queue.push(wp);
		_size++;
        //lock.unlock();
        _cond.notify_one();
	};

	inline bool try_pop(T *wp) {
        boost::mutex::scoped_lock lock(_mutex);
		if (_queue.empty())
			return false;

		*wp = _queue.front();
		_queue.pop();
		_size--;

		return true;
	}

	inline bool pop(T *wp) {
        boost::mutex::scoped_lock lock(_mutex);
        while(_queue.empty())
        {
            _cond.wait(lock);
        }
        *wp = _queue.front();
        _queue.pop();
		_size--;
/*
		std::queue<T> tmpQ = _queue;
		while(!tmpQ.empty()){
	        T val = tmpQ.front();
	        tmpQ.pop();

	        RW_ASSERT(val!=*wp);
    	}
    	*/

		return true;
	};

	bool has(T value){
		boost::mutex::scoped_lock lock(_mutex);
		std::queue<T> tmpQ = _queue;
		while(!tmpQ.empty()){
	        T val = tmpQ.front();
	        tmpQ.pop();
	        if(value==val)
	        	return true;
		}
		return false;
	}

	inline bool popAndPrint(T *wp) {
        boost::mutex::scoped_lock lock(_mutex);
        while(_queue.empty())
        {
            _cond.wait(lock);
        }

		std::cout << "-------------------------" << std::endl;
		std::queue<T> tmpQ = _queue;
		while(!tmpQ.empty()){
	        T val = tmpQ.front();
	        tmpQ.pop();
			std::cout << val << std::endl;

		}


        *wp = _queue.front();
        _queue.pop();
		_size--;

		std::cout << "-------------------------" << std::endl;
		return true;
	};

	inline size_t size() { return _size; };

private:
	std::queue<T> _queue;

	mutable boost::mutex _mutex;
	boost::condition _cond;

	size_t _size;
};


#endif
