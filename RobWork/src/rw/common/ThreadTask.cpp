/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ThreadTask.hpp"
#include "ThreadPool.hpp"
#include "ThreadSafeVariable.hpp"
#include "macros.hpp"
#include "Exception.hpp"

#include <boost/bind.hpp>
#include <boost/foreach.hpp>

using namespace rw::common;

ThreadTask::ThreadTask(ThreadTask::Ptr parent):
	_pool(new ThreadSafeVariable<ThreadPool::Ptr>(parent != NULL ? parent->getThreadPool() : NULL)),
	_state(new ThreadSafeVariable<TaskState>(INITIALIZATION)),
	_keepAlive(new ThreadSafeVariable<bool>(false)),
	_blockFinalize(new ThreadSafeVariable<bool>(false)),
	_children(new ThreadSafeVariable<std::vector<ThreadTask::Ptr> >(std::vector<ThreadTask::Ptr>())),
	_childrenMissing(new ThreadSafeVariable<unsigned int>(0)),
	_parentCallback(new ThreadSafeVariable<ParentCallback>(ParentCallback())),
	_exceptions(new ThreadSafeVariable<std::list<Exception> >(std::list<Exception>()))
{
}

ThreadTask::ThreadTask(ThreadPool::Ptr pool):
	_pool(new ThreadSafeVariable<ThreadPool::Ptr>(pool)),
	_state(new ThreadSafeVariable<TaskState>(INITIALIZATION)),
	_keepAlive(new ThreadSafeVariable<bool>(false)),
	_blockFinalize(new ThreadSafeVariable<bool>(false)),
	_children(new ThreadSafeVariable<std::vector<ThreadTask::Ptr> >(std::vector<ThreadTask::Ptr>())),
	_childrenMissing(new ThreadSafeVariable<unsigned int>(0)),
	_parentCallback(new ThreadSafeVariable<ParentCallback>(ParentCallback())),
	_exceptions(new ThreadSafeVariable<std::list<Exception> >(std::list<Exception>()))
{
}

ThreadTask::~ThreadTask() {
	delete _pool;
	delete _state;
	delete _keepAlive;
	delete _blockFinalize;
	delete _children;
	delete _childrenMissing;
	delete _parentCallback;
	delete _exceptions;
}

bool ThreadTask::setThreadPool(ThreadPool::Ptr pool) {
	// Do a quick check on the current state
	if (_state->getVariable() != INITIALIZATION)
		return false;
	// Lock mutex to get exclusive access to change private member variables.
	// (blocks execute() from moving from INITILIZATION state and launching the job before pool has been set)
	boost::mutex::scoped_lock lock(_mutex);
	if (_state->getVariable() != INITIALIZATION)
		return false;
	_pool->setVariable(pool);
	return true;
}

ThreadPool::Ptr ThreadTask::getThreadPool() {
	return _pool->getVariable();
}

void ThreadTask::run() {

}

void ThreadTask::subTaskDone(ThreadTask* subtask) {
	BOOST_FOREACH(const Exception& e, subtask->getExceptions()) {
		registerFailure(e);
	}
}

void ThreadTask::idle() {

}

void ThreadTask::done() {

}

bool ThreadTask::execute() {
	// Do a quick check on the state
	if (_state->getVariable() != INITIALIZATION)
		return false;
	std::vector<ThreadTask::Ptr> children;
	{
		// Lock mutex to get exclusive access to change private member variables.
		// This ensures:
		// 1. setThreadPool() and addSubTask() is not currently changing the pool or adding subtasks
		// 2. execute() is only called once (otherwise the state would no longer be INITIALIZATION)
		boost::mutex::scoped_lock lock(_mutex);
		if (_state->getVariable() != INITIALIZATION)
			return false;
		_state->setVariable(IN_QUEUE);
		children = _children->getVariable();
	}
	// At this point new subtasks can again be added
	//  - in this case they are launched from the addSubTasks() function and not from here
	//  - note that a subtask added at this point might actually start executing before we add this task to the pool
	// The ThreadPool can not change anymore (state is IN_QUEUE) so we just read it without holding the lock:
	ThreadPool::Ptr pool = _pool->getVariable();
	// Work is now added to the pool, where it is queued (it might start executing immediately).
	if (pool != NULL) {
		ThreadPool::WorkFunction workFct = boost::bind(&ThreadTask::runWrap,this,_1);
		pool->addWork(workFct);
	} else {
		runWrap(NULL);
	}
	BOOST_FOREACH(ThreadTask::Ptr subtask, children) {
		subtask->execute();
	}
	return true;
}

ThreadTask::TaskState ThreadTask::wait(ThreadTask::TaskState previous) {
	return _state->waitForUpdate(previous);
}

void ThreadTask::waitUntilDone() {
	ThreadTask::TaskState state = _state->getVariable();
	while (state != DONE) {
		state = wait(state);
	}
}

ThreadTask::TaskState ThreadTask::getState() {
	return _state->getVariable();
}

bool ThreadTask::addSubTask(ThreadTask::Ptr subtask) {
	// Do a quick check on the state
	TaskState state = _state->getVariable();
	if (state == POSTWORK || state == DONE)
		return false;
	{
		// Lock mutex to get exclusive access to change private member variables.
		// This ensures:
		// 1. only one addSubTask() function at a time is adding subtasks to the list
		// 2. if execute() and addSubTask() is invoked at the same time we must be sure that the
		//    task is launched from exactly one of the functions
		// 3. if task is executing, the setKeepAlive(), runWrap() or callbackParent() functions
		//    must be blocked from moving to the POSTWORK or DONE states
		boost::mutex::scoped_lock lock(_mutex);
		state = _state->getVariable();
		if (state == POSTWORK || state == DONE)
			return false; // it was too late to add new subtask
		// Add the subtask:
		std::vector<ThreadTask::Ptr> children = _children->getVariable();
		children.push_back(subtask);
		_children->setVariable(children);
		subtask->_parentCallback->setVariable(boost::bind(&ThreadTask::callbackParent,this,_1));
		_childrenMissing->setVariable(_childrenMissing->getVariable()+1);
		// If the task is currently IDLE, we change the state to CHILDREN.
		if (state == IDLE) {
			_state->setVariable(CHILDREN);
			state = CHILDREN;
		}
	}
	// Launch the task if parent task is already executing
	if (state != INITIALIZATION)
		subtask->execute();
	return true;
}

std::vector<ThreadTask::Ptr> ThreadTask::getSubTasks() {
	return _children->getVariable();
}

void ThreadTask::setKeepAlive(bool keepAlive) {
	// Make a quick check that a change is actually requested
	if (_keepAlive->getVariable() == keepAlive)
		return;
	bool doFinish = false;
	{
		// Lock mutex to get exclusive access to change private member variables.
		// This is done to be sure that the state does not change to and from IDLE while keepAlive is changing
		// this influences if finish() should be invoked or not.
		boost::mutex::scoped_lock lock(_mutex);
		// If keepAlive is switched off and the task is IDLE, we finish it.
		if (_keepAlive->getVariable() && !keepAlive) {
			if (_state->getVariable() == IDLE) {
				if (!_blockFinalize->getVariable()) { // only finish task if no subtasks are currently invoking idle
					_state->setVariable(POSTWORK); // addSubTasks() can not be used any longer
					doFinish = true;
				}
			}
		}
		_keepAlive->setVariable(keepAlive);
	}
	if (doFinish)
		finish();
}

bool ThreadTask::keepAlive() {
	return _keepAlive->getVariable();
}

void ThreadTask::registerFailure(const Exception& e) {
	boost::mutex::scoped_lock lock(_mutex);
	std::list<Exception> exceptions = _exceptions->getVariable();
	exceptions.push_back(e);
	_exceptions->setVariable(exceptions);
}

std::list<Exception> ThreadTask::getExceptions() const {
	return _exceptions->getVariable();
}

void ThreadTask::runWrap(ThreadPool* pool) {
	// runWrap is invoked only once - the state is IN_QUEUE
	// State is set to EXECUTING
	{
		boost::mutex::scoped_lock lock(_mutex);
		_state->setVariable(EXECUTING);
	}
	try {
		run();
	} catch(boost::thread_interrupted const&) {
		RW_THROW("Please catch boost::thread_interrupted exception thrown in ThreadTask run funtion!");
	}
	// State is set to CHILDREN causing children to try to move to IDLE when they finish (if keep alive is disabled).
	{
		boost::mutex::scoped_lock lock(_mutex);
		_state->setVariable(CHILDREN);
	}
	// Now check if task should change to IDLE (if there is no subtasks for instance)
	tryIdle();
}

void ThreadTask::callbackParent(ThreadTask* task) {
	// Notify the parent that the subtask has now finished
	subTaskDone(task);
	// Decrement children counter
	{
		// Lock the mutex to be sure that:
		// 1. no other children are trying to decrement counter at the same time
		// 2. addSubTasks() is not trying to increment counter at the same time
		boost::mutex::scoped_lock lock(_mutex);
		_childrenMissing->setVariable(_childrenMissing->getVariable()-1);
	}
	// Now check if task should change to IDLE
	tryIdle();
}

void ThreadTask::tryIdle() {
	bool doIdle = false;
	{
		// Lock the mutex to be sure that:
		// 1. Children counter is not currently changing
		// 2. Change from CHILDREN to IDLE state only happens once (and idle() is invoked once)
		boost::mutex::scoped_lock lock(_mutex);
		if (_childrenMissing->getVariable() == 0) {
			if (_state->getVariable() == CHILDREN) {
				_state->setVariable(IDLE);
				doIdle = true;
				_blockFinalize->setVariable(true); // to make sure that setKeepAlive(false) will not finish task to early
			}
		}
	}
	bool doFinish = false;
	if (doIdle) {
		idle();
		{
			// Lock the mutex to be sure that:
			// 1. Children counter is not currently changing
			// 2. If keep alive is disabled, finish is only invoked from here or from setKeepAlive
			boost::mutex::scoped_lock lock(_mutex);
			if (_childrenMissing->getVariable() == 0) {
				if (_state->getVariable() == IDLE) {
					if (!_keepAlive->getVariable()) {
						_state->setVariable(POSTWORK); // addSubTasks() can not be used any longer
						doFinish = true;
					}
				}
			}
			_blockFinalize->setVariable(false);
		}
	}
	if (doFinish) {
		finish();
	}
}

void ThreadTask::finish() {
	// Finish is guaranteed to only be invoked once - state is POSTWORK
	try {
		done();
	} catch(boost::thread_interrupted const&) {
		RW_THROW("Please catch boost::thread_interrupted exception thrown in ThreadTask done funtion!");
	}
	ParentCallback parentCallback;
	{
		boost::mutex::scoped_lock lock(_mutex);
		parentCallback = _parentCallback->getVariable();
	}
	{
		boost::mutex::scoped_lock lock(_mutex);
		_state->setVariable(DONE);
	}
	if (!parentCallback.empty())
		parentCallback(this);
}
