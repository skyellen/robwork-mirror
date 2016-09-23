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

#include "ThreadPool.hpp"
#include "ThreadSafeVariable.hpp"

#include <rw/common/macros.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

using namespace rw::common;

ThreadPool::ThreadPool(int threads):
	_work(new boost::asio::io_service::work(_service)),
	_threadsNumber(threads < 0 ? boost::thread::hardware_concurrency() : threads),
	_postStop(new ThreadSafeVariable<bool>(false)),
	_queueSize(new ThreadSafeVariable<unsigned int>(0))
{
	for (unsigned int i = 0; i < _threadsNumber; i++) {
		boost::function<std::size_t()> fct = boost::bind( static_cast< std::size_t (boost::asio::io_service::*) ()>(&boost::asio::io_service::run) , &_service);
		_threads.create_thread(fct);
	}
}

ThreadPool::~ThreadPool() {
	stop();
	delete _postStop;
	delete _queueSize;
}

unsigned int ThreadPool::getNumberOfThreads() const {
	return _threadsNumber;
}

void ThreadPool::stop() {
	_postStop->setVariable(true);
	_service.stop();
	delete _work;
	_work = NULL;
	_threads.interrupt_all();
	_threads.join_all();
	_queueSize->setVariable(0);
}

bool ThreadPool::isStopping() {
	return _postStop->getVariable();
}

void ThreadPool::addWork(WorkFunction work) {
	{
		boost::mutex::scoped_lock lock(_queueSizeMutex);
		_queueSize->setVariable(_queueSize->getVariable()+1);
	}
	if (_threads.size() == 0) {
		runWrap(work);
	} else {
		_service.post(boost::bind(&ThreadPool::runWrap,this,work));
	}
}

unsigned int ThreadPool::getQueueSize() {
	return _queueSize->getVariable();
}

void ThreadPool::waitForEmptyQueue() {
	unsigned int var = _queueSize->getVariable();
	while (var != 0) {
		var = _queueSize->waitForUpdate(var);
	}
}

void ThreadPool::runWrap(WorkFunction work) {
	try {
		work(this);
	} catch(boost::thread_interrupted const&) {
		RW_THROW("Please catch boost::thread_interrupted exception thrown in work function for ThreadPool!");
	}
	{
		boost::mutex::scoped_lock lock(_queueSizeMutex);
		_queueSize->setVariable(_queueSize->getVariable()-1);
	}
}
