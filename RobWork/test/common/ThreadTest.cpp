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

#include "../TestSuiteConfig.hpp"

#include <rw/common/Ptr.hpp>
#include <rw/common/Timer.hpp>
#include <rw/common/ThreadPool.hpp>
#include <rw/common/ThreadSafeVariable.hpp>
#include <rw/common/ThreadTask.hpp>
#include <rw/math/Random.hpp>

#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/test/unit_test_monitor.hpp>

using namespace rw::common;
using rw::math::Random;

static const unsigned int THREADS = 3;
static const unsigned int JOBS = 50;
static const unsigned int STOPJOB = 10; // ThreadPool should be able to shut down before reaching job JOBS
static ThreadSafeVariable<bool> requestStop(false);
static boost::mutex jobMutex;
static bool jobDone[JOBS];
static boost::mutex testMutex;

void workFunction(ThreadPool* pool, unsigned int id) {
	Timer::sleepMs(50);
	{
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(pool->getQueueSize() > 0);
		BOOST_CHECK(pool->isStopping() == false);
	}
	try {
		if (pool->isStopping())
			return;
		boost::mutex::scoped_lock lock(jobMutex);
		jobDone[id] = true;
	} catch(boost::thread_interrupted &e) {
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(pool->isStopping() == true);
	}
}

void workFunctionStop(ThreadPool* pool, unsigned int id) {
	try {
		Timer::sleepMs(50);
		{
			boost::mutex::scoped_lock lock(testMutex);
			BOOST_CHECK(pool->getQueueSize() > 0);
		}
		if (id >= STOPJOB) {
			requestStop = true;
			Timer::sleepMs(500); // slow it a bit down to give main thread time to call stop
		}
		if (pool->isStopping())
			return;
		boost::mutex::scoped_lock lock(jobMutex);
		jobDone[id] = true;
	} catch(boost::thread_interrupted &e) {
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(pool->isStopping() == true);
	}
}

int runPool(bool stop) {
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(THREADS));
	{
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(pool->getNumberOfThreads() == THREADS);
		BOOST_CHECK(pool->getQueueSize() == 0);
	}

	for (unsigned int i = 0; i < JOBS; i++) {
		jobDone[i] = false;
		ThreadPool::WorkFunction work;
		if (!stop)
			work = boost::bind(&workFunction,_1,i);
		else
			work = boost::bind(&workFunctionStop,_1,i);
		pool->addWork(work);
	}

	if (stop) {
		bool reqStop = requestStop();
		while (!reqStop) reqStop = requestStop.waitForUpdate(reqStop);
		pool->stop();
		boost::mutex::scoped_lock lock(testMutex);
    	BOOST_CHECK(pool->isStopping() == true);
        BOOST_CHECK(pool->getQueueSize() == 0);
	}

	pool->waitForEmptyQueue();

	boost::mutex::scoped_lock lock(testMutex);
    BOOST_CHECK(pool->getQueueSize() == 0);

    if (stop) {
    	// We can only test the first and last jobs (and hope that we are lucky)
    	BOOST_CHECK_MESSAGE(jobDone[0], "First job did not run.");
    	BOOST_CHECK_MESSAGE(!jobDone[JOBS-1], "Last job (" << JOBS << ") ran (should have been stopped before reaching this).");
    } else {
    	for (unsigned int i = 0; i < JOBS; i++) {
    		BOOST_CHECK_MESSAGE(jobDone[i], "Job " << i << " of " << JOBS << " did not run.");
    	}
    }
    pool = NULL;

	return 0;
}

int runTaskEmpty() {
	boost::mutex::scoped_lock lock(testMutex);
	ThreadPool::Ptr dummyPool = ownedPtr(new ThreadPool(THREADS));
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(THREADS));

	ThreadTask::Ptr task = ownedPtr(new ThreadTask(pool));

	BOOST_CHECK(task->getState() == ThreadTask::INITIALIZATION);
	BOOST_CHECK(task->getThreadPool() == pool);
	BOOST_CHECK(task->setThreadPool(dummyPool) == true);
	BOOST_CHECK(task->getThreadPool() == dummyPool);
	BOOST_CHECK(task->setThreadPool(pool) == true);
	BOOST_CHECK(task->getThreadPool() == pool);
	BOOST_CHECK(task->keepAlive() == false);
	task->setKeepAlive(true);
	BOOST_CHECK(task->keepAlive() == true);
	task->setKeepAlive(false);
	BOOST_CHECK(task->keepAlive() == false);
	BOOST_CHECK(task->getSubTasks().size() == 0);

	ThreadTask::Ptr subtask1 = ownedPtr(new ThreadTask(task));
	BOOST_CHECK(task->addSubTask(subtask1) == true);
	BOOST_CHECK(task->getSubTasks().size() == 1);

	task->setKeepAlive(true);
	BOOST_CHECK(task->execute() == true);
	BOOST_CHECK(task->execute() == false);
	BOOST_CHECK(task->getState() != ThreadTask::INITIALIZATION);

	BOOST_CHECK(task->setThreadPool(dummyPool) == false);
	dummyPool = NULL;
	BOOST_CHECK(task->getThreadPool() == pool);

	ThreadTask::Ptr subtask2 = ownedPtr(new ThreadTask(task));
	BOOST_CHECK(task->addSubTask(subtask2) == true);
	BOOST_CHECK(task->getSubTasks().size() == 2);
	task->setKeepAlive(true);

	ThreadTask::TaskState state;
	state = task->getState();
	while (state != ThreadTask::IDLE) state = task->wait(state);
	BOOST_CHECK(task->getState() == ThreadTask::IDLE);

	task->setKeepAlive(false);

	task->waitUntilDone();
	BOOST_CHECK(subtask1->getState() == ThreadTask::DONE || subtask2->getState() == ThreadTask::POSTWORK);
	BOOST_CHECK(subtask2->getState() == ThreadTask::DONE || subtask2->getState() == ThreadTask::POSTWORK);
	BOOST_CHECK(task->getState() == ThreadTask::DONE);

	return 0;
}

struct Event {
	typedef enum TaskType {
		MainTask,
		Task,
		SubTask
	} TaskType;
	typedef enum TaskFunction {
		Run,
		SubTaskDone,
		Idle,
		Done
	} TaskFunction;
	Event(TaskType type, TaskFunction function, unsigned int taskID, unsigned int subTaskID): type(type), function(function), taskID(taskID), subTaskID(subTaskID) {};
	TaskType type;
	TaskFunction function;
	unsigned int taskID;
	unsigned int subTaskID;
};
static boost::mutex eventMutex;
static std::vector<Event> events;

class SubTask: public ThreadTask {
public:
	SubTask(ThreadPool::Ptr pool, unsigned int taskID, unsigned int id): ThreadTask(pool), _taskID(taskID), _id(id) {};
	unsigned int getID() const { return _id; }
	void run() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::Run,_taskID,_id));
		Timer::sleepMs(Random::ranI(0,25));
	};
	void subTaskDone(ThreadTask* subtask) { // should not be called!
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::SubTaskDone,_taskID,_id));
	};
	void idle() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::Idle,_taskID,_id));
	};
	void done() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::SubTask,Event::Done,_taskID,_id));
	};
private:
	const unsigned int _taskID;
	const unsigned int _id;
};

class Task: public ThreadTask {
public:
	Task(ThreadPool::Ptr pool, unsigned int id): ThreadTask(pool), _id(id), _idleTaskAdded(false) {}
	unsigned int getID() const { return _id; }
	void run() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::Run,_id,0));
		}
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,0))) == true);
		BOOST_CHECK(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,1))) == true);
	};
	void subTaskDone(ThreadTask* subtask) {
		SubTask* stask = static_cast<SubTask*>(subtask);
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::SubTaskDone,_id,stask->getID()));
		}
		if (stask->getID() == 1) {
			boost::mutex::scoped_lock lock(testMutex);
			BOOST_CHECK(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,2))) == true);
		}
	};
	void idle() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::Idle,_id,0));
		}
		if (!_idleTaskAdded) {
			_idleTaskAdded = true;
			boost::mutex::scoped_lock lock(testMutex);
			BOOST_CHECK(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,3))) == true);
		}
	};
	void done() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::Task,Event::Done,_id,0));
		}
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(addSubTask(ownedPtr(new SubTask(getThreadPool(),_id,4))) == false);
	};
private:
	const unsigned int _id;
	bool _idleTaskAdded;
};

class MainTask: public ThreadTask {
public:
	MainTask(ThreadPool::Ptr pool): ThreadTask(pool) {};
	void run() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::MainTask,Event::Run,0,0));
		}
		for(unsigned int i = 0; i < JOBS; i++) {
			ThreadTask::Ptr task = ownedPtr(new Task(getThreadPool(),i));
			boost::mutex::scoped_lock lock(testMutex);
			BOOST_CHECK(addSubTask(task) == true);
		}
	};
	void subTaskDone(ThreadTask* subtask) {
		Task* task = static_cast<Task*>(subtask);
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::MainTask,Event::SubTaskDone,task->getID(),0));
	};
	void idle() {
		boost::mutex::scoped_lock lock(eventMutex);
		events.push_back(Event(Event::MainTask,Event::Idle,0,0));
	};
	void done() {
		{
			boost::mutex::scoped_lock lock(eventMutex);
			events.push_back(Event(Event::MainTask,Event::Done,0,0));
		}
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(addSubTask(ownedPtr(new Task(getThreadPool(),100))) == false);
	};
};

int runTask() {
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(THREADS));
	ThreadTask::Ptr task = ownedPtr(new MainTask(pool));
	{
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(task->execute() == true);
		BOOST_CHECK(task->execute() == false);
		BOOST_CHECK(task->getState() != ThreadTask::INITIALIZATION);
	}
	task->waitUntilDone();
	{
		boost::mutex::scoped_lock lock(testMutex);
		BOOST_CHECK(task->getSubTasks().size() == JOBS);
		BOOST_CHECK(task->getState() == ThreadTask::DONE);
	}

	std::vector<Event> eventList;
	{
		boost::mutex::scoped_lock lock(eventMutex);
		eventList = events;
	}
	// Print list
	/*BOOST_FOREACH(const Event &event, eventList) {
		if (event.type == Event::MainTask)
			std::cout << "MainTask ";
		else if (event.type == Event::Task)
			std::cout << "Task ";
		else if (event.type == Event::SubTask)
			std::cout << "SubTask ";
		if (event.function == Event::Run)
			std::cout << "Run ";
		else if (event.function == Event::SubTaskDone)
			std::cout << "SubTaskDone ";
		else if (event.function == Event::Idle)
			std::cout << "Idle ";
		else if (event.function == Event::Done)
			std::cout << "Done ";
		std::cout << event.taskID << " " << event.subTaskID << std::endl;
	}*/
	// Verify the event order
	bool mainRun = false;
	bool mainSubTask[JOBS];
	bool mainIdle = false;
	bool mainDone = false;
	bool taskRun[JOBS];
	bool taskSubTask[JOBS][4];
	bool taskSubTaskNewAdded[JOBS];
	bool taskIdleFirst[JOBS];
	bool taskIdleSecond[JOBS];
	bool taskDone[JOBS];
	bool subTaskRun[JOBS][4];
	bool subTaskIdle[JOBS][4];
	bool subTaskDone[JOBS][4];
	for (unsigned int i = 0; i < JOBS; i++) {
		mainSubTask[i] = false;
		taskRun[i] = false;
		taskSubTaskNewAdded[i] = false;
		taskIdleFirst[i] = false;
		taskIdleSecond[i] = false;
		taskDone[i] = false;
		for (unsigned int j = 0; j < 4; j++) {
			taskSubTask[i][j] = false;
			subTaskRun[i][j] = false;
			subTaskIdle[i][j] = false;
			subTaskDone[i][j] = false;
		}
	}
	boost::mutex::scoped_lock lock(testMutex);
	BOOST_FOREACH(const Event &event, eventList) {
		if (!mainRun) {
			if (event.type == Event::MainTask && event.function == Event::Run)
				mainRun = true;
			BOOST_CHECK(mainRun == true);
		} else {
			BOOST_CHECK(event.taskID < JOBS);
			BOOST_CHECK(event.subTaskID < 4);
			if (event.function == Event::Run) {
				BOOST_CHECK(!(event.type == Event::MainTask));
				if (event.type == Event::Task) {
					BOOST_CHECK(taskRun[event.taskID] == false);
					taskRun[event.taskID] = true;
				} else if (event.type == Event::SubTask) {
					BOOST_CHECK(taskRun[event.taskID] == true);
					BOOST_CHECK(subTaskRun[event.taskID][event.subTaskID] == false);
					subTaskRun[event.taskID][event.subTaskID] = true;
				}
			} else if (event.function == Event::SubTaskDone) {
				BOOST_CHECK(!(event.type == Event::SubTask));
				if (event.type == Event::MainTask) {
					BOOST_CHECK(mainSubTask[event.taskID] == false);
					BOOST_CHECK(taskDone[event.taskID] == true);
					mainSubTask[event.taskID] = true;
				} else if (event.type == Event::Task) {
					BOOST_CHECK(taskRun[event.taskID] == true);
					BOOST_CHECK(taskSubTask[event.taskID][event.subTaskID] == false);
					BOOST_CHECK(subTaskDone[event.taskID][event.subTaskID] == true);
					if (!taskSubTaskNewAdded[event.taskID]) {
						BOOST_CHECK(event.subTaskID < 2);
						if (event.subTaskID == 1)
							taskSubTaskNewAdded[event.taskID] = true;
					} else {
						if (!taskIdleFirst[event.taskID]) {
							BOOST_CHECK(event.subTaskID < 3);
						}
					}
					taskSubTask[event.taskID][event.subTaskID] = true;
				}
			} else if (event.function == Event::Idle) {
				if (event.type == Event::MainTask) {
					for (unsigned int i = 0; i < JOBS; i++) {
						BOOST_CHECK(mainSubTask[i] == true);
					}
					mainIdle = true;
				} else if (event.type == Event::Task) {
					if (!taskIdleFirst[event.taskID]) {
						BOOST_CHECK(taskSubTask[event.taskID][0] == true);
						BOOST_CHECK(taskSubTask[event.taskID][1] == true);
						BOOST_CHECK(taskSubTask[event.taskID][2] == true);
						taskIdleFirst[event.taskID] = true;
					} else {
						BOOST_CHECK(taskIdleSecond[event.taskID] == false);
						BOOST_CHECK(taskSubTask[event.taskID][3] == true);
						taskIdleSecond[event.taskID] = true;
					}
				} else if (event.type == Event::SubTask) {
					BOOST_CHECK(subTaskIdle[event.taskID][event.subTaskID] == false);
					BOOST_CHECK(subTaskRun[event.taskID][event.subTaskID] == true);
					subTaskIdle[event.taskID][event.subTaskID] = true;
				}
			} else if (event.function == Event::Done) {
				if (event.type == Event::MainTask) {
					BOOST_CHECK(mainDone == false);
					BOOST_CHECK(mainIdle == true);
					mainDone = true;
				} else if (event.type == Event::Task) {
					BOOST_CHECK(taskDone[event.taskID] == false);
					BOOST_CHECK(taskIdleSecond[event.taskID] == true);
					taskDone[event.taskID] = true;
				} else if (event.type == Event::SubTask) {
					BOOST_CHECK(subTaskDone[event.taskID][event.subTaskID] == false);
					BOOST_CHECK(subTaskIdle[event.taskID][event.subTaskID] == true);
					subTaskDone[event.taskID][event.subTaskID] = true;
				}
			}
		}
	}
	BOOST_CHECK(mainRun == true);
	BOOST_CHECK(mainIdle == true);
	BOOST_CHECK(mainDone == true);
	for (unsigned int i = 0; i < JOBS; i++) {
		BOOST_CHECK(mainSubTask[i] == true);
		BOOST_CHECK(taskRun[i] == true);
		BOOST_CHECK(taskSubTaskNewAdded[i] == true);
		BOOST_CHECK(taskIdleFirst[i] == true);
		BOOST_CHECK(taskIdleSecond[i] == true);
		BOOST_CHECK(taskDone[i] == true);
		for (unsigned int j = 0; j < 4; j++) {
			BOOST_CHECK(taskSubTask[i][j] == true);
			BOOST_CHECK(subTaskRun[i][j] == true);
			BOOST_CHECK(subTaskIdle[i][j] == true);
			BOOST_CHECK(subTaskDone[i][j] == true);
		}
	}

	return 0;
}

BOOST_AUTO_TEST_CASE(ThreadTest) {
    BOOST_TEST_MESSAGE("- ThreadTest");

	BOOST_TEST_MESSAGE("-- ThreadPoolTest");
    // get the execution monitor instance
    boost::unit_test::unit_test_monitor_t& theMonitor = boost::unit_test::unit_test_monitor_t::instance();

	BOOST_TEST_MESSAGE("--- Ordinary test");
    // Do a test where thread pool is allowed to finish all work (timeout = 5 seconds)
    theMonitor.p_timeout.set( 5 );
    try {
    	theMonitor.execute( boost::bind( &runPool, false ) );
    } catch(boost::execution_exception &e) {
    	if (e.code() == boost::execution_exception::timeout_error) {
			boost::mutex::scoped_lock lock(testMutex);
    		BOOST_CHECK_MESSAGE(false, "Timeout while testing ThreadPool - possibly a deadlock.");
    	} else {
    		throw(e);
    	}
    }
	BOOST_TEST_MESSAGE("--- Abort test");
    // Do a test where thread pool is stopped before work is finished (timeout = 10 seconds)
    theMonitor.p_timeout.set( 20 );
    try {
    	theMonitor.execute( boost::bind( &runPool, true ) );
    } catch(boost::execution_exception &e) {
    	if (e.code() == boost::execution_exception::timeout_error) {
			boost::mutex::scoped_lock lock(testMutex);
    		BOOST_CHECK_MESSAGE(false, "Timeout while testing ThreadPool - possibly a deadlock.");
    	} else {
    		throw(e);
    	}
    }

	BOOST_TEST_MESSAGE("-- ThreadTaskTest");
	BOOST_TEST_MESSAGE("--- Simple test");
    // Do a simple test with empty tasks
    theMonitor.p_timeout.set( 5 );
    try {
    	theMonitor.execute( boost::bind( &runTaskEmpty ) );
    } catch(boost::execution_exception &e) {
    	if (e.code() == boost::execution_exception::timeout_error) {
			boost::mutex::scoped_lock lock(testMutex);
    		BOOST_CHECK_MESSAGE(false, "Timeout while testing ThreadTask - possibly a deadlock.");
    	} else {
    		throw(e);
    	}
    }
	BOOST_TEST_MESSAGE("--- Invocation order test");
    // Do a complex test where order of function calls is tested
    theMonitor.p_timeout.set( 20 );
    try {
    	theMonitor.execute( boost::bind( &runTask ) );
    } catch(boost::execution_exception &e) {
    	if (e.code() == boost::execution_exception::timeout_error) {
			boost::mutex::scoped_lock lock(testMutex);
    		BOOST_CHECK_MESSAGE(false, "Timeout while testing ThreadTask - possibly a deadlock.");
    	} else {
    		throw(e);
    	}
    }
}
