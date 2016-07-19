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

#ifndef RW_COMMON_THREADTASK_HPP_
#define RW_COMMON_THREADTASK_HPP_

/**
 * @file ThreadTask.hpp
 *
 * \copydoc rw::common::ThreadTask
 */

#include "Ptr.hpp"

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

#include <vector>
#include <list>

namespace rw {
namespace common {

// Forward declarations
class Exception;
class ThreadPool;
template <typename T> class ThreadSafeVariable;

//! @addtogroup common

//! @{
/**
 * @brief A task that facilitates the use of a hierarchic tree of tasks and subtasks.
 *
 * Often parallel processing can be done at multiple levels. Typically it is not known beforehand
 * if some task can be split into multiple smaller subtasks or not. The ThreadTask keeps track of
 * the state of all its subtasks - only when all subtasks have been processed, the parent task will
 * be able to finish. Instead of finishing, it can also choose to add new subtasks that depends
 * on the result of the previously run subtasks.
 *
 * The ThreadTask can utilize a ThreadPool of arbitrary size (down to 0 threads).
 * When 0 threads are used, the addSubTask() function will be blocking and execute the work immediately.
 * If more than 0 threads are used, the addSubTask function will
 * return immediately, and the task is instead added to the work queue for processing when a thread
 * becomes available.
 *
 * There are two ways to use the ThreadTask:
 *
 *  - Use it as a grouping mechanism: here one or more subtasks can be added for parallel processing.
 *  One can use the ThreadPool size to adjust the number of threads one wants to use for a specific
 *  application.
 *
 *  - Subclass it and override the four standard functions to make more complex "branch-and-combine-results"
 *  type of tasks.
 *
 *  The four standard functions are as follows:
 *
 *  run() is the main work unit of the ThreadTask.
 *
 *  subTaskDone() is called each time a subtask has ended.
 *
 *  idle() is called when the task has finished its run() function and all subtasks has ended.
 *
 *  done() is the final function called before the ThreadTask is ended completely.
 *
 * Please remember that the four functions can in general not be expected to be run in the same thread!
 * In the first three functions it is allowed to add new subtasks to keep the thread running. In the done()
 * function this is not allowed (it is simply ignored as the task will end immediately after this function
 * has been called).
 */
class ThreadTask {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<ThreadTask> Ptr;

    /**
     * @brief The different execution states of a task.
     */
    typedef enum TaskState {
    	INITIALIZATION,//!< Before execute has been called the first time.
    	IN_QUEUE,      //!< Task has added its work to the pool, waiting for thread to become available.
    	EXECUTING,     //!< The run function of the task is executing in the pool.
    	CHILDREN,      //!< The run loop has finished, but children are still running.
    	IDLE,          //!< The idle function is called and subtasks can still be added. If the keep alive has been enabled, the task will stay in this state and not continue to the POSTWORK state.
    	POSTWORK,      //!< The task does not accept any more new subtasks - the done function is invoked.
    	DONE           //!< The done function has been invoked, and the task is now completed and dead.
    } TaskState;

    /**
     * @brief Create task that will inherit parents thread pool.
     *
     * @param parent [in] the parent task to take the thread pool from.
     */
	ThreadTask(ThreadTask::Ptr parent);

    /**
     * @brief Create task that will use a specific thread pool.
     *
     * If no thread pool is given, the task will not use parallelization.
     *
     * @param pool [in] (optional) a pointer to the ThreadPool to use.
     */
	ThreadTask(rw::common::Ptr<ThreadPool> pool = NULL);

    /**
     * @brief Destruct this task.
     */
	virtual ~ThreadTask();

	/**
	 * @brief Set which ThreadPool to use to do the actual execution of work.
	 *
	 * When execution is started the pool can not be changed anymore.
	 *
	 * @param pool [in] pointer to the pool
	 * @return true if change was successful, false otherwise.
	 */
	bool setThreadPool(rw::common::Ptr<ThreadPool> pool);

	/**
	 * @brief Get the ThreadPool that is used by this task currently.
	 *
	 * @return pointer to the ThreadPool.
	 */
	rw::common::Ptr<ThreadPool> getThreadPool();

    /**
     * @name Functions that can be implemented by subclasses.
     * @brief It is optional to implement these functions, but normally at least one should be implemented.
     */
    ///@{
	//! @brief Function is the first function executed to do the actual work (new subtasks can be added in this function).
	virtual void run();

	/**
	 * @brief Function is executed each time a subtask has finished (new subtasks can be added in this function).
	 *
	 * If #registerFailure is used to register failures in subtasks, this function should handle or propagate
	 * the failures.
	 *
	 * The default implementation of this function is as follows:
	 * \verbatim
	  	BOOST_FOREACH(const Exception& e, subtask->getExceptions()) {
			registerFailure(e);
		}
		\endverbatim
	 *
	 * @param subtask [in] the subtask that just finished.
	 */
	virtual void subTaskDone(ThreadTask* subtask);

	//! @brief Function is executed when the task becomes idle (new subtasks can be added in this function).
	virtual void idle();

	//! @brief Function is executed when work is finished (at this point new subtasks can NOT be added).
	virtual void done();
    ///@}

    /**
     * @brief Start executing the work in this task and all subtasks already added, by using the assigned ThreadPool.
     * @note There is no guarantee that the parent run() function starts executing before the childrens run() functions.
     * @return true if execution started successfully, false if already running or thread has finished.
     */
    bool execute();

    /**
     * @brief Wait until state of task changes (blocking).
     *
     * Remember to check if the new State is the desired (for instance DONE).
     *
     * @param previous [in] the previous state (wait for changes from this)
     * @return the new TaskState
     */
    TaskState wait(ThreadTask::TaskState previous);

    //! @brief Wait until state of task changes to DONE (blocking).
    void waitUntilDone();

    /**
     * @brief Get the current state of the task (non-blocking).
     * @return the current TaskState.
     */
    TaskState getState();

    /**
     * @brief Add a child task to this task.
     *
     * This task will not end before all child tasks has ended.
     * Never call execute on the child tasks - they will be executed by the parent task.
     *
     * @param subtask the ThreadTask to add as child.
     * @return true if child was added successfully (only if task has not already ended)
     */
    bool addSubTask(ThreadTask::Ptr subtask);

    /**
     * @brief Get the subtasks currently added to the ThreadTask.
     * @return a vector of subtasks.
     */
    std::vector<ThreadTask::Ptr> getSubTasks();

    /**
     * @brief Choose if the thread should exit automatically when all work and children has finished (this is the default).
     *
     * Remember to set this to false when there is no more work to be done by the task, to allow the task to stop.
     *
     * @param enable [in] true if the thread should NOT finish automatically.
     */
    void setKeepAlive(bool enable);

    /**
     * @brief Check is the task has keep alive option enabled.
     * @return true if task should be kept alive, false otherwise.
     */
    bool keepAlive();

    /**
     * @brief Mark the task as a failure by registering an exception.
     *
     * The user should override the #subTaskDone function in the parent task to
     * either handle the exceptions appropriately or propagate them further on.
     *
     * @param e [in] an exception describing the problem.
     */
    void registerFailure(const Exception& e);

    /**
     * @brief Get a list of exceptions registered in task and subtasks.
     * @return a list of exceptions.
     */
    std::list<Exception> getExceptions() const;

private:
    typedef boost::function<void(ThreadTask*)> ParentCallback;

    void runWrap(ThreadPool* pool);
    void doneWrap(ThreadPool* pool);
    void callbackParent(ThreadTask* task);
    void tryIdle();
    void finish();

	// Thread-safe data
    ThreadSafeVariable<rw::common::Ptr<ThreadPool> >* _pool;
	ThreadSafeVariable<TaskState>* _state;
	ThreadSafeVariable<bool>* _keepAlive;
	ThreadSafeVariable<bool>* _blockFinalize;
    ThreadSafeVariable<std::vector<rw::common::Ptr<ThreadTask> > >* _children;
    ThreadSafeVariable<unsigned int>* _childrenMissing;
    ThreadSafeVariable<ParentCallback>* _parentCallback;
    ThreadSafeVariable<std::list<Exception> >* _exceptions;

    // Mutex for exclusive access to atomic manipulation of all data
    boost::mutex _mutex;
};
//! @}
} /* namespace common */
} /* namespace rw */
#endif /* RW_COMMON_THREADTASK_HPP_ */
