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

#ifndef RW_COMMON_THREADPOOL_HPP_
#define RW_COMMON_THREADPOOL_HPP_

/**
 * @file ThreadPool.hpp
 *
 * \copydoc rw::common::ThreadPool
 */

#include <boost/asio/io_service.hpp>
#include <boost/function/function_fwd.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace rw {
namespace common {

// Forward declarations
template <class T> class Ptr;
template <typename T> class ThreadSafeVariable;

//! @addtogroup common

//! @{
/**
 * @brief A thread pool that can be assigned work.
 *
 * Work is handled in a FIFO manner, and the pool is intended to be very simple and basic.
 *
 * For more complex behaviour please look at the ThreadTask type,
 * which has a higher abstraction for adding tasks to a ThreadPool.
 */
class ThreadPool {
public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<ThreadPool> Ptr;

    /**
     * @brief Create new thread pool using the given number of threads.
     *
     * If no argument is given, the maximum number of hardware threads on the system is used (number of CPUs, cores or hyperthreading units).
     *
     * If number of threads is set to 0, the addWork function will be blocking (work is executed in thread that invokes addWork).
     *
     * @param threads number of threads to use - default is the number of hardware threads available on the system.
     */
    ThreadPool(int threads = -1);

    //! @brief Destruct the pool and all threads (this calls the stop function first).
    virtual ~ThreadPool();

    //! @brief Get number of threads in the pool.
    unsigned int getNumberOfThreads() const;

    /**
     * @brief Stop processing more work in the queue, and try to stop running work if possible.
     *
     * Long-running work should check if the isStopping function returns true and shut down gracefully.
     *
     * Interrupts are issued, so if there is interruption points in the WorkFunction,
     * the work should check for boost::thread_interrupted exceptions and shut down gracefully.
     */
    void stop();

    /**
     * @brief Check if work tasks are supposed to shut itself down.
     *
     * This function should be called from long-running worker functions to let them shut down gracefully.
     *
     * @return true if thread should shut down.
     */
    bool isStopping();

    //! @brief The type for a work function that can be assigned to the pool.
    typedef boost::function<void(ThreadPool*)> WorkFunction;

    //! @brief Add work to the thread pool.
    void addWork(WorkFunction work);

    /**
     * @brief Get the number of current tasks in the queue (tasks are removed from queue when done).
     * @return the number of current tasks.
     */
    unsigned int getQueueSize();

    //! @brief Wait until the task queue becomes empty.
    void waitForEmptyQueue();

private:
    void runWrap(WorkFunction work);

    boost::asio::io_service _service;
    boost::asio::io_service::work* _work;
    boost::thread_group _threads;
    const unsigned int _threadsNumber;
    ThreadSafeVariable<bool>* _postStop;
    ThreadSafeVariable<unsigned int>* _queueSize;
    boost::mutex _queueSizeMutex;

};
//! @}
} /* namespace common */
} /* namespace rw */
#endif /* RW_COMMON_THREADPOOL_HPP_ */
