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

#ifndef RW_COMMON_THREADSAFEQUEUE_HPP_
#define RW_COMMON_THREADSAFEQUEUE_HPP_

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

namespace rw {
namespace common {

    /**
     * @brief Queue class which is thread safe, eg. multiple threads may
     * use it at the same time.
     */
    template <class T>
    class ThreadSafeQueue {
    public:
        //! constructor
        ThreadSafeQueue() : _size(0) {};

        /**
         * @brief test if the queue is empty
         * @return true if queue is empty, false otherwise
         */
        inline bool empty() {
            boost::mutex::scoped_lock lock(_mutex);
            return _queue.empty();
        };

        /**
         * @brief add data to the queue
         * @param wp [in] data to add to queue
         */
        inline void push(T wp) {
            boost::mutex::scoped_lock lock(_mutex);
            _queue.push(wp);
            _size++;
            //lock.unlock();
            _cond.notify_one();
        };

        /**
         * @brief try to pop data from the queue. If no data is available then false is returned
         * if data is available then true is returned and wp is set.
         * @param wp [out]
         * @return true is wp set, false otherwise
         */
        inline bool try_pop(T *wp) {
            boost::mutex::scoped_lock lock(_mutex);
            if (_queue.empty())
                return false;

            *wp = _queue.front();
            _queue.pop();
            _size--;

            return true;
        }

        /**
         * @brief pop data from the queue in blocking manner. That is it will wait until
         * data is added to the queue if it is initially empty.
         * @param wp [out] data that is popped from the queue
         * @return
         */
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

        /**
         * @brief test if the queue contain a specific data value. This is slow O(N)
         * so keep that in mind when using it.
         * @param value [in] the value to compare with.
         * @return
         */
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
}
}

#endif
