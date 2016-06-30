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



#ifndef RWLIBS_TASK_TASKUTILS_HPP
#define RWLIBS_TASK_TASKUTILS_HPP

#include "Task.hpp"
#include "Target.hpp"

namespace rwlibs {
namespace task {


    /** @addtogroup task */
    /*@{*/

/**
 * @brief Collection of utility function associated to tasks
 */
class TaskUtils
{
public:
    virtual ~TaskUtils() {}

    /**
     * @brief Constructs a task with type T from a path.
     *
     * The first template argument T specifies the type of task, e.g. rw::math::Q or rw::math::Transform3D.
     * The second template argument M specifies which type of motion, e.g. P2PMotion or LinearMotion. It is required
     * that the motion M has a constructor taking two arguments of type T.
     *
     * @param path [in] Path to convert to task
     * @return The Task constructed
     */
    template <class T, class M>
	static typename Task<T>::Ptr pathToTask(const std::vector<T>& path) {
		typename Task<T>::Ptr task = ownedPtr(new Task<T>());
		typedef typename Target<T>::Ptr TargetPtr;
        TargetPtr previous = NULL;
        for (typename std::vector<T>::const_iterator it = path.begin(); it != path.end(); ++it) {
            TargetPtr target = task->addTargetByValue(*it);
            if (previous != NULL)
                task->addMotion(ownedPtr(new M(previous, target)));
            previous = target;
        }
        return task;
    }



private:
    TaskUtils() {};

};

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
