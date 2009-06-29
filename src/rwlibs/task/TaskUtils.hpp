/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/


#ifndef RWLIBS_TASK_TASKUTILS_HPP
#define RWLIBS_TASK_TASKUTILS_HPP

#include "Task.hpp"
#include "Motion.hpp"
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
    static rw::common::Ptr<Task<T> > pathToTask(const std::vector<T>& path) {
        rw::common::Ptr<Task<T> > task = ownedPtr(new Task<T>());
        typedef rw::common::Ptr<Target<T> > TargetPtr;
        TargetPtr previous = NULL;
        for (typename std::vector<T>::const_iterator it = path.begin(); it != path.end(); ++it) {
        	std::cout<<"."<<std::endl;
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
