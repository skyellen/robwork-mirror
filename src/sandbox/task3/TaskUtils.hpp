/*
 * TaskUtils.hpp
 *
 *  Created on: Feb 5, 2009
 *      Author: lpe
 */

#ifndef RW_TASK3_TASKUTILS_HPP
#define RW_TASK3_TASKUTILS_HPP

#include "Task.hpp"
#include "Motion.hpp"
#include "Target.hpp"

namespace rw {
namespace task3 {


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
            TargetPtr target = task->addTarget(*it);
            if (previous != NULL)
                task->addMotion(ownedPtr(new M(previous, target)));
            previous = target;
        }
        return task;
    }



private:
    TaskUtils() {};

};

} //end namespace task3
} //end namespace rw

#endif //end include guard
