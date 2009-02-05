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


class TaskUtils
{
public:
    virtual ~TaskUtils() {}

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
