#ifndef VISGRABGRASPTASK_HPP_
#define VISGRABGRASPTASK_HPP_

#include <rw/common/Ptr.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwsim/simulator/GraspTask.hpp>

/**
 * @brief a container for describing one or multiple grasping tasks. It is based on the rwlibs::tasks library
 *
 * Definition of VisGraBGraspTask xml format
 *
 */

class VisGraBGraspTask: public GraspTask {
public:
    typedef rw::common::Ptr<VisGraBGraspTask> Ptr;

    //! the possible discrete outcomes of a single task simulation
    typedef enum Status {
        UnInitialized = 0,
        Success, CollisionInitially,
        ObjectMissed, ObjectDropped,
        ObjectSlipped, TimeOut,
        SimulationFailure,
        InvKinFailure,
        PoseEstimateFailure
     } TestStatus;

    VisGraBGraspTask(rwlibs::task::CartesianTask::Ptr task):GraspTask(task){}

    static void saveVisGraBResult(VisGraBGraspTask::Ptr task, const std::string& name );

    static std::vector<VisGraBGraspTask::Ptr> load(const std::string& name);
};

#endif /* GRASPTASK_HPP_ */
