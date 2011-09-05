/*
 * GraspTask.hpp
 *
 *  Created on: Aug 15, 2011
 *      Author: jimali
 */

#ifndef GRASPTASK_HPP_
#define GRASPTASK_HPP_

#include <rw/common/Ptr.hpp>
#include <rwlibs/task/Task.hpp>

/**
 * @brief a container for describing one or multiple grasping tasks. It is based on the rwlibs::tasks library
 *
 *
 * Definition of GraspTask xml format
 *
 * GraspTask<br>
 *  - p:string:"GripperName" - name of the
 *  - p:string:"ControllerName" - defaults to GraspController
 *  - p:string:"TCP" - name of the TCP frame
 *
 *
 *
 *
 */

class GraspTask {
public:
    typedef rw::common::Ptr<GraspTask> Ptr;

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

    GraspTask(rwlibs::task::CartesianTask::Ptr task):_task(task){}

    rwlibs::task::CartesianTask::Ptr getRootTask(){ return _task; }

    /**
     * @brief save as UIBK format
     * @param task
     * @param name
     */
    static void saveUIBK(GraspTask::Ptr task, const std::string& name );

    /**
     *
     * @param task
     * @param name
     */
    static void saveRWTask(GraspTask::Ptr task, const std::string& name );

    /**
     * @brief save grasp task in a comma seperated format
     * @param task
     * @param name
     */
    static void saveText(GraspTask::Ptr task, const std::string& name );

private:

    rwlibs::task::CartesianTask::Ptr _task;
};

#endif /* GRASPTASK_HPP_ */
