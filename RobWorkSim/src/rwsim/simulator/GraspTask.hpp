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
#include <rw/sensor/Contact3D.hpp>



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
        Success,
        CollisionInitially,
        ObjectMissed,
        ObjectDropped,
        ObjectSlipped,
        TimeOut,
        SimulationFailure,
        InvKinFailure,
        PoseEstimateFailure,
        CollisionFiltered,
        CollisionObjectInitially,
        CollisionEnvironmentInitially,
        SizeOfStatusArray
     } TestStatus;

    GraspTask(){}
     //:_task(rw::common::ownedPtr(new rwlibs::task::CartesianTask() ) ){}
    GraspTask(rwlibs::task::CartesianTask::Ptr task);

    rwlibs::task::CartesianTask::Ptr toCartesianTask();

    std::string getGripperID();
    std::string getTCPID();
    std::string getGraspControllerID();
    void setGripperID(const std::string& id);
    void setTCPID(const std::string& id);
    void setGraspControllerID(const std::string& id);

    void addSubTask( class GraspSubTask& stask){ _subtasks.push_back(stask); };

    /**
     * @brief get the subtasks
     * @return
     */
    std::vector<class GraspSubTask>& getSubTasks(){ return _subtasks;};


    void filterTasks( std::vector<GraspTask::TestStatus> &includeMask);


    static std::string toString(TestStatus status){
        static std::string strArr[] = {"UnInitialized",
                                        "Success",
                                        "CollisionInitially",
                                        "ObjectMissed",
                                        "ObjectDropped",
                                        "ObjectSlipped",
                                        "TimeOut",
                                        "SimulationFailure",
                                        "InvKinFailure",
                                        "PoseEstimateFailure",
                                        "CollisionFiltered",
                                        "CollisionObjectInitially",
                                        "CollisionEnvironmentInitially",
                                        "SizeOfStatusArray"};
        return strArr[status];
    }

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


    /**
     * @brief load a GraspTask from file
     * @param name
     * @return
     */
    static GraspTask::Ptr load(const std::string& name);


private:
    std::vector<class GraspSubTask> _subtasks;
    //rwlibs::task::CartesianTask::Ptr _task;
    std::string _gripperID, _tcpID, _graspControllerID;
};


struct GraspResult {
    typedef rw::common::Ptr<GraspResult> Ptr;

    GraspResult():testStatus(GraspTask::UnInitialized),liftresult(0.0){}

    int testStatus;
    // the distance that
    double liftresult;

    // configuration of gripper when grasp is done
    rw::math::Q gripperConfigurationGrasp;
    // configuration of gripper when lift is done
    rw::math::Q gripperConfigurationLift;

    // quality of grasp before lifting
    rw::math::Q qualityBeforeLifting;
    // quality of grasp after lifting
    rw::math::Q qualityAfterLifting;

    // transform of target in object frame
    rw::math::Transform3D<> objectTtcpTarget;
    // transform after approach
    rw::math::Transform3D<> objectTtcpApproach;
    // transform after grasp
    rw::math::Transform3D<> objectTtcpGrasp;
    // transform after lift
    rw::math::Transform3D<> objectTtcpLift;

    // transform after approach
    std::vector<rw::math::Transform3D<> > gripperTobjects;
    // all contacts
    std::vector<rw::sensor::Contact3D> contactsGrasp, contactsLift;
};

class GraspTarget {
public:
    GraspTarget(){}
    GraspTarget(const rw::math::Transform3D<> &p):pose(p){}
    rw::math::Transform3D<> pose;

    GraspResult::Ptr getResult(){
        if(result==NULL)
            result = rw::common::ownedPtr( new GraspResult() );
        return result;
    }

    GraspResult::Ptr result;
};

class GraspSubTask {
public:

    void addTarget(const rw::math::Transform3D<>& target){
        targets.push_back( GraspTarget( target) );
    }

    void addTarget(const GraspTarget& target){
        targets.push_back(target);
    }



    rw::math::Q& getOpenQ(){ return openQ; }
    rw::math::Q& getCloseQ(){ return closeQ; }
    rw::math::Q& getTauMax(){ return tauMax; }

    void setOpenQ( const rw::math::Q& q){ openQ = q; }
    void setCloseQ( const rw::math::Q& q){ closeQ = q; }
    void setTauMax( const rw::math::Q& q){ tauMax = q; }

    const rw::math::Transform3D<>& getOffset(){ return offset; }

    void setRetract( const rw::math::Transform3D<>& t3d){ retract = t3d; }
    bool hasRetract(){ return !(retract == rw::math::Transform3D<>::identity()); }
    const rw::math::Transform3D<>& getRetract(){ return retract; }

    void setApproach( const rw::math::Transform3D<>& t3d){ approach = t3d; }
    bool hasApproach(){ return !(approach == rw::math::Transform3D<>::identity()); }
    const rw::math::Transform3D<>& getApproach(){ return approach; }

    void setRefFrame(const std::string& rframe){
        refframe = rframe;
    }
    std::string getRefFrame(){
        if(refframe=="")
            return "WORLD";
        return refframe;
    }

    std::vector<GraspTarget>& getTargets(){ return targets;};

    std::string getTaskID(){ return taskID; }
    void setTaskID(const std::string& ID){ taskID = ID; }

    std::string refframe;
    std::string objectID;
    rw::math::Transform3D<> offset, approach, retract;
    rw::math::Q openQ, closeQ, tauMax;
    std::vector<GraspTarget> targets;

    std::string taskID;
};


#endif /* GRASPTASK_HPP_ */
