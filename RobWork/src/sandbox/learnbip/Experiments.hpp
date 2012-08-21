/*
 * Experiments.hpp
 *
 *  Created on: Aug 16, 2012
 *      Author: thomas
 */

#ifndef EXPERIMENTS_HPP_
#define EXPERIMENTS_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/sensor/Contact3D.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/GraspTask.hpp>

class Experiments {
public:
    typedef rw::common::Ptr<Experiments> Ptr;

    Experiments(){}

    void addExperiment( const struct Experiment& experiment){ _experiments.push_back(experiment); };
    std::vector<class Experiment>& getExperiments(){ return _experiments;};

    static void saveRWTask(Experiments::Ptr task, const std::string& name );
    static Experiments::Ptr load(const std::string& name);

private:
    Experiments(rwlibs::task::CartesianTask::Ptr task);
    rwlibs::task::CartesianTask::Ptr toCartesianTask();

    std::vector<class Experiment> _experiments;
};


struct Experiment {
    typedef rw::common::Ptr<Experiment> Ptr;

    Experiment():id(-1),testStatus(rwlibs::task::GraspTask::UnInitialized){}

    int id;
    rwlibs::task::GraspTask::TestStatus testStatus;

    rw::math::Transform3D<> worldTobjectReal;
    rw::math::Transform3D<> worldTobjectDetected;

    rw::math::Q gripperConfigurationTarget;
    rw::math::Q gripperConfigurationGrasp;
    rw::math::Q gripperConfigurationLift;
    rw::math::Q gripperConfigurationRelease;

    rw::math::Transform3D<> objectTtcpTarget;
    rw::math::Transform3D<> objectTtcpGrasp;
    rw::math::Transform3D<> objectTtcpLift;
    rw::math::Transform3D<> objectTtcpRelease;
};

#endif /* EXPERIMENTS_HPP_ */
