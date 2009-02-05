/*
 * Task.h
 *
 *  Created on: Jan 29, 2009
 *      Author: lpe
 */

#ifndef RW_TASK_TASK_HPP
#define RW_TASK_TASK_HPP

#include "Target.hpp"
#include "Entity.hpp"
#include "Action.hpp"
#include "Motion.hpp"
#include <rw/common/Ptr.hpp>

#include <map>
#include <string>

namespace rw {
namespace task3 {


class TaskBase;
typedef rw::common::Ptr<TaskBase> TaskBasePtr;

class TaskBase: public Entity
{
public:
    TaskBase():
        Entity(EntityType::Task)
    {}
    virtual ~TaskBase() {}
};

typedef rw::common::Ptr<TaskBase> TaskPtr;

template <class T>
class Task: public TaskBase {
public:
    //typedef Task<int> TTask;
    typedef rw::common::Ptr<Task<T> > TaskPtr;
    typedef rw::common::Ptr<Target<T> > TargetPtr;
    typedef rw::common::Ptr<Motion<T> > MotionPtr;
    typedef rw::common::Ptr<Action> ActionPtr;

    Task()
    {}

    virtual ~Task() {}

    void addAugmentation(rw::common::Ptr<Task<T> > task, const std::string& id) {
        _augmentations[id] = task;
    }

    bool hasAugmentation(const std::string& id) {
        return _augmentations.find(id) != _augmentations.end();
    }

    TaskPtr getAugmentation(const std::string& id) {
        typename std::map<std::string, TaskPtr>::iterator it = _augmentations.find(id);
        if (it == _augmentations.end())
            RW_THROW("Unable to find augmentation named \""<<id);

        return (*it).second;
    }

    std::map<std::string, TaskPtr >& getAugmentations() {
        return _augmentations;
    }
    const std::map<std::string, TaskPtr>& getAugmentations() const {
        return _augmentations;
    }

    void addTarget(rw::common::Ptr<Target<T> > target) {
        _targets.push_back(target);
    }

    //void addTarget

    std::vector<rw::common::Ptr<Target<T> > >& getTargets() {
        return _targets;
    }

    const std::vector<TargetPtr>& getTargets() const {
        return _targets;
    }

    void addAction(ActionPtr action) {
        addEntity(action);
        _actions.push_back(action);
    }


    std::vector<ActionPtr>& getActions() {
        return _actions;
    }


    void addMotion(MotionPtr motion) {
        addEntity(motion);
        _motions.push_back(motion);
    }

    std::vector<MotionPtr>& getMotions() {
        return _motions;
    }

    void addTask(TaskPtr task) {
        addEntity(task);
        _tasks.push_back(task);
    }

    std::vector<TaskPtr>& getTasks() {
        return _tasks;
    }

    void addEntity(rw::common::Ptr<Entity> entity) {
        entity->setOrderIndex(_entities.size());
        _entities.push_back(entity);
    }

    std::vector<rw::common::Ptr<Entity> >& getEntities() {
        return _entities;
    }

    const std::vector<rw::common::Ptr<Entity> >& getEntities() const {
        return _entities;
    }


    void setDeviceName(const std::string& name) {
        _deviceName = name;
    }

    const std::string& getDeviceName() const {
        return _deviceName;
    }


private:
    std::map<std::string, rw::common::Ptr<Task<T> > > _augmentations;

    std::vector<rw::common::Ptr<Target<T> > > _targets;

    std::vector<rw::common::Ptr<Motion<T> > > _motions;

    std::vector<ActionPtr> _actions;

    std::vector<TaskPtr> _tasks;

    std::vector<rw::common::Ptr<Entity> > _entities;

    std::string _deviceName;
};


typedef Task<rw::math::Q> QTask;
typedef Task<rw::math::Transform3D<> > CartesianTask;

typedef rw::common::Ptr<QTask> QTaskPtr;
typedef rw::common::Ptr<CartesianTask> CartesianTaskPtr;



} //end namespace task
} //end namespace rw

#endif //end include guard
