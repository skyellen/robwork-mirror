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
#include <rw/common/Ptr.hpp>



namespace rw {
namespace task2 {


class Task;
typedef rw::common::Ptr<Task> TaskPtr;

class Task: public Entity
{
public:
    Task() {}
    virtual ~Task() {}

    void addAugmentation(TaskPtr task, const std::string& id) {
        _augmentations[id] = task;
    }

    bool hasAugmentation(const std::string& id) {
        return _augmentations.find(id) != _augmentations.end();
    }

    TaskPtr getAugmentation(const std::string& id) {
        std::map<std::string, TaskPtr>::iterator it = _augmentations.find(id);
        if (it == _augmentations.end())
            RW_THROW("Unable to find augmentation named \""<<id);

        return (*it).second;
    }

    std::map<std::string, TaskPtr>& getAugmentations() {
        return _augmentations;
    }
    const std::map<std::string, TaskPtr>& getAugmentations() const {
        return _augmentations;
    }

    void addTarget(TargetPtr target) {
        _targets.push_back(target);
    }

    std::vector<TargetPtr>& getTargets() {
        return _targets;
    }
    const std::vector<TargetPtr>& getTargets() const {
        return _targets;
    }

    void addEntity(EntityPtr entity) {
        _entities.push_back(entity);
    }

    std::vector<EntityPtr>& getEntities() {
        return _entities;
    }

    const std::vector<EntityPtr>& getEntities() const {
        return _entities;
    }


    void setDeviceName(const std::string& name) {
        _deviceName = name;
    }

    const std::string& getDeviceName() const {
        return _deviceName;
    }

    int type() const {
        return TASK;
    }


private:
    std::map<std::string, TaskPtr> _augmentations;

    std::vector<TargetPtr> _targets;

    std::vector<EntityPtr> _entities;

    std::string _deviceName;
};



} //end namespace task
} //end namespace rw

#endif //end include guard
