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

#ifndef RWLIBS_TASK_TASK_HPP
#define RWLIBS_TASK_TASK_HPP

#include "Target.hpp"
#include "Entity.hpp"
#include "Action.hpp"
#include "Motion.hpp"
#include <rw/common/Ptr.hpp>

#include <map>
#include <string>

namespace rwlibs {
namespace task {


class TaskBase;

typedef rw::common::Ptr<TaskBase> TaskBasePtr;



/** @addtogroup task */
/*@{*/

/**
 * @brief Base class for tasks
 */
class TaskBase: public Entity
{

public:
    /**
     * Convenience definition of pointer to Action
     */
    typedef rw::common::Ptr<Action> ActionPtr;


    /**
     * @brief Constructs a task with a given type
     * @param type [in] Type of task
     */
    TaskBase(Type type, const std::string& id = ""):
        Entity(EntityType::Task, id),
        _type(type)
    {}

    /**
     * @brief Destructor
     */
    virtual ~TaskBase() {}

    /**
     * @brief Returns the type of the task
     */
    Type type() const {
        return _type;
    }

    /**
     * @brief Add an augmentation of the task
     * @param task [in] Task representing the augmentation
     * @param id [in] Id associated to augmentation
     */
    void addAugmentation(TaskBasePtr task, const std::string& id) {
        _augmentations[id] = task;
    }

    /**
     * @brief Returns whether the task has an augmentation with a given \b id
     * @param id [in] id of augmentation to look for
     * @return True if augmentation exists. False otherwise
     */
    bool hasAugmentation(const std::string& id) {
        return _augmentations.find(id) != _augmentations.end();
    }

    /**
     * @brief Returns augmentation associated to \b id.
     *
     * If no augmentation exists the method throws a rw::common::Exception
     * @param id [in] id of task
     * @return Pointer to the augmenting task
     */
    TaskBasePtr getAugmentation(const std::string& id) {
        std::map<std::string, TaskBasePtr>::iterator it = _augmentations.find(id);
        if (it == _augmentations.end())
            RW_THROW("Unable to find augmentation named \""<<id);

        return (*it).second;
    }

    /**
     * @brief Returns map with ids and augmentations
     * @return Reference to map with ids and augmentations
     */
    std::map<std::string, TaskBasePtr >& getAugmentations() {
        return _augmentations;
    }

    /**
     * @brief Returns map with ids and augmentations
     * @return Reference to map with ids and augmentations
     */
    const std::map<std::string, TaskBasePtr>& getAugmentations() const {
        return _augmentations;
    }

    /**
     * @brief Adds \b entity to the task.
     *
     * When adding an entity is is assigned an orderIndex specifying
     * when it is added. This index can later be used to find the relative
     * order between motions, actions and subtasks.
     *
     * @param entity [in] Entity to add
     */
    void addEntity(rw::common::Ptr<Entity> entity) {
        entity->setIndex(_entities.size());
        _entities.push_back(entity);
    }

    /**
     * @brief Returns list of entities
     *
     * The order of the entities corresponds to the expected order of execution.
     * @return Reference to list of entities
     */
    std::vector<rw::common::Ptr<Entity> >& getEntities() {
        return _entities;
    }

    /**
     * @brief Returns list of entities
     *
     * The order of the entities corresponds to the expected order of execution.
     * @return Reference to list of entities
     */
    const std::vector<rw::common::Ptr<Entity> >& getEntities() const {
        return _entities;
    }


    /**
     * @brief Adds \b action to the task
     * @param action [in] Action to add
     */
    void addAction(ActionPtr action) {
        addEntity(action);
        _actions.push_back(action);
    }

    /**
     * @brief Returns list of actions
     * @return Reference to list of actions
     */
    std::vector<ActionPtr>& getActions() {
        return _actions;
    }

    /**
     * @brief Returns list of actions
     * @return Reference to list of actions
     */
    const std::vector<ActionPtr>& getActions() const {
        return _actions;
    }


    /**
     * @brief Sets name of device associated to the task
     * @param name [in] Device name
     */
    void setDeviceName(const std::string& name) {
        _deviceName = name;
    }

    /**
     * @brief Returns name of device associated to the task
     * @return Name of the device
     */
    const std::string& getDeviceName() const {
        return _deviceName;
    }

protected:
    Type _type;

    std::map<std::string, TaskBasePtr> _augmentations;

    std::vector<ActionPtr> _actions;

    std::vector<rw::common::Ptr<Entity> > _entities;

    std::string _deviceName;

};


/**
 * @brief Implements a template based and generic version of a task.
 *
 * The template arguments \b TASK, \b TARGET and \b MOTION represents
 * specifies which kind of task, target and motion to use.
 */
template <class TASK, class TARGET, class MOTION>
class GenericTask: public TaskBase {
public:
	/** Convenience definition of pointer to task */
	typedef rw::common::Ptr<TASK> TaskPtr;

	/** Convenience definition of pointer to target */
	typedef rw::common::Ptr<TARGET> TargetPtr;

	/** Convenience definition of pointer to motion */
	typedef rw::common::Ptr<MOTION> MotionPtr;

    /**
     * @brief Constrcts Task
     *
     * When constructing a task the type T is automatically added to the TypeRepository
     * and the the associated value is set as the type.
     */
	GenericTask(Type type = -1, const std::string& id = ""):
        TaskBase(type, id)
    {

    }

    /**
     * @brief Destructor
     */
    virtual ~GenericTask() {}


    /**
     * @brief Adds \b target to the task
     * @param target [in] Target to add
     */
    void addTarget(TargetPtr target) {
		addEntity(target); //andb
        _targets.push_back(target);
    }


    /**
     * @brief Returns list of targets
     * @return Reference to list of targets
     */
    std::vector<TargetPtr>& getTargets() {
        return _targets;
    }

    /**
     * @brief Returns list of targets
     * @return Reference to list of targets
     */
    const std::vector<TargetPtr>& getTargets() const {
        return _targets;
    }

    /**
     * @brief Adds \b motion to the task
     * @param motion [in] Motion to add
     */
    void addMotion(MotionPtr motion) {
        addEntity(motion);
        _motions.push_back(motion);
    }

    /**
     * @brief Returns list of motions
     * @return Reference to list of motions
     */
    std::vector<MotionPtr>& getMotions() {
        return _motions;
    }

    /**
     * @brief Returns list of motions
     * @return Reference to list of motions
     */
    const std::vector<MotionPtr>& getMotions() const {
        return _motions;
    }

    /**
     * @brief Adds \b task as a subtask
     * @param task [in] Task to add
     */
    void addTask(TaskPtr task) {
        addEntity(task);
        _tasks.push_back(task);
    }

    /**
     * @brief Returns list of tasks
     * @return Reference to list of tasks
     */
    std::vector<TaskPtr>& getTasks() {
        return _tasks;
    }

    /**
     * @brief Returns list of tasks
     * @return Reference to list of tasks
     */
    const std::vector<TaskPtr>& getTasks() const {
        return _tasks;
    }


protected:
    std::vector<TargetPtr> _targets;

    std::vector<MotionPtr> _motions;

    std::vector<TaskPtr> _tasks;
};

/**
 * @brief Definition of rw::common::Ptr to a GenericTask working on the base classes
 * of Task, Target and Motion.
 */
typedef rw::common::Ptr<GenericTask<TaskBase, TargetBase, MotionBase> > GenericTaskPtr;


/**
 * @brief Template based implementation of Task
 */
template <class T>
class Task: public GenericTask<Task<T>, Target<T>, Motion<T> > {
public:
    /**
     * Convenience definition of pointer to Task with type T
     */
    typedef rw::common::Ptr<Task<T> > TaskPtr;

    /**
     * Convenience definition of pointer to Target with type T
     */
    typedef rw::common::Ptr<Target<T> > TargetPtr;

    /**
     * Convenience definition of pointer to Motion with type T
     */
    typedef rw::common::Ptr<Motion<T> > MotionPtr;


    /**
     * @brief Constrcts Task
     *
     * When constructing a task the type T is automatically added to the TypeRepository
     * and the their associated value is set as the type.
     */
    Task(const std::string& id = ""):
    	GenericTask<Task<T>, Target<T>, Motion<T> >(TypeRepository::instance().get<T>(true /*Add if it does not exist*/), id)
    {
    }

    /**
     * @brief Destructor
     */
    virtual ~Task() {}


    /**
     * @brief Adds target to task based on \b value
     * @param value [in] Value of the target.
     * @return Pointer to the target object constructed and added.
     */
    rw::common::Ptr<Target<T> > addTargetByValue(const T& value) {
        addTarget(ownedPtr(new Target<T>(value)));
        return this->_targets.back();
    }

    /**
     * @brief Adds values of targets in the task to \b result.
     *
     * Utility function which can be used to construct a list of via points
     * from a task.
     *
     * See also Task::getPath() below.
     *
     * @param task [in] Task to run through
     * @param result [in] Vector into which targets pointer should be placed
     */
    void addToPath(TaskPtr task, std::vector<T>& result) {
    	std::vector<rw::common::Ptr<Entity> >& entities = this->getEntities();
        for (std::vector<rw::common::Ptr<Entity> >::const_iterator it = entities.begin(); it != entities.end(); ++it) {
            switch ((*it)->entityType()) {
            case EntityType::Task:
                addToPath((*it)->cast<Task<T>*>(), result);
                break;
            case EntityType::Motion: {
                Motion<T>* motion = (*it)->cast<Motion<T>*>();
                result.push_back(motion->start());
                break;
                }
            } //end switch ((*it)->entityType())
        }
        if (this->_motions.size() > 0) {
            result.push_back(this->_motions.back()->end());
        }
    }

    /**
     * @brief Returns a vector with template type T, describing the target values visited
     * when executing the task.
     *
     * This function can be used to make a simple path from a task.
     * @return Vector with target values visited.
     */
    std::vector<T> getPath() {
        std::vector<T> result;
        addToPath(this, result);
        return result;
    }


private:
   /* std::vector<rw::common::Ptr<Target<T> > > _targets;

    std::vector<rw::common::Ptr<Motion<T> > > _motions;

    std::vector<TaskPtr> _tasks;
*/
};

/**
 * Definition of task with type rw::math::Q
 */
typedef Task<rw::math::Q> QTask;

/**
 * Definition of task with type rw::math::Transform3D
 */
typedef Task<rw::math::Transform3D<> > CartesianTask;

/**
 * Definition of rw::common::Ptr to QTask
 */
typedef rw::common::Ptr<QTask> QTaskPtr;

/**
 * Definition of rw::common::Ptr to CartesianTask
 */
typedef rw::common::Ptr<CartesianTask> CartesianTaskPtr;

/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
