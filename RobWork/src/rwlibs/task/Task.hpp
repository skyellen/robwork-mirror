/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

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
class TaskBase: public Entity {

public:
	//! @brief smart pointer type to this class
    typedef rw::common::Ptr<TaskBase> Ptr;
	/**
	 * Convenience definition of pointer to Action
	 */
	typedef rw::common::Ptr<Action> ActionPtr;

	/**
	 * @brief Constructs a task with a given type
	 * @param type [in] Type of task
	 * @param id [in] optional identifier
	 */
	TaskBase(Type type, const std::string& id = "") :
		Entity(EntityType::Task, id), _type(type) {
	}

	/**
	 * @brief Destructor
	 */
	virtual ~TaskBase() {
	}

	/*virtual rw::common::Ptr<TaskBase> clone() {
	 RW_THROW("Cloning on TaskBase level not supported!");
	 }
	 */
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
		std::map<std::string, TaskBasePtr>::iterator it = _augmentations.find(
				id);
		if (it == _augmentations.end())
			RW_THROW("Unable to find augmentation named \""<<id);

		return (*it).second;
	}

	/**
	 * @brief Returns map with ids and augmentations
	 * @return Reference to map with ids and augmentations
	 */
	std::map<std::string, TaskBasePtr>& getAugmentations() {
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
	    entity->setIndex((int)_entities.size());
	    _entities.push_back(entity);
	}


    void addEntityToFront(rw::common::Ptr<Entity> entity) {
        BOOST_FOREACH(rw::common::Ptr<Entity> ent, _entities) {
            ent->setIndex(ent->getIndex() + 1);
        }
        _entities.insert(_entities.begin(), entity);
        entity->setIndex(0);
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

    void addActionToFront(ActionPtr action) {
        addEntityToFront(action);
        _actions.insert(_actions.begin(), action);
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

	/**
	 * @brief Removes a specific action
	 */
	bool removeAction(Action::Ptr action) {
		std::vector<Action::Ptr>::iterator it = std::find(_actions.begin(), _actions.end(), action);
		if (it != _actions.end()) {
			_actions.erase(it);
			return removeEntity(action);			
		} else {
			return false;
		}	
	}

	/**
	 * @brief Reverse the order of the task
	 */
	virtual void reverse() = 0;

protected:
	Type _type;

	std::map<std::string, TaskBasePtr> _augmentations;

	std::vector<ActionPtr> _actions;

	std::vector<rw::common::Ptr<Entity> > _entities;

	std::string _deviceName;

	void doReverseBase() {
		std::vector<ActionPtr>& actions = getActions();
		std::vector<ActionPtr> tmp = actions;
		actions.clear();
		for (std::vector<ActionPtr>::reverse_iterator it = tmp.rbegin(); it
				!= tmp.rend(); ++it)
			actions.push_back(*it);

		for (std::map<std::string, TaskBasePtr>::iterator it =
				_augmentations.begin(); it != _augmentations.end(); ++it)
			(*it).second->reverse();

		typedef std::vector<rw::common::Ptr<Entity> > EntityVector;
		EntityVector entities = _entities;
		_entities.clear();
		for (EntityVector::reverse_iterator it = entities.rbegin(); it
				!= entities.rend(); ++it) {
			addEntity(*it);
		}
	}


	void copyBase(TaskBasePtr target) {
		target->_type = _type;
		target->_deviceName = _deviceName;

		for (std::map<std::string, TaskBasePtr>::iterator it = _augmentations.begin(); it != _augmentations.end(); ++it) {
			target->_augmentations[(*it).first] = NULL;//(*it).second->doClone();
		}
	}

    virtual TaskBasePtr doClone() {
        return NULL;
    }

	bool removeEntity(Entity::Ptr entity) {
		std::vector<Entity::Ptr>::iterator it = std::find(_entities.begin(), _entities.end(), entity);
		if (it != _entities.end()) {
			_entities.erase(it);
			return true;
		} else {
			return false;
		}	
	}		


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
		//! @brief smart pointer type to this class
		//typedef rw::common::Ptr<GenericTask<TASK, TARGET, MOTION> > Ptr;
		//typedef typename TASK::INT TaskPtr;
		/** Convenience definition of pointer to task */
		typedef rw::common::Ptr<TASK> TaskPtr;

		/** Convenience definition of pointer to target */
		typedef rw::common::Ptr<TARGET> TargetPtr;

		/** Convenience definition of pointer to motion */
		typedef rw::common::Ptr<MOTION> MotionPtr;

        //typedef rw::common::Ptr<GenericTask> GenericTaskPtr;

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
			addEntity(target);
			_targets.push_back(target);
		}

		void addTargetToFront(TargetPtr target) {
            addEntityToFront(target);
            _targets.insert(_targets.begin(), target);
        }


		/**
		 * @brief Removes a specific target
		 */
		bool removeTarget(TargetPtr target) {
			typename std::vector<TargetPtr>::iterator it = std::find(_targets.begin(), _targets.end(), target);
			if(it != _targets.end()) {
				_targets.erase(it);
				return removeEntity(target);			
			} else {
				return false;
			}	
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
		 * @brief Adds \b motion to the front of the task
		 * @param motion [in] Motion to add
		 */
		void addMotionToFront(MotionPtr motion) {
            addEntityToFront(motion);
            _motions.insert(_motions.begin(), motion);
        }

		/**
		 * @brief Removes a specific motion
		 */
		bool removeMotion(MotionPtr motion) {
			typename std::vector<MotionPtr>::iterator it = std::find(_motions.begin(), _motions.end(), motion);
			if (it != _motions.end()) {
				_motions.erase(it);
				return removeEntity(motion);			
			} else {
				return false;
			}	
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

		void addTaskToFront(TaskPtr task) {
            addEntityToFront(task);
            _tasks.insert(_tasks.begin(), task);
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

		void reverse() {
			std::vector<TargetPtr> targets = _targets;
			_targets.clear();
			for (typename std::vector<TargetPtr>::reverse_iterator it = targets.rbegin(); it != targets.rend(); ++it)
			_targets.push_back(*it);

			std::vector<MotionPtr> motions = _motions;
			_motions.clear();
			for (typename std::vector<MotionPtr>::reverse_iterator it = motions.rbegin(); it != motions.rend(); ++it) {
				MotionPtr motion = *it;
				motion->reverse();
				_motions.push_back(motion);
			}

			std::vector<TaskPtr> tasks = _tasks;
			_tasks.clear();
			for (typename std::vector<TaskPtr>::reverse_iterator it = tasks.rbegin(); it != tasks.rend(); ++it) {
				TaskPtr task = *it;
				task->reverse();
				_tasks.push_back(task);
			}

			doReverseBase();

		}


	protected:
		std::vector<TargetPtr> _targets;

		std::vector<MotionPtr> _motions;

		std::vector<TaskPtr> _tasks;




	};

	/**
	 * @brief Template based implementation of Task
	 */
	template <class T>
	class Task: public GenericTask<Task<T>, Target<T>, Motion<T> > {
	public:
		//! @brief smart pointer type to this class
		typedef typename rw::common::Ptr<Task<T> > Ptr;

		typedef int INT;

		/**
		 * Convenience definition of pointer to Task with type T
		 */
		//typedef rw::common::Ptr<Task<T> > TaskPtr;

		/**
		 * Convenience definition of pointer to Target with type T
		 */
		//typedef rw::common::Ptr<Target<T> > TargetPtr;

		/**
		 * Convenience definition of pointer to Motion with type T
		 */
		//typedef rw::common::Ptr<Motion<T> > MotionPtr;

		/**
		 * @brief Constructs Task
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
		typename Target<T>::Ptr addTargetByValue(const T& value) {
			this->addTarget(rw::common::ownedPtr(new Target<T>(value)));
			return this->_targets.back();
		}

		typename Target<T>::Ptr addTargetByValueToFront(const T& value) {
            this->addTargetToFront(rw::common::ownedPtr(new Target<T>(value)));
            return this->_targets.front();
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
		void addToPath(typename Task<T>::Ptr task, std::vector<T>& result) {
			std::vector<Entity::Ptr>& entities = this->getEntities();

			for (std::vector<Entity::Ptr>::const_iterator it = entities.begin(); it != entities.end(); ++it) {
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
			if (this->_motions.size()> 0) {
				result.push_back(this->_motions.back()->end());
			}
		}

		void addToTargetPath(typename Task<T>::Ptr task, std::vector<typename Target<T>::Ptr>& result) {
			std::vector<Entity::Ptr>& entities = this->getEntities();
			for (std::vector<Entity::Ptr>::const_iterator it = entities.begin(); it != entities.end(); ++it) {
				switch ((*it)->entityType()) {
					case EntityType::Task:
					addToTargetPath((*it)->cast<Task<T>*>(), result);
					break;
					case EntityType::Motion: {
						Motion<T>* motion = (*it)->cast<Motion<T>*>();
						result.push_back(motion->startTarget());
						break;
					}
				} //end switch ((*it)->entityType())
			}
			if (this->_motions.size()> 0) {
				result.push_back(this->_motions.back()->endTarget());
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

		std::vector<typename Target<T>::Ptr> getTargetPath() {
			std::vector<typename Target<T>::Ptr> result;
			addToTargetPath(this, result);
			return result;
		}

		virtual typename Task<T>::Ptr clone() {
			TaskBase::Ptr base = doClone();
			typename Task<T>::Ptr task = base.cast<Task<T> >();
			return task;
		}

	protected:

		virtual TaskBase::Ptr doClone() {
			typename Task<T>::Ptr result = rw::common::ownedPtr(new Task<T>(this->getId()));

			std::vector<typename Target<T>::Ptr > newTargets;
			BOOST_FOREACH(typename Target<T>::Ptr target, this->getTargets()) {
				newTargets.push_back(target->clone());
			}

			BOOST_FOREACH(Entity::Ptr entity, this->getEntities()) {
				switch (entity->entityType()) {
				case EntityType::Target:
					BOOST_FOREACH(typename Target<T>::Ptr target, newTargets) {
						if (target->getIndex() == entity->getIndex()) {
							result->addTarget(target);
							break;
						}
					}
					break;
				case EntityType::Motion:
					result->addMotion(entity.cast<Motion<T> >()->clone(newTargets));
					break;
				case EntityType::Action:
					result->addAction(entity.cast<Action>()->clone());
					break;
				case EntityType::Task:
					result->addTask(entity.cast<Task>()->clone());
					break;

				}

			}
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

	/** @} */

} //end namespace task
} //end namespace rwlibs

#endif //end include guard
