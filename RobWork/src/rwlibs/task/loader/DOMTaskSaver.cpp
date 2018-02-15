/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "DOMTaskSaver.hpp"
#include "DOMTaskFormat.hpp"

#include <rw/common/DOMParser.hpp>
#include <rw/common/DOMElem.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#include <rw/loaders/dom/DOMPropertyMapFormat.hpp>
#include <rwlibs/task/Task.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;
using namespace rwlibs::task;

DOMTaskSaver::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
        DOMBasisTypes::Initializer init1;
        DOMTaskFormat::Initializer init2;
		done = true;
	}
}

const DOMTaskSaver::Initializer DOMTaskSaver::initializer;

namespace {

template <class T>
class Identifiers {
 public:
     static const std::string& taskId();
     static const std::string& targetId();
 };

template<> const std::string& Identifiers<Q>::taskId() {
    return DOMTaskFormat::idQTask();
}

template<> const std::string& Identifiers<Q>::targetId() {
    return DOMTaskFormat::idQTarget();
}


template<> const std::string& Identifiers<Transform3D<> >::taskId() {
    return DOMTaskFormat::idCartesianTask();
}

template<> const std::string& Identifiers<Transform3D<> >::targetId() {
    return DOMTaskFormat::idCartesianTarget();
}


template <class T>
class ElementCreator {
public:
    static DOMElem::Ptr createElement(const T& element, DOMElem::Ptr parent);
};

template<> DOMElem::Ptr ElementCreator<Q>::createElement(const Q& element, DOMElem::Ptr parent) {
    return DOMBasisTypes::createQ(element, parent);
}

template<> DOMElem::Ptr ElementCreator<Transform3D<> >::createElement(const Transform3D<>& element, DOMElem::Ptr parent) {
    return DOMBasisTypes::createTransform3D(element, parent);
}

} // end anonymous namespace


void DOMTaskSaver::writeEntityInfo(Entity::Ptr entity, DOMElem::Ptr parent) {
    DOMElem::Ptr idElement = parent->addChild(DOMTaskFormat::idEntityId());
    idElement->setValue(entity->getId());

    DOMElem::Ptr indexElement = parent->addChild(DOMTaskFormat::idEntityIndex());
    indexElement->setValue(entity->getIndex());

    DOMPropertyMapSaver::save(entity->getPropertyMap(),parent);
}

template <class T>
void DOMTaskSaver::writeTargets(typename Task<T>::Ptr task, DOMElem::Ptr parent) {
    DOMElem::Ptr element = parent->addChild(DOMTaskFormat::idTargets());

	std::vector<rw::common::Ptr<Target<T> > > targets = task->getTargets();

	int targetId = 0;
	BOOST_FOREACH(rw::common::Ptr<Target<T> > target, targets) {
        DOMElem::Ptr targetElement = element->addChild(Identifiers<T>::targetId());
        DOMElem::Ptr attr = targetElement->addAttribute(DOMTaskFormat::idTargetIdAttr());
        attr->setValue(targetId);
        _targetMap[target] = attr->getValue();
        ElementCreator<T>::createElement(target->get(), targetElement);
        writeEntityInfo(target, targetElement);
		targetId++;
	}
}

template <class T>
void DOMTaskSaver::writeMotion(typename Motion<T>::Ptr motion, DOMElem::Ptr parent) {
    DOMElem::Ptr motionElement = parent->addChild(DOMTaskFormat::idMotion());

    DOMElem::Ptr typeAttr = motionElement->addAttribute(DOMTaskFormat::idMotionTypeAttr());
	switch (motion->motionType()) {
	case MotionType::Linear:
        typeAttr->setValue(DOMTaskFormat::idLinearMotion());
		break;
	case MotionType::P2P:
        typeAttr->setValue(DOMTaskFormat::idP2PMotion());
		break;
	case MotionType::Circular:
        typeAttr->setValue(DOMTaskFormat::idCircularMotion());
		break;
	}

    DOMElem::Ptr targetElement = motionElement->addChild(DOMTaskFormat::idMotionStart());
    targetElement->addChild(_targetMap[motion->startTarget()]); // Maby set value

	if (motion->motionType() == MotionType::Circular) {
        targetElement = motionElement->addChild(DOMTaskFormat::idMotionMid());
        targetElement->addChild(_targetMap[motion.template cast<CircularMotion<T> >()->midTarget()]);
	}

    targetElement = motionElement->addChild(DOMTaskFormat::idMotionEnd());
    targetElement->addChild(_targetMap[motion->endTarget()]);

    writeEntityInfo(motion, motionElement);
}

void DOMTaskSaver::writeAction(Action::Ptr action, DOMElem::Ptr parent) {
    DOMElem::Ptr actionElement = parent->addChild(DOMTaskFormat::idAction());

    DOMElem::Ptr typeAttr = actionElement->addAttribute(DOMTaskFormat::idActionTypeAttr());
    typeAttr->setValue(action->actionType());

    writeEntityInfo(action, actionElement);
}

template <class T>
void DOMTaskSaver::writeEntities(typename Task<T>::Ptr task, DOMElem::Ptr parent) {
	DOMElem::Ptr entriesElement = parent->addChild(DOMTaskFormat::idEntities());

	std::vector<Entity::Ptr> entities = task->getEntities();
	BOOST_FOREACH(Entity::Ptr entity, entities) {
		switch (entity->entityType()) {
		case EntityType::Task:
			writeTask(entity.cast<Task<T> >(), entriesElement);
			break;
		case EntityType::Motion:
			writeMotion<T>(entity.cast<Motion<T> >(), entriesElement);
			break;
		case EntityType::Action:
			writeAction(entity.cast<Action>(), entriesElement);
			break;
		case EntityType::User:
			RW_THROW("Unable to save user specified types");
			break;
		}
	}


}

template <class T>
void DOMTaskSaver::writeTaskImpl(typename Task<T>::Ptr task, DOMElem::Ptr parent) {
    DOMElem::Ptr taskElement = parent->addChild(Identifiers<T>::taskId());
    saveImpl<T>(task, taskElement);
}


void DOMTaskSaver::writeTask(TaskBase::Ptr task, DOMElem::Ptr parent) {
	Task<Q>::Ptr qtask = task.cast<Task<Q> >();
	if (qtask != NULL) {
        writeTaskImpl<Q>(qtask, parent);
		return;
	}

	Task<Transform3D<> >::Ptr carttask = task.cast<Task<Transform3D<> > >();
	if (carttask != NULL) {
        writeTaskImpl<Transform3D<> >(carttask, parent);
		return;
	}
}

template <class T>
void DOMTaskSaver::saveImpl(typename Task<T>::Ptr task, DOMElem::Ptr parent) {
    writeTargets<T>(task, parent);
    writeEntities<T>(task, parent);
    writeEntityInfo(task, parent);
}


bool DOMTaskSaver::save(rwlibs::task::QTask::Ptr task, std::ostream& outstream) {
    DOMParser::Ptr parser = DOMParser::make();
    DOMElem::Ptr doc = parser->getRootElement();
    DOMElem::Ptr parent = doc->addChild(DOMTaskFormat::idQTask());
    saveImpl<Q>(task, parent);
    parser->save(outstream);
    return true;
}

bool DOMTaskSaver::save(rwlibs::task::CartesianTask::Ptr task, std::ostream& outstream) {
    DOMParser::Ptr parser = DOMParser::make();
    DOMElem::Ptr doc = parser->getRootElement();
    DOMElem::Ptr parent = doc->addChild(DOMTaskFormat::idCartesianTask());
    saveImpl<Transform3D<>>(task, parent);
    parser->save(outstream);
    return true;
}

bool DOMTaskSaver::save(QTask::Ptr task, const std::string& filename) {
    std::string ext = StringUtil::toUpper(StringUtil::getFileExtension(filename));
    if (!ext.empty())
    	ext = ext.substr(1); // remove dot
    if (!DOMParser::Factory::hasDOMParser(ext)) {
    	RW_WARN("No DOMParser can be created for a file with the extension " << ext << "!");
    	return false;
    }
    DOMParser::Ptr parser = DOMParser::Factory::getDOMParser(ext);
    DOMElem::Ptr doc = parser->getRootElement();
    doc->setName(DOMTaskFormat::idQTask());
    DOMElem::Ptr parent = doc->addChild(DOMTaskFormat::idQTask());
    saveImpl<Q>(task, parent);
    parser->save(filename);
    return true;
}

bool DOMTaskSaver::save(CartesianTask::Ptr task, const std::string& filename) {
    std::string ext = StringUtil::toUpper(StringUtil::getFileExtension(filename));
    if (!ext.empty())
    	ext = ext.substr(1); // remove dot
    if (!DOMParser::Factory::hasDOMParser(ext)) {
    	RW_WARN("No DOMParser can be created for a file with the extension " << ext << "!");
    	return false;
    }
    DOMParser::Ptr parser = DOMParser::make();
    DOMElem::Ptr doc = parser->getRootElement();
    DOMElem::Ptr parent = doc->addChild(DOMTaskFormat::idCartesianTask());
    saveImpl<Transform3D<>>(task, parent);
    parser->save(filename);
    return true;
}
