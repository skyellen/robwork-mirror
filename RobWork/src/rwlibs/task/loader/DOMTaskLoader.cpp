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

#include "DOMTaskLoader.hpp"
#include "DOMTaskFormat.hpp"

#include <rw/common/DOMParser.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>
#include <rw/loaders/dom/DOMPropertyMapFormat.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;
using namespace rwlibs::task;

namespace {

int readIntAttribute(DOMElem::Ptr element, const std::string& id) {
    if (!element->hasAttribute(id)) {
        RW_THROW("Unable to find attribute: \""<<id<<"\"");
    }
    return element->getAttributeValueAsInt(id);
}

std::string readStringAttribute(DOMElem::Ptr element, const std::string& id) {
    if (!element->hasAttribute(id)) {
        RW_THROW("Unable to find attribute: \""<<id<<"\" it is instead: " << element->getName());
    }
    return element->getAttributeValue(id);
}

MotionType readMotionTypeAttribute(DOMElem::Ptr element, const std::string& id) {
	std::string type = readStringAttribute(element, id);
    if (type == DOMTaskFormat::idLinearMotion() )
		return MotionType::Linear;
    if (type == DOMTaskFormat::idP2PMotion())
		return MotionType::P2P;
    if (type == DOMTaskFormat::idCircularMotion())
		return MotionType::Circular;
	RW_THROW("Unsupported type of motion: \""<<type<<"\"");
}

template <class T>
class ElementReader {
public:
    static T readElement(DOMElem::Ptr element);
};

template<> Q ElementReader<Q>::readElement(DOMElem::Ptr element) {
    return DOMBasisTypes::readQ(element, false);
}

template<> Transform3D<> ElementReader<Transform3D<> >::readElement(DOMElem::Ptr element) {
    return DOMBasisTypes::readTransform3D(element, false);
}

template<> std::string ElementReader<std::string>::readElement(DOMElem::Ptr element) {
    return DOMBasisTypes::readString(element, false);
}

} //end anonymous namespace

void DOMTaskLoader::load(std::istream& instream, const std::string& schemaFileName) {
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load(instream);
    DOMElem::Ptr elementRoot = parser->getRootElement();

    _task = readTask(elementRoot);
}

void DOMTaskLoader::load(const std::string& filename, const std::string& schemaFileName) {
    std::string ext = StringUtil::toUpper(StringUtil::getFileExtension(filename));
    if (!ext.empty())
    	ext = ext.substr(1); // remove dot
    if (!DOMParser::Factory::hasDOMParser(ext))
    	RW_THROW("No DOMParser can be created for a file with the extension " << ext << "!");
    DOMParser::Ptr parser = DOMParser::Factory::getDOMParser(ext);
    parser->setSchema(schemaFileName);
    parser->load(filename);
    DOMElem::Ptr elementRoot = parser->getRootElement();

    _task = readTask(elementRoot);
}

QTask::Ptr DOMTaskLoader::getQTask() {
	if (_qTask == NULL)
		RW_THROW("No QTask Loaded");
	return _qTask;
}

CartesianTask::Ptr DOMTaskLoader::getCartesianTask() {
	if (_cartTask == NULL)
		RW_THROW("No CartesianTask Loaded");
	return _cartTask;
}

TaskBase::Ptr DOMTaskLoader::getTask() {
	if (_task == NULL)
		RW_THROW("No Task Loaded");
	return _task;
}

TaskLoader::Ptr DOMTaskLoader::clone() const {
	return ownedPtr(new DOMTaskLoader());
}

void DOMTaskLoader::readEntityData(DOMElem::Ptr element, Entity::Ptr entity) {
    DOMElem::Ptr child;
    if(element->hasChildren()){
        BOOST_FOREACH(child, element->getChildren() ){

            if(child != NULL){
                if(child->isName(DOMTaskFormat::idEntityIndex())){
                    entity->setIndex(DOMBasisTypes::readInt(child,false));
                }
                if(child->isName(DOMTaskFormat::idEntityId())){
                    try {
                        entity->setId(DOMBasisTypes::readString(child, false));
                    } catch (...) {}
                }
                if(child->isName(DOMPropertyMapFormat::idPropertyMap())){
                    entity->setPropertyMap(DOMPropertyMapLoader::readProperties(child, false));
                }
            }
        }
    }
}

template <class T>
typename Target<T>::Ptr DOMTaskLoader::readTarget(DOMElem::Ptr element) {
    DOMElem::Ptr child;
    T value;
    std::string id = readStringAttribute(element, DOMTaskFormat::idTargetIdAttr());

    if(element->hasChildren()){
        BOOST_FOREACH(child, element->getChildren() ){
            if(!child.isNull()){
                if(child->isName(DOMBasisTypes::idQ())){
                    value = ElementReader<T>::readElement(child);
                }
                if(child->isName(DOMBasisTypes::idTransform3D())){
                    value = ElementReader<T>::readElement(child);
                }
            }
        }
    }
	typename Target<T>::Ptr target = ownedPtr(new Target<T>(value));
	readEntityData(element, target);
	if (_targetMap.find(id) != _targetMap.end()) {
		RW_THROW("Multiple targets with id \""<<id<<"\" exists");
	}
	_targetMap[id] = target;
	return target;
}

Action::Ptr DOMTaskLoader::readAction(DOMElem::Ptr element) {
    int type = readIntAttribute(element, DOMTaskFormat::idActionTypeAttr());

	Action::Ptr action = ownedPtr(new Action(type));
	readEntityData(element, action);
	return action;
}

template <class T>
typename Motion<T>::Ptr DOMTaskLoader::readMotion(DOMElem::Ptr element) {
    MotionType type = readMotionTypeAttribute(element, DOMTaskFormat::idMotionTypeAttr());

	std::string start;
	std::string mid = "";
	std::string end;

    DOMElem::Ptr child;
    if(element->hasChildren()){
        BOOST_FOREACH(child, element->getChildren() ){
            if(!child.isNull()){
                if (child->isName(DOMTaskFormat::idMotionStart())) {
                    start = ElementReader<std::string>::readElement(child);
                } else if (child->isName(DOMTaskFormat::idMotionMid())) {
                    mid = ElementReader<std::string>::readElement(child);
                } else if (child->isName(DOMTaskFormat::idMotionEnd())) {
                    end = ElementReader<std::string>::readElement(child);
                }
            }

        }
    }
	typedef std::map<std::string, TargetBase::Ptr> MyMap;
	typename MyMap::iterator it, endIt;
	it = _targetMap.find(start);
	endIt = _targetMap.end();
	//if (it == endIt)
	//	RW_THROW("Unable to find Target named \""<<start<<"\"");
	typename Target<T>::Ptr startTarget;
	if (it != endIt)
		 startTarget = (*it).second.cast<Target<T> >();

	typename Target<T>::Ptr midTarget;
	if (mid != "") {
		it = _targetMap.find(mid);
		if (it != endIt)
			midTarget = (*it).second.cast<Target<T> >();
			//RW_THROW("Unable to find Target named \""<<mid<<"\"");
		
	}

	it = _targetMap.find(end);
	typename Target<T>::Ptr endTarget;
	if (it != endIt)
		 endTarget = (*it).second.cast<Target<T> >();	
		//RW_THROW("Unable to find Target named \""<<end<<"\"");
	

	typename Motion<T>::Ptr motion;
	switch (type) {
	case MotionType::Linear:
		motion = ownedPtr(new LinearMotion<T>(startTarget, endTarget));
		break;
	case MotionType::Circular:
		motion = ownedPtr(new CircularMotion<T>(startTarget, midTarget, endTarget));
		break;
	case MotionType::P2P:
		motion = ownedPtr(new P2PMotion<T>(startTarget, endTarget));
		break;
	}
	readEntityData(element, motion);
	return motion;
}

template <class T>
void DOMTaskLoader::readEntities(DOMElem::Ptr element, typename Task<T>::Ptr task) {
    DOMElem::Ptr child;
    if(element->hasChildren()){
        BOOST_FOREACH(child, element->getChildren() ){
            if(!child.isNull()){
                if (child->isName(DOMTaskFormat::idMotion())) {
                    task->addMotion(readMotion<T>(child));
                } else if (child->isName(DOMTaskFormat::idAction())) {
                    task->addAction(readAction(child));
                } else if (child->isName(DOMTaskFormat::idQTask()) || child->isName(DOMTaskFormat::idCartesianTask())) {

                    //A subtask may have duplicates of the target names. We therefore
                    //store the old targets temporarily and
                    TargetMap tmp = _targetMap;
                    _targetMap.clear();
                    task->addTask(readTemplateTask<T>(child));
                    _targetMap.clear();
                    _targetMap = tmp;
                }
            }
		}
	}
}

template <class T>
void DOMTaskLoader::readTargets(DOMElem::Ptr element, typename Task<T>::Ptr task) {
    DOMElem::Ptr child;
    if(element->hasChildren()){
        BOOST_FOREACH(child, element->getChildren() ){
            if(!child.isNull()){
                if (child->isName(DOMTaskFormat::idQTarget()) || child->isName(DOMTaskFormat::idCartesianTarget())){
                    task->addTarget(readTarget<T>(child));
                }
            }
		}
	}
}

void DOMTaskLoader::readAugmentations(DOMElem::Ptr element, TaskBase::Ptr task) {
    DOMElem::Ptr child;
    if(element->hasChildren()){
        BOOST_FOREACH(child, element->getChildren() ){
            if (!child.isNull()) {
                DOMTaskLoader loader;
                TaskBasePtr augmentation = loader.readTask(child);
                task->addAugmentation(augmentation, augmentation->getId());
            }
        }
	}
}

template <class T>
typename Task<T>::Ptr DOMTaskLoader::readTemplateTask(DOMElem::Ptr element) {
	typename Task<T>::Ptr task = ownedPtr(new Task<T>());

    DOMElem::Ptr child;
    if(element->hasChildren()){
        BOOST_FOREACH(child, element->getChildren() ){
            if (!child.isNull()) {
                if (child->isName(DOMTaskFormat::idTargets())) {
                    readTargets<T>(child, task);
                } else if (child->isName(DOMTaskFormat::idEntities())) {
                    readEntities<T>(child, task);
                } else if (child->isName(DOMTaskFormat::idAugmentations())) {
                    readAugmentations(child, task);
                }
            }
        }
	}
	readEntityData(element, task);
	return task;
}

TaskBase::Ptr DOMTaskLoader::readTask(DOMElem::Ptr element) {
    DOMElem::IteratorPair itpair = element->getChildren();
    for (DOMElem::Iterator it = itpair.first; it != itpair.second; ++it) {
        if(!(*it).isNull()){
            if ((*it)->isName(DOMTaskFormat::idQTask())) {
                _qTask = readTemplateTask<Q>((*it));
                return _qTask;
            } else if ((*it)->isName(DOMTaskFormat::idCartesianTask())) {
                _cartTask = readTemplateTask<Transform3D<> >((*it));
                return _cartTask;
            } else {
                RW_THROW("Element name does not match " + DOMTaskFormat::idQTask() + " or " + DOMTaskFormat::idCartesianTask() + " as expected, but equals: " + (*it)->getName());
            }
        }
    }
    RW_WARN("Element does not seem to contain any children. NULL Task is returned.");
    return NULL;
}
