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



#include "XMLTaskLoader.hpp"
#include "XMLTaskFormat.hpp"

#include <iostream>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>

#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMNodeIterator.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMText.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/XMLDouble.hpp>

#include <xercesc/validators/common/Grammar.hpp>
#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>


#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <rw/loaders/xml/XercesErrorHandler.hpp>
#include <rw/loaders/xml/XMLBasisTypes.hpp>
#include <rw/loaders/xml/XercesUtils.hpp>
#include <rw/loaders/xml/XMLPropertyFormat.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
using namespace xercesc;

using namespace rwlibs::task;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::common;

XMLTaskLoader::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		XMLBasisTypes::Initializer init1;
		XMLPropertyFormat::Initializer init2;
		XMLPropertyLoader::Initializer init3;
		XMLTaskFormat::Initializer init4;
		done = true;
	}
}

const XMLTaskLoader::Initializer XMLTaskLoader::initializer;

namespace {

int readIntAttribute(xercesc::DOMElement* element, const XMLCh* id) {
    if (!element->hasAttribute(id)) {
        RW_THROW("Unable to find attribute: \""<<XMLStr(id).str()<<"\"");
    }
    const XMLCh* attr = element->getAttribute(id);
    XMLDouble value(attr);
    return (int)value.getValue();
}

std::string readStringAttribute(xercesc::DOMElement* element, const XMLCh* id) {
    if (!element->hasAttribute(id)) {
        RW_THROW("Unable to find attribute: \""<<XMLStr(id).str()<<"\"");
    }
    const XMLCh* attr = element->getAttribute(id);
    char* buffer = XMLString::transcode(attr);
    std::string str = buffer;
    delete buffer;
    return str;
}

MotionType readMotionTypeAttribute(xercesc::DOMElement* element, const XMLCh* id) {
	std::string type = readStringAttribute(element, id);
	if (type == XMLStr(XMLTaskFormat::idLinearMotion()).str())
		return MotionType::Linear;
	if (type == XMLStr(XMLTaskFormat::idP2PMotion()).str())
		return MotionType::P2P;
	if (type == XMLStr(XMLTaskFormat::idCircularMotion()).str())
		return MotionType::Circular;
	RW_THROW("Unsupported type of motion: \""<<type<<"\"");
}


template <class T>
class ElementReader {
public:
    static T readElement(xercesc::DOMElement* element);
};

template<> Q ElementReader<Q>::readElement(xercesc::DOMElement* element) {
    return XMLBasisTypes::readQ(element, false);
}

template<> Transform3D<> ElementReader<Transform3D<> >::readElement(xercesc::DOMElement* element) {
    return XMLBasisTypes::readTransform3D(element, false);
}

template<> std::string ElementReader<std::string>::readElement(xercesc::DOMElement* element) {
    return XMLBasisTypes::readString(element, false);
}




} //end anonymous namespace




void XMLTaskLoader::readEntityData(xercesc::DOMElement* element, Ptr<Entity> entity) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::idEntityIndex(), child->getNodeName())) {
				entity->setIndex(XMLBasisTypes::readInt(child, false));
			}
			if (XMLString::equals(XMLTaskFormat::idEntityId(), child->getNodeName())) {
				try {
					entity->setId(XMLBasisTypes::readElementText(child, false));
				} catch (...) {}
			}
			if (XMLString::equals(XMLPropertyFormat::idPropertyMap(), child->getNodeName())) {
				entity->setPropertyMap(XMLPropertyLoader::readProperties(child, false));
			}
		}
	}
}



template <class T>
typename Target<T>::Ptr XMLTaskLoader::readTarget(xercesc::DOMElement* element) {
	T value;
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();
	std::string id = readStringAttribute(element, XMLTaskFormat::idTargetIdAttr());
	
	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLBasisTypes::idQ(), child->getNodeName())) {
				value = ElementReader<T>::readElement(child);
			} else if (XMLString::equals(XMLBasisTypes::idTransform3D(), child->getNodeName())) {
				value = ElementReader<T>::readElement(child);
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




Action::Ptr XMLTaskLoader::readAction(xercesc::DOMElement* element) {
	int type = readIntAttribute(element, XMLTaskFormat::idActionTypeAttr());

	Action::Ptr action = ownedPtr(new Action(type));
	readEntityData(element, action);
	return action;
}





template <class T>
typename Motion<T>::Ptr XMLTaskLoader::readMotion(xercesc::DOMElement* element) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();
	MotionType type = readMotionTypeAttribute(element, XMLTaskFormat::idMotionTypeAttr());

	std::string start;
	std::string mid = "";
	std::string end;
	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::idMotionStart(), child->getNodeName())) {
				start = ElementReader<std::string>::readElement(child);
			} else if (XMLString::equals(XMLTaskFormat::idMotionMid(), child->getNodeName())) {
				mid = ElementReader<std::string>::readElement(child);
			} else if (XMLString::equals(XMLTaskFormat::idMotionEnd(), child->getNodeName())) {
				end = ElementReader<std::string>::readElement(child);
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
void XMLTaskLoader::readEntities(xercesc::DOMElement* element, typename Task<T>::Ptr task) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::idMotion(), child->getNodeName())) {
				task->addMotion(readMotion<T>(child));
			} else if (XMLString::equals(XMLTaskFormat::idAction(), child->getNodeName())) {
				task->addAction(readAction(child));
			} else if (XMLString::equals(XMLTaskFormat::idQTask(), child->getNodeName()) ||
			           XMLString::equals(XMLTaskFormat::idCartesianTask(), child->getNodeName())) {

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

template <class T>
void XMLTaskLoader::readTargets(xercesc::DOMElement* element, typename Task<T>::Ptr task) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();
	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::idQTarget(), child->getNodeName()) ||
                XMLString::equals(XMLTaskFormat::idCartesianTarget(), child->getNodeName()))
			{				
				task->addTarget(readTarget<T>(child));
			}
		}
	}
}


void XMLTaskLoader::readAugmentations(xercesc::DOMElement* element, TaskBase::Ptr task) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
		if (child != NULL) {
			XMLTaskLoader loader;
			TaskBasePtr augmentation = loader.readTask(child);
			task->addAugmentation(augmentation, augmentation->getId());
		}
	}
}




template <class T>
typename Task<T>::Ptr XMLTaskLoader::readTemplateTask(xercesc::DOMElement* element) {
	typename Task<T>::Ptr task = ownedPtr(new Task<T>());

	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::idTargets(), child->getNodeName())) {
				readTargets<T>(child, task);
			} else if (XMLString::equals(XMLTaskFormat::idEntities(), child->getNodeName())) {
				readEntities<T>(child, task);
			} else if (XMLString::equals(XMLTaskFormat::idAugmentations(), child->getNodeName())) {
				readAugmentations(child, task);
			}
		}
	}
	readEntityData(element, task);
	return task;
}


TaskBase::Ptr XMLTaskLoader::readTask(xercesc::DOMElement* element) {

	if (XMLString::equals(XMLTaskFormat::idQTask(), element->getNodeName())) {
		_qTask = readTemplateTask<Q>(element);
		return _qTask;
	} else if (XMLString::equals(XMLTaskFormat::idCartesianTask(), element->getNodeName())) {
		_cartTask = readTemplateTask<Transform3D<> >(element);
		return _cartTask;
	} else {
		RW_THROW("Element name does not match " + XMLStr(XMLTaskFormat::idQTask()).str() + " or " + XMLStr(XMLTaskFormat::idCartesianTask()).str() + " as expected");
	}
}


void XMLTaskLoader::load(std::istream& instream, const std::string& schemaFileName) {
    XercesDOMParser parser;
    //The document is owned by the parser.
	xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, instream, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    _task = readTask(elementRoot);
	
}

void XMLTaskLoader::load(const std::string& filename, const std::string& schemaFileName) {
    XercesDOMParser parser;
	//The document is owned by the parser.
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, schemaFileName);
		
	xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    _task = readTask(elementRoot);
	


}


QTask::Ptr XMLTaskLoader::getQTask() {
	if (_qTask == NULL)
		RW_THROW("No QTask Loaded");
	return _qTask;
}

CartesianTask::Ptr XMLTaskLoader::getCartesianTask() {
	if (_cartTask == NULL)
		RW_THROW("No CartesianTask Loaded");
	return _cartTask;
}

TaskBase::Ptr XMLTaskLoader::getTask() {
	if (_task == NULL)
		RW_THROW("No Task Loaded");
	return _task;

}

TaskLoader::Ptr XMLTaskLoader::clone() const {
	return ownedPtr(new XMLTaskLoader());
}
