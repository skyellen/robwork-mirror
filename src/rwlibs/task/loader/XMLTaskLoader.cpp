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

XMLTaskLoader::XMLTaskLoader() {
	// TODO Auto-generated constructor stub

}

XMLTaskLoader::~XMLTaskLoader() {
	// TODO Auto-generated destructor stub
}


namespace {

int readIntAttribute(DOMElement* element, const XMLCh* id) {
    if (element->hasAttribute(id)) {
        const XMLCh* attr = element->getAttribute(id);
        XMLDouble value(attr);
        return (int)value.getValue();
    }
    RW_THROW("Unable to find attribute: \""<<XMLStr(id).str()<<"\"");
}

std::string readStringAttribute(DOMElement* element, const XMLCh* id) {
    if (element->hasAttribute(id)) {
        const XMLCh* attr = element->getAttribute(id);
        return XMLString::transcode(attr);
    }
    RW_THROW("Unable to find attribute: \""<<XMLStr(id).str()<<"\"");
}

MotionType readMotionTypeAttribute(DOMElement* element, const XMLCh* id) {
	std::string type = readStringAttribute(element, id);
	if (type == XMLStr(XMLTaskFormat::LinearMotionId).str())
		return MotionType::Linear;
	if (type == XMLStr(XMLTaskFormat::P2PMotionId).str())
		return MotionType::P2P;
	if (type == XMLStr(XMLTaskFormat::CircularMotionId).str())
		return MotionType::Circular;
	RW_THROW("Unsupported type of motion: \""<<type<<"\"");
}


template <class T>
class ElementReader {
public:
    static T readElement(DOMElement* element);
};

template<> Q ElementReader<Q>::readElement(DOMElement* element) {
    return XMLBasisTypes::readQ(element, false);
}

template<> Transform3D<> ElementReader<Transform3D<> >::readElement(DOMElement* element) {
    return XMLBasisTypes::readTransform3D(element, false);
}

template<> std::string ElementReader<std::string>::readElement(DOMElement* element) {
    return XMLBasisTypes::readString(element, false);
}




} //end anonymous namespace




void XMLTaskLoader::readEntityData(DOMElement* element, Ptr<Entity> entity) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::EntityIndexId, child->getNodeName())) {
				entity->setIndex(XMLBasisTypes::readInt(child, false));
			}
			if (XMLString::equals(XMLTaskFormat::EntityIdId, child->getNodeName())) {
				try {
					entity->setId(XMLBasisTypes::readElementText(child, false));
				} catch (...) {}
			}
			if (XMLString::equals(XMLPropertyFormat::PropertyMapId, child->getNodeName())) {
				entity->setPropertyMap(XMLPropertyLoader::readProperties(child, false));
			}
		}
	}
}



template <class T>
Ptr<Target<T> > XMLTaskLoader::readTarget(DOMElement* element) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();
	std::string id = readStringAttribute(element, XMLTaskFormat::TargetIdAttrId);
	T value;
	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLBasisTypes::QId, child->getNodeName())) {
				value = ElementReader<T>::readElement(child);
				std::cout<<"Got Value = "<<value<<std::endl;
			} else if (XMLString::equals(XMLBasisTypes::Transform3DId, child->getNodeName())) {
				value = ElementReader<T>::readElement(child);
				std::cout<<"Got Value = "<<value<<std::endl;
			}

		}
	}
	Ptr<Target<T> > target = ownedPtr(new Target<T>(value));
	readEntityData(element, target);
	if (_targetMap.find(id) != _targetMap.end()) {
		RW_THROW("Multiple targets with id \""<<id<<"\" exists");
	}
	_targetMap[id] = target;
	return target;
}




ActionPtr XMLTaskLoader::readAction(DOMElement* element) {
	int type = readIntAttribute(element, XMLTaskFormat::ActionTypeAttrId);

	ActionPtr action = ownedPtr(new Action(type));
	readEntityData(element, action);
	return action;
}





template <class T>
Ptr<Motion<T> > XMLTaskLoader::readMotion(DOMElement* element) {
	std::cout<<"Read Motion"<<std::endl;
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();
	MotionType type = readMotionTypeAttribute(element, XMLTaskFormat::MotionTypeAttrId);
	std::cout<<"Got Type = "<<type<<std::endl;

	std::string start;
	std::string mid = "";
	std::string end;
	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::MotionStartId, child->getNodeName())) {
				start = ElementReader<std::string>::readElement(child);
			} else if (XMLString::equals(XMLTaskFormat::MotionMidId, child->getNodeName())) {
				mid = ElementReader<std::string>::readElement(child);
			} else if (XMLString::equals(XMLTaskFormat::MotionEndId, child->getNodeName())) {
				end = ElementReader<std::string>::readElement(child);
			}
		}
	}


	typedef std::map<std::string, TargetPtr> MyMap;
	typename MyMap::iterator it, endIt;
	it = _targetMap.find(start);
	endIt = _targetMap.end();
	if (it == endIt)
		RW_THROW("Unable to find Target named \""<<start<<"\"");

	Ptr<Target<T> > startTarget = (*it).second.cast<Target<T> >();

	Ptr<Target<T> > midTarget;
	if (mid != "") {
		it = _targetMap.find(mid);
		if (it == endIt)
			RW_THROW("Unable to find Target named \""<<mid<<"\"");
		midTarget = (*it).second.cast<Target<T> >();
	}

	it = _targetMap.find(end);
	if (it == endIt)
		RW_THROW("Unable to find Target named \""<<end<<"\"");
	Ptr<Target<T> > endTarget = (*it).second.cast<Target<T> >();

	Ptr<Motion<T> > motion;
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
void XMLTaskLoader::readEntities(DOMElement* element, Ptr<Task<T> > task) {
	std::cout<<"Read entities"<<std::endl;
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::MotionId, child->getNodeName())) {
				task->addMotion(readMotion<T>(child));
			} else if (XMLString::equals(XMLTaskFormat::ActionId, child->getNodeName())) {
				task->addAction(readAction(child));
			} else if (XMLString::equals(XMLTaskFormat::QTaskId, child->getNodeName()) ||
			           XMLString::equals(XMLTaskFormat::CartesianTaskId, child->getNodeName())) {

			    //A subtask may have duplicates of the target names. We therefore
			    //store the old targets temporarily and
			    std::cout<<"Craetes new tmp target map"<<std::endl;
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
void XMLTaskLoader::readTargets(DOMElement* element, Ptr<Task<T> > task) {
	std::cout<<"Read targets"<<std::endl;
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::QTargetId, child->getNodeName()) ||
                XMLString::equals(XMLTaskFormat::CartesianTargetId, child->getNodeName())){
				task->addTarget(readTarget<T>(child));
			}
		}
	}
	std::cout<<"Target Count = "<<_targetMap.size()<<std::endl;
}


void XMLTaskLoader::readAugmentations(DOMElement* element, TaskBasePtr task) {
	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
		if (child != NULL) {
			XMLTaskLoader loader;
			TaskBasePtr augmentation = loader.readTask(child);
			task->addAugmentation(augmentation, augmentation->getId());
		}
	}
}




template <class T>
Ptr<Task<T> > XMLTaskLoader::readTemplateTask(DOMElement* element) {
	Ptr<Task<T> > task = ownedPtr(new Task<T>());

	DOMNodeList* children = element->getChildNodes();
	const  XMLSize_t nodeCount = children->getLength();

	for(XMLSize_t i = 0; i < nodeCount; ++i ) {
		DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
		if (child != NULL) {
			if (XMLString::equals(XMLTaskFormat::TargetsId, child->getNodeName())) {
				readTargets<T>(child, task);
			} else if (XMLString::equals(XMLTaskFormat::EntitiesId, child->getNodeName())) {
				readEntities<T>(child, task);
			} else if (XMLString::equals(XMLTaskFormat::AugmentationsId, child->getNodeName())) {
				readAugmentations(child, task);
			}
		}
	}
	readEntityData(element, task);
	return task;
}


TaskBasePtr XMLTaskLoader::readTask(DOMElement* element) {

	if (XMLString::equals(XMLTaskFormat::QTaskId, element->getNodeName())) {
		_qTask = readTemplateTask<Q>(element);
		return _qTask;
	} else if (XMLString::equals(XMLTaskFormat::CartesianTaskId, element->getNodeName())) {
		_cartTask = readTemplateTask<Transform3D<> >(element);
		return _cartTask;
	} else {
		RW_THROW("Element name does not match " + XMLStr(XMLTaskFormat::QTaskId).str() + " or " + XMLStr(XMLTaskFormat::CartesianTaskId).str() + " as expected");
	}
}

void XMLTaskLoader::load(const std::string& filename, const std::string& schemaFileName) {
    try
    {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    }
    catch( XMLException& e )
    {
       RW_THROW("Xerces initialization Error"<<XMLStr(e.getMessage()).str());
    }

    XercesDOMParser parser;

    XercesErrorHandler errorHandler;

    parser.setDoNamespaces( true );
    parser.setDoSchema( true );
    if (schemaFileName.size() != 0)
        parser.setExternalNoNamespaceSchemaLocation(schemaFileName.c_str());


    parser.setErrorHandler(&errorHandler);
    parser.setValidationScheme(XercesDOMParser::Val_Auto);
    parser.parse(filename.c_str() );
    if (parser.getErrorCount() != 0) {
        std::cerr<<std::endl<<std::endl<<"Error(s) = "<<std::endl<<errorHandler.getMessages()<<std::endl;
        RW_THROW(""<<parser.getErrorCount()<<" Errors: "<<std::endl<<errorHandler.getMessages());
    }


    // no need to free this pointer - owned by the parent parser object
    DOMDocument* xmlDoc = parser.getDocument();

    // Get the top-level element: Name is "root". No attributes for "root"
    DOMElement* elementRoot = xmlDoc->getDocumentElement();
    _task = readTask(elementRoot);
}


QTaskPtr XMLTaskLoader::getQTask() {
	if (_qTask == NULL)
		RW_THROW("No QTask Loaded");
	return _qTask;
}

CartesianTaskPtr XMLTaskLoader::getCartesianTask() {
	if (_cartTask == NULL)
		RW_THROW("No CartesianTask Loaded");
	return _cartTask;
}

TaskBasePtr XMLTaskLoader::getTask() {
	if (_task == NULL)
		RW_THROW("No Task Loaded");
	return _task;

}

