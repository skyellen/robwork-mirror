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



#include "XMLTaskSaver.hpp"

#include "XMLTaskFormat.hpp"

#include <rw/loaders/xml/XercesUtils.hpp>
#include <rw/loaders/xml/XMLBasisTypes.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#include <rw/math/Q.hpp>

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>

#include <xercesc/dom/DOMImplementationLS.hpp>

#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>

#include <boost/foreach.hpp>

using namespace xercesc;
using namespace rw::common;
using namespace rw::math;
using namespace rw::loaders;
using namespace rwlibs::task;

XMLTaskSaver::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		XMLBasisTypes::Initializer init1;
		XMLPropertySaver::Initializer init2;
		XMLTaskFormat::Initializer init3;
		done = true;
	}
}

const XMLTaskSaver::Initializer XMLTaskSaver::initializer;

XMLTaskSaver::XMLTaskSaver() {
	// TODO Auto-generated constructor stub

}

XMLTaskSaver::~XMLTaskSaver() {
	// TODO Auto-generated destructor stub
}


namespace {

template <class T>
class Identifiers {
 public:
     static const XMLCh* taskId();
     static const XMLCh* targetId();
 };

template<> const XMLCh* Identifiers<Q>::taskId() {
    return XMLTaskFormat::idQTask();
}

template<> const XMLCh* Identifiers<Q>::targetId() {
    return XMLTaskFormat::idQTarget();
}


template<> const XMLCh* Identifiers<Transform3D<> >::taskId() {
    return XMLTaskFormat::idCartesianTask();
}

template<> const XMLCh* Identifiers<Transform3D<> >::targetId() {
    return XMLTaskFormat::idCartesianTarget();
}


template <class T>
class ElementCreator {
public:
    static xercesc::DOMElement* createElement(const T& element, xercesc::DOMDocument* doc);
};

template<> xercesc::DOMElement* ElementCreator<Q>::createElement(const Q& element, xercesc::DOMDocument* doc) {
    return XMLBasisTypes::createQ(element, doc);
}

template<> xercesc::DOMElement* ElementCreator<Transform3D<> >::createElement(const Transform3D<>& element, xercesc::DOMDocument* doc) {
    return XMLBasisTypes::createTransform3D(element, doc);
}



}


void XMLTaskSaver::writeEntityInfo(Entity::Ptr entity, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {

    xercesc::DOMElement* idElement = doc->createElement(XMLTaskFormat::idEntityId());
    DOMText* txt = doc->createTextNode(XMLStr(entity->getId()).uni());
    idElement->appendChild(txt);
	element->appendChild(idElement);


	xercesc::DOMElement* indexElement = doc->createElement(XMLTaskFormat::idEntityIndex());
	txt = doc->createTextNode(XMLStr(entity->getIndex()).uni());
	indexElement->appendChild(txt);
	element->appendChild(indexElement);

	xercesc::DOMElement* propertyMapElement = XMLPropertySaver::save(entity->getPropertyMap(), doc);
	element->appendChild(propertyMapElement);

}

template <class T>
void XMLTaskSaver::writeTargets(typename Task<T>::Ptr task, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* targetsElement = doc->createElement(XMLTaskFormat::idTargets());
    element->appendChild(targetsElement);

	std::vector<rw::common::Ptr<Target<T> > > targets = task->getTargets();

	int targetId = 0;
	BOOST_FOREACH(rw::common::Ptr<Target<T> > target, targets) {
		xercesc::DOMElement* targetElement = doc->createElement(Identifiers<T>::targetId());
		targetsElement->appendChild(targetElement);

		DOMAttr* idAttr = doc->createAttribute(XMLTaskFormat::idTargetIdAttr());
		idAttr->setValue(XMLStr(targetId).uni());
		_targetMap[target] = XMLStr(idAttr->getValue()).str();
		targetElement->setAttributeNode(idAttr);

		targetElement->appendChild(ElementCreator<T>::createElement(target->get(), doc));
		writeEntityInfo(target, targetElement, doc);

		targetId++;
	}


}


template <class T>
void XMLTaskSaver::writeMotion(typename Motion<T>::Ptr motion, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {
	xercesc::DOMElement* motionElement = doc->createElement(XMLTaskFormat::idMotion());
	element->appendChild(motionElement);

	DOMAttr* typeAttr = doc->createAttribute(XMLTaskFormat::idMotionTypeAttr());
	switch (motion->motionType()) {
	case MotionType::Linear:
		typeAttr->setValue(XMLTaskFormat::idLinearMotion());
		break;
	case MotionType::P2P:
		typeAttr->setValue(XMLTaskFormat::idP2PMotion());
		break;
	case MotionType::Circular:
		typeAttr->setValue(XMLTaskFormat::idCircularMotion());
		break;
	}

	motionElement->setAttributeNode(typeAttr);

	xercesc::DOMElement* targetElement = doc->createElement(XMLTaskFormat::idMotionStart());
	motionElement->appendChild(targetElement);
	targetElement->appendChild(doc->createTextNode(XMLStr(_targetMap[motion->startTarget()]).uni()));


	if (motion->motionType() == MotionType::Circular) {
		targetElement = doc->createElement(XMLTaskFormat::idMotionMid());
		motionElement->appendChild(targetElement);
		targetElement->appendChild(doc->createTextNode(XMLStr(_targetMap[motion.template cast<CircularMotion<T> >()->midTarget()]).uni()));
	}

	targetElement = doc->createElement(XMLTaskFormat::idMotionEnd());
	motionElement->appendChild(targetElement);
	targetElement->appendChild(doc->createTextNode(XMLStr(_targetMap[motion->endTarget()]).uni()));


	writeEntityInfo(motion, motionElement, doc);
}

void XMLTaskSaver::writeAction(Action::Ptr action, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {
	xercesc::DOMElement* actionElement = doc->createElement(XMLTaskFormat::idAction());
	element->appendChild(actionElement);

	DOMAttr* typeAttr = doc->createAttribute(XMLTaskFormat::idActionTypeAttr());
	typeAttr->setValue(XMLStr(action->actionType()).uni());
	actionElement->setAttributeNode(typeAttr);

	writeEntityInfo(action, actionElement, doc);
}

template <class T>
void XMLTaskSaver::writeEntities(typename Task<T>::Ptr task, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* entriesElement = doc->createElement(XMLTaskFormat::idEntities());
    element->appendChild(entriesElement);

	std::vector<Entity::Ptr> entities = task->getEntities();
	BOOST_FOREACH(Entity::Ptr entity, entities) {
		switch (entity->entityType()) {
		case EntityType::Task:
			writeTask(entity.cast<Task<T> >(), entriesElement, doc);
			break;
		case EntityType::Motion:
			writeMotion<T>(entity.cast<Motion<T> >(), entriesElement, doc);
			break;
		case EntityType::Action:
			writeAction(entity.cast<Action>(), entriesElement, doc);
			break;
		case EntityType::User:
			RW_THROW("Unable to save user specified types");
			break;
		}
	}


}



template <class T>
void XMLTaskSaver::writeTaskToElement(typename Task<T>::Ptr task, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {
    writeTargets<T>(task, element, doc);
    writeEntities<T>(task, element, doc);
    writeEntityInfo(task, element, doc);
}

template <class T>
void XMLTaskSaver::writeTaskImpl(typename Task<T>::Ptr task, xercesc::DOMElement* parent, xercesc::DOMDocument* doc) {
    xercesc::DOMElement* taskElement = doc->createElement(Identifiers<T>::taskId());
    parent->appendChild(taskElement);
    writeTaskToElement<T>(task, taskElement, doc);
}


void XMLTaskSaver::writeTask(TaskBase::Ptr task, xercesc::DOMElement* parent, xercesc::DOMDocument* doc) {
	Task<Q>::Ptr qtask = task.cast<Task<Q> >();
	if (qtask != NULL) {
		writeTaskImpl<Q>(qtask, parent, doc);
		return;
	}

	Task<Transform3D<> >::Ptr carttask = task.cast<Task<Transform3D<> > >();
	if (carttask != NULL) {
		writeTaskImpl<Transform3D<> >(carttask, parent, doc);
		return;
	}


}

template <class T>
bool XMLTaskSaver::saveImpl(typename Task<T>::Ptr task, XMLFormatTarget* target) {
    XMLCh* features = XMLString::transcode("Core");
	//It appears that this value originates from a registry and should therefore not be deleted.
    DOMImplementation* impl =  DOMImplementationRegistry::getDOMImplementation(features);
    XMLString::release(&features);	
    if (impl != NULL)
    {
		
        try
        {
			rw::common::Ptr<xercesc::DOMDocument> doc = NULL;
             doc = ownedPtr(impl->createDocument(0,               // root element namespace URI.
                                        Identifiers<T>::taskId(), // root element name
                                        0));                   	  // We do not wish to specify a document type

            xercesc::DOMElement* root = doc->getDocumentElement();
            writeTaskToElement<T>(task, root, doc.get());


            XercesDocumentWriter::writeDocument(doc.get(), target);

        }
        catch (const OutOfMemoryException&)
        {
            RW_THROW("XMLTaskSaver: OutOfMemory");
        }
        catch (const DOMException& e)
        {
            RW_THROW("XMLTaskSaver: DOMException:  " << XMLString::transcode(e.getMessage()));
        }
        catch (const rw::common::Exception& exp) {
            throw exp;
        } 
        catch (...)
        {
            RW_THROW("XMLTaskSaver: Unknown Exception while creating saving path");
		} 
    }
    else
    {
        RW_THROW("XMLTaskSaver: Unable to find a suitable DOM Implementation");
    }
    return true;
}


bool XMLTaskSaver::save(rwlibs::task::QTask::Ptr task, std::ostream& outstream) {
    XMLTaskSaver saver;
    OutStreamFormatTarget target(outstream);
    return saver.saveImpl<Q>(task, &target);
}

bool XMLTaskSaver::save(rwlibs::task::CartesianTask::Ptr task, std::ostream& outstream) {	
    XMLTaskSaver saver;
    OutStreamFormatTarget target(outstream);
    return saver.saveImpl<Transform3D<> >(task, &target);
}



bool XMLTaskSaver::save(QTask::Ptr task, const std::string& filename) {
	XMLTaskSaver saver;
    LocalFileFormatTarget target(filename.c_str());
    return saver.saveImpl<Q>(task, &target);
}


bool XMLTaskSaver::save(CartesianTask::Ptr task, const std::string& filename) {
    XMLTaskSaver saver;
    LocalFileFormatTarget target(filename.c_str());
    return saver.saveImpl<Transform3D<> >(task, &target);
}
