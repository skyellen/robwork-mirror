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

#include "XMLProximitySetupLoader.hpp"

#include <iostream>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>
#include <xercesc/dom/DOMElement.hpp>
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

#include "XercesErrorHandler.hpp"
#include "XercesUtils.hpp"

using namespace xercesc;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::proximity;

XMLProximitySetupFormat::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
		static XercesInitializer initializer;
		idProximitySetup();
		idExcludeRule();
		idIncludeRule();
		idIncludeAllAttribute();
		idExcludeStaticAttributePairs();
		idPatternAAttribute();
		idPatternBAttribute();
		done = true;
	}
}

const XMLProximitySetupFormat::Initializer XMLProximitySetupFormat::initializer;

const XMLCh* XMLProximitySetupFormat::idProximitySetup() {
	static const XMLStr id("ProximitySetup");
	return id.uni();
}

const XMLCh* XMLProximitySetupFormat::idExcludeRule() {
	static const XMLStr id("Exclude");
	return id.uni();
}

const XMLCh* XMLProximitySetupFormat::idIncludeRule() {
	static const XMLStr id("Include");
	return id.uni();
}

const XMLCh* XMLProximitySetupFormat::idIncludeAllAttribute() {
	static const XMLStr id("UseIncludeAll");
	return id.uni();
}

const XMLCh* XMLProximitySetupFormat::idExcludeStaticAttributePairs() {
	static const XMLStr id("UseExcludeStaticPairs");
	return id.uni();
}

const XMLCh* XMLProximitySetupFormat::idPatternAAttribute() {
	static const XMLStr id("PatternA");
	return id.uni();
}

const XMLCh* XMLProximitySetupFormat::idPatternBAttribute() {
	static const XMLStr id("PatternB");
	return id.uni();
}

XMLProximitySetupLoader::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		XMLProximitySetupFormat::Initializer init1;
		done = true;
	}
}

const XMLProximitySetupLoader::Initializer XMLProximitySetupLoader::initializer;

ProximitySetup XMLProximitySetupLoader::load(const std::string& filename, const std::string& schemaFileName)
{
    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    return readProximitySetup(elementRoot);
}


ProximitySetup XMLProximitySetupLoader::load(std::istream& instream, const std::string& schemaFileName) {    
    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, instream, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    return readProximitySetup(elementRoot);
}


ProximitySetup XMLProximitySetupLoader::readProximitySetup(xercesc::DOMElement* element) {
    if (XMLString::equals(XMLProximitySetupFormat::idProximitySetup(), element->getNodeName())) {
        ProximitySetup setup;

		if (element->hasAttributes()) {
			if (element->hasAttribute(XMLProximitySetupFormat::idIncludeAllAttribute())) {
		        const XMLCh* attr = element->getAttribute(XMLProximitySetupFormat::idIncludeAllAttribute());
				if (XMLStr(attr).str() == "true") {
					setup.setUseIncludeAll(true);
				} else if (XMLStr(attr).str() == "false"){
					setup.setUseIncludeAll(false);
				} else {
					RW_THROW("Unknown value for "<<XMLStr(XMLProximitySetupFormat::idIncludeAllAttribute()).str()<<" attribute. Please specify \"true\" or \"false\"");
				}
			} else if (element->hasAttribute(XMLProximitySetupFormat::idExcludeStaticAttributePairs())) {
		        const XMLCh* attr = element->getAttribute(XMLProximitySetupFormat::idExcludeStaticAttributePairs());
				if (XMLStr(attr).str() == "true") {
					setup.setUseExcludeStaticPairs(true);
				} else if (XMLStr(attr).str() == "false"){
					setup.setUseExcludeStaticPairs(false);
				} else {
					RW_THROW("Unknown value for "<<XMLStr(XMLProximitySetupFormat::idExcludeStaticAttributePairs()).str()<<" attribute. Please specify \"true\" or \"false\"");
				}
			} else {
				RW_THROW("Unknown attribute on "<<XMLStr(XMLProximitySetupFormat::idProximitySetup()).str());
			}
        } 

        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (child != NULL) {
				if (XMLString::equals(child->getNodeName(), XMLProximitySetupFormat::idExcludeRule())) {
					setup.addProximitySetupRule(ProximitySetupRule::makeExclude(readFramePatternAttributes(child)));					
				} else if (XMLString::equals(child->getNodeName(), XMLProximitySetupFormat::idIncludeRule())) {
					setup.addProximitySetupRule(ProximitySetupRule::makeInclude(readFramePatternAttributes(child)));					
				}
			}
		}
		return setup;
	}
	RW_THROW("Failed to parse element \""<<XMLStr(element->getNodeName()).str()<<" as ProximitySetup");
}

std::pair<std::string, std::string> XMLProximitySetupLoader::readFramePatternAttributes(xercesc::DOMElement* element) {
	if (element->hasAttribute(XMLProximitySetupFormat::idPatternAAttribute()) &&
		element->hasAttribute(XMLProximitySetupFormat::idPatternBAttribute()))
	{
		const XMLCh* attr1 = element->getAttribute(XMLProximitySetupFormat::idPatternAAttribute());
		const XMLCh* attr2 = element->getAttribute(XMLProximitySetupFormat::idPatternBAttribute());
		return std::make_pair(XMLStr(attr1).str(), XMLStr(attr2).str());
	}
	RW_THROW("Element does not have the expected attributes");
}
