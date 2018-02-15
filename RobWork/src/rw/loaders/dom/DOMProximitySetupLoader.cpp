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

#include "DOMProximitySetupLoader.hpp"

#include <iostream>
#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::proximity;

ProximitySetup DOMProximitySetupLoader::load(const std::string& filename, const std::string& schemaFileName)
{
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load(filename);
    DOMElem::Ptr root = parser->getRootElement();
    DOMElem::Ptr psetupRoot = root->getChild("ProximitySetup", false);
    return readProximitySetup(psetupRoot);
}


ProximitySetup DOMProximitySetupLoader::load(std::istream& instream, const std::string& schemaFileName) {
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load(instream);
    DOMElem::Ptr root = parser->getRootElement();
    DOMElem::Ptr psetupRoot = root->getChild("ProximitySetup", false);
    return readProximitySetup(psetupRoot);
}


ProximitySetup DOMProximitySetupLoader::readProximitySetup(DOMElem::Ptr element) {
    if (!element->isName("ProximitySetup") )
    	RW_THROW("Failed to parse element \""<<element->getName()<<" as ProximitySetup");

	ProximitySetup setup;
	bool incall = element->getAttributeValueAsBool("UseIncludeAll", true);
	bool exstatic = element->getAttributeValueAsBool("UseExcludeStaticPairs", true);
	setup.setUseIncludeAll(incall);
	setup.setUseExcludeStaticPairs(exstatic);

	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
		if(child->isName("Exclude")){
			setup.addProximitySetupRule(ProximitySetupRule::makeExclude(readFramePatternAttributes(child)));
		} else if (child->isName("Include")) {
			setup.addProximitySetupRule(ProximitySetupRule::makeInclude(readFramePatternAttributes(child)));
		} else {
			if(!child->isName("<xmlcomment>"))
				RW_THROW("Unknown element \"" << child->getName() << "\" in ProximitySetup, should be Include or Exclude!");
		}
	}
	return setup ;
}

std::pair<std::string, std::string> DOMProximitySetupLoader::readFramePatternAttributes(DOMElem::Ptr element) {
	if ( !(element->hasAttribute("PatternA") && element->hasAttribute("PatternB")) )
		RW_THROW("Element does not have the expected attributes");

	const std::string attr1 = element->getAttributeValue("PatternA");
	const std::string attr2 = element->getAttributeValue("PatternB");
	return std::make_pair(attr1, attr2);
}
