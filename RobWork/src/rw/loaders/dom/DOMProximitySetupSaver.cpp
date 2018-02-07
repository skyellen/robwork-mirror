/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/common/DOMParser.hpp>
#include <rw/loaders/dom/DOMProximitySetupSaver.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/common/DOMElem.hpp>
#include <rw/proximity/ProximitySetup.hpp>
#include <rw/proximity/ProximitySetupRule.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::proximity;

DOMProximitySetupSaver::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		DOMBasisTypes::Initializer init1;
		done = true;
	}
}

const DOMProximitySetupSaver::Initializer DOMProximitySetupSaver::initializer;

void DOMProximitySetupSaver::save(const ProximitySetup& prox, Ptr<DOMElem> parent) {
    Ptr<DOMElem> root = parent->addChild("ProximitySetup");
    if (prox.useIncludeAll())
        root->addAttribute("UseIncludeAll")->setValueString("true");
    else
        root->addAttribute("UseIncludeAll")->setValueString("false");

    if (prox.useExcludeStaticPairs())
        root->addAttribute("UseExcludeStaticPairs")->setValueString("true");
    else
        root->addAttribute("UseExcludeStaticPairs")->setValueString("false");

    const std::vector<ProximitySetupRule>& prox_rules = prox.getProximitySetupRules();

    for (const auto &prox_rule : prox_rules) {
        Ptr<DOMElem> element;
        if (prox_rule.type() == ProximitySetupRule::EXCLUDE_RULE) {
            element = root->addChild("Exclude");
            std::pair<std::string, std::string> patterns = prox_rule.getPatterns();
            element->addAttribute("PatternA")->setValue(patterns.first);
            element->addAttribute("PatternB")->setValue(patterns.second);
        }
        else if (prox_rule.type() == ProximitySetupRule::INCLUDE_RULE) {
            element = root->addChild("Include");
            std::pair<std::string, std::string> patterns = prox_rule.getPatterns();
            element->addAttribute("PatternA")->setValue(patterns.first);
            element->addAttribute("PatternB")->setValue(patterns.second);
        }
        else
            RW_THROW("The ProximitySetupRule has an unknown type!");
    }
}

void DOMProximitySetupSaver::save(const ProximitySetup& prox, const std::string& filename) {
    DOMParser::Ptr parser = DOMParser::make();
    createDOMDocument(prox, parser);
    parser->save(filename);
}

void DOMProximitySetupSaver::write(const ProximitySetup& prox, std::ostream& outstream) {
    DOMParser::Ptr parser = DOMParser::make();
    createDOMDocument(prox, parser);
    parser->save(outstream);
}

DOMElem::Ptr DOMProximitySetupSaver::createDOMDocument(const ProximitySetup& prox, Ptr<DOMParser> parser) {
    Ptr<DOMElem> doc = parser->getRootElement();
    save(prox, doc);
    return doc;
}
