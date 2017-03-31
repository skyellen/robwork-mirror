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

#include "DOMWorkCellSaver.hpp"

#include <rw/common/DOMElem.hpp>
#include <rw/common/DOMParser.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

using namespace rw::common;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw;

namespace {
    class ElementCreator {
    public:
        ElementCreator(DOMElem::Ptr root) :
                _root(root) {

        }
        template<class T>
        DOMElem::Ptr createElement(T object, DOMElem::Ptr parent);

    private:
        DOMElem::Ptr _root;
    };

    template<>
    DOMElem::Ptr ElementCreator::createElement<FixedFrame*>(FixedFrame* frame, DOMElem::Ptr parent)
    {
        DOMElem::Ptr element = parent->addChild("Frame");

        // Set attributes
        element->addAttribute("name")->setValue(frame->getName());

        // The WORLD frame should not be added to the file
        if (frame->getName() != "WORLD")
        {
            element->addAttribute("refframe")->setValue(frame->getParent()->getName());
        }

        const std::string type = "Fixed";
        element->addAttribute("type")->setValue(type);

        //Add the rotation in RPY
        DOMBasisTypes::createRPY(RPY<>(frame->getFixedTransform().R()), element);

        //Add the position
        DOMBasisTypes::createPos(frame->getFixedTransform().P(), element);
        return element;
    }

    void createDOMDocument(DOMElem::Ptr rootDoc, rw::models::WorkCell::Ptr workcell) {
        DOMElem::Ptr rootElement = rootDoc->addChild("WorkCell");
        rootElement->addAttribute("name")->setValue(workcell->getName());
        ElementCreator creator(rootElement);

        const std::vector<Frame*> wc_frames = workcell->getFrames();

        for(const auto &frame : wc_frames)
        {
            if(FixedFrame* ff = dynamic_cast<FixedFrame*>(frame))
            {
                std::cout << "The frame type was Fixed!" << std::endl;
                creator.createElement<FixedFrame*>(ff, rootDoc);
            }
            else
            {
                std::cout << "The frame type was not Fixed!" << std::endl;
            }
        }
    }
} // end of anonymous namespace

void DOMWorkCellSaver::save(rw::models::WorkCell::Ptr workcell, std::string fileName) {
    DOMParser::Ptr doc = DOMParser::make();
    DOMElem::Ptr root = doc->getRootElement();

    createDOMDocument(root, workcell);

    // save to file
    doc->save( fileName );
}

void DOMWorkCellSaver::save(rw::models::WorkCell::Ptr workcell, std::ostream& ostream) {
    DOMParser::Ptr doc = DOMParser::make();
    DOMElem::Ptr root = doc->getRootElement();

    createDOMDocument(root, workcell);

    // save to stream
    doc->save( ostream );
}

