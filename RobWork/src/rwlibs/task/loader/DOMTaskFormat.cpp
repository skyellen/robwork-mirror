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

#include "DOMTaskFormat.hpp"

#include <rw/loaders/dom/DOMBasisTypes.hpp>

using rw::loaders::DOMBasisTypes;
using rwlibs::task::DOMTaskFormat;

DOMTaskFormat::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
        DOMBasisTypes::Initializer init;
        idQTask();
        idCartesianTask();
        idTargets();
        idEntities();
        idAugmentations();
        idQTarget();
        idCartesianTarget();
        idMotion();
        idAction();
        idEntityIndex();
        idEntityId();
        idTargetIdAttr();
        idMotionTypeAttr();
        idMotionStart();
        idMotionMid();
        idMotionEnd();
        idLinearMotion();
        idP2PMotion();
        idCircularMotion();
        idActionTypeAttr();

        done = true;
	}
}

const DOMTaskFormat::Initializer DOMTaskFormat::initializer;

const std::string& DOMTaskFormat::idQTask(){
    static const std::string id("QTask");
    return id;
}
const std::string& DOMTaskFormat::idCartesianTask(){
    static const std::string id("CartesianTask");
    return id;
}
const std::string& DOMTaskFormat::idTargets(){
    static const std::string id("Targets");
    return id;
}
const std::string& DOMTaskFormat::idEntities(){
    static const std::string id("Entities");
    return id;
}
const std::string& DOMTaskFormat::idAugmentations(){
    static const std::string id("Augmentations");
    return id;
}
const std::string& DOMTaskFormat::idQTarget(){
    static const std::string id("QTarget");
    return id;
}
const std::string& DOMTaskFormat::idCartesianTarget(){
    static const std::string id("CartesianTarget");
    return id;
}
const std::string& DOMTaskFormat::idMotion(){
    static const std::string id("Motion");
    return id;
}
const std::string& DOMTaskFormat::idAction(){
    static const std::string id("Action");
    return id;
}
const std::string& DOMTaskFormat::idEntityIndex(){
    static const std::string id("Index");
    return id;
}
const std::string& DOMTaskFormat::idEntityId(){
    static const std::string id("Id");
    return id;
}
const std::string& DOMTaskFormat::idTargetIdAttr(){
    static const std::string id("id");
    return id;
}
const std::string& DOMTaskFormat::idMotionTypeAttr(){
    static const std::string id("type");
    return id;
}
const std::string& DOMTaskFormat::idMotionStart(){
    static const std::string id("Start");
    return id;
}
const std::string& DOMTaskFormat::idMotionMid(){
    static const std::string id("Mid");
    return id;
}
const std::string& DOMTaskFormat::idMotionEnd(){
    static const std::string id("End");
    return id;
}
const std::string& DOMTaskFormat::idLinearMotion(){
    static const std::string id("Linear");
    return id;
}
const std::string& DOMTaskFormat::idP2PMotion(){
    static const std::string id("P2P");
    return id;
}
const std::string& DOMTaskFormat::idCircularMotion(){
    static const std::string id("Circular");
    return id;
}
const std::string& DOMTaskFormat::idActionTypeAttr(){
    static const std::string id("type");
    return id;
}
