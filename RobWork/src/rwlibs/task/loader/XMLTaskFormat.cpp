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


#include "XMLTaskFormat.hpp"

using namespace rwlibs::task;
using namespace rw::loaders;
using namespace xercesc;

//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
const XercesInitializer XMLTaskFormat::initializer;

const XMLCh* XMLTaskFormat::QTaskId = XMLString::transcode("QTask");

const XMLCh* XMLTaskFormat::CartesianTaskId = XMLString::transcode("CartesianTask");

const XMLCh* XMLTaskFormat::TargetsId = XMLString::transcode("Targets");

const XMLCh* XMLTaskFormat::EntitiesId  = XMLString::transcode("Entities");

const XMLCh* XMLTaskFormat::AugmentationsId  = XMLString::transcode("Augmentations");


const XMLCh* XMLTaskFormat::QTargetId = XMLString::transcode("QTarget");

const XMLCh* XMLTaskFormat::CartesianTargetId = XMLString::transcode("CartesianTarget");


const XMLCh* XMLTaskFormat::MotionId = XMLString::transcode("Motion");
const XMLCh* XMLTaskFormat::ActionId = XMLString::transcode("Action");


const XMLCh* XMLTaskFormat::EntityIndexId = XMLString::transcode("Index");
const XMLCh* XMLTaskFormat::EntityIdId = XMLString::transcode("Id");
const XMLCh* XMLTaskFormat::TargetIdAttrId = XMLString::transcode("id");
const XMLCh* XMLTaskFormat::ActionTypeAttrId = XMLString::transcode("type");

const XMLCh* XMLTaskFormat::MotionTypeAttrId = XMLString::transcode("type");



const XMLCh* XMLTaskFormat::MotionStartId = XMLString::transcode("Start");
const XMLCh* XMLTaskFormat::MotionMidId = XMLString::transcode("Mid");
const XMLCh* XMLTaskFormat::MotionEndId = XMLString::transcode("End");

const XMLCh* XMLTaskFormat::LinearMotionId = XMLString::transcode("Linear");
const XMLCh* XMLTaskFormat::P2PMotionId = XMLString::transcode("P2P");
const XMLCh* XMLTaskFormat::CircularMotionId = XMLString::transcode("Circular");




