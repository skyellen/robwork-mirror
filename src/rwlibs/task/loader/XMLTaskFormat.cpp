/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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




