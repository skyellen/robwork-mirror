#include "GraspMotorProgram.hpp"

#include <bstrap/core/BrainState.hpp>

#include <rwsim/control/PoseController.hpp>

using rw::common::PropertyMap;
using rw::math::Transform3D;

GraspMotorProgram::GraspMotorProgram(const std::string& name,
                                     rwsim::control::PoseController::Ptr graspController):
                                     MotorProgram(name)
{

}

void GraspMotorProgram::update(const BrainState& bstate){

    const PropertyMap &visualObjects = bstate.getMap().get<PropertyMap>("VisualObjects", PropertyMap());
    // get the object, if its there...
    if( visualObjects.has(_objName) == false ){
        //TODO: perhaps signal that parameters are wrong
        return;
    }
    const PropertyMap &object = visualObjects.get<PropertyMap>(_objName);
    Transform3D<> t3d = object.get<Transform3D<> >("transform", Transform3D<>::identity());
    _graspController->setTarget( t3d );

}

void GraspMotorProgram::setParameters(rw::common::Ptr<rw::common::PropertyMap> parameters, const BrainState& bstate){
    // PropertyMap input;
    // input.set<std::string>("objectName","object");
    // input.set<Transform3D<> >("objectTransform", Transform3D<>() );

    // extract parameters for grasping
    _objName = parameters->get<std::string>("objectName");

    const PropertyMap &visualObjects = bstate.getMap().get<PropertyMap>("VisualObjects", PropertyMap());
    // get the object, if its there...
    if( visualObjects.has(_objName) == false ){
        //TODO: perhaps signal that parameters are wrong
        return;
    }
    const PropertyMap &object = visualObjects.get<PropertyMap>(_objName);
    Transform3D<> t3d = object.get<Transform3D<> >("transform", Transform3D<>::identity());
    _graspController->setTarget( t3d );
}

void GraspMotorProgram::executionloop(){
    // do stuff in a loop
    // call stop() when finished

    // we check if the controller reached its destination


}
