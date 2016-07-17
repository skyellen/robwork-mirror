#include "VisionSensor.hpp"

#include <bstrap/core/BrainState.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

using rw::math::Transform3D;
using rw::common::PropertyMap;
using rw::kinematics::Kinematics;
using rwsim::dynamics::Body;

VisionSensor::VisionSensor(rw::kinematics::Frame *visionFrame,
		rwsim::dynamics::DynamicWorkCell::Ptr dwc):
	_dwc(dwc),_visionFrame(visionFrame)
{
}


void VisionSensor::update(BrainState& cstate, Memory& mem){

    // for now we see all objects that are in front of the visionFrame (z-axis) ;). this can be improved later on

    // we use a VisualObjects property
    //PropertyMap &objectsMap = cstate.getMap().add("VisualObjects", "A list of visual objects", PropertyMap())->getValue();

    Transform3D<> wTvision = Kinematics::worldTframe(_visionFrame, cstate.getRobWorkState() );
    BOOST_FOREACH(Body::Ptr b, _dwc->getBodies() ){
        Transform3D<> wTobj = b->getTransformW( cstate.getRobWorkState() );

        // transform wTobj to vision coordinates, and only record objects that are in the positive z-coordinate
        Transform3D<> visionTobj = inverse(wTvision) * wTobj;
        if( visionTobj.P()[2]<0 )
            continue;

        PropertyMap &objMap = cstate.getMap().add(b->getName(), "An object", PropertyMap())->getValue();
        objMap.set("transform", wTobj);

    }

}
