#include "VisionSensor.hpp"

#include <rw/kinematics/Kinematics.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;

using namespace rwsim::dynamics;

VisionSensor::VisionSensor(rw::kinematics::Frame *visionFrame,
                 rwsim::simulator::ThreadSimulator::Ptr sim,
                 rwsim::simulator::DynamicSimulator::Ptr dsim,
                 rwsim::dynamics::DynamicWorkCell::Ptr dwc):
                 _visionFrame(visionFrame),_sim(sim),_dsim(dsim),_dwc(dwc)
{

}


void VisionSensor::update(BrainState& cstate, Memory& mem){

    // for now we see all objects that are in front of the visionFrame (z-axis) ;). this can be improved later on

    // we use a VisualObjects property
    PropertyMap &objectsMap = cstate.getMap().add("VisualObjects", "A list of visual objects", PropertyMap())->getValue();

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
