#include "SoftBeamObject.hpp"

#include <rw/common/macros.hpp>


using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::graphics;
using namespace rw::math;
using namespace rw::models;




rw::models::SoftBeamObject::SoftBeamObject(WorkCell* workcell, Frame* baseframe, const int nFrames):
    Object(baseframe),
    _wc(workcell),
    _nFrames(nFrames)
{
    rw::kinematics::StateStructure::Ptr tree = workcell->getStateStructure();

    for (int i = 0; i < _nFrames; i++) {
        std::ostringstream ss;
        ss << "softbeamframe" << i+1;

        MovableFrame *newFrame = new MovableFrame( ss.str() );
        addFrame(newFrame);

        // add to workcell
        tree->addFrame(newFrame, getBase());

        // make sure to retrieve the newly changed workcell state
        _state = tree->getDefaultState();
    }
}



rw::models::SoftBeamObject::~SoftBeamObject()
{
    // TODO delete frames in vector (need to do this on Object.hpp
}



void SoftBeamObject::setConstraints(const std::vector< SoftBeamObject::Constraint >& constraints)
{

}



void SoftBeamObject::update()
{
    const double hx = 0.1;

    for (int i = 0; i < _nFrames; i++) {
        MovableFrame *frame = static_cast<MovableFrame *> ( (getFrames()[i]) );

        Transform3D<> trans = Transform3D<>::identity();
        trans.P()[0] = i * hx + 0.1;

        frame->setTransform(trans, _state);
    }
}



State SoftBeamObject::getState(void )
{
    return _state;
}





