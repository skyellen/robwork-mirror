#include "SuctionCup.hpp"
#include "DynamicWorkCell.hpp"

#include <rw/common/Ptr.hpp>

#include <string>

using namespace rwsim::dynamics;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rw::common;

namespace {

    class SuctionCupDevice: public Device {
    public:
        SuctionCupDevice(std::string name, Frame* base, Frame* end):
            Device(name),
            _base(base),
            _end(end),
            _pressure(1,1.0)
        {
            _bounds = QBox(Q(1,0.0),Q(1,1.0));
        }

        Frame* getBase(){ return _base;}

        const Frame* getBase() const{ return _base; }

        Frame* getEnd(){ return _end; }

        const Frame* getEnd() const{ return _end; }

        void setQ(const Q& q, State& state) const { _pressure = q;}
        Q getQ(const State& state) const { return _pressure; }
        QBox getBounds() const{ return _bounds; };
        void setBounds(const Device::QBox& bounds){ _bounds = bounds; };
        Q getVelocityLimits() const{ return Q::zero(1); }
        void setVelocityLimits(const Q& vellimits){  };
        Q getAccelerationLimits() const{return Q::zero(1);}
        void setAccelerationLimits(const Q& acclimits){ }
        size_t getDOF() const {return 1;};
        Jacobian baseJend(const State& state) const{ return Jacobian(6,6);};
        JacobianCalculatorPtr baseJCframes(const std::vector<Frame*>& frames, const State& state) const{
            return NULL;
        };

        QBox _bounds;
        Frame *_base, *_end;
        mutable Q _pressure;
    };

}


SuctionCup::SuctionCup(const std::string& name, Body::Ptr baseBody, RigidBody::Ptr end,
                       const rw::math::Transform3D<>& bTb2,
                       double radi,
                       double height,
                       Q sc1,
                       Q sc2):
    DynamicDevice(baseBody, ownedPtr(new SuctionCupDevice(name, baseBody->getBodyFrame(), end->getBodyFrame()) )),
    _baseBody(baseBody),
    _endBody(end),
    _radius(radi),
    _height(height),
    _bTb2(bTb2),
    _springConstant1(sc1),
    _springConstant2(sc2),
    _closedState(false),
    _contactBodyState(NULL)
{
    _links.push_back(_baseBody);
    _links.push_back(_endBody);
    _kindev = getKinematicModel();
}

SuctionCup::~SuctionCup(){

}

void SuctionCup::addToWorkCell(DynamicWorkCell::Ptr dwc){
    //dwc->getWorkcell()->addDevice( getKinematicModel() );


    /*
    StateStructure::Ptr sstruct = dwc->getWorkcell()->getStateStructure();

    //sstruct->addFrame(getBodyFrame(), parent);
    //sstruct->addData(_baseBody);
    BOOST_FOREACH(Frame *frame, _frames){
        sstruct->addFrame(frame, getBodyFrame());
    }

    BOOST_FOREACH(Body *body, _bodies){
        sstruct->addData(body);
        dwc->addBody( body );
    }

    // now add contact sensing to the bodies
    BOOST_FOREACH(BodyContactSensor::Ptr sensor, _sensors){
        dwc->addSensor( sensor );
    }

    // update the default state such that the transforms of the movables is valid
    State state = sstruct->getDefaultState();
    for(size_t i=0;i<_frames.size();i++){
        _frames[i]->setTransform(_bodyTransforms[i], state);
    }

    sstruct->setDefaultState(state);
    */
}
