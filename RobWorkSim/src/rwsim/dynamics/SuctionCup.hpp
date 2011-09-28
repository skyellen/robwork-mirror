
#ifndef RWSIM_DYNAMICS_SUCTIONCUP_HPP_
#define RWSIM_DYNAMICS_SUCTIONCUP_HPP_


#include <rw/kinematics/MovableFrame.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include "DynamicDevice.hpp"

namespace rwsim {
namespace dynamics {
    /**
     * @brief a suction cup with circular end effector.
     *
     * This is a complex device. That use a CompositeBody to model a semi-elastic
     * geometry which is a composite of several rigid parts. It use a sensor for
     * detecting contacting state with surface of another body and a controller for
     * controlling the forces acting due to the suction.
     *
     *
     *
     */
    class SuctionCup : public DynamicDevice {
    public:

    public:

        typedef rw::common::Ptr<SuctionCup> Ptr;

        SuctionCup(const std::string& name,
                   rwsim::dynamics::Body* base,
                   rwsim::dynamics::RigidBody* end,
                   const rw::math::Transform3D<>& bTb2,
                   double radi,
                   double height,
                   rw::math::Q springConstant1,
                   rw::math::Q springConstant2);

        virtual ~SuctionCup();

        rwsim::dynamics::Body* getBaseBody(){ return _baseBody; }

        rwsim::dynamics::Body* getEndBody(){ return _endBody; };

        void addToWorkCell(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

        double getRadius(){ return _radius; }

        double getHeight(){ return _height; }

        rw::math::Q getSpringParamsOpen(){ return _springConstant1; }

        rw::math::Q getSpringParamsClosed(){ return _springConstant2; }

        rw::math::Q getVelocity(const rw::kinematics::State& state){
            return rw::math::Q::zero(6);
        }

        void setVelocity(const rw::math::Q &vel, const rw::kinematics::State& state){

        }

        void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state){

        }

        rw::math::Transform3D<> getOffset(){ return _bTb2; }
    private:
        rw::models::Device *_kindev;
        rwsim::dynamics::Body *_baseBody;
        rwsim::dynamics::RigidBody *_endBody;
        double _radius, _height;
        rw::math::Q _springConstant1, _springConstant2;
        rw::math::Transform3D<> _bTb2;


    };
}
}

#endif /* SUCTIONCUP_HPP_ */
