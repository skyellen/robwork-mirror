
#ifndef RWSIM_DYNAMICS_SUCTIONCUP_HPP_
#define RWSIM_DYNAMICS_SUCTIONCUP_HPP_

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include "DynamicDevice.hpp"

namespace rwsim {
namespace dynamics {
	class DynamicWorkCell;

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
        //! smart pointer type of SuctionCup
        typedef rw::common::Ptr<SuctionCup> Ptr;

        SuctionCup(const std::string& name,
                   rwsim::dynamics::Body::Ptr base,
                   rwsim::dynamics::RigidBody::Ptr end,
                   const rw::math::Transform3D<>& bTb2,
                   double radi,
                   double height,
                   rw::math::Q springConstant1,
                   rw::math::Q springConstant2);

        virtual ~SuctionCup();

        rwsim::dynamics::Body::Ptr getBaseBody(){ return _baseBody; }

        rwsim::dynamics::Body::Ptr getEndBody(){ return _endBody; };

        void addToWorkCell(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc);

        double getRadius(){ return _radius; }

        double getHeight(){ return _height; }

        rw::math::Q getSpringParamsOpen(){ return _springConstant1; }

        rw::math::Q getSpringParamsClosed(){ return _springConstant2; }

        rw::math::Q getJointVelocities(const rw::kinematics::State& state){
            return rw::math::Q::zero(6);
        }

        void setJointVelocities(const rw::math::Q &vel, rw::kinematics::State& state){ }

        void addForceTorque(const rw::math::Q &forceTorque, rw::kinematics::State& state){
        }

        rw::math::Transform3D<> getOffset(){ return _bTb2; }

        const std::vector<rwsim::dynamics::Body::Ptr>& getLinks(){
            return _links;
        }

        bool isClosed(const rw::kinematics::State& state){ return _closedState; }
        void setClosed(bool closed, rw::kinematics::State& state){ _closedState = closed; }

        rwsim::dynamics::Body::Ptr getContactBody(const rw::kinematics::State& state){ return _contactBodyState; }
        void setContactBody(rwsim::dynamics::Body::Ptr b, rw::kinematics::State& state){ _contactBodyState = b; }

        double getPressure(const rw::kinematics::State& state){ return _kindev->getQ(state)[0]; }
        void setPressure(double pressure, rw::kinematics::State& state){ return _kindev->setQ(rw::math::Q(1,pressure), state); }

    private:
        rw::models::Device::Ptr _kindev;
        rwsim::dynamics::Body::Ptr _baseBody;
        rwsim::dynamics::RigidBody::Ptr _endBody;
        std::vector<rwsim::dynamics::Body::Ptr> _links;
        double _radius, _height;
        rw::math::Transform3D<> _bTb2;
        rw::math::Q _springConstant1, _springConstant2;

        // TODO: variables that should be put into the state
        bool _closedState;
        rwsim::dynamics::Body::Ptr _contactBodyState;
    };
}
}

#endif /* SUCTIONCUP_HPP_ */
