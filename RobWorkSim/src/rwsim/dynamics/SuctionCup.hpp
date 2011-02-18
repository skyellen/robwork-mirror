
#ifndef RWSIM_DYNAMICS_SUCTIONCUP_HPP_
#define RWSIM_DYNAMICS_SUCTIONCUP_HPP_


#include <rw/kinematics/MovableFrame.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

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
    class SuctionCup {
    public:

    public:

        typedef rw::common::Ptr<SuctionCup> Ptr;

        SuctionCup(rwsim::dynamics::Body* base,
                   const rw::math::Transform3D<>& bTb2,
                   double radi,
                   double height,
                   double elasticity);

        virtual ~SuctionCup(){};

        rwsim::dynamics::Body* getBodyPart();

        rw::kinematics::Frame* getBodyFrame();

        std::vector<rwsim::dynamics::RigidBody*>& getBodyParts();

        std::vector<rwsim::sensor::BodyContactSensor::Ptr>& getBodySensors(){ return _sensors; }

        std::vector<rw::kinematics::MovableFrame*>& getFrameParts();

        void addToWorkCell(rwsim::dynamics::DynamicWorkCell::Ptr dwc);

        double getRadius(){ return _radius; }

        double getHeight(){ return _height; }

        double getElasticity(){ return _elasticity; }

    private:

        rwsim::dynamics::Body* _baseBody;
        double _radius, _height, _elasticity;

        std::vector<rw::kinematics::MovableFrame*> _frames;
        std::vector<rwsim::dynamics::RigidBody*> _bodies;
        std::vector<rwsim::sensor::BodyContactSensor::Ptr> _sensors;
        std::vector<rw::math::Transform3D<> > _bodyTransforms;
        rw::math::Transform3D<> _bTb2;


    };
}
}

#endif /* SUCTIONCUP_HPP_ */
