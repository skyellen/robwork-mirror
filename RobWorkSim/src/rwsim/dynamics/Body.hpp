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

#ifndef RWSIM_DYNAMICS_BODY_HPP_
#define RWSIM_DYNAMICS_BODY_HPP_

//! @file Body.hpp

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Object.hpp>

#include <rw/math/InertiaMatrix.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

#include <boost/foreach.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/kinematics/StateData.hpp>
#include <rw/kinematics/Stateless.hpp>

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{

    struct BodyInfo {
    public:
        BodyInfo():
        	material(""),
        	objectType(""),
        	mass(0),
        	masscenter(0,0,0)
        {};

        std::string material;
        std::string objectType;
        double mass;
        rw::math::Vector3D<> masscenter;
        rw::math::InertiaMatrix<> inertia;
        std::string integratorType;

        std::vector<rw::models::Object::Ptr> objects;

        void print() const{
        	std::cout << "Material: " << material << "\n";
        	std::cout << "Mass    : " << mass << "\n";
        	//std::cout << "Frames: \n";
        	//RW_ASSERT(frames.size()>0);
        	//std::cout << "- " << frames[0]->getName() << "\n";
        	//BOOST_FOREACH(rw::kinematics::Frame* frame, frames){
        	//	std::cout <<"-- "<< frame->getName() << "\n";
        	//}
        	std::cout << std::endl;
        }

        void print(std::ostream& ostr) const{
        	ostr << "Material: " << material << "\n";
        	ostr << "Mass    : " << mass << "\n";
        	//ostr << "Frames: \n";
        	//RW_ASSERT(frames.size()>0);
        	//ostr << "- " << frames[0]->getName() << "\n";
        	//BOOST_FOREACH(rw::kinematics::Frame* frame, frames){
        	//	ostr <<"-- "<< frame->getName() << "\n";
        	//}
        	ostr << std::endl;
        }
    };

	/**
	 * @brief The body interface describe the basic interface of some physical entity
	 * in the virtual world. That is as a minimum the body has a geometric description,
	 * and a material identity.
	 *
	 * The body interface is used to add impulses, calculate basic velocity
	 * stuff and saving/updating the velocity and position states.
	 */
    class Body: public rw::kinematics::Stateless
    {
    protected:
        /**
         * @brief A Body with \b ssize of room in state vector
         *
         * \b ssize must be non-negative.
         *
         * A newly created body can be added to the StateStructure
         *
         * The size allocate in the state vector is constant throughout
         * the lifetime of the body.
         *
         * @param dof [in] The number of degrees of freedom of the frame.
         * @param info [in] general information of this body
         * @param bodyframe [in]
         */
        Body(const BodyInfo& info, rw::models::Object::Ptr obj);


    public:

        typedef rw::common::Ptr<Body> Ptr;

    	/**
    	 * @brief destructor
    	 */
    	virtual ~Body(){};

    	/**
    	 * @brief name of body which is the name of the BodyFrame
    	 * @return
    	 */
    	const std::string& getName() const {
    	    return _bodyframe->getName();
    	}

    	/**
    	 * @brief gets the frame that the bodies dynamic variables
    	 * are described relative to.
    	 */
        rw::kinematics::Frame* getBodyFrame() const {
            return _obj->getBase();
        }

        /**
         * @brief get all geometry associated with this body
         */
        const std::vector<rw::geometry::Geometry::Ptr>& getGeometry(const rw::kinematics::State& state){
            return _obj->getGeometry(state);
        }

        const std::vector<rw::geometry::Geometry::Ptr>& getGeometry(){
        	return getGeometry( _obj->getStateStructure()->getDefaultState() );
        }

        /**
         * @brief gets all frames that is staticly connected to this Body
         * @return list of frames
         */
        const std::vector<rw::kinematics::Frame*>& getFrames(){
            return _frames;
        }

        /**
         * @brief get the body info
         * @return
         */
        const BodyInfo& getInfo() const {return _info;};

        /**
         * @brief retrieve body information
         *
         * NOTICE: changing this will not force a changed event.
         */
        BodyInfo& getInfo(){return _info;};

        /**
         * @brief matrial identifier of this object
         */
        const std::string& getMaterialID() const { return _info.material; };

        /**
         * @brief get the inertia matrix of this body. The inertia is described
         * around the center of mass and relative to the parent frame.
         */
        const rw::math::InertiaMatrix<>& getInertia() const { return _info.inertia; };

        typedef enum{MassChangedEvent} BodyEventType;
        /**
         * @brief Defines a state changed listener.
         *
         * Listeners to this event is called when a change of the state occurs.
         *
         * StateChangedListener describes the signature of a callback method.
         *
         * Example usage in a plugin:
         * \code
         * void MyPlugin::initialize()
         * {
         *     getRobWorkStudio()->stateChangedEvent().add(
         *         boost::bind(&MyPlugin::stateChangedListener, this, _1), this);
         * }
         *
         * void MyPlugin::stateChangedListener(const State& state)
         * {
         * ...
         * }
         * \endcode
         */
        typedef boost::function<void(BodyEventType)> BodyChangedListener;
        typedef rw::common::Event<BodyChangedListener, BodyEventType> BodyChangedEvent;

        /**
         * @brief Returns StateChangeEvent needed for subscribing and firing the event.
         * @return Reference to the StateChangedEvent
         */
        BodyChangedEvent& changedEvent() { return _bodyChangedEvent; }
        BodyChangedEvent _bodyChangedEvent;

        void setMass(double m){
            _info.mass = m;
            _bodyChangedEvent.fire(MassChangedEvent);
        }

        void setMass(double m, const rw::math::InertiaMatrix<>& inertia){
            _info.mass = m;
            _info.inertia = inertia;
            _bodyChangedEvent.fire(MassChangedEvent);
        }

        void setMass(double m, const rw::math::InertiaMatrix<>& inertia, const rw::math::Vector3D<>& com){
            _info.mass = m;
            _info.inertia = inertia;
            _info.masscenter = com;
            _bodyChangedEvent.fire(MassChangedEvent);
        }
        
        /**
         * @brief Replaces object belonging to the body.
         * 
         * Geometry is replaced. WARNING: BodyInfo is not changed! This has to be done manually.
         * 
         * @todo Make BodyInfo adjust automatically.
         */
        void setObject(rw::models::Object::Ptr obj);

        //--------------------------- interface functions ------------------
        /**
         * @brief calculates the relative velocity of a point p on the body
         * described in world frames.
         */
        virtual rw::math::Vector3D<>
        	getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const;

        /**
         * @brief gets the velocity of this body relative to the parent frame
         * @param state
         * @return
         */
        virtual rw::math::VelocityScrew6D<> getVelocity(const rw::kinematics::State &state) const = 0;

        /**
         * @brief returns the linear velocity described in parent frame
         */
        virtual rw::math::Vector3D<> getLinVel(const rw::kinematics::State& state) const{
            return rw::math::Vector3D<>(0,0,0);
        }

        /**
         * @brief returns the angular velocity described in parent frame
         */
        virtual rw::math::Vector3D<> getAngVel(const rw::kinematics::State& state) const{
            return rw::math::Vector3D<>(0,0,0);
        }

        /**
         * @brief returns the linear velocity described in world frame
         */
        rw::math::Vector3D<> getLinVelW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state).R() *getLinVel(state);
        }

        /**
         * @brief returns the angular velocity described in world frame
         */
        rw::math::Vector3D<> getAngVelW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state).R() *getAngVel(state);
        }




        /**
         * @brief reset the state variables of this body
         */
        virtual void reset(rw::kinematics::State &state) = 0;

        /**
         * @brief calculates and returns the total energy of this body
         * @return
         */
        virtual double calcEnergy(const rw::kinematics::State& state) = 0;

        /**
         * @brief Sets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual void setForce(const rw::math::Vector3D<>& f, rw::kinematics::State& state) = 0;

        /**
         * @brief Gets the force described in parent frame acting on
         * the center mass of this body.
         */
        virtual rw::math::Vector3D<> getForce(const rw::kinematics::State& state) const = 0;

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addForce(const rw::math::Vector3D<>& force, rw::kinematics::State& state) = 0;

        /**
         * @brief set the torque of this body with torque t, where t is
         * described in body frame
         */
        virtual void setTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state) = 0;

        /**
         * @brief Adds a force described in parent frame to the
         * center of mass of this body
         */
        virtual void addTorque(const rw::math::Vector3D<>& t, rw::kinematics::State& state) = 0;

        /**
         * @brief returns torque described in body frame
         */
        virtual rw::math::Vector3D<> getTorque(const rw::kinematics::State& state) const = 0;



        virtual rw::kinematics::Frame* getParentFrame(const rw::kinematics::State& state) const {
            return getBodyFrame()->getParent(state);
        }

        /**
         * @brief Sets the force described in world frame acting on
         * the center mass of this body.
         *
         * @note more efficient to ude setForce()
         */
        virtual void setForceW(const rw::math::Vector3D<>& f, rw::kinematics::State& state){
            setForce( rw::math::inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state)).R() * f, state);
        }

        /**
         * @brief Gets the force described in world frame acting on
         * the center mass of this body.
         */
        virtual rw::math::Vector3D<> getForceW(const rw::kinematics::State& state) const{
            return rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state).R() * getForce(state);
            //return rw::kinematics::Kinematics::worldTframe(_bodyframe, state).R() * getForce(state);
        }

        /**
         * @brief Adds a force described in world frame to the
         * center of mass of this body
         */
        virtual void addForceW(const rw::math::Vector3D<>& force, rw::kinematics::State& state){
            addForce( rw::math::inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state), state)).R() * force, state );
        }

        /**
         * @brief Adds a force described in parent frame to this body
         * which is working on a specific position pos that is described relative to
         * this body.
         */
        void addForceToPos(const rw::math::Vector3D<>& force,
                                   const rw::math::Vector3D<>& pos,
                                   rw::kinematics::State& state){
            // calculate the center force contribution
            addForce(force, state);

            // calculate the torque contribution
            addTorque( cross( pos, force ) , state);
        }

        /**
         * @brief Adds a force described in world frame to this body
         * which is worked on a specific position pos that is described
         * relative to world
         */
        virtual void addForceWToPosW(const rw::math::Vector3D<>& force,
                                     const rw::math::Vector3D<>& pos,
                                     rw::kinematics::State& state)
        {
            using namespace rw::kinematics;
            rw::math::Transform3D<> pTw = inverse(Kinematics::worldTframe(getParentFrame(state), state));
            rw::math::Transform3D<> wTb = Kinematics::worldTframe(getBodyFrame(), state);

            // transform the force into body frame description
            rw::math::Vector3D<> forcebody = pTw.R() * force;
            // calculate the center force contribution
            addForce( forcebody, state);

            rw::math::Vector3D<> posOnBody = pTw.R() * (pos - wTb.P());
            // calculate the torque contribution
            addTorque( cross( posOnBody, forcebody ) , state);
        }


        /**
         * @brief set the torque of this body with torque t, where t is
         * described in world frame
         */
        virtual void setTorqueW(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            setTorque( inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state),state)).R() * t , state);
        }

        virtual void addTorqueW(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            addTorque( inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state),state)).R() * t , state);
        }

        /**
         * @brief returns torque described in world frame
         */
        virtual rw::math::Vector3D<> getTorqueW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe( getParentFrame(state) , state).R() * getTorque(state);
        }

        virtual rw::math::Transform3D<> getTransformW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe( _bodyframe , state);
        }

        rw::math::Transform3D<> pTbf(const rw::kinematics::State& state) const {
            return _bodyframe->getTransform(state);
        }

        rw::math::Transform3D<> pTcom(const rw::kinematics::State& state) const {
            rw::math::Transform3D<> t3d = _bodyframe->getTransform(state);
            t3d.P() += _info.masscenter;
            return t3d;
        }

        rw::math::Transform3D<> wTbf(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe( _bodyframe , state);
        }
        // world
        rw::math::Transform3D<> wTcom(const rw::kinematics::State& state) const {
            rw::math::Transform3D<> t3d = rw::kinematics::Kinematics::worldTframe( _bodyframe , state);
            return rw::math::Transform3D<>( t3d.P() + t3d.R()*_info.masscenter , t3d.R() );
        }

        rw::models::Object::Ptr getObject() const { return _obj; };

    private:
        rw::kinematics::Frame *_bodyframe;
        std::vector<rw::kinematics::Frame*> _frames;


        rw::models::Object::Ptr _obj;
    protected:
    	BodyInfo _info;

    };
    //! @}
}
}

#endif /*BODY_HPP_*/
