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

#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/models/Object.hpp>

#include <rw/math/InertiaMatrix.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VelocityScrew6D.hpp>

//#include <boost/foreach.hpp>
#include <rw/geometry/Geometry.hpp>

#include <rw/kinematics/Stateless.hpp>

namespace rw { namespace kinematics { class State; } }

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{

	//! @brief Body info holds the values that determine the dynamic behavior of the body, such as mass and inertia.
    struct BodyInfo {
    public:
    	//! @brief Construct empty body info.
        BodyInfo():
        	material(""),
        	objectType(""),
        	mass(0),
        	masscenter(0,0,0)
        {};

        //! @brief The type of material of the body. This determines the frictional effects.
        std::string material;
        //! @brief The object type determines the behaviour when bodies collide.
        std::string objectType;
        //! @brief Mass of a body in kg.
        double mass;
        //! @brief The location of the center of mass, given relative to the body frame.
        rw::math::Vector3D<> masscenter;
        //! @brief The inertia of the body given in the in the body frame around the \b masscenter .
        rw::math::InertiaMatrix<> inertia;
        //! @brief Specify the type of integrator to use to integrate the motion - the effect is highly dependent on the PhysicsEngine used.
        std::string integratorType;
        //! @brief The objects that compose the body.
        std::vector<rw::models::Object::Ptr> objects;

        //! @brief Output body info to std output.
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

        /**
         * @brief Output body info to output stream.
         * @param ostr [out] the stream to output to.
         */
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
         * @brief Construct a new dynamic body.
         * @param dof [in] general information about the dynamic properties of this body.
         * @param object [in] information about the body reference frame and geometry.
         */
        Body(const BodyInfo& info, rw::models::Object::Ptr obj);


    public:
        //! @brief Smart pointer type for a Body.
        typedef rw::common::Ptr<Body> Ptr;

    	//! @brief Destructor.
    	virtual ~Body(){};

    	/**
    	 * @brief name of body which is the name of the BodyFrame
    	 * @return the name.
    	 */
    	const std::string& getName() const {
    	    return _bodyframe->getName();
    	}

    	/**
    	 * @brief Returns the frame that the bodies dynamic variables are described relative to.
    	 * @return pointer to the body reference frame.
    	 */
        rw::kinematics::Frame* getBodyFrame() const {
            return _obj->getBase();
        }

        /**
         * @brief Get all geometry associated with this body.
         * @param state [in] the current state (for deformable bodies).
         * @return a vector of all geometries for the body.
         */
        const std::vector<rw::geometry::Geometry::Ptr>& getGeometry(const rw::kinematics::State& state){
            return _obj->getGeometry(state);
        }

        /**
         * @brief Get all geometry associated with this body.
         * @return a vector of all geometries for the body.
         * @deprecated please use getGeometry(rw::kinematics::State&) instead.
         */
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
         * @return the body info.
         */
        const BodyInfo& getInfo() const {return _info;};

        /**
         * @brief retrieve body information
         * @return the body information.
         * @note changing this will not force a changed event.
         */
        BodyInfo& getInfo(){return _info;};

        /**
         * @brief Material identifier of this object.
         * @return the identifier.
         */
        const std::string& getMaterialID() const { return _info.material; };

        /**
         * @brief get the inertia matrix of this body. The inertia is described
         * around the center of mass and relative to the parent frame.
         * @return the inertia matrix.
         */
        const rw::math::InertiaMatrix<>& getInertia() const { return _info.inertia; };

        //! @brief Types of events a body can emit.
        typedef enum{
        	MassChangedEvent//!< If mass, inertia or center of mass of a body is changed.
        } BodyEventType;

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

        //! @brief Type for body events.
        typedef rw::common::Event<BodyChangedListener, BodyEventType> BodyChangedEvent;

        /**
         * @brief Returns StateChangeEvent needed for subscribing and firing the event.
         * @return Reference to the StateChangedEvent
         */
        BodyChangedEvent& changedEvent() { return _bodyChangedEvent; }
        //! @brief The event where listeners can be registered.
        BodyChangedEvent _bodyChangedEvent;

        /**
         * @brief Set the mass of the body.
         * @note a BodyChangedEvent is fired.
         * @param m [in] the new mass of the body.
         */
        void setMass(double m){
            _info.mass = m;
            _bodyChangedEvent.fire(MassChangedEvent);
        }

        /**
         * @brief Set the mass and inertia of the body.
         * @note a BodyChangedEvent is fired.
         * @param m [in] the new mass of the body.
         * @param inertia [in] the new inertia of the body.
         */
        void setMass(double m, const rw::math::InertiaMatrix<>& inertia){
            _info.mass = m;
            _info.inertia = inertia;
            _bodyChangedEvent.fire(MassChangedEvent);
        }

        /**
         * @brief Set the mass, inertia and center of mass of the body.
         * @note a BodyChangedEvent is fired.
         * @param m [in] the new mass of the body.
         * @param inertia [in] the new inertia of the body.
         * @param com [in] the new center of mass.
         */
        void setMass(double m, const rw::math::InertiaMatrix<>& inertia, const rw::math::Vector3D<>& com){
            _info.mass = m;
            _info.inertia = inertia;
            _info.masscenter = com;
            _bodyChangedEvent.fire(MassChangedEvent);
        }
        
        /**
         * @brief Replaces object belonging to the body.
         * 
         * Geometry is replaced.
         * @warning BodyInfo is not changed! This has to be done manually.
         * 
         * @todo Make BodyInfo adjust automatically.
         * @param obj [in] the new object.
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
         * @brief Calculates the total energy of the body.
         * @param state [in] the state with position and velocity of the body.
         * @param gravity [in] (optional) the gravity to use for calculation of potential energy
         *  - if not specified, no potential energy is calculated.
         * @param potZero [in] (optional) the zero point for calculation of potential energy.
         * @return the total energy of the body.
         */
        virtual double calcEnergy(const rw::kinematics::State& state,
        		const rw::math::Vector3D<>& gravity = rw::math::Vector3D<>::zero(),
				const rw::math::Vector3D<>& potZero = rw::math::Vector3D<>::zero()) const = 0;

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

        /**
         * @brief Get the parent frame of this body.
         * @param state [in] the state.
         * @return a pointer to the parent frame.
         */
        virtual rw::kinematics::Frame* getParentFrame(const rw::kinematics::State& state) const {
            return getBodyFrame()->getParent(state);
        }

        /**
         * @brief Sets the force described in world frame acting on
         * the center mass of this body.
         *
         * @note more efficient to use setForce()
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

        /**
         * @brief Add a torque to the body.
         * @param t [in] the torque to add, described in world frame.
         * @param state [in] the state giving the current pose of the object relative to the world frame.
         */
        virtual void addTorqueW(const rw::math::Vector3D<>& t, rw::kinematics::State& state){
            addTorque( inverse(rw::kinematics::Kinematics::worldTframe(getParentFrame(state),state)).R() * t , state);
        }

        /**
         * @brief returns torque described in world frame
         */
        virtual rw::math::Vector3D<> getTorqueW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe( getParentFrame(state) , state).R() * getTorque(state);
        }

        /**
         * @brief Get the current position and orientation of the body frame relative to world frame.
         * @param state [in] the state giving the current pose.
         * @return the pose of the body frame relative to the world frame.
         */
        virtual rw::math::Transform3D<> getTransformW(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe( _bodyframe , state);
        }

        /**
         * @brief Get the current position and orientation of the body frame relative to the body parent frame.
         * @param state [in] the state giving the current pose.
         * @return the pose of the body frame relative to the body parent frame.
         */
        rw::math::Transform3D<> pTbf(const rw::kinematics::State& state) const {
            return _bodyframe->getTransform(state);
        }

        /**
         * @brief Get the current position of the body center of mass relative to the body parent frame.
         * @param state [in] the state giving the current pose.
         * @return the position of the body center of mass relative to the body parent frame.
         */
        rw::math::Transform3D<> pTcom(const rw::kinematics::State& state) const {
            rw::math::Transform3D<> t3d = _bodyframe->getTransform(state);
            t3d.P() += _info.masscenter;
            return t3d;
        }

        /**
         * @copydoc getTransformW
         * @note This is equivalent to getTransformW()
         */
        rw::math::Transform3D<> wTbf(const rw::kinematics::State& state) const {
            return rw::kinematics::Kinematics::worldTframe( _bodyframe , state);
        }

        /**
         * @brief Get the current position of the body center of mass relative to the world frame.
         * @param state [in] the state giving the current pose.
         * @return the position of the body center of mass relative to the world frame.
         */
        rw::math::Transform3D<> wTcom(const rw::kinematics::State& state) const {
            rw::math::Transform3D<> t3d = rw::kinematics::Kinematics::worldTframe( _bodyframe , state);
            return rw::math::Transform3D<>( t3d.P() + t3d.R()*_info.masscenter , t3d.R() );
        }

        /**
         * @brief Get the geometry information for the body.
         * @return the object.
         */
        rw::models::Object::Ptr getObject() const { return _obj; };

    private:
        rw::kinematics::Frame *_bodyframe;
        std::vector<rw::kinematics::Frame*> _frames;


        rw::models::Object::Ptr _obj;
    protected:
        //! @brief The dynamic parameters for the body.
    	BodyInfo _info;

    };
    //! @}
}
}

#endif /*BODY_HPP_*/
