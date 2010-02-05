#ifndef DYNAMICS_BODY_HPP_
#define DYNAMICS_BODY_HPP_

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>

#include <rw/math/InertiaMatrix.hpp>
#include <rw/math/Vector3D.hpp>

#include <boost/foreach.hpp>
#include <sandbox/geometry/Geometry.hpp>
//#include "Geometry.hpp"

namespace dynamics {

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
        std::vector<rw::kinematics::Frame*> frames;

        void print(){
        	std::cout << "Material: " << material << "\n";
        	std::cout << "Mass    : " << mass << "\n";
        	std::cout << "Frames: \n";
        	std::cout << "- " << frames[0]->getName() << "\n";
        	BOOST_FOREACH(rw::kinematics::Frame* frame, frames){
        		std::cout <<"-- "<< frame->getName() << "\n";
        	}
        	std::cout << std::endl;
        }

        void print(std::ostream& ostr){
        	ostr << "Material: " << material << "\n";
        	ostr << "Mass    : " << mass << "\n";
        	ostr << "Frames: \n";
        	ostr << "- " << frames[0]->getName() << "\n";
        	BOOST_FOREACH(rw::kinematics::Frame* frame, frames){
        		ostr <<"-- "<< frame->getName() << "\n";
        	}
        	ostr << std::endl;
        }
    };

	/**
	 * The body interface describe the basic interface of some physical entity
	 * in the virtual world.
	 *
	 * The body interface is used to add impulses, calculate basic velocity
	 * stuff and saving/updating the velocity and position states.
	 *
	 */
    class Body
    {
    protected:
        Body(const BodyInfo& info,
        	 rw::kinematics::Frame *bodyframe,
             const std::vector<rw::geometry::sandbox::Geometry*>& geometry):
                _bodyframe(*bodyframe),
                _geometry(geometry),
                _info(info)
        {
           /*BOOST_FOREACH(Geometry *geom, geometry){
               BOOST_FOREACH(rw::kinematics::Frame *frame, geom->frames){
                   _frames.push_back(frame);
               }
           }*/
        };

    public:
    	/**
    	 * @brief destructor
    	 */
    	virtual ~Body(){};

    	/**
    	 * @brief gets the frame that the bodies dynamic variables
    	 * are described relative to.
    	 */
        rw::kinematics::Frame& getBodyFrame() const {
            return _bodyframe;
        }

        const std::vector<rw::geometry::sandbox::Geometry*>& getGeometry(){
            return _geometry;
        }

        //void setAssociatedFrames(const std::vector<rw::kinematics::Frame*>& frames){
        //
        //}

        /**
         * @brief saves the current state in the rollback variables.
         */
        virtual void saveState(double h, rw::kinematics::State& state)= 0;

        /**
         * @brief rolls back to the last saved state
         */
        virtual void rollBack(rw::kinematics::State& state)= 0;

        /**
         * @brief integrates forces over timestep h to update the velocity of the body
         */
        virtual void updateVelocity(double h, rw::kinematics::State& state)= 0;

        /**
         * @brief integrates velocity over timestep h to update the position of the body
         */
        virtual void updatePosition(double h, rw::kinematics::State& state) = 0;

        /**
         * @brief updates the velocity of this body with the accumulated linear impulse
         * and angular impulse
         */
        virtual void updateImpulse() = 0;

        /**
         * @brief calculates the relative velocity of a point p on the body
         * described in world frames.
         */
        virtual rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p) = 0;

        /**
         * @brief calculates the effective mass of this object
         * seen from some contact point wPc
         * described in world coordinates.
         * @param wPc [in] contact point position
         * @return effective mass of object in the contact point
         */
        virtual rw::math::InertiaMatrix<> getEffectiveMassW(const rw::math::Vector3D<>& wPc) = 0;

        /**
         * @brief gets all frames that is staticly connected to this Body
         * @return list of frames
         */
        const std::vector<rw::kinematics::Frame*>& getFrames(){
            return _info.frames;
        }

        /**
         *
         */
        virtual void reset() = 0;

        virtual void resetState(rw::kinematics::State &state) = 0;

        /**
         * @brief gets the material identifier for this body.
         * @return string material identifier
         */
        virtual const std::string& getMaterial() = 0;

        /**
         * @brief this is called to precalculate all auxiliary
         * variables.
         */
        virtual void calcAuxVarialbles(rw::kinematics::State& state) = 0;

        /**
         * @brief calculates and returns the total energy of this body
         * @return
         */
        virtual double calcEnergy() = 0;

        /**
         * @brief get the body info
         * @return
         */
        const BodyInfo& getInfo() const {return _info;};
        BodyInfo& getInfo(){return _info;};
    private:
        rw::kinematics::Frame &_bodyframe;

        std::vector<rw::geometry::sandbox::Geometry*> _geometry;
    protected:
    	BodyInfo _info;

    };

}

#endif /*BODY_HPP_*/
