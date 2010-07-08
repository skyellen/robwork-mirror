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

//! @file.hpp

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>

#include <rw/math/InertiaMatrix.hpp>
#include <rw/math/Vector3D.hpp>

#include <boost/foreach.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/kinematics/StateData.hpp>

namespace rwsim {
namespace dynamics {
	//! @addtogroup dynamics @{

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
	 * @brief The body interface describe the basic interface of some physical entity
	 * in the virtual world.
	 *
	 * The body interface is used to add impulses, calculate basic velocity
	 * stuff and saving/updating the velocity and position states.
	 *
	 */
    class Body: public rw::kinematics::StateData
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
        Body(
        	 int dof,
             const BodyInfo& info,
        	 rw::kinematics::Frame *bodyframe,
             const std::vector<rw::geometry::GeometryPtr>& geometry):
            	 rw::kinematics::StateData(dof, bodyframe->getName()),
                _bodyframe(*bodyframe),
                _geometry(geometry),
                _info(info)
        {

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

        /**
         * @brief get all geometry associated with this body
         */
        const std::vector<rw::geometry::GeometryPtr>& getGeometry(){
            return _geometry;
        }

        /**
         * @brief calculates the relative velocity of a point p on the body
         * described in world frames.
         */
        virtual rw::math::Vector3D<>
        	getPointVelW(const rw::math::Vector3D<>& p, const rw::kinematics::State& state) const = 0;

        /**
         * @brief gets all frames that is staticly connected to this Body
         * @return list of frames
         */
        const std::vector<rw::kinematics::Frame*>& getFrames(){
            return _info.frames;
        }

        /**
         * @brief reset the state variables of this body
         */
        virtual void resetState(rw::kinematics::State &state) = 0;

        /**
         * @brief calculates and returns the total energy of this body
         * @return
         */
        virtual double calcEnergy(const rw::kinematics::State& state) = 0;

        /**
         * @brief get the body info
         * @return
         */
        const BodyInfo& getInfo() const {return _info;};

        /**
         * @brief retrieve body information
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


    private:
        rw::kinematics::Frame &_bodyframe;

        std::vector<rw::geometry::GeometryPtr> _geometry;
    protected:
    	BodyInfo _info;

    };
    //! @}
}
}

#endif /*BODY_HPP_*/
