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

#ifndef RW_MODELS_OBJECT_HPP_
#define RW_MODELS_OBJECT_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/Stateless.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/graphics/Model3D.hpp>
#include <rw/math/InertiaMatrix.hpp>

#include <vector>

namespace rw { namespace kinematics { class State; } }

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief The object class represents a physical thing in the scene which has geometry.
     * An object has a base frame (similar to a Device) and may have a number of associated frames.
     *
     */
    class Object: public rw::kinematics::Stateless
    {
    public:
    	//! smart pointer
        typedef rw::common::Ptr<Object> Ptr;
		//! const smart pointer
		typedef rw::common::Ptr<const Object> CPtr;

    protected:
        //! constructor
        Object(rw::kinematics::Frame* baseframe);
        //! constructor - first frame is base
        Object(std::vector<rw::kinematics::Frame*> frames);

    public:
        //! destructor
        virtual ~Object();

        /**
         * @brief get name of this object. Name is always the same as the name of the
         * base frame.
         * @return name of object.
         */
        const std::string& getName(){ return _base->getName(); };

        /**
         * @brief get base frame of this object
         * @return base frame of object
         */
        rw::kinematics::Frame* getBase();
        const rw::kinematics::Frame* getBase() const;

        /**
         * @brief get all associated frames of this object
         * @return a vector of frames
         */
        const std::vector<rw::kinematics::Frame*>& getFrames();

        /**
         * @brief associate a frame to this Object.
         * @param frame [in] frame to associate to object
         */
        void addFrame(rw::kinematics::Frame* frame);

        /**
         * @brief get default geometries
         * @return geometry for collision detection
         */
        const std::vector<rw::geometry::Geometry::Ptr>& getGeometry() const {
        	return doGetGeometry(this->getStateStructure()->getDefaultState());
        }

        /**
         * @brief get the default models
         * @return models for vizualization
         */
        const std::vector<rw::graphics::Model3D::Ptr>& getModels() const {
        	return doGetModels(this->getStateStructure()->getDefaultState());
        }

        // stuff that should be implemented by deriving classes
        /**
         * @brief get geometry of this object
         * @return geometry for collision detection.
         */
        const std::vector<rw::geometry::Geometry::Ptr>& getGeometry(const rw::kinematics::State& state) const{ return doGetGeometry(state); }

        /**
         * @brief get visualization models of this object
         * @return models for visualization
         */
        const std::vector<rw::graphics::Model3D::Ptr>& getModels(const rw::kinematics::State& state) const{ return doGetModels(state);}


	    /**
	     * @brief get mass in Kg of this object
	     * @return mass in kilo grams
	     */
	    virtual double getMass(rw::kinematics::State& state) const = 0;

	    /**
	     * @brief get center of mass of this object
	     * @param state [in] the state in which to get center of mass
	     * @return
	     */
	    virtual rw::math::Vector3D<> getCOM(rw::kinematics::State& state) const = 0;

	    /**
	     * @brief returns the inertia matrix of this body calculated around COM with the orientation
	     * of the base frame.
	     */
	    virtual rw::math::InertiaMatrix<> getInertia(rw::kinematics::State& state) const = 0;

    protected:
        friend class WorkCell;

        /**
         * @brief get geometry of this object
         * @return geometry for collision detection.
         */
        virtual const std::vector<rw::geometry::Geometry::Ptr>& doGetGeometry(const rw::kinematics::State& state) const = 0;

        /**
         * @brief get visualization models of this object
         * @return models for visualization
         */
        virtual const std::vector<rw::graphics::Model3D::Ptr>& doGetModels(const rw::kinematics::State& state) const = 0;


    private:
        rw::kinematics::Frame *_base;
        std::vector<rw::kinematics::Frame*> _frames;
        std::vector<rw::geometry::Geometry::Ptr> _geometry;
        std::vector<rw::graphics::Model3D::Ptr> _models;


    };

    /*@}*/
}}

#endif /* OBJECT_HPP_ */
