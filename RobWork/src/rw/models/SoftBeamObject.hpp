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

#ifndef RW_MODELS_SOFTBEAMOBJECT_HPP_
#define RW_MODELS_SOFTBEAMOBJECT_HPP_

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/graphics/Model3D.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include "Object.hpp"

#include <vector>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief An object is an element in the scene with a geometric representation. The Object
       has a base frame and may have additional frames attached.
    */
    class SoftBeamObject: public Object
    {
    public:
        typedef rw::common::Ptr<SoftBeamObject> Ptr;

	
        SoftBeamObject(rw::models::WorkCell *workcell, rw::kinematics::Frame* baseframe, const int nFrames);
	/*
        SoftBeamObject(rw::kinematics::Frame* baseframe, rw::geometry::Geometry::Ptr geom);
        SoftBeamObject(rw::kinematics::Frame* baseframe, std::vector<rw::geometry::Geometry::Ptr> geom);

        SoftBeamObject(std::vector<rw::kinematics::Frame*> frames);
        SoftBeamObject(std::vector<rw::kinematics::Frame*> frames, rw::geometry::Geometry::Ptr geom);
        SoftBeamObject(std::vector<rw::kinematics::Frame*> frames, std::vector<rw::geometry::Geometry::Ptr> geom);
        */

        virtual ~SoftBeamObject();

        struct Constraint {
        	//
        };

        void setConstraints( const std::vector<Constraint>& constraints);
        void update();


    protected:
        friend class WorkCell;
        
        
    private:
      //std::vector< rw::math::Transform3D<>  >
      rw::models::WorkCell *_wc;
	int _nFrames;
	rw::kinematics::State _state;
    };

    /*@}*/
}}

#endif /* OBJECT_HPP_ */
