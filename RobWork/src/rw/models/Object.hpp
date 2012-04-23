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
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/StateStructure.hpp>
#include <rw/geometry/Geometry.hpp>
#include <rw/graphics/Model3D.hpp>

#include <vector>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief An object is an element in the scene with a geometric representation. The Object
       has a base frame and may have additional frames attached.
    */
    class Object
    {
    public:
        typedef rw::common::Ptr<Object> Ptr;

        Object(rw::kinematics::Frame* baseframe);
        Object(rw::kinematics::Frame* baseframe, rw::geometry::Geometry::Ptr geom);
        Object(rw::kinematics::Frame* baseframe, std::vector<rw::geometry::Geometry::Ptr> geom);

        Object(std::vector<rw::kinematics::Frame*> frames);
        Object(std::vector<rw::kinematics::Frame*> frames, rw::geometry::Geometry::Ptr geom);
        Object(std::vector<rw::kinematics::Frame*> frames, std::vector<rw::geometry::Geometry::Ptr> geom);

        virtual ~Object();

        void addGeometry(rw::geometry::Geometry::Ptr geom);

        void addModel(rw::graphics::Model3D::Ptr model);

        void addFrame(rw::kinematics::Frame* frame);

        void removeGeometry(rw::geometry::Geometry::Ptr geom);

        const std::vector<rw::geometry::Geometry::Ptr>& getGeometry();
        const std::vector<rw::graphics::Model3D::Ptr>& getModels();

        rw::kinematics::Frame* getBase();

        const rw::kinematics::Frame* getBase() const;

        const std::vector<rw::kinematics::Frame*>& getFrames();

        const std::string& getName(){ return _base->getName(); };

    protected:
        friend class WorkCell;
        virtual void registerStateData(rw::kinematics::StateStructure::Ptr sstruct){};
        virtual void removeStateData(rw::kinematics::StateStructure::Ptr sstruct){};

    private:
        rw::kinematics::Frame *_base;
        std::vector<rw::kinematics::Frame*> _frames;
        std::vector<rw::geometry::Geometry::Ptr> _geometry;
        std::vector<rw::graphics::Model3D::Ptr> _models;


    };

    /*@}*/
}}

#endif /* OBJECT_HPP_ */
