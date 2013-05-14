/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef RWLIBS_SOFTBODY_BEAMOBSTACLEPLANE_HPP
#define RWLIBS_SOFTBODY_BEAMOBSTACLEPLANE_HPP

#include <rw/geometry/Plane.hpp>
#include <rw/math/Transform3D.hpp>



namespace rwlibs {
namespace softbody {
/** @addtogroup softbody */
/*@{*/
    /**
     * @brief A plane obstacle for a beam
     **/
    class BeamObstaclePlane {
    public:
        /**
         * @brief Constructor
         *
         * @param plane the plane defining the obstacle
         * @param trans the world to plane transformation
         **/
        BeamObstaclePlane(const rw::geometry::Plane &plane, const rw::math::Transform3D<> &trans);
        
        
        
        /**
         * @brief returns the world to plane transformation
         *
         * @return world to plane transformation
         **/
        
        
        
        rw::math::Transform3D<> getTransform(void) const;
        /**
         * @brief sets the world to plane transformation
         *
         * @param trans world to plane transformation
         **/
        void setTransform(const rw::math::Transform3D<> &trans);
        
        
        
        /**
         * @brief given the plane to beam tranformation, returns yTCP
         *
         * @param planeTbeam the transformation from the plane to the beam..
         * @return yTCP in millimeters
         **/        
        double get_yTCP(const rw::math::Transform3D<> &planeTbeam) const;
        
        
        
        /**
         * @brief given the plane to beam transformation, returns thetaTCP
         *
         * @param planeTbeam the transformation from the plane to the beam
         * @return thetaTCP in radians
         **/
        double get_thetaTCP(const rw::math::Transform3D<> &planeTbeam) const;
        
        
        
        /**
         * @brief computes the plane to beam transformation, given the world to beam transformation
         *
         * @param Tbeam world to beam transformation
         * @return plane to beam transformation
         **/
        rw::math::Transform3D<> compute_planeTbeam(const rw::math::Transform3D<> &Tbeam);
        
        
        
        /**
         * @brief returns the plane
         *
         * @return the plane
         **/        
        const rw::geometry::Plane &getPlane(void) const;
        
    private:
        rw::geometry::Plane _plane;
        rw::math::Transform3D<> _trans;
    };
    /*@}*/
}}

#endif // BEAMOBSTACLEPLANE_HPP
