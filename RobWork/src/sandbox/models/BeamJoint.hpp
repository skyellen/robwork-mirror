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


#ifndef RW_MODELS_BEAMJOINT_HPP
#define RW_MODELS_BEAMJOINT_HPP

#include <rw/models/Joint.hpp>

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/
  
    /**
       @brief Beam joint.

       BeamJoint implements a deformable cantilever beam as a joint with one degree of freedom.
       To calculate the deformation, this class needs three parameters for the beam. These are
       the length L, the elastic modulus (Young's modulus) E and the second moment of area/inertia I.
       
       The control parameter is a force F which acts at the end of the beamm, i.e. at z = L.
       The beam extends along the z-axis and the deflection happens along the y-axis.
       
       For calculation of I for rectangular profiles, some helper functions are provided.
    */
    class BeamJoint : public Joint
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<BeamJoint> Ptr;

        /**
         * @brief Constructs BeamJoint
         *
         * @param name [in] Name of the joints
         * @param transform [in] Static transform of the joint
         */
        BeamJoint(const std::string& name, const math::Transform3D<>& transform);

        //! @brief destructor
        virtual ~BeamJoint();

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * @param q [in] Force values at the beam end
         *
         * @return The transform of the frame relative to its parent.
         */
        math::Transform3D<> getJointTransform(const math::Q& F) const;
        
        //! @copydoc Joint::getFixedTransform()        
        rw::math::Transform3D<> getFixedTransform() const { return _transform; }
        
        //! @copydoc Joint::getJacobian
        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint,
                         const math::Transform3D<>& tcp, math::Jacobian& jacobian) const;
        
        inline void setLength(double L) { _L = L; }
        inline void setElasticModulus(double E) { _E = E; }
        inline void setSecondMomentOfArea(double I) { _I = I; }  
        
        inline double getLength() const { return _L; }
        inline double getElasticModulus() const { return _E; }
        inline double getSecondMomentOfArea() const { return _I; }
        
        inline double secondMomentOfAreaRectangle(double widthX, double heightY, bool baseAxis = false) const {
            if(baseAxis)
                return widthX * heightY * heightY * heightY / 12.0;
            else
                return widthX * heightY * heightY * heightY / 3.0;
        }
        
        inline double secondMomentOfAreaCircle(double radius, bool baseAxis = false) const {
            if(baseAxis)
                RW_THROW("Second moment of area not defined for circles with respect to base axis!");
            else
                return math::Pi * radius * radius * radius * radius / 4.0;
        }
        
        inline double secondMomentOfAreaSemicircle(double radius, bool baseAxis = false) const {
            if(baseAxis)
                return math::Pi * radius * radius * radius * radius / 8.0;
            else
                return ( math::Pi / 8.0 - 8.0 / (9.0 * math::Pi) ) * radius * radius * radius * radius;
        }
        
    protected:


        //! @copydoc rw::kinematics::Frame::doMultiplyTransform
        void doMultiplyTransform(const math::Transform3D<>& parent,
                                 const kinematics::State& state,
                                 math::Transform3D<>& result) const;


        //! @copydoc rw::kinematics::Frame::doGetTransform
        math::Transform3D<> doGetTransform(const kinematics::State& state) const;
        
    private:
        math::Transform3D<> getTransformImpl(double F) const;
        
        math::Transform3D<> _transform;
        
        // Material parameters
        double _L, _E, _I;
        
        // Deflection y at a point 0 <= z <= L for an input force F acting at z = L
        inline double deflection(double F, double z) const {
            return (3.0 * _L - z) * F * z * z / (6.0 * _E * _I);
        }
        // Slope dy at a point 0 <= z <= L for an input force F acting at z = L
        inline double slope(double F, double z) const {
            return (2.0 * _L - z) * F * z / (2.0 * _E * _I);
        }
        
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
