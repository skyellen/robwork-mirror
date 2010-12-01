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
       
       The control parameter is a force F which acts at the end of the beam. However, the position l
       at which F acts can also be changed.
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
        
        //! @copydoc Joint::getJacobian
        void getJacobian(size_t row, size_t col, const math::Transform3D<>& joint,
                         const math::Transform3D<>& tcp, math::Jacobian& jacobian) const;
        
        inline void setLength(double L) { _L = L; }
        inline void setElasticModulus(double E) { _E = E; }
        inline void setSecondMomentOfArea(double I) { _I = I; }
        inline void setForcePosition(double l) { _l = l; }       
        
        inline double getLength() const { return _L; }
        inline double getElasticModulus() const { return _E; }
        inline double getSecondMomentOfArea() const { return _I; }
        inline double getForcePosition() const { return _l; }
        
    private:
        math::Transform3D<> _transform;
        
        // Material parameters
        double _L, _E, _I;
        // Force position
        double _l;
        
        // Deflection at a point 0 <= x <= L for an input force F acting at x = l
        double deflection(double F, double x) const;
        // Slope at a point 0 <= x <= L for an input force F acting at x = l
        double slope(double F, double x) const;
        
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
