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

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/models/Joint.hpp>
#include <rw/trajectory/Interpolator.hpp>

namespace rw { namespace kinematics {
    class State;
}} // end namespaces

namespace rw { namespace models {
    /** @addtogroup models */
    /*@{*/
  
    /**
       @brief Beam joint.

       BeamJoint implements a deformable cantilever beam as a joint with one degree of freedom.
       The beam has a length of L.
       
       The control parameter is an angle a at the end of the beam, i.e. at z = L.
       The beam extends along the z-axis and the deflection occurs along the y-axis.
    */
    class BeamJoint : public Joint {
      private:
        /*
         * Interpolator along the beam profile
         */
        class BeamJointInterpolator : public rw::trajectory::Interpolator<rw::math::Transform3D<> > {          
          public:
            BeamJointInterpolator(const BeamJoint* bj, const rw::kinematics::State& state) : _bj(bj) {
               RW_ASSERT(_bj!=NULL);
              _F = _bj->getData(state)[0];
              _zF = _bj->projectedLength(_F);
            }
            
            virtual ~BeamJointInterpolator() {}
            
            rw::math::Transform3D<> x(double z) const { return _bj->getJointTransform(_F, z*_zF); }
            
            rw::math::Transform3D<> dx(double z) const { RW_THROW("First derivative undefined for beams!"); }
            
            rw::math::Transform3D<> ddx(double z) const { RW_THROW("Second derivative undefined for beams!"); }
            
            double duration() const { return 1; }
            
          private:
            const BeamJoint* _bj;
            double _F, _zF;
        };
        
        friend class BeamJointInterpolator;
      
      /*
       * Class definitions
       */
      public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<BeamJoint> Ptr;

        rw::trajectory::Interpolator<rw::math::Transform3D<> >::Ptr getInterpolator(const rw::kinematics::State& state) {
          return rw::common::ownedPtr(new BeamJointInterpolator(this, state));
        }

        /**
         * @brief Constructs BeamJoint
         *
         * @param name [in] Name of the joints
         * @param transform [in] Static transform of the joint
         */
        BeamJoint(const std::string& name, const rw::math::Transform3D<>& transform); 

        //! @brief destructor
        virtual ~BeamJoint();

        /**
         * @brief The transform of the joint relative to its parent.
         *
         * @param F [in] Force value at the beam end
         *
         * @return The transform of the frame relative to its parent.
         */
        rw::math::Transform3D<> getJointTransform(const rw::math::Q& a);
        
        //! @copydoc Joint::getFixedTransform()        
        rw::math::Transform3D<> getFixedTransform() const { return _transform; }
        
        //! @copydoc Joint::getJacobian
        void getJacobian(size_t row,
                         size_t col,
                         const rw::math::Transform3D<>& joint,
                         const rw::math::Transform3D<>& tcp,
                         const rw::kinematics::State& state,
                         rw::math::Jacobian& jacobian) const;
        
        /**
         * @brief Sets the beam length.
         *
         * @param L [in] The beam length
         */
        inline void setLength(double L) { _L = L; }
        
        /**
         * @brief Gets the length of the beam.
         *
         * @return the length
         */
        inline double getLength() const { return _L; }
        
        void setBounds(const std::pair<const math::Q, const math::Q>& bounds);
        
      protected:
        //! @copydoc rw::kinematics::Frame::doMultiplyTransform
        void doMultiplyTransform(const rw::math::Transform3D<>& parent,
                                 const rw::kinematics::State& state,
                                 rw::math::Transform3D<>& result) const {
          result = parent * getJointTransform(getData(state)[0]);
        }


        //! @copydoc rw::kinematics::Frame::doGetTransform
        math::Transform3D<> doGetTransform(const rw::kinematics::State& state) const {
          return getJointTransform(getData(state)[0]);
        }
        
      private:
        rw::math::Transform3D<> getJointTransform(double a) const;
        
        rw::math::Transform3D<> getJointTransform(double a, double z) const;
        
        rw::math::Transform3D<> _transform;
        
        // Material parameters
        double _L;
        
        // Deflection y(z) at a point 0 <= z <= zL for an angle a at z
        inline double deflection(double a, double z) const {
            return -std::tan(a) * (z*z - 3.0*_L*z) / (3.0*z - 6.0*_L);
        }
        
        /*
         * The z value zL corresponding to the z-distance to the tip of the beam (the projected length)
         * under a given deflection.
         *
         * TODO: The input to deflection()
         */
        inline double projectedLength(double a) const {
          const double y = deflection(a, _L);
          return std::sqrt(_L*_L - y*y);
        }
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
