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
       To calculate the deformation, this class needs three parameters for the beam. These are
       the length L, the elastic modulus (Young's modulus) E and the second moment of area/inertia I.
       
       The control parameter is a force F which acts at the end of the beamm, i.e. at z = L.
       The beam extends along the z-axis and the deflection occurs along the y-axis.
       
       For calculation of I for rectangular and (semi)circular profiles,
       some helper functions are provided.
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
        rw::math::Transform3D<> getJointTransform(const rw::math::Q& F) { return getJointTransform(F[0]); }
        
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
         * @brief Sets the elastic modulus (Young's modulus) of the beam
         *
         * @param E [in] The elastic modulus
         */
        inline void setElasticModulus(double E) { _E = E; }
        
        /**
         * @brief Sets the second moment of area/inertia.
         *
         * @param I [in] The second moment of area
         */
        inline void setSecondMomentOfArea(double I) { _I = I; }
        
        /**
         * @brief Gets the length of the beam.
         *
         * @return the length
         */
        inline double getLength() const { return _L; }
        
        /**
         * @brief Gets the elastic modulus (Young's modulus) of the beam.
         *
         * @return the elastic modulus
         */
        inline double getElasticModulus() const { return _E; }
        
        /**
         * @brief Gets the second moment of area/inertia of the beam.
         *
         * @return the second moment of area
         */
        inline double getSecondMomentOfArea() const { return _I; }
        
        /**
         * @brief Calculates the second moment of area of a rectangular cross-section
         *
         * @param w [in] The width
         * @param h [in] The height
         * @param baseAxis [in] Whether the second moment of area should be calculated w.r.t.
         * the base axis or center axis
         *
         * @return the second moment of area
         */
        static inline double secondMomentOfAreaRectangle(double w, double h, bool baseAxis = false) {
            return baseAxis ? w * h * h * h / 12.0 :
                              w * h * h * h / 3.0;
        }
        
        /**
         * @brief Calculates the second moment of area of a circular cross-section
         *
         * @param r [in] The radius
         * @param baseAxis [in] Whether the second moment of area should be calculated w.r.t.
         * the base axis or center axis
         *
         * @return the second moment of area
         */
        static inline double secondMomentOfAreaCircle(double radius, bool baseAxis = false) {
            return baseAxis ? 5.0 * rw::math::Pi * radius * radius * radius * radius / 4.0 :
                              rw::math::Pi * radius * radius * radius * radius / 4.0;
        }
        
        /**
         * @brief Calculates the second moment of area of a semicircular cross-section
         *
         * @param r [in] The radius
         * @param baseAxis [in] Whether the second moment of area should be calculated w.r.t.
         * the base axis or center axis
         *
         * @return the second moment of area
         */
        static inline double secondMomentOfAreaSemicircle(double radius, bool baseAxis = false) {
            return baseAxis ? rw::math::Pi * radius * radius * radius * radius / 8.0 :
                              ( rw::math::Pi / 8.0 - 8.0 / (9.0 * rw::math::Pi) ) * radius * radius * radius * radius;
        }
        
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
        rw::math::Transform3D<> getJointTransform(double F) const;
        
        rw::math::Transform3D<> getJointTransform(double F, double z) const;
        
        rw::math::Transform3D<> _transform;
        
        // Material parameters
        double _L, _E, _I;
        
        // Deflection y(z) at a point 0 <= z <= zL for an input force F acting at z = L
        inline double deflection(double F, double z) const {
            return (3.0 * _L - z) * F * z * z / (6.0 * _E * _I);
        }
        
        // Slope dy/dz at a point 0 <= z <= zL for an input force F acting at z = L
        inline double slope(double F, double z) const {
            return (2.0 * _L - z) * F * z / (2.0 * _E * _I);
        }
        
        /*
        // Curvature d^2y/dz^2 at a point 0 <= z <= zL for an input force F acting at z = L
        inline double curvature(double F, double z) const {
            return (2.0 * _L - z) * F / (2.0 * _E * _I);
        }
        */
        
        // Angle of the beam at a point 0 <= z <= zL for an input force F acting at z = L
        inline double angle(double F, double z) const {
          return -std::atan(slope(F, z));
        }
        
        /*
         * The arc length l of the beam at z for an input force F acting at z = L
         * l gives the numerical integral (using Simpson's rule):
         * 
         * l \approx \int_{0}^{z}\sqrt{1 + \left(\frac{dy}{dx}\right)^2}dz
         */
        double arcLength(double F, double z) const;
        
        /*
         * The z value zL corresponding to the z-distance to the tip of the beam (the projected length)
         * under a given deflection caused by an input force F acting at the tip.
         *
         * zL gives the numerical solution to the equation (using a starting guess of L):
         * 
         * \int_{0}^{zL}\sqrt{ 1 + \left(\frac{dy}{dz}\right)^2 }dz = L \Leftrightarrow
         * \int_{0}^{zL}\sqrt{ 1 + \left(\frac{dy}{dz}\right)^2 }dz = L
         */
        double projectedLength(double F,
                               unsigned int ntrial = 20,
                               double toly = 0.00001,
                               double tolz = 0.00001) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
