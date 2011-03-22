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

       BeamJoint implements a deformable cantilever beam as a joint with two degrees of freedom.
       The beam has a length of L, an elastic modulus (Young's modulus) E and a second moment of inertia (area) I.

       The control parameters are the deflection and the angle at the end of the beam.
       The beam extends along the z-axis and the deflection occurs along the y-axis.
 */
class BeamJoint : public Joint {
    /*
     * Class definitions
     */
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<BeamJoint> Ptr;
        
        /**
         * @brief Constructor
         *
         * @param name [in] Name of the joint
         * @param transform [in] Static transform of the joint
         */
        BeamJoint(const std::string& name, const rw::math::Transform3D<>& transform); 
        
        //! @brief Destructor
        virtual ~BeamJoint();
        
        /**
         * @brief The transform of the joint relative to its parent.
         *
         * @param q [in] Translation and angle values of the beam tip
         *
         * @return The transform of the frame relative to its parent
         */
        rw::math::Transform3D<> getJointTransform(const rw::math::Q& q) const;
        
        /**
         * @brief Solver for the internal parameters of the beam, force F, moment M and tip coordinate z.
         * The control inputs are the deflection and the angle of the tip. The function uses Newton's method
         * for calculating the F, M, z triplet corresponding to the input using a 3-dimensional objective function.
         * 
         * @param q [in] Translation and angle values of the beam tip
         * 
         * @return The equivalent force F, moment M and z-coordinate for the control input
         */
        std::vector<double> solveParameters(const rw::math::Q& q) const;
        
        
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
         * @brief Gets the beam length.
         *
         * @return The length
         */
        inline double getLength() const { return _L; }
        
        /**
         * @brief Sets the elastic modulus of the beam.
         *
         * @param E [in] The elastic modulus of the beam 
         */
        inline void setE(double E) { _E = E; }
        
        inline void setControlMode(bool controlMode) { _controlMode = controlMode; }
        
        /**
         * @brief Gets the elastic modulus of the beam.
         *
         * @return The elastic modulus of the beam
         */
        inline double getE() const { return _E; }
        
        /**
         * @brief Sets the second moment of inertia of the beam.
         *
         * @param I [in] The second moment of inertia of the beam 
         */
        inline void setI(double I) { _I = I; }
        
        /**
         * @brief Gets the second moment of inertia of the beam.
         *
         * @return The second moment of inertia of the beam
         */
        inline double getI() const { return _I; }
        
        inline double getF() const { return _F; }
        inline double getM() const { return _M; }
        
        rw::math::Q getQ(const rw::kinematics::State& state) const {
            return rw::math::Q(2, getData(state));
        }
        
        /**
         * @brief Interpolator over the beam profile for the current state
         *
         * @param state [in] The current state
         * 
         * @return An interpolator over the beam profile
         */
        rw::trajectory::Interpolator<rw::math::Transform3D<> >::Ptr getInterpolator(const rw::kinematics::State& state) {
            if(_controlMode) {
                // Solve for internal parameters and instantiate interpolator
                const std::vector<double> sol = solveParameters(rw::math::Q(2, getData(state)));
                return rw::common::ownedPtr(new BeamJointInterpolator(this, sol[0], sol[1], _L));
            } else {
                const rw::math::Q q = rw::math::Q(2, getData(state));
                return rw::common::ownedPtr(new BeamJointInterpolator(this, q[0], q[1], _L));
            }
        }
    
    protected:
        //! @copydoc rw::kinematics::Frame::doMultiplyTransform
        void doMultiplyTransform(const rw::math::Transform3D<>& parent,
                const rw::kinematics::State& state,
                rw::math::Transform3D<>& result) const {
            result = parent * getJointTransform(rw::math::Q(2, getData(state)));
        }
        
        
        //! @copydoc rw::kinematics::Frame::doGetTransform
        rw::math::Transform3D<> doGetTransform(const rw::kinematics::State& state) const {
            return getJointTransform(rw::math::Q(2, getData(state)));
        }
    
    
    private:
        // The transform of the joint based on the internal parameters
        rw::math::Transform3D<> getJointTransform(double F, double M, double s) const;
        
        /**
         * @brief Shooting method for solving for the beam profile given an end load
         *
         * @param F [in] End force
         * @param M [in] End moment
         */
        bool shooting(double F, double M) const;
        
        
        /**
         * @brief Shooting method for solving for the end load given an end transform.
         * The solution can be retrieved by getF() and getM().
         *
         * @param a [in] End angle
         * @param y [in] End y-coordinate
         */
        bool shootingBack(double a, double y) const;

        // Fixed joint transform
        rw::math::Transform3D<> _transform;
        
        // Joint profile
        mutable unsigned int _n;
        mutable std::vector<double> _a, _z, _y;
        
        // End load
        mutable double _F, _M, _zEnd;
        
        // Material parameters
        double _L, _E, _I;
        
        // Controlled by kinematics (true) or force (false)
        bool _controlMode;
        
        /*
         * Interpolator along the beam profile
         */
        class BeamJointInterpolator : public rw::trajectory::Interpolator<rw::math::Transform3D<> > {          
            public:
                BeamJointInterpolator(const BeamJoint* bj, double F, double M, double z) : _bj(bj) {
                    RW_ASSERT(_bj);
                    _F = F;
                    _M = M;
                    _z = z;
                }
                
                virtual ~BeamJointInterpolator() {}
                
                rw::math::Transform3D<> x(double t) const {
                    RW_ASSERT(t >= 0.0 && t <= 1.0);
                    return _bj->getJointTransform(_F, _M, t*_z);
                }
                
                rw::math::Transform3D<> dx(double t) const {
                    RW_ASSERT(t >= 0.0 && t <= 1.0);
                    RW_THROW("First derivative undefined for beams!");
                }
                
                rw::math::Transform3D<> ddx(double t) const {
                    RW_ASSERT(t >= 0.0 && t <= 1.0);
                    RW_THROW("Second derivative undefined for beams!");
                }
                
                double duration() const { return 1.0; }
            
            private:
                const BeamJoint* _bj;
                double _F, _M, _z;
        };
        
        friend class BeamJointInterpolator;
};

/*@}*/
}} // end namespaces

#endif // end include guard
