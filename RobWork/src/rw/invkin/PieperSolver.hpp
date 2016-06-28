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


#ifndef RW_INVKIN_PIEPERSOLVER_HPP
#define RW_INVKIN_PIEPERSOLVER_HPP

/**
 * @file PieperSolver.hpp
 */


#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <vector>

#include "ClosedFormIK.hpp"

namespace rw { namespace models { class DHParameterSet; } }
namespace rw { namespace models { class SerialDevice; } }

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/


    /**
     * @brief Calculates the closed form inverse kinematics of
     * a device using Piepers method
     *
     * To use Piepers method it is required that the device has
     * 6 DOF revolute joints, and that last three axis intersects.
     * In this implementation it will be assumed that the that
     * rotation of these last three axis are equivalent to an
     * Euler ZYZ or Z(-Y)Z rotation.
     *
     * See Introduction to Robotics Mechanics and Control, by
     * John J. Craig for further information about the algorithm.
     */
    class PieperSolver: public ClosedFormIK {
    public:

		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<PieperSolver> Ptr;

        /**
         * @brief Constructor
         * @param dhparams [in] DH-parameters corresponding to the device
         * @param joint6Tend [in] transform from the 6th joint to the end of the device
         * @param baseTdhRef [in] Transformation between the robot base and the reference frame for the DH-parameters.
         */
        PieperSolver(const std::vector<rw::models::DHParameterSet>& dhparams,
                     const rw::math::Transform3D<>& joint6Tend,
                     const rw::math::Transform3D<>& baseTdhRef = rw::math::Transform3D<>::identity());

        /**
         * @brief Constructor - the DH parameters is expected to be on each joint
         * in the serial device. When specifying the DH params in the workcell file
         * this constructor can be used.
         * @param dev [in] the device for which to extract the DH parameters.
         * @param joint6Tend [in] transform from the 6th joint to the end of the device
         * @param state [in] State using which the transformation between robot base and the DH-parameters reference frame are calculated.
         * @note throws an exception if the device has no DH params
         */
        PieperSolver(rw::models::SerialDevice& dev,
                     const rw::math::Transform3D<>& joint6Tend,
                     const rw::kinematics::State& state);

        /**
         * @copydoc ClosedFormIK::solve
         */
        virtual std::vector<math::Q> solve(const rw::math::Transform3D<>& baseTend, const rw::kinematics::State& state) const;

        /**
         * @copydoc InvKinSolver::setCheckJointLimits
         */
        virtual void setCheckJointLimits(bool check);

    private:
        std::vector<rw::models::DHParameterSet> _dhparams;
        rw::math::Transform3D<> _baseTdhRef;
        rw::math::Transform3D<> _0Tbase;

        rw::math::Transform3D<> _endTjoint6;

        void init();

        void solveTheta456(double theta1,
                           double theta2,
                           double theta3,
                           rw::math::Transform3D<>& T06,
                           std::vector<rw::math::Q>& result) const;


        /**
         * @brief Solves the case with a1 = 0 (Case 1 in Craig)
         */
        std::vector<double> solveTheta3Case1(double z) const;

        /**
         * @brief Solves the case with a1 != 0 and sin(alpha1) = 0 (Case 2 in Craig)
         */
        std::vector<double> solveTheta3Case2(double r) const;

        /**
         * @brief Solves the general case with a1 != 0 and sin(alpha1) != 0 (Case 3 in Craig)
         */
        std::vector<double> solveTheta3Case3(double r, double z) const;


        double solveTheta2(double r, double z, double theta3) const;

        std::vector<double> solveTheta2Case1(double z, double theta3) const;
        std::vector<double> solveTheta2Case2(double r, double theta3) const;

        double solveTheta1(double x, double y, double theta2, double theta3) const;

        double f(double x) const;
        double df(double x) const;
        double ddf(double x) const;


        std::vector<double> fSolve() const;
        std::vector<double> fSolve(double s1, double s2, double s3) const;
        std::vector<double> dfSolve(double s1, double s2) const;
        std::vector<double> ddfSolve() const;

        void setupCoefficients(double r, double z) const;


        /*
         * Variables used for calculating
         * Defined here to avoid allocating memory for them all the time
         */
        mutable double a,b,c,d,e;

        mutable double alpha0, a0, calpha0, salpha0, d1;
        mutable double alpha1, a1, calpha1, salpha1, d2;
        mutable double alpha2, a2, calpha2, salpha2, d3;
        mutable double alpha3, a3, calpha3, salpha3, d4;
        mutable double alpha4, a4, calpha4, salpha4, d5;
        mutable double alpha5, a5, calpha5, salpha5, d6;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
