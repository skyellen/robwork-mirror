/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_iksolvers_PieperSolver_HPP
#define rw_iksolvers_PieperSolver_HPP

/**
 * @file PieperSolver.hpp
 */


#include <rw/math/Transform3D.hpp>
#include <rw/math/Q.hpp>
#include <vector>

#include "ClosedFormIK.hpp"

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/

    /**
     * @brief Simple struct to help represent a set of Denavit-Hartenberg
     * parameters
     */
    struct DHSet
    {
        /** @brief \f$\alpha_{i-1}\f$ **/
        double _alpha;
        /** @brief \f$a_{i-1}\f$ **/
        double _a;
        /** @brief \f$d_{i} \f$ **/
        double _d;
        /** $brief \f$\theta_{i} \f$ **/
        double _theta;

        /**
         * @brief Constructor
         * @param alpha [in] \f$\alpha_{i-1}\f$
         * @param a [in] \f$a_{i-1}\f$
         * @param d [in] \f$d_{i}\f$
         * @param theta [in] \f$\theta_{i-1}\f$
         */
        DHSet(double alpha, double a, double d, double theta) :
            _alpha(alpha),
            _a(a),
            _d(d),
            _theta(theta)
        {}
    };

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
        /**
         * @brief Constructor
         * @param dhparams [in] DH-parameters corresponding to the device
         * @param joint6Tend [in] transform from the 6th joint to the end of the device
         */
        PieperSolver(
            const std::vector<DHSet>& dhparams,
            const rw::math::Transform3D<>& joint6Tend);

        /**
         * @copydoc rw::inversekinematics::ClosedFormIK::solve
         */
        virtual std::vector<math::Q> solve(rw::math::Transform3D<>& baseTend) const;

    private:
        std::vector<DHSet> _dhparams;
        rw::math::Transform3D<> _0Tbase;
        rw::math::Transform3D<> _endTjoint6;

        void solveTheta456(
            double theta1,
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

        std::vector<double> fSolve(double s1, double s2, double s3) const;
        std::vector<double> dfSolve(double s1, double s2) const;
        std::vector<double> ddfSolve() const;

        void setupCoefficients(double r, double z) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
