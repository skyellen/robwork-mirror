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


#ifndef RWLIBS_ALGORITHMS_QPCONTROLLER_IKGPMMSOLVER_HPP
#define RWLIBS_ALGORITHMS_QPCONTROLLER_IKGPMMSOLVER_HPP

/**
 * @file IKGPMMSolver.hpp
 */

#include "BasicGPMM.hpp"

#include <rw/invkin/IterativeMultiIK.hpp>
#include <rw/kinematics/State.hpp>

namespace rw { namespace models {
    class SerialDevice;
}} // end namespaces

namespace rwlibs { namespace algorithms {

    /** \addtogroup algorithms */
    /*@{*/

    /**
     * @brief Iterative inverse kinematics solved based on the QPController
     *
     * The IKGPMMSolver works very similar to the SimpleSolver. However, instead
     * of using a simple inverse Jacobian it uses the rw::algorithms::QPController
     * making it robust to singularities and ensuring joint limits.
     *
     * Usually the IKGPMMSolver runs order of magnitudes slower than the standard
     * SimpleSolver. Solutions returned by the IKGPMMSolver might not be exact
     * solutions, but are least square fits. Given a target outside the robot workspace
     * the IKGPMMSolver will, given enough iterations, return the least square solution.
     *
     * Notice that the IKGPMMSolver is a local method. It there is thus no guarantee that
     * the solution returned will be the global least square.
     */
    class IKGPMMSolver: public rw::invkin::IterativeMultiIK {
    public:

        /**
         * @brief Constructs BasicGPMM for TreeDevice. Uses the default
         * end effectors of the treedevice
         * @param device [in] Device to work with
         * @param state [in] Default state
         * @param qhome [in] Configuration somewhere between the lower and upper limit and towards which the joints should move
         * @param dt [in] Step size
         */
        IKGPMMSolver(const rw::models::TreeDevice* device,
                  const rw::kinematics::State& state);

        /**
         * @brief Constructs IKGPMMSolver for device
         *
         * It is required that the device has correct joint position, velocity
         * and acceleration limits.
         *
         * @param device [in] Device to solve for         * @param state [in] State of the workcell
         */
        IKGPMMSolver(const rw::models::JointDevice* device,
                  const std::vector<rw::kinematics::Frame*>& foi,
                  const rw::kinematics::State& state);

        /**
         * @copydoc IterativeMultiIK::solve
         */
        std::vector<rw::math::Q> solve(const std::vector<rw::math::Transform3D<> >& baseTend,
                                   const rw::kinematics::State& state) const;

        /**
         * @brief performs a local search toward the the target bTed. No via points
         * are generated to support the convergence and robustness.
         * @param bTed [in] the target end pose
         * @param maxError [in] the maximal allowed error
         * @param state [in/out] the starting position for the search. The end position will
         * also be saved in this state.
         * @param maxIter [in] max number of iterations
         * @param untilSmallChange [in] if true the error difference between two
         * succesive iterations need to be smaller than \b maxError. If false then the
         * error of a iteration need to be smaller than \b maxError.
         * @return true if error is below max error
         * @note the result will be saved in state
         */
        bool solveLocal(const std::vector<rw::math::Transform3D<> > &bTed,
                        std::vector<double>& maxError,
                        rw::kinematics::State &state,
                        int maxIter,
                        bool untilSmallChange=false) const;


        void setMaxLocalStep(double qlength, double plength){_maxQuatStep=qlength;};

    private:
        mutable BasicGPMM _controller;
        const rw::models::JointDevice* _device;
        rw::common::PropertyMap _properties;
        rw::kinematics::State _state;
        double _maxQuatStep;
        std::vector<rw::kinematics::Frame*> _foi; // frames of interest, end frames
        std::vector<boost::shared_ptr<rw::kinematics::FKRange> > _fkranges;
        bool _returnBestFit;


        bool performLocalSearch(const rw::models::SerialDevice *device,
                       			const rw::math::Transform3D<> &bTed,
                       			double maxError,
                       			rw::kinematics::State &state,
                       			unsigned int maxIter) const;
    };

}} // end namespaces

#endif // end include guard
