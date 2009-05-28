/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_INVKIN_CCDSOLVER_HPP
#define RW_INVKIN_CCDSOLVER_HPP

/**
 * @file CCDSolver.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/models/JacobianCalculator.hpp>

#include "IterativeIK.hpp"


namespace rw { namespace models {
    class SerialDevice;
}} // end namespaces

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/

    /**
     * \brief This inverse kinematics method is a heuristic search technique called
     * the Cyclic-Coordinate Descent method. The method attempts to minimize position
     * and orientation errors by varying individual joints at a time.
     *
     * Notice that the CCDSolver only work on devices with 1-dof joints.
     */
    class CCDSolver : public IterativeIK
    {
    public:
        /**
         * @brief Constructor
         */
        CCDSolver(const models::SerialDevice* device, const kinematics::State& state);

        /**
         * @brief Sets the maximal size of a local step
         * @param quatlength [in] Maximal length for quartenion
         */
        void setMaxLocalStep(double quatlength);

        /**
         * \copydoc IterativeIK::solve
         *
         * Example:\n
         * CCDAlgorithm r;\n
         * r.inverseKinematics(device, Ttarget);
         */
        std::vector<math::Q> solve(const math::Transform3D<>& baseTend,
                                   const kinematics::State& state) const;

        /**
         * @brief performs a local search toward the the target bTed. No via points
         * are generated to support the convergence and robustness.
         * @param bTed [in] the target end pose
         * @param maxError [in] the maximal allowed error
         * @param state [in/out] the starting position for the search. The end position will
         * also be saved in this state.
         * @param maxIter [in] max number of iterations
         * @return true if error is below max error
         * @note the result will be saved in state
         */
        bool solveLocal(const math::Transform3D<> &bTed,
                        double maxError,
                        kinematics::State &state,
                        int maxIter) const;

    private:
        double _wpos;
        double _worin;
        double _scale;

        double _maxQuatStep;

        const models::SerialDevice* _device;

        common::PropertyMap _properties;

        kinematics::FKRange _fkrange;
        rw::common::Ptr<models::JacobianCalculator> _devJac;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
