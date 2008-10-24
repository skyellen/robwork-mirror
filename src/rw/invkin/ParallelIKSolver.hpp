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

#ifndef RW_INVKIN_PARALLELIKSOLVER_HPP
#define RW_INVKIN_PARALLELIKSOLVER_HPP

/**
 * @file ParallelIKSolver.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/models/ParallelDevice.hpp>
#include "IterativeIK.hpp"

namespace rw { namespace models { class ParallelDevice; }}

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/

    /**
     * \brief This inverse kinematics method is a heuristic search technique
     */

    class ParallelIKSolver : public IterativeIK
    {
    public:
        /**
         * @brief Constructor
         */
        ParallelIKSolver(const models::ParallelDevice* device);

        virtual ~ParallelIKSolver();

        /**
         * \copydoc rw::inversekinematics::IterativeIK::solve
         */
        virtual std::vector<math::Q> solve(const math::Transform3D<>& baseTend,
                                           const kinematics::State &state) const;

    private:
        typedef std::pair<rw::math::Vector3D<double>, rw::math::Quaternion<double> > QPose;

        const models::ParallelDevice* _device;
        rw::common::PropertyMap _properties;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
