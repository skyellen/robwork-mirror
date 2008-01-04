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

#ifndef rw_iksolvers_CCDSolver_HPP
#define rw_iksolvers_CCDSolver_HPP

/**
 * @file CCDSolver.hpp
 */

#include <rw/math/Transform3D.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/common/PropertyMap.hpp>

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
     */
    class CCDSolver : public IterativeIK
    {
    public:
        /**
         * @brief Constructor
         */
        CCDSolver(const models::SerialDevice* device);

        void setMaxLocalStep(double quatlength, double poslength);
        
        /**
         * \copydoc rw::inversekinematics::IterativeIK::solve
         *
         * Example:\n
         * CCDAlgorithm r;\n
         * r.inverseKinematics(device, Ttarget);
         */
        std::vector<math::Q> solve(
            const math::Transform3D<>& baseTend,
            const kinematics::State& state) const;

    private:
        math::Transform3D<double> forwardKinematics(
            kinematics::Frame *b,
            kinematics::Frame *e) const;

        double _wpos;
        double _worin;
        double _scale;

        double _maxQuatStep;

        const models::SerialDevice* _device;

        common::PropertyMap _properties;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
