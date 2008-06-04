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

#ifndef rw_proximity_DistanceToleranceStrategy_HPP
#define rw_proximity_DistanceToleranceStrategy_HPP
/**
 * @file DistanceStrategy.hpp
 */

#include <rw/kinematics/Frame.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/geometry/Face.hpp>

#include "ProximityStrategy.hpp"

namespace rw { namespace proximity {

    /** @addtogroup proximity */
    /*@{*/


	/**
	 * @brief DistanceResult contains basic information about the distance
	 * result between two frames.
	 */
	struct MultiDistanceResult {
		 //! @brief reference to the first frame
		const kinematics::Frame* f1;

		//! @brief reference to the second frame
		const kinematics::Frame* f2;

		// TODO: is this correct?? vector pointing along shortest distance axis from f1 towards f2

		//! Closest point on f1 to f2, described in f1 reference frame
		math::Vector3D<double> p1;

		//! Closest point on f2 to f1, described in f2 reference frame
		math::Vector3D<double> p2;

		//! @brief distance between frame f1 and frame f1
		double distance;


		std::vector< math::Vector3D<> > p1s;
		std::vector< math::Vector3D<> > p2s;
		std::vector< double > distances;
	};


    /**
     * @brief The DistanceStrategy interface is used to abstract away
     * specific collision detection algorithms or strategies.
     */
    class DistanceToleranceStrategy {

    public:
        /**
         * @brief Destroys object
         */
        virtual ~DistanceToleranceStrategy();

        /**
         * @brief Calculates the distance between two given frames @f$ \mathcal{F}_a @f$ and
         * @f$ \mathcal{F}_b @f$
         *
         * @param a [in] @f$ \mathcal{F}_a @f$
         *
         * @param wTa [in] @f$ \robabx{w}{a}{\mathbf{T}} @f$
         *
         * @param b [in] @f$ \mathcal{F}_b @f$
         *
         * @param wTb [in] @f$ \robabx{w}{b}{\mathbf{T}} @f$
         *
         * @param rel_err [in] relative acceptable error
         *
         * @param abs_err [in] absolute acceptable error
         *
         * @return shortest distance if @f$ \mathcal{F}_a @f$ and @f$ \mathcal{F}_b @f$ are
         * separated and not in collision.
         */
        virtual bool getDistances(
            MultiDistanceResult &result,
            const kinematics::Frame* a,
            const math::Transform3D<>& wTa,
            const kinematics::Frame* b,
            const math::Transform3D<>& wTb,
            double tolerance,
            double rel_err = 0.0,
            double abs_err = 0.0) = 0;

    private:
        DistanceToleranceStrategy(const DistanceToleranceStrategy&);
        DistanceToleranceStrategy& operator=(const DistanceToleranceStrategy&);

    protected:
        /**
         * @brief Creates object
         */
        DistanceToleranceStrategy();
    };

    /*@}*/
}} // end namespaces

#endif /* rw_proximity_DistanceToleranceStrategy_HPP */
