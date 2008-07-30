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

#ifndef rw_interpolator_LloydHaySegment_HPP
#define rw_interpolator_LloydHaySegment_HPP

/**
 * @file LloydHaySegment.hpp
 */

#include "FunctionSegment.hpp"

#include <rw/math/Q.hpp>
namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This class represents a Lloyd Hayward blended segment.
     *
     */
    class LloydHaySegment : public FunctionSegment
    {
    private:
        const FunctionSegment *_x1,*_x2;
        double _k;
        double _tau;
        //double _length; now part of FunctionSegment

    public:
        /**
         * @brief Constructs a Lloyd Hayward blend between two FunctionSegments. The end of x1 and
         * start of x2 is required to be the same. Also the dimension of x1 should equal the dimension
         * of x2.
         * @param x1 [in] The first of the two segments.
         * @param x2 [in] The second of the two segments.
         * @param k [in] A parameter that controls the amount of acceleration compensation to apply.
         * @param tau [in] The half of the blend time.         
         */
        LloydHaySegment(const FunctionSegment& x1, const FunctionSegment& x2, double k, double tau);

        /**
         * @brief Destructor
         */
        virtual ~LloydHaySegment(){};

        /**
         * @copydoc FunctionSegment::getX
         */
        rw::math::Q getX(double t) const;

        /**
         * @copydoc FunctionSegment::getXd
         */
        rw::math::Q getXd(double t) const;

        /**
         * @copydoc FunctionSegment::getXdd
         */
        rw::math::Q getXdd(double t) const;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
