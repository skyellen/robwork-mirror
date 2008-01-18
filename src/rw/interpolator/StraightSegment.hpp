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

#ifndef rw_interpolator_StraightSegment_HPP
#define rw_interpolator_StraightSegment_HPP

/**
 * @file StraightSegment.hpp
 */

#include "FunctionSegment.hpp"
#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This is a basic straightline segment.
     * 
     * The output of the segment is
     * defined by @f$ \bf{f}(t/length)= \bf{a} + \bf{b} \cdot t @f$, where
     * t is the interpolation parameter.
     */
    class StraightSegment : public FunctionSegment
    {
    private:
        /**
         * @brief The parameters of the straight line function
         */
        rw::math::Q _a;
        rw::math::Q _b;

    public:

        /**
         * @brief Constructor
         * 
         * Constructs a StraightSegment with parameters \b a and \b b         
         * @param a [in] the a parameter in the straight line equation
         * @param b [in] the b parameter in the straight line equation
         * @param length [in] length of the segment
         */
        StraightSegment(const rw::math::Q& a,
                        const rw::math::Q& b,
                        //const std::pair<double, double> &interval)
                        double length):            
            FunctionSegment(length),
            _a(a),
            _b(b)
        {
            if (length<=0)
                RW_THROW("Lenght of segment must to positive");    
        }

        /**
         * @brief sets all the parameters of this StraightLine segment
         * @param a [in] the a parameter in the straight line equation
         * @param b [in] the b parameter in the straight line equation
         * @param length [in] the length of the segment
         */
        void setParameters(const rw::math::Q& a,
                           const rw::math::Q& b,
                           double length)
        {
            if (length <= 0)
                RW_THROW("Length of segment must be positive");
            _a = a;
            _b = b;
            _length = length;
        }

        /**
         * @brief Deconstructor
         */
        virtual ~StraightSegment() {}

        /**
         * @copydoc FunctionSegment::getX
         */
        rw::math::Q getX(double t) const
        {
            t /= _length;
            return _a + t * _b;
        };

        /**
         * @copydoc FunctionSegment::getXd
         */
        rw::math::Q getXd(double t) const
        {
            return _b;
        };

        /**
         * @copydoc FunctionSegment::getXdd
         */
        rw::math::Q getXdd(double t) const {
            return rw::math::Q(rw::math::Q::ZeroBase(_a.size()));
        }
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
