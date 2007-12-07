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

#ifndef rw_interpolator_Interpolator_HPP
#define rw_interpolator_Interpolator_HPP

/**
 * @file Interpolator.hpp
 */

#include <rw/math/Q.hpp>
#include "InterpolatorIterator.hpp"

#include <boost/numeric/ublas/vector.hpp>
#include <vector>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This abstract interpolator class defines a general interpolator interface.
     */
    class Interpolator
    {
    public:

    protected:
        /**
         * @brief Creates empty object.
         */
        Interpolator();

    public:
        /**
         * @brief Destroys object
         */
        virtual ~Interpolator();

        /**
         * @brief returns a configuration Q at some specific discreticed place in the
         * interpolated path.
         * @param d [in] a double that specifies at what place in the path the
         * configuration Q is to be returned. @f$ d\in [0;length] @f$ where length is
         * the complete length of the interpolated path.
         * @return a configuration of a device
         */
        virtual rw::math::Q getX(double d) const = 0;

        /**
         * @brief the length of the interpolated path is defined as the complete
         * length of the interpolated path. The length can be parameterized with an
         * arbitrary size difined by the specialized class.
         * @return the length of the interpolated path
         */
        virtual double getLength() const = 0;

        /**
         * @brief returns the set of viaPoints that this interpolator
         * interpolates over, including start and end point.
         * @return a vector of via points
         */
        virtual std::vector<rw::math::Q> getViaPoints() const = 0;

        /**
         * @brief The array of segments can be used if efficient random access is nessesary.
         * @return vector of pointers to FunctionSegments.
         */
        virtual const std::vector<FunctionSegment*>& getSegments() const = 0;

        /**
         * @brief This function constructs a iterator for a this Interpolator. This Iterator
         * is bi-directional and enables efficient sequential access.
         * @return InterpolatorIterator that can be used to iterate through the
         * interpolated path.
         */        
        virtual InterpolatorIterator getIterator() const {
            return InterpolatorIterator(&getSegments());
        }

        /**
         * @brief returns the first configuration in path
         * @return first configuration in path
         */
        rw::math::Q getFirstX() 
        {
            return getSegments().front()->getX(0);
        }

        /**
         * @brief returns the last configuration in the path
         * @return last configuration in the path
         */
        rw::math::Q getLastX()
        {
            FunctionSegment *segment = getSegments().back();
            return segment->getX(segment->getLength());
        }

    private:
        Interpolator(const Interpolator&);
        Interpolator& operator=(const Interpolator&);
    };
    /*@}*/
}} // end namespaces

#endif // end include guard
