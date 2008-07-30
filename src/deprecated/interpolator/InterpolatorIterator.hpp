/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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

#ifndef rw_interpolator_InterpolatorIterator_HPP
#define rw_interpolator_InterpolatorIterator_HPP

/**
 * @file InterpolatorIterator.hpp
 */

#include <rw/math/Q.hpp>
#include "FunctionSegment.hpp"

#include <vector>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This class represents a structure for iterating efficiently through a
     * path defined by a vector of FunctionSegments.
     *
     * The iterator is bi-directional and is able to return a "position" and its
     * 1st and 2nd derivatives.
     */
    class InterpolatorIterator
    {
    private:
        bool _endReached;
        bool _beginReached;

        /**
         * @brief a pointer to a vector of FunctionSegment pointers
         */
        const std::vector<FunctionSegment*> *_segments;

        /**
         * @brief points to the current segment in the _segments vector.
         */
        int _index;

        /**
         * @brief is the current offset from the start of the current segment.
         */
        double _offset;

    public:
        /**
         * @brief Constructor
         *
         * An exception is thrown if \b segments is NULL.
         *
         * @param segments [in] a pointer to a vector of FunctionSegments
         */
        InterpolatorIterator(const std::vector<FunctionSegment*>* segments);

        /**
         * @brief This function can be used to decrease the iterator position.
         *
         * The position can be decreased no longer than to the length 0.
         * 
         * However, each time a new double value is subtracted, a small error can
         * be introduced due to inaccuracy of floating point numbers.
         * As long as the error is within \f$10^{-10}\f$, the old segment of the path
         * is used.
         *
         * @param val [in] a double that describes how much to decrease the
         * iterator position
         */
        void operator-=(double val);

        /**
         * @brief This function can be used to increase the iterator position.
         *
         * The position can be increased no longer than the length og the
         * complete iterated path.
         * 
         * However, each time a new double value is added, a small error can
         * be introduced due to inaccuracy of floating point numbers.
         * As long as the error is within \f$10^{-10}\f$, the old segment of the path
         * is used.
         *
         * @param val [in] a double that describes how much to increase the
         * iterator position
         */
        void operator+=(double val);

        /**
         * @brief used to test if the end of the interpolated path is reached.
         *
         * @return true if the iterator has reached the end of the interpolated
         * path false otherwise.
         */
        bool isEnd() { return _endReached; }

        /**
         * @brief used to test if the beginning of the interpolated path is reached.
         *
         * When the iterator initially starts out, the iterator is positioned at
         * the beginning of the path and isBegin() returns true.
         *
         * @return true if the iterator has reached the beginning of the
         * interpolated path false otherwise.
         */
        bool isBegin() { return _beginReached; }

        /**
         * @brief Extracts a point at the current position in the interpolated
         * path.
         *
         * @param iter [in] the InterpolatorIterator
         *
         * @return the point at the current position in the path.
         */
        friend rw::math::Q operator*(InterpolatorIterator& iter) { return iter.getX(); }

        /**
         * @brief Extracts a point at the current position in the interpolated
         * path.
         *
         * @return the point at the current position in the interpolated path.
         *
         * TODO: handle when _segments == NULL
         */
        rw::math::Q getX() const { return getFun().getX(_offset); }

        /**
         * @brief Extracts a point of the derivative of the interpolated path at
         * the current position in the path
         *
         * @return the derived point at the current position in the interpolated
         * path.
         */
        rw::math::Q getXd() const { return getFun().getXd(_offset); }

        /**
         * @brief Extracts a point of the double derivative of the interpolated path
         * at the current position in the path
         *
         * @return the double derived point at the current position in the
         * interpolated path.
         */
        rw::math::Q getXdd() const { return getFun().getXdd(_offset); }

    private:
        FunctionSegment& getFun() const { return *(*_segments)[_index]; }
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
