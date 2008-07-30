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

#ifndef rw_interpolator_PathIterator_HPP
#define rw_interpolator_PathIterator_HPP

/**
 * @file TrajectoryIterator.hpp
 */

#include "InterpolatorIterator.hpp"
#include "Interpolator.hpp"

#include <vector>

namespace rw { namespace interpolator {
    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief This is a bidirectional iterator class for a Path.
     */
    class TrajectoryIterator
    {
    private:
        const std::vector<interpolator::Interpolator*> *_interpolators;
        unsigned int _currIndex;
        double _currOffset;
        bool _endReached,_beginReached;
        double _length;
        interpolator::InterpolatorIterator _currIIterator;

    public:
        /**
         * @brief Constructor
         *
         * @param interpolators [in] a pointer to a vector of FunctionSegments
         */
        TrajectoryIterator(const std::vector<interpolator::Interpolator*>* interpolators = NULL);

        /**
         * @brief This function can be used to decrease the iterator position.
         * The position can be decreased no longer than to the length 0.
         *
         * @param val [in] a double that describes how much to decrease the
         * iterator position
         */
        void operator -=(double val);

        /**
         * @brief This function can be used to increase the iterator position.
         * The position can be increased no longer than the length og the
         * complete iterated path.
         *
         * @param val [in] a double that describes how much to increase the
         * iterator position
         */
        void operator +=(double val);

        /**
         * @brief used to test if the end of the path is reached.
         *
         * @return true if the iterator has reached the end of the path false
         * otherwise.
         */
        bool isEnd(){ return _endReached;};

        /**
         * @brief used to test if the beginning of the path is reached.
         *
         * @return true if the iterator has reached the beginning of the path
         * false otherwise.
         */
        bool isBegin(){ return _beginReached;};

        /**
         * @brief Extracts a point at the current position in the path.
         *
         * @param iter [in] the path iterator
         *
         * @return the point at the current position in the path.
         */
        friend math::Q operator*(TrajectoryIterator& iter){
            return iter.getX();
        }

        /**
         * @brief Extracts a point at the current position in the path.
         *
         * @return the point at the current position in the path.
         */
        math::Q getX() const {
            return _currIIterator.getX();
        }

        /**
         * @brief Extracts a point of the derivative of the path
         * at the current position in the path
         *
         * @return the derived point at the current position in the path.
         */
        math::Q getXd() const {
            return _currIIterator.getXd();
        }

        /**
         * @brief Extracts a point of the double derivative of the path at the
         * current position in the path
         *
         * @return the double derived point at the current position in the path.
         */
        math::Q getXdd() const {
            return _currIIterator.getXdd();
        }
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
