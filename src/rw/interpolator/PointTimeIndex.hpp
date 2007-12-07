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

#ifndef RW_INTERPOLATOR_POINTTIMEINDEX_HPP
#define RW_INTERPOLATOR_POINTTIMEINDEX_HPP

/**
   @file PointTimeIndex.hpp
   @brief Class rw::interpolator::PointTimeIndex.
*/

#include <vector>

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
     * @brief Lookup by time for a sequence of time step values.
     *
     *  Suppose you are given a list of points 
     * \verbatim
     *    p0, p1, p2, ..., p(N).
     * \endverbatim
     *  and that each point is positioned in time at
     * \verbatim
     *    t0, t1, t2, ..., t(N)
     * \endverbatim
     * Given a time \b t you wish to find the index \b i in the list such that 
     * \verbatim
     *   p(i) <= t <= p(i + 1)
     * \endverbatim
     *  The PointTimeIndex helps you do that.
     *
     *  The input for PointTimeIndex is the sequence of time steps
     * \verbatim
     *   t1 - t0, t2 - t1, t3 - t2, ..., t(N) - t(N - 1)
     * \endverbatim
     * To find the index for time \b t you lookup in your PointTimeIndex by the value
     * \verbatim
     *   t - t0
     * \endverbatim
     *  The lookup runs in time O(log N) for large N and O(N) for small N.
     */
    class PointTimeIndex
    {
    public:
        /**
         * @brief Index for a sequence of positive time steps \b timeSteps.
         *
         * The time steps can be calculated as
         * \verbatim
         *   t1 - t0, t2 - t1, t3 - t2, ..., t(N) - t(N - 1)
         * \endverbatim
         * where
         * \verbatim
         *   t0, t1, t2, t3, ..., t(N)
         * \endverbatim
         * is a growing sequence of numbers related to some events (called
         * points in this documentation). It is OK for a time step to be zero,
         *  but it can not be negative.
         */
        PointTimeIndex(const std::vector<double>& timeSteps);

        /**
         * @brief The index in point list as well as the start and end times 
         * for that segment.
         */
        struct Key
        {
            /** Start */
            double _start;
            /** End */
            double _end;
            /** Index */
            int _index;

            /**
             * @brief Constructor
             * @param start [in] start value
             * @param end [in] end value
             * @param index [in] index
             */
            Key(double start, double end, int index) :
                _start(start), _end(end), _index(index)
            {}
        };

        /**
         * @brief The index of the \e first element of the point times that is
         * ahead in time of \b time.
         *
         * If \b time is less than zero or greater than the position in time of
         * the last point, then -1 is returned.
         *
         * @param time [in] time for which to get the key
         * @return the key
         */
        Key getKey(double time);

        /**
         * @brief The position in time of the last point.
         * @return the end time
         */
        double getEndTime() const;

        /**
         * @brief The time steps given as input.
         * @return vector with time step values
         */
        std::vector<double>& getTimeSteps() const;

        /**
         * Destructor.
         */
        ~PointTimeIndex();

    private:
        struct Impl;
        Impl* impl;

    private:
        PointTimeIndex(const PointTimeIndex&);
        PointTimeIndex& operator=(const PointTimeIndex&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
