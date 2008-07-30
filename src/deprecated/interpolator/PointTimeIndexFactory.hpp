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

#ifndef RW_INTERPOLATOR_POINTTIMEINDEXFACTORY_HPP
#define RW_INTERPOLATOR_POINTTIMEINDEXFACTORY_HPP

#include "Timed.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>

#include <vector>
#include <memory>

namespace rw { namespace kinematics {
    class State;
}}
namespace rw { namespace models {
    class WorkCell;
}}

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    class PointTimeIndex;

    /**
       @brief PointTimeIndex constructors
    */
    class PointTimeIndexFactory
    {
    public:
        /**
           @brief Index for the straight line path \b path that is linearly
           traversed with maximum speeds \b speed.

           The path must contain at least two configurations.

           The speed must be of the same dimension as the path configurations.

           The index returns values between 0 and N - 1 where N is the number of the
           configurations of the path. The index i is returned for time positions in
           between elements i and i + 1 of the path.

           The path is assumed to start at time 0.
        */
        static
        std::auto_ptr<PointTimeIndex> make(
            const math::Q& speed,
            const std::vector<math::Q>& path);

        /**
           @brief Index for the straight line path \b path that is linearly
           traversed with the maximum speeds for the work cell \b workcell.

           The path must contain at least two configurations.

           The index returns values between 0 and N - 1 where N is the number of the
           configurations of the path. The index i is returned for time positions in
           between elements i and i + 1 of the path.

           The path is assumed to start at time 0.
        */
        static
        std::auto_ptr<PointTimeIndex> make(
            const models::WorkCell& workcell,
            const std::vector<kinematics::State>& path);

        /**
           @brief Index for the path \b path with the provided time stamps.

           The sequence of time stamp values must be increasing.
        */
        template <class T>
        static
        std::auto_ptr<PointTimeIndex> make(const std::vector<Timed<T> >& path)
        {
            RW_ASSERT(path.size() >= 2);

            typedef typename std::vector<Timed<T> >::const_iterator I;

            std::vector<double> timeSteps;
            I p = path.begin();
            double prev = p->getTime();
            for (++p; p != path.end(); ++p) {
                const double now = p->getTime();
                const double step = now - prev;
                if (step < 0)
                    RW_THROW(
                        "Negative step in timed path."
                        " A step from time " << prev <<
                        " to time " << now << " was attempted.");

                timeSteps.push_back(step);

                prev = now;
            }

            return makeIndex(timeSteps);
        }

    private:
        static
        std::auto_ptr<PointTimeIndex> makeIndex(const std::vector<double>& timeSteps);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
