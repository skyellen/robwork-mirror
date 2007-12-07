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

#ifndef RW_ROBP_STATETRAJECTORY_HPP
#define RW_ROBP_STATETRAJECTORY_HPP

#include <vector>

namespace rw { namespace kinematics {
    class State;
}}

namespace rw { namespace interpolator {

    /** @addtogroup interpolator */
    /*@{*/

    /**
       @brief A trajectory of configurations.

       A trajectory is a mapping from the time interval [0, end] into the
       space of work cell states.

       The mapping won't necessarily be continuous. For example, the state
       trajectory may change the attachment of DAFs.
    */
    class StateTrajectory
    {
    public:
        /**
           @brief The configuration for time \b time.

           An exception is thrown if \b time is out of range.
        */
        virtual kinematics::State get(double time) const = 0;

        /**
           @brief The end time of the mapped time interval.

           The start time is zero always.
        */
        virtual double getEndTime() const = 0;

        /**
           @brief Destructor.
        */
        virtual ~StateTrajectory() {}

    protected:
        /**
           @brief Constructor.
         */
        StateTrajectory() {}

    private:
        StateTrajectory(const StateTrajectory&);
        StateTrajectory& operator=(const StateTrajectory&);
    };

    /*@}*/
}} // end namespaces


#endif // end include guard
