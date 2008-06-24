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

#ifndef rw_pathplanning_StateConstraint_hpp
#define rw_pathplanning_StateConstraint_hpp

/**
   @file StateConstraint.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class StateConstraint;

    //! A pointer to a StateConstraint.
    typedef rw::common::Ptr<StateConstraint> StateConstraintPtr;

    /**
       @brief Interface for the checking for collisions for work cell states.
    */
    class StateConstraint
    {
    public:
        /**
           @brief True if the work cell is considered to be in collision for the
           work cell state \b state.
         */
        virtual bool inCollision(const rw::kinematics::State& state) const = 0;

        /**
           Destructor
        */
        virtual ~StateConstraint() {}

        // Factory functions follow below.

        /**
           @brief Map a collision detector to a state constraint.
        */
        static std::auto_ptr<StateConstraint> make(
            rw::proximity::CollisionDetectorPtr detector);

        /**
           @brief Combine a set of state constraints into a single state
           constraint.
        */
        static std::auto_ptr<StateConstraint> make(
            const std::vector<StateConstraintPtr>& constraints);

    protected:
        /**
           Constructor
        */
        StateConstraint() {}

    private:
        StateConstraint(const StateConstraint&);
        StateConstraint& operator=(const StateConstraint&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
