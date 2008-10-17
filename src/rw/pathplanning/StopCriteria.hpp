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

#ifndef rw_pathplanning_StopCriteria_hpp
#define rw_pathplanning_StopCriteria_hpp

/**
   @file StopCriteria.hpp
*/

#include <boost/function.hpp>
#include <rw/common/Ptr.hpp>
#include <vector>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    class StopCriteria;

    //! A pointer to a StopCriteria.
    typedef rw::common::Ptr<StopCriteria> StopCriteriaPtr;

    /**
       @brief StopCriteria is a class for specifying an instant a compution
       should be aborted.

       The computation determines when to stop by repeatedly polling the
       StopCriteria::stop() method. Therefore the stop() method should be
       implemented to have a very short, preferably deterministic running time.
    */
    class StopCriteria
    {
    public:
        /**
           @brief True is returned when the computation should be stopped.
        */
        bool stop() const;

        /**
           @brief A new instance of the property constructed to match the
           original initial state of the criteria.

           This implies, for example, that instance() called for a time criteria
           creates a new criteria that stops after the same amount of time that
           was specified for the original stop criteria.

           Not all stop criteria returned are required to behave this way. For
           some types of stop criteria, the instances of the stop criteria will
           be effectively identical to the stop criteria itself.
        */
        StopCriteriaPtr instance() const;

        /**
           @brief Destructor
        */
        virtual ~StopCriteria() {}

        /**
           @brief Stop the computation after \b time seconds from now.

           RobWork does not link with a thread library, and therefore this
           function is implemented by polling a timer. This makes the function
           relatively slow for its purpose.
        */
        static StopCriteriaPtr stopAfter(double time);

        /**
           @brief Never stop the computation.
        */
        static StopCriteriaPtr stopNever();

        /**
           @brief Immediately stop the computation.
        */
        static StopCriteriaPtr stopNow();

        /**
           @brief Stop the computation when \b stop says so.

           \b stop must be non-null.

           Ownership of \b stop is not taken.
        */
        static StopCriteriaPtr stopByFlag(bool* stop);

        /**
           @brief Stop the computation when \b fun says so.
        */
        static StopCriteriaPtr stopByFun(boost::function<bool ()> fun);

        /**
           @brief Stop the computation after \b cnt calls of the stop criteria.
        */
        static StopCriteriaPtr stopCnt(int cnt);

        /**
           @brief Stop if either of \b criteria says stop.
        */
        static StopCriteriaPtr stopEither(
            const std::vector<StopCriteriaPtr>& criteria);

        /**
           @brief Stop if either \b a or \b b says stop.
        */
        static StopCriteriaPtr stopEither(
            const StopCriteriaPtr& a,
            const StopCriteriaPtr& b);

    protected:
        //! Constructor
        StopCriteria() {}

        /**
           @brief Subclass implementation of the stop() method.
        */
        virtual bool doStop() const = 0;

        /**
           @brief Subclass implementation of the instance() method.
         */
        virtual StopCriteriaPtr doInstance() const = 0;

    private:
        StopCriteria(const StopCriteria&);
        StopCriteria& operator=(const StopCriteria&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
