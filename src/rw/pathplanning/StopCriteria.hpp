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
           @brief Destructor
        */
        virtual ~StopCriteria() {}

        /**
           @brief Stop the computation after \b time seconds from now.

           RobWork does not link with a thread library, and therefore this
           function is implemented by polling a timer. This makes the function
           relatively slow for its purpose.
        */
        static std::auto_ptr<StopCriteria> stopAfter(double time);

        /**
           @brief Never stop the computation.
        */
        static std::auto_ptr<StopCriteria> stopNever();

        /**
           @brief Immediately stop the computation.
        */
        static std::auto_ptr<StopCriteria> stopNow();

        /**
           @brief Stop the computation when \b stop says so.

           \b stop must be non-null.

           Ownership of \b stop is not taken.
        */
        static std::auto_ptr<StopCriteria> stopByFlag(bool* stop);

        /**
           @brief Stop the computation when \b fun says so.
        */
        static std::auto_ptr<StopCriteria> stopByFun(boost::function<bool ()> fun);

        /**
           @brief Stop the computation after \b cnt calls of the stop criteria.
        */
        static std::auto_ptr<StopCriteria> stopCnt(int cnt);

    protected:
        //! Constructor
        StopCriteria() {}

        /**
           @brief Subclass implementation of the stop() method.
        */
        virtual bool doStop() const = 0;

    private:
        StopCriteria(const StopCriteria&);
        StopCriteria& operator=(const StopCriteria&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
