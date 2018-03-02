/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef RW_PATHPLANNING_STOPCRITERIA_HPP
#define RW_PATHPLANNING_STOPCRITERIA_HPP

/**
   @file StopCriteria.hpp
*/

#include <boost/function.hpp>
#include <rw/common/Ptr.hpp>
#include <vector>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/
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
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<StopCriteria> Ptr;

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
		StopCriteria::Ptr instance() const;

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
		static StopCriteria::Ptr stopAfter(double time);

        /**
           @brief Never stop the computation.
        */
		static StopCriteria::Ptr stopNever();

        /**
           @brief Immediately stop the computation.
        */
		static StopCriteria::Ptr stopNow();

        /**
           @brief Stop the computation when \b stop says so.

           \b stop must be non-null.

           Ownership of \b stop is not taken.
        */
		static StopCriteria::Ptr stopByFlag(bool* stop);

        /**
           @brief Stop the computation when \b fun says so.
        */
		static StopCriteria::Ptr stopByFun(boost::function<bool ()> fun);

        /**
           @brief Stop the computation after \b cnt calls of the stop criteria.
        */
		static StopCriteria::Ptr stopCnt(int cnt);

        /**
           @brief Stop if either of \b criteria says stop.
        */
		static StopCriteria::Ptr stopEither(
			const std::vector<StopCriteria::Ptr>& criteria);

        /**
           @brief Stop if either \b a or \b b says stop.
        */
		static StopCriteria::Ptr stopEither(
			const StopCriteria::Ptr& a,
			const StopCriteria::Ptr& b);

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
		virtual StopCriteria::Ptr doInstance() const = 0;

    private:
        StopCriteria(const StopCriteria&);
        StopCriteria& operator=(const StopCriteria&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
