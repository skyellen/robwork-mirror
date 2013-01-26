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


#ifndef RW_MATH_FUNCTION_HPP
#define RW_MATH_FUNCTION_HPP

/**
 * @file rw/math/Math.hpp
 */



namespace rw { namespace math {

    /** @addtogroup math */
    /*@{*/

    /**
       @brief Interfaces for 1 dimensional function
    */
	template<class T = double>
    class Function
    {
    public:
		typedef rw::common::Ptr<Function> Ptr;

		virtual T x(T q) = 0;
    };

    /**
       @brief Interfaces for 1 dimensional function which are 1 time differentiable
    */
	template <class T = double>
	class Function1Diff: public Function<T>
	{
	public:
		typedef rw::common::Ptr<Function1Diff> Ptr;
		virtual T dx(T q) = 0;
	};

    /*@}*/
}} // end namespaces

#endif // end include guard
