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
     *  @brief Interface for functions
     */
	template<class RES_T = double, class ARG_T = double>
    class Function
    {
    public:
		//! Smart pointer to this type of class.
		typedef rw::common::Ptr<Function> Ptr;

	public:
		/**
		 * @brief Returns function value for arguments q.
		 */
		virtual RES_T f(ARG_T q) = 0;
		
		/**
		 * @brief Wraps the evaluation of x() with operator().
		 */
		RES_T operator()(ARG_T q) {
			return f(q);
		}
    };

    /**
     * @brief Interface for functions which are 1 time differentiable
     */
	template<class RES_T = double, class ARG_T = double, class GRAD_T = double>
	class Function1Diff: virtual public Function<RES_T, ARG_T>
	{
	public:
		//! Smart pointer to this type of class.
		typedef rw::common::Ptr<Function1Diff> Ptr;
		
	public:
		/**
		 * @brief Returns gradient(derivative) of the function
		 */
		virtual GRAD_T df(ARG_T q) = 0;
	};
	

    /*@}*/
}} // end namespaces

#endif // end include guard
