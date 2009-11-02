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


#ifndef RW_COMMON_PTR_HPP
#define RW_COMMON_PTR_HPP

/**
   @file Ptr.hpp
*/

#include <memory>
#include <boost/shared_ptr.hpp>

namespace rw { namespace common {

	/** @addtogroup common */
	/*@{*/

    /**
       @brief Ptr stores a pointer and optionally takes ownership of the value.
    */
    template <class T>
    class Ptr
    {
    public:
        //! The internal type of shared pointer used.
        typedef boost::shared_ptr<T> shared_ptr;

        //! Pointer type
        typedef T* pointer;

        //! Reference type
        typedef T& reference;

        //! Value type
        typedef T value_type;

        /**
           @brief Default constructor yielding a NULL-pointer.
        */
        Ptr() :
            _ptr(0),
            _owned_ptr()
        {}

        /**
           @brief Do not take ownership of \b ptr.

           \b ptr can be null.

           The constructor is implicit on purpose.
        */
        Ptr(T* ptr) :
            _ptr(ptr),
            _owned_ptr()
        {}



        /**
           @brief Take ownership of \b ptr.

           \b ptr can be null.

           The constructor is implicit on purpose.
        */
        Ptr(shared_ptr ptr) :
            _ptr(ptr.get()),
            _owned_ptr(ptr)
        {}

        /**
           @brief Take ownership of \b ptr.

           \b ptr can be null.

           The constructor is implicit on purpose.
        */
        Ptr(std::auto_ptr<T> ptr) :
            _ptr(ptr.get()),
            _owned_ptr(ptr.release())
        {}


        template <class S>
        Ptr<S> cast() {
            if (_owned_ptr)
                return Ptr<S>(boost::dynamic_pointer_cast<S>(_owned_ptr));
            else
                return Ptr<S>(dynamic_cast<S*>(_ptr));
        }

        /**
         *  @brief Implicit conversion to a superclass of a ptr.
         */
        template <class S>
        operator Ptr<S> ()
        {
            if (_owned_ptr)
                return Ptr<S>(_owned_ptr);
            else
                return Ptr<S>(_ptr);
        }


        /**
           @brief The pointer stored in the object.
        */
        pointer get() const { return _ptr; }

        /**
           @brief Dereferencing operator.
        */
        reference operator*() const { return *get(); }

        /**
           @brief Member access operator.
        */
        pointer operator->() const { return get(); }

        /**
           @brief Support for implicit conversion to bool.
        */
        operator void* () const { return get(); }

    private:
        T* _ptr;
        boost::shared_ptr<T> _owned_ptr;
    };

    /**
       @brief A Ptr that takes ownership over a raw pointer \b ptr.

       @relates Ptr
    */
    template <class T>
    Ptr<T> ownedPtr(T* ptr) { return Ptr<T>(std::auto_ptr<T>(ptr)); }

	/*@}*/
}} // end namespaces

#endif // end include guard
