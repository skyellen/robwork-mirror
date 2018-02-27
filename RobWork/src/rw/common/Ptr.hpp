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

//#include <memory>
#include <boost/shared_ptr.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_base_of.hpp>
//#include "Any.hpp"

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
         * @brief Cast the smart pointer to a different type.
         * @return smart pointer that can be null if cast was not possible.
         */
        template <class S>
        Ptr<S> cast() {
        	// this should test if we cast FROM an Any type
        	/*
        	if( ::boost::is_same<Any, T>::value ){
        		// The any type needs to be handled specially, get the real content of Any
        		if (_ptr!=NULL)
                    return Ptr<S>(boost::dynamic_pointer_cast<S>( ((Any*)_ptr)->content ));
                return Ptr<S>();
        	}

        	// this should test if we cast TO an Any type
        	if( ::boost::is_same<Any, S>::value ){
        		// The any type needs to be handled specially, get the real content of Any

        		//if (_owned_ptr)
                //    return ownedPtr<S>( new Any(_owned_ptr) );
                //if (_ptr!=NULL)
                //	return ownedPtr<S>( new Any(_ptr) );

                return Ptr<S>();
        	}
			*/

			if (_owned_ptr)
				return Ptr<S>(boost::dynamic_pointer_cast<S>(_owned_ptr));
			else
				return Ptr<S>(dynamic_cast<S*>(_ptr));
        }

        //! @copydoc cast()
		template <class S>
        Ptr<S> cast() const {
            if (_owned_ptr)
                return Ptr<S>(boost::dynamic_pointer_cast<S>(_owned_ptr));
            else
                return Ptr<S>(dynamic_cast<S*>(_ptr));
        }

		/**
		 * @brief Cast the smart pointer statically to a different type.
		 *
		 * This is more efficient if it is already known that the object is of a certain type.
		 * If this is not known, please use the more safe cast() instead.
		 *
         * @return smart pointer that can be null if cast was not possible.
		 */
        template <class S>
        Ptr<S> scast() {
			if (_owned_ptr)
				return Ptr<S>(boost::static_pointer_cast<S>(_owned_ptr));
			else
				return Ptr<S>(static_cast<S*>(_ptr));
        }

        //! @copydoc scast()
		template <class S>
        Ptr<S> scast() const {
            if (_owned_ptr)
                return Ptr<S>(boost::static_pointer_cast<S>(_owned_ptr));
            else
                return Ptr<S>(static_cast<S*>(_ptr));
        }
        /**
         *  @brief Implicit conversion to a superclass of a ptr.
         */
        /*
         * This conversion operator yields compile errors inside Ptr class when
         * setting A::Ptr s = B::Ptr(); where A inherit from B.
         * instead it should produce errors on the actual line.
        template <class S>
        operator Ptr<S> ()
        {
            if (_owned_ptr)
                return Ptr<S>(_owned_ptr);
            else
                return Ptr<S>(_ptr);
        }*/

        template <class S>
        Ptr<T>( Ptr<S> const& p, typename boost::enable_if
                                 < boost::is_base_of< T, S>
                                 >::type* = 0 )
        {
            _owned_ptr = p.getSharedPtr();
            _ptr = p.get();
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
        operator const void* () const { return get(); }

        /**
         * @brief equallity operator, this only tests if the pointers to the referenced objects are the same
         * and NOT if the smart pointers are the same.
         * @param p [in] smart pointer to compare with
         * @return true if the referenced objects are the same
         */
        template<class A>
        bool operator==(const Ptr<A>& p) const { return get()==p.get(); }

		/**
		 * @brief Tests if the smart pointer points to the same instance as \b p
		 */
        bool operator==(void* p) const { return get()==p; }

        /**
         * @brief check if this Ptr has shared ownership or none
         * ownership
         * @return true if Ptr has shared ownership, false if it has no ownership.
         */
        bool isShared(){
            if (_owned_ptr)
                return true;
            else
                return false;
        }

		/**
		 * @brief Returns true is the smart pointer is null
		 */
		bool isNull() const {
			return get() == NULL;
		}

		/**
		 * @brief Returns the boost shared pointer used internally
		 */
        boost::shared_ptr<T> getSharedPtr() const { return _owned_ptr; }



    private:
        T* _ptr;
        boost::shared_ptr<T> _owned_ptr;
    };

	/**
	 * @brief Comparator for comparing an ordinary pointer with a smart pointer
	 *
	 * @note If comparing two instances of a class without specifying the equal operator 
	 * this method might be called.
	 */
    template <class T, class R>
    bool operator==(void* p, const Ptr<R>& g) { return p==g.get(); }

    /**
       @brief A Ptr that takes ownership over a raw pointer \b ptr.

       @relates Ptr
    */
    template <class T>
    Ptr<T> ownedPtr(T* ptr) { return Ptr<T>(boost::shared_ptr<T>(ptr)); }

	/*@}*/
}} // end namespaces

#endif // end include guard
