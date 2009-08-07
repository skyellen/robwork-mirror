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


#ifndef RW_COMMON_VECTORITERATOR_HPP
#define RW_COMMON_VECTORITERATOR_HPP

/**
 * @file VectorIterator.hpp
 */

#include <vector>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    template <typename T>
    class ConstVectorIterator;

    /**
     * @brief Forward iterator for vectors of pointers to T
     */
    template <typename T>
    class VectorIterator
    {
        typedef std::vector<T*> PtrTVector;
        typedef typename PtrTVector::const_iterator I;
        I pos;

    public:
        /** Iterator category. */
        typedef typename I::iterator_category iterator_category;

        /** Value type. */
        typedef T value_type;

        /** Pointer type. */
        typedef T* pointer;

        /** Reference type. */
        typedef T& reference;

        /** Difference type. */
        typedef typename I::difference_type difference_type;

        /**
           @brief Iterator for the element at \b pos.
         */
        explicit VectorIterator(I pos) : pos(pos) {}

        /**
         * @brief Reference to the T element
         */
        T& operator*() const { return **pos; }

        /**
         * @brief Pointer to the T element
         */
        T* operator->() const { return *pos.operator->(); }

        /**
         * @brief Increments the position of the iterator
         * @return Reference to the incremented iterator
         */
        VectorIterator& operator++() { ++pos; return *this; }

        /**
         * @brief Increments the position of the iterator
         * @return the VectorIterator with the value before the incrementation
         */
        VectorIterator operator++(int) { return VectorIterator(pos++); }

        /**
         * @brief Tests whether the positions of two iterators are equal
         *
         * @param other [in] VectorIterator to compare with
         *
         * @return true if equal
         */
        bool operator==(const VectorIterator& other) const
        { return pos == other.pos; }

        /**
         * @brief Tests whether the positions of two iterators are unequal
         *
         * @param other [in] VectorIterator to compare with
         *
         * @return true if unequal
         */
        bool operator!=(const VectorIterator& other) const
        { return pos != other.pos; }

        friend class ConstVectorIterator<T>;
    };

    /**
     * @brief Forward iterator for vectors of pointers to const T
     */
    template <typename T>
    class ConstVectorIterator
    {
        typedef std::vector<T*> PtrTVector;
        typedef typename PtrTVector::const_iterator I;
        I pos;

    public:
        /** Iterator category. */
        typedef typename I::iterator_category iterator_category;

        /** Value type. */
        typedef T const value_type;

        /** Pointer type. */
        typedef T const* pointer;

        /** Reference type. */
        typedef T const& reference;

        /** Difference type. */
        typedef typename I::difference_type difference_type;

        /**
           @brief Iterator for the element at \b pos.
         */
        explicit ConstVectorIterator(I pos) : pos(pos) {}

        /**
         * @brief Reference to the T element
         */
        const T& operator*() const { return **pos; }

        /**
         * @brief Pointer to the T element
         */
        const T* operator->() const { return *pos.operator->(); }

        /**
         * @brief Increments the position of the iterator
         *
         * @return Reference to the incremented iterator
         */
        ConstVectorIterator& operator++() { ++pos; return *this; }

        /**
         * @brief Increments the position of the iterator
         *
         * @return the VectorIterator with the value before the incrementation
         */
        ConstVectorIterator operator++(int) { return ConstVectorIterator(pos++); }

        /**
         * @brief Tests whether the positions of two iterators are equal
         *
         * @param other [in] VectorIterator to compare with
         *
         * @return true if equal
         */
        bool operator==(const ConstVectorIterator& other) const
        { return pos == other.pos; }

        /**
         * @brief Tests whether the positions of two iterators are unequal
         *
         * @param other [in] VectorIterator to compare with
         *
         * @return true if unequal
         */
        bool operator!=(const ConstVectorIterator& other) const
        { return pos != other.pos; }

        /**
         * @brief Implicit conversion from iterators.
         */
        ConstVectorIterator(VectorIterator<T> pos) : pos(pos.pos) {}
    };

    /* @} */
}} // end namespaces

#endif // end include guard
