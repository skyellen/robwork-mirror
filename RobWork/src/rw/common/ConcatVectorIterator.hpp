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


#ifndef RW_COMMON_CONCATVECTORITERATOR_HPP
#define RW_COMMON_CONCATVECTORITERATOR_HPP

/**
 * @file ConcatVectorIterator.hpp
 */

#include <vector>
#include <iterator>
#include <cstddef>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    template <typename T>
    class ConstConcatVectorIterator;

    /**
     * @brief Forward iterator for the concatenation of a pair of vectors of
     * pointers to T
     */
    template <typename T>
    class ConcatVectorIterator
    {
        typedef std::vector<T*> PtrTVector;
        typedef typename PtrTVector::const_iterator I;

        const PtrTVector* curr;
        I pos;
        const PtrTVector* next;

        friend class ConstConcatVectorIterator<T>;

    public:
        /** Iterator category. */
        typedef std::forward_iterator_tag iterator_category;

        /** Value type. */
        typedef T value_type;

        /** Pointer type. */
        typedef T* pointer;

        /** Reference type. */
        typedef T& reference;

        /** Difference type. */
        typedef ptrdiff_t difference_type;

        /**
           @brief Constructor

           The iterator \b pos points into \b curr. When the iterator reaches
           the end of \b curr it then switches to the start of \b next.

           Given a pair of vectors \b curr and \b next, the start and end
           iterator for the concatenated sequence are given by
\code
const ConcatVectorIterator<T> begin(curr, curr->begin(), next);
const ConcatVectorIterator<T> end(next, next->end(), 0);
\endcode

           You can use ConcatVectorIterator for iterating through a single
           sequence by letting \b next be NULL.
         */
        ConcatVectorIterator(const PtrTVector* curr, I pos, const PtrTVector* next) :
			curr(curr),
			pos(pos),
			next(next)
        {
            skipIfEndOfCurr();
        }

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
        ConcatVectorIterator& operator++()
        {
            inc();
            return *this;
        }

        /**
         * @brief Increments the position of the iterator
         * @return the ConcatVectorIterator with the value before the incrementation
         */
        ConcatVectorIterator operator++(int)
        {
            ConcatVectorIterator before = *this;
            inc();
            return before;
        }

        /**
         * @brief Tests whether the positions of two iterators are equal
         * @param other [in] ConcatVectorIterator to compare with
         * @return true if equal
         */
        bool operator==(const ConcatVectorIterator& other) const
        {
            return
                curr == other.curr &&
                pos == other.pos &&
                next == other.next;
        }

        /**
         * @brief Tests whether the positions of two iterators are unequal
         * @param other [in] ConcatVectorIterator to compare with
         * @return true if unequal
         */
        bool operator!=(const ConcatVectorIterator& other) const
        {

			// If only comparing pos and other.pos Visual Studio will generate
			// an error message when pos and other.pos does not come from the
			// same vector. A test curr != other.curr has thus been added
			return this->curr != other.curr || this->pos != other.pos;
		}

    private:
        void inc()
        {
            ++pos;
            skipIfEndOfCurr();
        }

        void skipIfEndOfCurr()
        {
            if (next && pos == curr->end()) {
                pos = next->begin();
                curr = next;
                next = 0;
            }
        }
    };

    /**
     * @brief Forward iterator for the concatenation of a pair of vectors of
     * pointers to T
     */
    template <typename T>
    class ConstConcatVectorIterator
    {
    public:
        /** Iterator category. */
        typedef std::forward_iterator_tag iterator_category;

        /** Value type. */
        typedef T const value_type;

        /** Pointer type. */
        typedef T const* pointer;

        /** Reference type. */
        typedef T const& reference;

        /** Difference type. */
        typedef ptrdiff_t difference_type;

        /**
         * @brief Constructor and implicit conversion from iterators.
         *
         * All ConstConcatVectorIterator are constructed via
         * ConcatVectorIterator values or copy construction.
         */
        ConstConcatVectorIterator(ConcatVectorIterator<T> pos) :
            pos(pos)
        {}

        /**
         * @brief Reference to the T element
         */
        const T& operator*() const { return *pos; }

        /**
         * @brief Pointer to the T element
         */
        const T* operator->() const { return pos.operator->(); }

        /**
         * @brief Increments the position of the iterator
         * @return Reference to the incremented iterator
         */
        ConstConcatVectorIterator& operator++() { ++pos; return *this; }

        /**
         * @brief Increments the position of the iterator
         * @return the ConstConcatVectorIterator with the value before the incrementation
         */
        ConstConcatVectorIterator operator++(int)
        { return ConstConcatVectorIterator(pos++); }

        /**
         * @brief Tests whether the positions of two iterators are equal
         * @param other [in] ConstConcatVectorIterator to compare with
         * @return true if equal
         */
        bool operator==(const ConstConcatVectorIterator& other) const
        { return pos == other.pos; }

        /**
         * @brief Tests whether the positions of two iterators are unequal
         * @param other [in] ConstConcatVectorIterator to compare with
         * @return true if unequal
         */
        bool operator!=(const ConstConcatVectorIterator& other) const
        { 			
			return pos != other.pos; 
		}

    private:
        // We simply forward to the non-const iterator.
        ConcatVectorIterator<T> pos;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
