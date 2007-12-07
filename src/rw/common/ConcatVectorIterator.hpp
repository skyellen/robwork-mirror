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

#ifndef rw_common_ConcatVectorIterator_HPP
#define rw_common_ConcatVectorIterator_HPP

/**
 * @file ConcatVectorIterator.hpp
 */

#include <vector>

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
			//If only comparing pos and other.pos Visual Studio will generate an error message 
			//when pos and other.pos does not come from the same vector. A test curr != other.curr
			//has thus been added
			return this->curr != other.curr || pos != other.pos; 
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
        { return pos != other.pos; }

    private:
        // We simply forward to the non-const iterator.
        ConcatVectorIterator<T> pos;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
