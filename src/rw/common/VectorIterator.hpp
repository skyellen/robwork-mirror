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

#ifndef rw_common_VectorIterator_HPP
#define rw_common_VectorIterator_HPP

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
         * @param other [in] VectorIterator to compare with
         * @return true if equal
         */
        bool operator==(const VectorIterator& other) const
        { return pos == other.pos; }

        /**
         * @brief Tests whether the positions of two iterators are unequal
         * @param other [in] VectorIterator to compare with
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
