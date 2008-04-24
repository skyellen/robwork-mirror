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

#ifndef rw_kinematics_FrameProperty_HPP
#define rw_kinematics_FrameProperty_HPP

/**
 * @file FrameProperty.hpp
 */

#include <rw/common/Property.hpp>

#include <cassert>

namespace rw { namespace kinematics {
    class Frame;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Interface for utilities for accessing a named property of a frame.
     *
     * FrameProperty is a utility for accessing the property of a frame as
     * painlessly as possible. The class may therefore happen to have a few
     * superflous methods that are abbreviations for combinations of the other
     * methods.
     *
     * FrameProperty hides the low-level manipulations of PropertyMap values
     * of frames.
     *
     * FrameProperty throws an exception to give good defaults for the common
     * case where your program has no option but to crash if some option is not
     * present.
     */
    template <typename T>
    class FrameProperty
    {
    public:
        /**
         * @brief The name of the property that is being accessed.
         *
         * @return The name of the property.
         */
        virtual const std::string& key() const = 0;

        /**
         * @brief The value of the property of a frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @return A pointer to the value of the property of the frame or NULL
         * if the property does not exist or if the value of the property is of
         * a wrong type.
         */
        virtual T* getPtr(Frame& frame) const = 0;

        /**
         * @copydoc getPtr
         */
        virtual const T* getPtr(const Frame& frame) const = 0;

        /**
         * @brief True iff the frame has the property.
         *
         * A call of the method is equivalent to
\code
getPtr(frame) != 0
\endcode
         *
         * @param frame [in] A frame containing properties.
         *
         * @return \b true iff the property exists in \b frame.
         */
        virtual bool has(const Frame& frame) const = 0;

        /**
         * @brief The value of the property of a frame.
         *
         * The method is superflous as it can be easily implemented in terms of
         * getPtr().
         *
         * @param frame [in] A frame containing properties.
         *
         * @return The value of the property of the frame.
         *
         * The property must be present in the frame or the program will throw. If
         * you don't like that policy you should use getPtr().
         */
        virtual const T& get(const Frame& frame) const = 0;

        /**
         * @brief The value of the property of a frame.
         *
         * The method is superflous as it can be easily implemented in terms of
         * getPtr().
         *
         * @param frame [in] A frame containing properties.
         *
         * @return The value of the property of the frame.
         *
         * The property must be present in the frame or the program will throw. If
         * you don't like that policy you should use getPtr().
         */
        virtual T& get(Frame& frame) const = 0;

        /**
         * @brief Assign a value to the property of the frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @param value [in] The value to assign to the property.
         *
         * [NB: We haven't quite decided yet what to do if a value is already
         * present (we could have single-assignment) or if the value present is
         * of a wrong type.]
         */
        virtual void set(Frame& frame, const T& value) const = 0;

        /**
         * @brief Remove a property of a frame.
         *
         * Following a call of erase() the has() will return false.
         *
         * If erase() is called on a frame that does not have the property, then
         * nothing is done.
         *
         * The property is removed also if its type does not match.
         *
         * @param frame [in] the frame from which to remove the property.
         */
        virtual void erase(Frame& frame) const = 0;

        /**
         * @brief Virtual destructor.
         */
        virtual ~FrameProperty() {}

    protected:
        FrameProperty() {}

    private:
        FrameProperty(const FrameProperty&);
        FrameProperty& operator=(const FrameProperty&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
