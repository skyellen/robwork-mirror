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

#ifndef rw_kinematics_FramePropertyImpl_HPP
#define rw_kinematics_FramePropertyImpl_HPP

/**
 * @file FramePropertyImpl.hpp
 */

#include "FrameProperty.hpp"
#include "Frame.hpp"
#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <boost/shared_ptr.hpp>

namespace rw { namespace kinematics {

    /**
     * @brief An implementation of the FrameProperty class.
     */
    template <typename T>
    class FramePropertyImpl : public FrameProperty<T>
    {
    public:
        /**
         * @brief An accessor for a named property of a frame.
         *
         * @param key [in] The name of property to access.
         *
         * @param description [in] The description of the property.
         */
        FramePropertyImpl(const std::string& key, const std::string& description) :
            _key(key),
            _description(description)
        {}

        /**
         * @brief The name of the property that is being accessed.
         *
         * @return The name of the property.
         */
        const std::string& key() const { return _key; }

        /**
         * @brief The value of the property of a frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @return A pointer to the value of the property of the frame or NULL
         * if the property does not exist or if the value of the property is of
         * a wrong type.
         */
        T* getPtr(Frame& frame) const
        {
            return frame.getPropertyMap().template getPtr<T>(_key);
        }

        /**
         * @brief The value of the property of a frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @return A pointer to the value of the property of the frame or NULL
         * if the property does not exist or if the value of the property is of
         * a wrong type.
         */
        const T* getPtr(const Frame& frame) const
        {
            // Forward to non-const version.
            return getPtr(const_cast<Frame&>(frame));
        }

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
        bool has(const Frame& frame) const
        {
            return this->getPtr(frame) != NULL;
        }

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
         * The property must be present in the frame or the program will throw.
         * If you don't like that policy you should use getPtr().
         */
        const T& get(const Frame& frame) const
        {
            // Forward to the non-const version
            return get(const_cast<Frame&>(frame));
        }

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
         * The property must be present in the frame or the program will throw.
         * If you don't like that policy you should use getPtr().
         */
        T& get(Frame& frame) const
        {
            T* value = this->getPtr(frame);
            if (!value) {
                // Perhaps this error message should include the description of
                // the property also.
                RW_THROW(
                    "No key (of some type T) named "
                    << rw::common::StringUtil::quote(key())
                    << " in frame "
                    << rw::common::StringUtil::quote(frame.getName()));
            }
            return *value;
        }

        /**
         * @brief Assign a value to the property of the frame.
         *
         * @param frame [in] A frame containing properties.
         *
         * @param value [in] The value to assign to the property.
         */
        void set(Frame& frame, const T& value) const
        {
            frame.getPropertyMap().set(_key, value);
        }

        /**
           @brief Erase the property of the frame.
         */
        void erase(Frame& frame) const
        {
            frame.getPropertyMap().erase(_key);
        }

    private:
        std::string _key;
        std::string _description;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
