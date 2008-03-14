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

#ifndef rw_common_Property_HPP
#define rw_common_Property_HPP

/**
 * @file Property.hpp
 */

#include <iostream>
#include "PropertyBase.hpp"

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Property class
     * The Property class is a template to support properties of any type. A Property
     * is characterized by a string identifier, string description and a value of the
     * template specified type.
     */
    template<class T>
    class Property: public PropertyBase
    {
    public:
        /**
         * @brief Constructs Property.
         *
         * Constructs a Property and tries to auto detect the type.
         *
         * @param identifier [in] identifier
         * @param description [in] description
         * @param value [in] value
         */
        Property(
            const std::string& identifier,
            const std::string& description,
            T value)
            :
            PropertyBase(identifier, description, PropertyType::getType(value)),
            _value(value)
        {}

        /**
         * @brief Constructs Property.
         * @param identifier [in] identifier
         * @param description [in] description
         * @param type [in] type of property
         * @param value [in] value
         */
        Property(
            const std::string& identifier,
            const std::string& description,
            const PropertyType& type,
            T value)
            :
            PropertyBase(identifier, description, type),
            _value(value)
        {}

        /**
         * @brief Destroys Property
         * If the property value is a pointer, the object pointed to will NOT be destroyed.
         */
        virtual ~Property() {};

        /**
         * @brief returns reference to the property value
         * @return value
         */
        T& getValue() {
            return _value;
        }

        /**
         * @brief returns const reference to the property value
         * @return value
         */
        const T& getValue() const {
            return _value;
        }

        /**
         * @brief Sets the property value
         * @param value [in] the new value of the Property
         */
        void setValue(const T& value) {
            _value = value;
        }

        /**
           @copydoc clone
        */
        Property<T>* clone() const
        {
            return new Property<T>(
                this->getIdentifier(),
                this->getDescription(),
                this->getType(),
                this->_value);
        }

    private:
        T _value;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
