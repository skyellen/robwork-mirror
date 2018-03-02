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


#ifndef RW_COMMON_PROPERTYBASE_HPP
#define RW_COMMON_PROPERTYBASE_HPP

/**
 * @file PropertyBase.hpp
 */

#include "PropertyType.hpp"
#include "Ptr.hpp"
#include "Event.hpp"

#include <boost/function.hpp>

#include <string>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Base class for Property handling
     */
    class PropertyBase
    {
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<PropertyBase> Ptr;

        /**
         * @brief Constructor
         *
         * @param identifier [in] identifier for the property
         * @param description [in] description of the property
         */
        PropertyBase(const std::string& identifier, const std::string& description);

        /**
         * @brief Constructor
         *
         * @param identifier [in] identifier for the property
         * @param description [in] description of the property
         * @param type [in] type of the property
         */
        PropertyBase(const std::string& identifier,
                     const std::string& description,
                     const PropertyType& type);

        /**
         * @brief Destroys PropertyBase
         */
        virtual ~PropertyBase();

        /**
         * @brief Returns the Property identifier
         * @return string identifier
         */
        const std::string& getIdentifier() const;

        /**
         * @brief Returns description
         * @return string description
         */
        const std::string& getDescription() const;

        /**
         * @brief set description
         */
        void setDescription(const std::string& desc){ _description = desc;}

        /**
           @brief Construct a clone of the property.
        */
        virtual PropertyBase* clone() const = 0;

        /**
         * @brief Method signature for a callback function
         */
        typedef boost::function<void(PropertyBase*)> PropertyListener;

        //! @brief Type for changed property events.
        typedef rw::common::Event<PropertyListener, PropertyBase*> ChangedEvent;

        /**
         * @brief get changed event
         *
         * to add listener use:
         * changedEvent().add(...)
         *
         */
        ChangedEvent& changedEvent() { 
			return _changedEvent; 
		}

        /**
         * @brief Returns the PropertyType
         * @return the PropertyType
         */
        const PropertyType& getType() const;

    private:
        /**
         * @brief Identifiers
         */
        std::string _identifier;

        /**
         * @brief Description
         */
        std::string _description;

        /**
         * @brief Type of property
         */
        PropertyType _propertyType;

        //! changed event handler
        ChangedEvent _changedEvent;
    private:
        PropertyBase(const PropertyBase&);
        PropertyBase& operator=(const PropertyBase&);
    };

    /** @} */
}} // end namespaces

#endif // end include guard
