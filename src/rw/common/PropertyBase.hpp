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

#ifndef rw_common_PropertyBase_HPP
#define rw_common_PropertyBase_HPP

/**
 * @file PropertyBase.hpp
 */

#include "PropertyType.hpp"

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <string>
#include <vector>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Base class for Property handling
     */
    class PropertyBase
    {
    public:
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
        PropertyBase(
            const std::string& identifier, 
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
           @brief Construct a clone of the property.
        */
        virtual PropertyBase* clone() const = 0;

        /**
         * @brief Method signature for a callback function
         */
        typedef boost::function<void(PropertyBase*)> PropertyChangedListener;

        /**
         * @brief Add listener to be call, when the property changes
         * @param callback [in] Callback method
         */
        void addChangedListener(PropertyChangedListener callback);

        /**
         * @brief Returns the PropertyType
         * @return the PropertyType
         */
        const PropertyType& getType() const;

    protected:
        /**
         * @brief Notifies listeners about a change in the Property
         */
        void notifyListeners();

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

        /**
         * @brief PropertyChanged Listeners
         */
        std::vector<PropertyChangedListener> _listeners;

    private:
        PropertyBase(const PropertyBase&);
        PropertyBase& operator=(const PropertyBase&);
    };

    /** @} */

}} // end namespaces

#endif // end include guard
