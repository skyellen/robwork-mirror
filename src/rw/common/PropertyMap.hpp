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

#ifndef rw_common_PropertyMap_HPP
#define rw_common_PropertyMap_HPP

/**
 * @file PropertyMap.hpp
 */

#include "PropertyBase.hpp"
#include "Property.hpp"
#include "StringUtil.hpp"
#include "macros.hpp"

#include <map>
#include <boost/shared_ptr.hpp>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Container for a collection of Property Objects
     */
    class PropertyMap {
    public:
        /**
         * @brief Constructs PropertyMap
         */
        PropertyMap();

        /**
         * @brief Destroys PropertyMap.
         *
         * All properties in the bag is before the PropertyMap is destroyed
         * PropertyMap
         */
        ~PropertyMap();


        /**
         * @brief Sets value of property
         *     
         * If a property with the specified identifier cannot be found
         * the method throws an exception
         
         * @param identifier [in] the property identifier
         * @param value [in] the new value
         */
        template<class T>
        void setValue(const std::string& identifier, const T& value) {
            PropertyBase* prop = find(identifier);
            Property<T>* propT = dynamic_cast<Property<T>*>(prop);
            if (propT != NULL)
                propT->setValue(value);
            else {
                addProperty(boost::shared_ptr<PropertyBase>(new Property<T>(identifier, "", value)));
            }
            
        }
        
        /**
         * @brief Returns value of property
         *
         * If a property with the specified identifier cannot be found
         * the method throws an exception
         *
         * @param identifier [in] the identifier of the property
         * @return the value of the property
         */
        template<class T>
        T& getValue(const std::string& identifier) {
            PropertyBase* prop = find(identifier);
            Property<T>* propT = dynamic_cast<Property<T>*>(prop);
            if (propT != NULL)
                return propT->getValue();
            else
                RW_THROW("Property named "<<StringUtil::Quote(identifier)<<" could not be found");
            
        }
        
        /**
         * @copydoc getValue
         */
        template<class T>
        const T& getValue(const std::string& identifier) const {
            const PropertyBase* prop = find(identifier);
            const Property<T>* propT = dynamic_cast<const Property<T>*>(prop);
            if (propT != NULL)
                return propT->getValue();
            else
                RW_THROW("Property named "<<StringUtil::Quote(identifier)<<" could not be found");
            
        }
        
        
        /**
         * @brief Returns whether a specific property exists
         * @param identifier [in] the identifier of the property
         * @return true if it exists
         */
        bool has(const std::string& identifier) const {
            const PropertyBase* prop = find(identifier);
            return prop != NULL;
        }
        
        
        /**
         * @brief Returns a Property
         *
         * The getProperty method find a property based on the string identifier
         * and returns it with proper template type. If the property is not
         * found or if the cast is invalid it will return NULL.
         *
         * Do NOT attempt to delete the pointer returned
         *
         * @param identifier [in] property string identifier
         *
         * @return pointer to Property object
         */
        template<class T>
        Property<T>* getProperty(const std::string& identifier) {
            PropertyBase* prop = find(identifier);
            return dynamic_cast<Property<T>*>(prop);
        }


        /**
         * @copydoc getProperty
         */
        template<class T>
        const Property<T>* getProperty(const std::string& identifier) const {
            const PropertyBase* prop = find(identifier);
            return dynamic_cast<const Property<T>*>(prop);
        }


        /**
         * @brief Finds a property based in identifier
         *
         * The find methods returns pointer to PropertyBase object. Use
         * getProperty(std::string) to obtain a Property object.
         *
         * Do NOT attempt to delete the pointer returned
         *
         * @param identifier [in] property string identifier
         */
        PropertyBase* find(const std::string& identifier);


        /**
         * @copydoc find
         */
        const PropertyBase* find(const std::string& identifier) const;

        /**
         * @brief Adds property
         *
         * The PropertyMap takes responsibility of the Property and destroyes it
         * when being destroyed.
         *
         * @return true if added, false if the identifier is already in use
         */
        bool addProperty(boost::shared_ptr<PropertyBase> property);

        /**
         * @brief Removes a property
         *
         * @return true if the property was successfully removed
         */
        bool removeProperty(const std::string& identifier);

        /**
         * @brief returns the number of properties
         *
         * @return number of properties
         */
        size_t size() const;

        /**
         * @brief Returns std::vector with pointers to PropertyBase objects
         *
         * @return std::vector
         */
        std::vector<boost::shared_ptr<PropertyBase> > properties() const;

    private:
        std::map<std::string, boost::shared_ptr<PropertyBase> > _properties;

    private:
        PropertyMap(const PropertyMap&);
        PropertyMap& operator=(const PropertyMap&);
    };

    /** @} */
}} // end namespaces

#endif // end include guard
