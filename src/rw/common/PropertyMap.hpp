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
        /*
          Some suggestions for a new interface:

              T get(const string& id);
              T* getPtr(const string& id);

              bool has(const string& id);

              void set(const string& id, const T& value);
              void set(const string& id, const string& description, const T& value);

              Property<T>* add(
                  const string& id, const string& description, const T& value);

              bool erase(const string& id);

              size_t size();
              bool empty();

          Perhaps set() and add() should have identical behaviour.

          These extra (expert) methods are only used rarely so they can have
          longer names:

              Property<T>* findProperty();
              PropertyBase* findPropertyBase();

           This one should probably be private:

              bool addPropertyBase(PropertyBase* base);

           This one should in principle be replaced by an iterator interface:

              std::vector<boost::shared_ptr<PropertyBase> > properties();

           If we don't provide an iterator interface, it should look like this:

              std::vector<PropertyBase*> properties();

           In other words, we provide something that is little bit more
           convenient and map-like and assures that the properties in the map
           are all of type Property<T>. This means that we can easily decide to
           not use boost:shared_ptr if we want to.

           We should probably store in a std::set instead of a std::map to make
           it easier to provide an iterator interface.
        */

        /**
         * @brief Constructor
         */
        PropertyMap();

        /**
         * @brief Destructor
         */
        ~PropertyMap();

        /**
         * @brief Set value of a property
         *
         * If a property with the given identifier cannot be found, the
         * method throws an exception
         *
         * @param identifier [in] the property identifier
         * @param value [in] the new value
         */
        template<class T>
        void setValue(const std::string& identifier, const T& value)
        {
            Property<T>* prop = getProperty<T>(identifier);
            if (prop) prop->setValue(value);
            else {
                // This would make sense according to the documentation:
                // RW_THROW(...);

                // This is what people expect:
                addProperty(identifier, "", value);
            }
        }

        /**
         * @brief Returns value of property
         *
         * If a property with the given identifier cannot be found, the
         * method throws an exception
         *
         * @param identifier [in] the identifier of the property
         * @return the value of the property
         */
        template<class T>
        T& getValue(const std::string& identifier)
        {
            Property<T>* prop = getProperty<T>(identifier);
            if (prop) return prop->getValue();
            else 
                RW_THROW(
                    "Property "
                    << StringUtil::Quote(identifier)
                    << " could not be found");
        }

        /**
         * @copydoc getValue
         */
        template<class T>
        const T& getValue(const std::string& identifier) const
        {
            // Forward to non-const method.
            return const_cast<PropertyMap*>(this)->getValue<T>(identifier);
        }

        /**
         * @brief Returns a Property
         *
         * The getProperty method find a property based on the string identifier
         * and returns it with proper template type. If the property is not
         * found or if the cast is invalid it will return NULL.
         *
         * @param identifier [in] property string identifier
         *
         * @return pointer to Property object
         */
        template<class T>
        Property<T>* getProperty(const std::string& identifier)
        {
            return dynamic_cast<Property<T>*>(find(identifier));
        }

        /**
         * @copydoc getProperty
         */
        template<class T>
        const Property<T>* getProperty(const std::string& identifier) const
        {
            return dynamic_cast<const Property<T>*>(find(identifier));
        }

        /**
           @brief Add a property to the map.

           @param identifier [in] Property identifier.
           @param description [in] Property description.
           @param value [in] Property value.
           @return true if added, false if the identifier is already in use
        */
        template <typename T>
        bool addProperty(
            const std::string& identifier,
            const std::string& description,
            const T& value)
        {
            return addProperty(
                boost::shared_ptr<Property<T> >(
                    new Property<T>(
                        identifier, description, value)));
        }

        /**
         * @brief Removes a property
         *
         * @return true if the property was successfully removed
         */
        bool removeProperty(const std::string& identifier);

        /**
         * @brief True if a specific property exists
         *
         * @param identifier [in] The identifier of the property
         * @return true if the property exists
         */
        bool has(const std::string& identifier) const;

        /**
         * @brief The number of properties
         */
        size_t size() const;

        /**
         * @brief All properties stored in the property map.
         */
        std::vector<boost::shared_ptr<PropertyBase> > properties() const;

        /**
         * @brief Find the property base for an identifier.
         *
         * The find methods returns pointer to PropertyBase object or NULL if
         * a property base with that identifier does not exist.
         *
         * See also getProperty() for a less primitive interface for retrieving
         * properties.
         *
         * @param identifier [in] identifier for the property base to find.
         */
        PropertyBase* find(const std::string& identifier);

        /**
         * @copydoc find
         */
        const PropertyBase* find(const std::string& identifier) const;

        /**
           @brief Add a property to the property map.

           @param property [in] The property to add.

           @return true if the property was added; false if the identifier of
           the property is already in use.
        */
        bool addProperty(boost::shared_ptr<PropertyBase> property);

    private:
        typedef std::map<std::string, boost::shared_ptr<PropertyBase> > MapType;
        MapType _properties;

    private:
        PropertyMap(const PropertyMap&);
        PropertyMap& operator=(const PropertyMap&);
    };

    /** @} */
}} // end namespaces

#endif // end include guard
