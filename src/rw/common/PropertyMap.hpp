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

#include <set>
#include <memory>

namespace rw { namespace common {

    /** @addtogroup common */
    /*@{*/

    /**
     * @brief Container for a collection of Property Objects
     *
     * This container is used to bind various user information to for example
     * Frame.
     *
     * Example: Getting a string property with ID "Camera" from a frame
     *
     * \code
     * const std::string* ptr = frame.getPropertyMap().getPtr<std::string>("Camera");
     * if (ptr) {
     *    std::cout << "Property 'Camera' has value " << *ptr << "\n";
     * }
     * \endcode
     *
     */
    class PropertyMap {
    public:
        /**
         * @brief Constructor
         */
        PropertyMap();

        /**
         * @brief Destructor
         */
        ~PropertyMap();

        /**
           @brief Copy constructor.
        */
        PropertyMap(const PropertyMap& other);

        /**
           @brief Assignment operator.
        */
        PropertyMap& operator=(const PropertyMap& other);

        /**
           @brief swap operator.
        */
        void swap(PropertyMap& other);

        /**
         * @brief Set the value of a property
         *
         * If a property with the given identifier cannot be found, a new
         * property with no description is created and inserted.
         *
         * @param identifier [in] the property identifier
         * @param value [in] the new value
         */
        template<class T>
        void set(const std::string& identifier, const T& value)
        {
            Property<T>* prop = findProperty<T>(identifier);
            if (prop) prop->setValue(value);
            else add(identifier, "", value);
        }

        /**
           @brief Add a property to the map.

           @param identifier [in] Property identifier.

           @param description [in] Property description.

           @param value [in] Property value.

           @return The property if added or NULL if the identifier is already in
           use.
        */
        template <typename T>
        Property<T>* add(
            const std::string& identifier,
            const std::string& description,
            const T& value)
        {
            std::auto_ptr<Property<T> > property(
                new Property<T>(identifier, description, value));

            const bool ok = insert(property.get());
            if (ok) {
                Property<T>* result = property.get();
                property.release();
                return result;
            }
            else return NULL;
        }

        /**
         * @brief Get the value of a property or NULL if no such property.
         *
         * If a property of the given identifier and type cannot be found, the
         * method returns NULL.
         *
         * @param identifier [in] the identifier of the property
         *
         * @return the value of the property
         */
        template<class T>
        T* getPtr(const std::string& identifier)
        {
            Property<T>* prop = findProperty<T>(identifier);
            if (prop) return &prop->getValue();
            else return NULL;
        }

        /**
         * @brief Get the value of a property or NULL if no such property.
         *
         * If a property of the given identifier and type cannot be found, the
         * method returns NULL.
         *
         * @param identifier [in] the identifier of the property
         *
         * @return the value of the property
         */
        template<class T>
        const T* getPtr(const std::string& identifier) const
        {
            // Forward to non-const method.
            return const_cast<PropertyMap*>(this)->getPtr<T>(identifier);
        }

        /**
         * @brief Get the value of a property
         *
         * If a property of the given identifier and type cannot be found, the
         * method throws an exception
         *
         * @param identifier [in] the identifier of the property
         *
         * @return the value of the property
         */
        template<class T>
        T& get(const std::string& identifier)
        {
            T* p = getPtr<T>(identifier);
            if (!p) {
                RW_THROW(
                    "Property "
                    << StringUtil::quote(identifier)
                    << " could not be found");
            }
            return *p;
        }

        /**
         * @brief Get the value of a property
         *
         * If a property of the given identifier and type cannot be found, the
         * method throws an exception
         *
         * @param identifier [in] the identifier of the property
         *
         * @return the value of the property
         */
        template<class T>
        const T& get(const std::string& identifier) const
        {
            // Forward to non-const method.
            return const_cast<PropertyMap*>(this)->get<T>(identifier);
        }

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
         * @brief True iff the property map contains no properties.
         */
        bool empty() const;

        /**
         * @brief Remove a property
         *
         * @return true if the property was successfully removed.
         */
        bool erase(const std::string& identifier);

        // The following methods are rarely used and are therefore given longer
        // names. They more strongly expose the internal use of Property<T>.

        /**
         * @brief Find the property for an identifier.
         *
         * The method finds the Property<T> object having a given identifier. If
         * no property with that identifier exists or if the value of the
         * property is not of type T then NULL is returned.
         *
         * @param identifier [in] property identifier
         *
         * @return Property object with that identifier
         */
        template<class T>
        Property<T>* findProperty(const std::string& identifier)
        {
            return dynamic_cast<Property<T>*>(
                findPropertyBase(identifier));
        }

        /**
         * @brief Find the property for an identifier.
         *
         * The method finds the Property<T> object having a given identifier. If
         * no property with that identifier exists or if the value of the
         * property is not of type T then NULL is returned.
         *
         * @param identifier [in] property identifier
         *
         * @return Property object with that identifier
         */
        template<class T>
        const Property<T>* findProperty(const std::string& identifier) const
        {
            return dynamic_cast<const Property<T>*>(
                findPropertyBase(identifier));
        }

        /**
         * @brief Find the property base for an identifier.
         *
         * The find methods returns pointer to PropertyBase object or NULL if a
         * property base with that identifier does not exist.
         *
         * @param identifier [in] identifier for the property base to find.
         */
        PropertyBase* findPropertyBase(const std::string& identifier);

        /**
         * @brief Find the property base for an identifier.
         *
         * The find methods returns pointer to PropertyBase object or NULL if a
         * property base with that identifier does not exist.
         *
         * @param identifier [in] identifier for the property base to find.
         */
        const PropertyBase* findPropertyBase(const std::string& identifier) const;

    private:
        struct CmpPropertyBase
        {
            bool operator()(
                const PropertyBase* a,
                const PropertyBase* b) const
            {
                return a->getIdentifier() < b->getIdentifier();
            }
        };

        typedef std::set<PropertyBase*, CmpPropertyBase> MapType;

    public:
        //! Iterator for PropertyBase const*.
        typedef MapType::const_iterator iterator;

        /**
           @brief Range of all PropertyBase* objects stored.

           Note that this low-level interface does permits the PropertyBase
           values to be modified even though the method itself is declared
           const.
        */
        std::pair<iterator, iterator> getProperties() const;

    public:
        //----------------------------------------------------------------------
        // Everything below is deprecated.

        /*
         * @brief REMOVED All properties stored in the property map.
         */
        // std::vector<boost::shared_ptr<PropertyBase> > properties() const;
        // We can't support this because we don't want to use shared_ptr
        // internally.

#ifndef RW_REMOVE_DEPRECATED
        /**
         * @brief \b DEPRECATED Find the property base for an identifier.
         *
         * The find methods returns pointer to PropertyBase object or NULL if a
         * property base with that identifier does not exist.
         *
         * See also getProperty() for a less primitive interface for retrieving
         * properties.
         *
         * @param identifier [in] identifier for the property base to find.
         */
        PropertyBase* find(const std::string& identifier)
        { return findPropertyBase(identifier); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /** \b DEPRECATED */
        const PropertyBase* find(const std::string& identifier) const
        { return findPropertyBase(identifier); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /**
         * @brief \b DEPRECATED Returns a Property
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
        { return findProperty<T>(identifier); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /** DEPRECATED */
        template<class T>
        const Property<T>* getProperty(const std::string& identifier) const
        { return findProperty<T>(identifier); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /**
           @brief \b DEPRECATED Add a property to the map.

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
        { return add<T>(identifier, description, value); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /**
         * @brief \b DEPRECATED Removes a property
         *
         * @return true if the property was successfully removed
         */
        bool removeProperty(const std::string& identifier)
        { return erase(identifier); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /**
         * @brief \b DEPRECATED Set value of a property
         *
         * If a property with the given identifier cannot be found, the
         * method throws an exception
         *
         * @param identifier [in] the property identifier
         * @param value [in] the new value
         */
        template<class T>
        void setValue(const std::string& identifier, const T& value)
        { set<T>(identifier, value); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /**
         * @brief \b DEPRECATED Returns value of property
         *
         * If a property with the given identifier cannot be found, the
         * method throws an exception
         *
         * @param identifier [in] the identifier of the property
         * @return the value of the property
         */
        template<class T>
        T& getValue(const std::string& identifier)
        { return get<T>(identifier); }
#endif /* RW_REMOVE_DEPRECATED */

#ifndef RW_REMOVE_DEPRECATED
        /** \b DEPRECATED */
        template<class T>
        const T& getValue(const std::string& identifier) const
        { return get<T>(identifier); }
#endif /* RW_REMOVE_DEPRECATED */

    private:
        bool insert(PropertyBase* property);

    private:
        MapType _properties;
    };

    /** @} */
}} // end namespaces

#endif // end include guard
