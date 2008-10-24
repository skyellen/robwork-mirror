/*********************************************************************
 * RobWork Version 0.3
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

#ifndef RW_LOADERS_TUL_TAG_HPP
#define RW_LOADERS_TUL_TAG_HPP

/**
 * @file Tag.hpp
 */

#include <rw/common/macros.hpp>
#include <rw/common/Property.hpp>
#include <rw/common/StringUtil.hpp>

#include <map>

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
     * @brief Tags of setup files.
     *
     * A tag is a mapping from key name to sequence of values.
     *
     * The values are stored as properties. For tags read from files the values
     * can be of type string, double or Q.
     *
     * A number of utilities have been provided to make it convenient to access
     * properties of tags.
     *
     * A tag file is a sequence of tags. A tag file can be loaded as follows:
     \code
     const std::vector<Tag>& tags = loadTagFile(file);
     \endcode
    */
    class Tag
    {
    public:
        /**
         * @brief A tag named \b name.
         *
         * It is great for error messages to know the file from which the tag
         * was read and therefore this is stored too.
         *
         * @param name [in] The name of the tag.
         *
         * @param file [in] The originating file.
         */
        Tag(const std::string& name, const std::string& file) :
            _name(name),
            _file(file)
        {}

        /**
         * @brief The name of the tag.
         */
        const std::string& getName() const { return _name; }

        /**
         * @brief The file from which the tag was read.
         *
         * This is great to have for error messages and also it is indispensible
         * when interpreting relative file names stored in tag attributes.
         */
        const std::string& getFile() const { return _file; }

        /**
         * @brief A sequence of property values.
         */
        typedef std::vector<
            boost::shared_ptr<common::PropertyBase> >
        PropertyList;

        /**
         * @brief A mapping from attribute name to attribute value.
         */
        typedef std::map<std::string, PropertyList> PropertyMap;

        /**
         * @brief The tag properties.
         */
        const PropertyMap& getPropertyMap() const { return _map; }

        /**
         * @brief The tag properties.
         */
        PropertyMap& getPropertyMap() { return _map; }

        /**
         * @brief Emit to \b out the tag \b tag in standard tag format.
         *
         * If \b tag contains properties that not among the types supported for tags
         * the the output will not be in the valid tag format, but the output may
         * still be useful for debugging.
         */
        friend std::ostream& operator<<(std::ostream& out, const Tag& tag);

    private:
        std::string _name;
        std::string _file;
        PropertyMap _map;
    };

    std::ostream& operator<<(std::ostream& out, const Tag& tag);

    /**
     * @brief The property value for attribute \b key at position \b pos.
     *
     * An exception is thrown if there is no such value.
     *
     * The utility function is parameterized by the type of value to access.
     *
     * @param tag [in] The tag to access.
     *
     * @param key [in] The name of the property.
     *
     * @param pos [in] The position of the property in the sequence of property
     * values.
     *
     * @return The value of the property.
     */
    template <typename T>
    T& getAttribute(
        Tag& tag,
        const std::string& key,
        int pos)
    {
        typedef Tag::PropertyMap::const_iterator I;
        const I p = tag.getPropertyMap().find(key);
        if (p == tag.getPropertyMap().end()) {
            RW_THROW(
                "No property named "
                << common::StringUtil::quote(key)
                << " in tag "
                << common::StringUtil::quote(tag.getName()));

            // To avoid a compiler warning.
            return getAttribute<T>(tag, key, pos);
        } else {
            const Tag::PropertyList& vals = p->second;
            if (pos < 0 || (int)vals.size() <= pos) {
                RW_THROW(
                    "Can't access index "
                    << pos
                    << " for attribute "
                    << common::StringUtil::quote(key)
                    << " of length "
                    << (int)vals.size()
                    << " in tag "
                    << common::StringUtil::quote(tag.getName()));

                // To avoid a compiler warning.
                return getAttribute<T>(tag, key, pos);
            } else {
                common::PropertyBase* prop = vals.at(pos).get();
                common::Property<T>* property =
                    dynamic_cast<common::Property<T>*>(prop);

                if (!property) {
                    RW_THROW(
                        "Value at index "
                        << pos
                        << " for attribute "
                        << common::StringUtil::quote(key)
                        << " with description "
                        << prop->getDescription()
                        << " in tag "
                        << common::StringUtil::quote(tag.getName())
                        << " is of an unexpected type.");

                    // To avoid a compiler warning.
                    return getAttribute<T>(tag, key, pos);

                } else {
                    return property->getValue();
                }
            }
        }
    }

    /**
     * @brief The property value for attribute \b key at position \b pos.
     *
     * An exception is thrown if there is no such value.
     *
     * The utility function is parameterized by the type of value to access.
     *
     * @param tag [in] The tag to access.
     *
     * @param key [in] The name of the property.
     *
     * @param pos [in] The position of the property in the sequence of property
     * values.
     *
     * @return The value of the property.
     */
    template <typename T>
    const T& getAttribute(
        const Tag& tag,
        const std::string& key,
        int pos)
    {
        // Forward to the non-const version.
        return getAttribute<T>(const_cast<Tag&>(tag),  key, pos);
    }

    /**
     * @brief The property value for attribute \b key at position \b pos.
     *
     * NULL is returned if there is no such property value.
     *
     * The utility function is parameterized by the type of value to access.
     *
     * @param tag [in] The tag to access.
     *
     * @param key [in] The name of the property.
     *
     * @param pos [in] The position of the property in the sequence of property
     * values.
     *
     * @return The value of the property or NULL if no such property.
     */
    template <typename T>
    T* getAttributePtr(
        Tag& tag,
        const std::string& key,
        int pos)
    {
        typedef Tag::PropertyMap::const_iterator I;
        const I p = tag.getPropertyMap().find(key);
        if (p == tag.getPropertyMap().end()) {
            return NULL;
        } else {
            const Tag::PropertyList& vals = p->second;
            if (pos < 0 || (int)vals.size() <= pos) {
                return NULL;
            } else {
                common::PropertyBase* prop = vals.at(pos).get();
                common::Property<T>* property =
                    dynamic_cast<common::Property<T>*>(prop);

                if (!property) {
                    return NULL;
                } else {
                    return &property->getValue();
                }
            }
        }
    }

    /**
     * @brief The property value for attribute \b key at position \b pos.
     *
     * NULL is returned if there is no such property value.
     *
     * The utility function is parameterized by the type of value to access.
     *
     * @param tag [in] The tag to access.
     *
     * @param key [in] The name of the property.
     *
     * @param pos [in] The position of the property in the sequence of property
     * values.
     *
     * @return The value of the property or NULL if no such property.
     */
    template <typename T>
    const T* getAttributePtr(
        const Tag& tag,
        const std::string& key,
        int pos)
    {
        // Forward to non-const version.
        return getAttributePtr<T>(const_cast<Tag&>(tag), key, pos);
    }

    /**
     * @brief The number of values stored for an attribute.
     *
     * An exception is thrown if the tag does not contain the attribute.
     *
     * @param tag [in] The tag containing the attributes.
     *
     * @param key [in] The name of the attributes.
     *
     * @return The number of attribute values.
     */
    int getAttributeSize(const Tag& tag, const std::string& key);

    /**
     * @brief True if an attribute named \b key has been registered for tag \b
     * tag.
     *
     * @param tag [in] The tag containing attributes.
     *
     * @param key [in] The name of an attribute.
     *
     * @return True iff \b tag contains the attribute named \b name.
     */
    bool hasAttribute(
        const Tag& tag, const std::string& key);

    /**
     * @brief Load the tags of the file \b file.
     *
     * @param file [in] The file to load.
     *
     * @return The tags of the file.
     */
    std::vector<Tag> loadTagFile(const std::string& file);

    /*@}*/
}} // end namespaces

#endif // end include guard
