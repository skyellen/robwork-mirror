
#ifndef RW_PLUGIN_EXTENSION_HPP
#define RW_PLUGIN_EXTENSION_HPP

#include <iostream>
#include <string>
#include <vector>
#include <rw/common/Ptr.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/AnyPtr.hpp>

namespace rw {
namespace common {
class Plugin;

/**
 * @brief The Extension class is used to provide additonal functionality from a
 * Plugin to other extension points of either the system or other plugins.
 *
 * extension points in RobWork
 * @copydoc extensionpoints
 */
class Extension {
public:
	typedef rw::common::Ptr<Extension> Ptr;
    /**
     * @brief an extension descriptor
     */
    struct Descriptor {
		Descriptor(){}
		Descriptor(const std::string& id_, const std::string& point_):id(id_),point(point_){}


        std::string id,name,point;
        rw::common::PropertyMap props;

        rw::common::PropertyMap& getProperties(){ return props;}
        const rw::common::PropertyMap& getProperties() const { return props;}
    };

public:

    /**
     * @brief Constructor
     * @param desc [in] Description and configuration of extension.
     */
    Extension(Descriptor desc, Plugin* owner);

    /**
     * @brief Constructor
     * @param id
     * @param point
     */
    Extension(const std::string& id, const std::string& point, Plugin* owner );

    Extension(const std::string& id, const std::string& point, Plugin* owner, rw::common::AnyPtr obj);

    //! @brief Destructor
    virtual ~Extension(){}

    //! @brief a unique id that uniquely identifies this extension in its owner Plugin
    const std::string& getId(){ return _desc.id; }

    //! @brief a human readable name of this plugin (may contain spaces)
    const std::string& getName(){ return _desc.name; }

    //! @brief a unique global identifier of the extension point that this extension is attached to
    const std::string& getPoint() const { return _desc.point; }

    //! @brief the properties/configuration of this extension
    const rw::common::PropertyMap& getProperties() const { return _desc.props; }

    rw::common::PropertyMap& getProperties(){ return _desc.props; }

    virtual rw::common::AnyPtr getObject(){ return _obj; }

    //! @brief get the owner plugin
    Plugin* getOwner(){ return _owner; };


private:
    friend class Plugin;

    //! @brief set owner plugin, NOTE use with care
    void setOwner(Plugin* owner){ _owner = owner;}
private:
    Descriptor _desc;
    Plugin* _owner;
    rw::common::AnyPtr _obj;
};

}
}

#endif
