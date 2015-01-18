
#ifndef RW_PLUGIN_EXTENSIONREGISTRY_HPP
#define RW_PLUGIN_EXTENSIONREGISTRY_HPP

#include "Extension.hpp"
#include <rw/common/Ptr.hpp>

namespace rw {
namespace common {


/**
 * @brief an extension point is a class that defines a point where Extension can be added.
 * This is typically used together with plugins, however any class may register extensions
 * to an extension point.
 */
class ExtensionRegistry {
public:
    //! smart pointer type of ExtensionPoint
    typedef rw::common::Ptr<ExtensionRegistry> Ptr;

    //! @brief Constructor
    ExtensionRegistry();


    //! get registry instance
    static rw::common::Ptr<ExtensionRegistry> getInstance();

    /**
     * @brief get all descriptors registered for a specific extension point id
     * @param ext_point_id [in] identifier of extension point
     * @return list of extension point descriptions
     */
    std::vector<Extension::Descriptor> getExtensionDescriptors(const std::string& ext_point_id) const;

    /**
     * @brief get all extensions of a specific extension point
     * @param ext_point_id [in] identifier of extension point
     * @return list of extensions
     */
    std::vector<rw::common::Ptr<Extension> > getExtensions(const std::string& ext_point_id) const;

    /**
     * @brief register extensions and extension points of a plugin
     * @param plugin [in] the plugin that is to be registered
     */
    void registerExtensions(rw::common::Ptr<Plugin> plugin);
    
    /**
     * @brief get a list of registered plugins
     * @return list of plugins
     */
    std::vector<rw::common::Ptr<Plugin> > getPlugins() const;

private:

    // maps extension point id's into description-plugin pair
    std::map<std::string, std::vector< std::pair<Extension::Descriptor, rw::common::Ptr<Plugin> > > > _descMap;

    std::vector<rw::common::Ptr<Plugin> > _plugins;
};

}
}


#endif
