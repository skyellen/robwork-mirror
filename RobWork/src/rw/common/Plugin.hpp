
#ifndef RW_COMMON_PLUGIN_HPP
#define RW_COMMON_PLUGIN_HPP

#include <string>
#include <vector>
#include <rw/common/Ptr.hpp>
#include <rw/common/os.hpp>

#include "Extension.hpp"

namespace rw {
namespace common {

/**
 * @brief this define is to be used whenever dynamically loadable RobWork libraries are
 * being created. Simply add this in the bottom of the .cpp file which defines you plugin.
 * RW_ADD_PLUGIN(MyPluginName)
 *
 * where MyPluginName is the name of your plugin including namespace. So if your plugin is
 * named: "rw::example::MyExamplePlugin" then you should use the complete name.
 */
#define RW_ADD_PLUGIN(name) \
    DLL_EXPORT void* createplugin(void) { \
        return new name(); \
    }

/**
 * @brief an interface for defining dynamically loadable plugins that define extensions and
 * extension points.
 */
class Plugin {
protected:
    /**
     * @brief constructor
     * @param id [in] unique identifier of this plugin
     * @param name [in] Human readable identifier of this plugin
     * @param version [in] version of this plugin
     */
    Plugin(const std::string& id, const std::string& name, const std::string& version);

public:
    //! @brief destructor
    virtual ~Plugin();

    //! @brief get all extension descriptors of this plugin
    virtual std::vector<Extension::Descriptor> getExtensionDescriptors() = 0;

    //! @brief get a specific extension using the unique extendion ID
    virtual rw::common::Ptr<Extension> makeExtension(const std::string& id) = 0;

    /**
     * @brief get a list of extension point ids which this plugin define
     * @return list of extension points ids.
     */
    virtual std::vector<std::string> getExtensionPointIDs();

    //! @brief get a
    //virtual rw::common::Ptr<ExtensionPoint> makeExtensionPoint(const std::string& id){ return NULL; };

    template<class T>
    rw::common::Ptr<T> makeExtension(const std::string& id){
        rw::common::Ptr<Extension> ext = makeExtension(id);
        return ext.cast<T>();
    }

    /**
     * @brief Get unique identifier of plugin.
     * @return the identifier.
     */
    const std::string& getId(){ return _id; }

    /**
     * @brief Get human readable identifier for the plugin.
     * @return the identifier.
     */
    const std::string& getName(){ return _name; }

    /**
     * @brief Get version of plugin.
     * @return the version.
     */
    const std::string& getVersion(){ return _version; }

    /**
     * @brief Load the plugin given by \b filename.
     *
     * A filename with .xml extension will be loaded as a lazy plugin.
     *
     * Notice that the smart pointer returned will automatically unload the plugin
     * when there are no more references to the plugin.
     *
     * @param filename [in] the filename.
     * @return the plugin, or NULL if load failed.
     */
    static rw::common::Ptr<Plugin> load(const std::string& filename);

    //! @brief Internal handle that makes unloading possible.
    struct OSHandle;

    /**
     * @brief Get the low-level handle of the plugin (for internal use).
     * @return the handle.
     */
    const OSHandle* getHandle();

protected:
    /**
     * @brief Close the plugin.
     * @param handle [in] the low-level handle of the plugin.
     */
    static void close(const OSHandle* handle);

private:
    static rw::common::Ptr<Plugin> loadDirect(const std::string& filename);
    static rw::common::Ptr<Plugin> loadLazy(const std::string& filename);

    std::vector<Extension::Descriptor> _descriptors;
    std::string _id, _name, _version;
    OSHandle* _handle;
};

}
}

#endif
