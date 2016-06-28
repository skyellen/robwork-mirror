
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

    const std::string& getId(){ return _id; };
    const std::string& getName(){ return _name; };
    const std::string& getVersion(){ return _version; };

    static rw::common::Ptr<Plugin> load(const std::string& filename);

private:
    static rw::common::Ptr<Plugin> loadDirect(const std::string& filename);
    static rw::common::Ptr<Plugin> loadLazy(const std::string& filename);

    std::vector<Extension::Descriptor> _descriptors;
    std::string _id, _name, _version;
};

}
}

#endif
