
#ifndef RW_PLUGIN_EXTENSIONPOINT_HPP
#define RW_PLUGIN_EXTENSIONPOINT_HPP

#include "ExtensionRegistry.hpp"

namespace rw {
namespace common {

class Plugin;

/**
 * @brief an extension point is a class that defines a point where Extension can be added.
 * This is typically used together with plugins, however any class may register extensions
 * to an extension point.
 */
template<class ExtensionInterface>
class ExtensionPoint {
public:
    //! smart pointer type of ExtensionPoint
    typedef rw::common::Ptr<ExtensionPoint> Ptr;

    /**
     * @brief Constructor
     * @param id [in] unique identifier of this extension point
     * @param name [in] human readable name of this extension point
     * @param plugin [in] the plugin from which this extension point is defined, NULL if not defined from plugin
     */
    ExtensionPoint(const std::string& id, const std::string& name, Plugin* plugin=NULL):
    	_id(id),_name(id),_owner(plugin)
    {
    }

    //! @brief get unique identifier of this extensionpoint
    const std::string& getId() const {return _id;}

    //! @brief get human readable name of this extension point
    const std::string& getName() const {return _name;}

    /**
     * @brief the schema describe the possible properties/configurations elements
     * which is used in the PropertyMap. It contain examples of all possible configuration
     * options. This can be used to configure any extensions that needs to attach to
     * this extension point.
     */
    const rw::common::PropertyMap& getSchema() const {return _schema; }

    //! @brief get all extension descriptions of this extension point
    std::vector<Extension::Descriptor> getExtensionDescriptors() const {
    	ExtensionRegistry::Ptr  reg = ExtensionRegistry::getInstance();
    	return reg->getExtensionDescriptors( _id );
    }

    //! @brief get all extensions of this extension point
    std::vector<rw::common::Ptr<Extension> > getExtensions() const {
    	ExtensionRegistry::Ptr reg = ExtensionRegistry::getInstance();
    	return reg->getExtensions( _id );
    }

protected:
    /**
     * @brief the schema describe the possible properties/configurations elements
     * which is used in the PropertyMap. The schema property map should just be loaded
     * with all possible configuration options which the extension might use.
     *
     * subclassing the ExtensionPoint class enables you to add extra configuration options.
     * This might be done as simply as this:
     *
     * \code
     * getSchema().add("SupportedFormats", "Comma seperated String of supported formats", std::string("GIF,PNG,JPEG"));
     * getSchema().add("ShowLights", "Should lights be on as default?", true);
     * ...
     * \endcode
     *
     * which will enable the loading of these options from the plugin xml file descriptor
     *
     * \code
     * <plugin ... >
     * ...
     * <extension>
     * ...
     * <SupportedFormats>GIF,PNG,JPEG</SupportedFormats>
     * <ShowLights>false</ShowLights>
     * </extension>
     * </plugin>
     * \endcode
     *
     * @return
     */
    rw::common::PropertyMap& getSchema(){ return _schema;};

private:
    std::string _id, _name;
    Plugin *_owner;
    rw::common::PropertyMap _schema;
};

}
}

#endif
