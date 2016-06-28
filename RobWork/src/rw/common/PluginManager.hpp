
#ifndef RW_PLUGIN_PLUGINMANAGER_HPP
#define RW_PLUGIN_PLUGINMANAGER_HPP

#include <string>

/**
 * @brief The plugin manager will manage all plugins in the system. It will handle
 * the loading of plugins and the default locations of plugins
 *
 *  the following directories will be searched
 *  {exedir}/plugins
 *
 *  On Unix systems
 *  $HOME/.config/RobWork/plugins.xml
 *  $HOME/.config/RobWork.xml
 *
 */
class PluginManagerRegistry {
public:
    //! smart pointer type of PluginManagerRegistry
    typedef rw::common::Ptr<PluginManagerRegistry> Ptr;

protected:

    /**
     * @brief Constructor
     * @param exedir [in] the directory in which the main executable was executed. From
     * this dir the manager will try and autodetect plugins and their locations.
     */
    PluginManager(const std::string& exedir);


    void searchForPlugins();


};


#endif
