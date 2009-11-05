/*
 * PluginServer.hpp
 *
 *  Created on: 20-06-2009
 *      Author: jimali
 */

#ifndef PLUGINSERVER_HPP_
#define PLUGINSERVER_HPP_

/**
 * @brief All plugins are registered in the plugin server.
 *
 */

class PluginServer {
public:

    PluginServer();
    virtual ~PluginServer();

    /**
     * @@brief the search path is a list of paths (relative or absolute) to directories
     * that may contain plugins.
     * @param str
     */
    void addSearchPath(const std::string& str);
    std::string getSearchPath(int i);
    std::string getSearchPaths();
    void removeSearchPath(int i);

    std::vector<std::string> findAllPlugins();

    /**
     *
     * @param pluginname
     */
    void loadPlugin(const std::string& pluginname);

    // devices
    void registerDeviceParser(DeviceParserFactoryPtr factory);

    //
    void registerRenderLoader(RenderLoaderFactoryPtr factory);


};


#endif /* PLUGINSERVER_HPP_ */
