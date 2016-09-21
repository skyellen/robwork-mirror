RobWork Plugins and Extensions   {#pageRobworkPlugins} 
===============================

This tutorial will cover the creation and usage of RobWork plugins. This should not be confused with
the QT based GUI plugins which are discussed in another tutorial: \ref pageRobWorkStudioPlugins.

# Introduction # 
The primary usage of the RobWork plugins are to extend the core functionality of RobWork. This might be
as simple as a loader to load an unsupported image file.

Loading of plugins in RobWork need to be done actively. This is most easily done by calling initialize 
on the RobWork instance:

~~~{.cpp}
RobWork::getInstance()->initialize();
~~~  

This will search for a settings file which may be located in the same folder as the executable. More 
detail on the plugin structure can be found in [Plugin structure](@ref page_rw_plugins).


# Tutorial 1 - A simple  plugin #
In this section we will see how to create a simple plugin which can be loaded in runtime by RobWork.
First the header file TemplatePlugin.hpp:

~~~{.cpp}
class TemplatePlugin: public rw::common::Plugin {
public:
    TemplatePlugin();
	
    virtual ~TemplatePlugin();

    std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

    rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& id);
};
~~~

as can be seen only two methods are necesary: getExtensionDescriptors() and makeExtension(). 
The getExtensionDescriptors() should return a list describing all extensions provided
by the plugin. The makeExtension take a string id and create the extension fitting the id. 





TODO:

# Tutorial 2 - ImageLoader plugin extension #
The class rw::loaders::ImageLoader define an extension point for loading images. In this
example we will extend the above plugin such that it will register an ImageLoader extension
to RobWork uppon loading.

TODO:

# Tutorial 3 - A simple xml based extension #
Extensions does not have to be coded. They can also be pure information. The extension structure in 
RobWork allow for xml files to be parsed which contain extension information. In this tutorial we 
will go through a simple example showing how to create such an extension.

TODO:

# Tutorial 4 - A simple lua based extension #
TODO:

# Tutorial 5 - A simple python based extension #
