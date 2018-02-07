RobWork Plugins  {#page_rw_plugins}
====================

[TOC]

# Introduction # {#sec_rw_plugins_intro}

RobWork is an extensible framework that allows addition of functionality through the use of plugins. Plugins can be dynamically loaded at runtime, and makes it possible to reduce the coupling and complexity. Furthermore, the framework provides the concept of lazily loaded plugins. Functionality in a lazy-loaded plugin will not be loaded before it is needed. To make the framework extensible, RobWork provides a bunch of \link rw::common::ExtensionPoint ExtensionPoints \endlink. Each extension point has a unique identifier, and by referring to the identifier, it is possible for \link rw::common::Extension Extension(s) \endlink to extend the extension point. A \link rw::common::Plugin Plugin \endlink can provide one or more extensions, and these do not need to be extensions for the same extension point. To keep track of all plugins and extensions, the \link rw::common::ExtensionRegistry ExtensionRegistry \endlink is used. On this page,  some of the existing extension points and extensions will be discussed to give a better understanding of how they can be used to produce better and more modular code. After that, there is more in-depth examples of how a user can write a plugin that provides one or more extensions, and how this plugin is actually loaded and used in a program.

# Extension Points in the RobWork Framework # {#sec_rw_plugins_extensionpoints}
We will here try to give an overview of extension points currently available in the RobWork framework. Notice that the \ref extensionpoints group should always contain a up-to-date comprehensive list of all the extension points.

## Loaders ## {#sec_rw_plugins_extensionpoints_loaders}
RobWork has various loaders, for XML files, images, geometry data and workcells. These are defined as extension points, such that the framework can be easily extended to support new formats and loaders.

<table>
<tr><th>Extension Point</th><th>Description</th><th>Known Extensions</th></tr>
<tr><td>rw::common::DOMParser::Factory</td><td>Different parsers understanding the Document Object Model.</td><td>
  <ul>
    <li>rw::common::BoostXMLParser</li>
  </ul>
</td></tr>
<tr><td>rw::loaders::ImageLoader::Factory</td><td>Loaders that are able to load an rw::sensor::Image</td><td>
  <ul>
    <li>rw::loaders::PGMLoader</li>
    <li>rw::loaders::PPMLoader</li>
    <li>rw::loaders::RGBLoader</li>
  </ul>
</td></tr>
<tr><td>rw::loaders::Model3DLoader::Factory</td><td>Loaders that will load a rw::graphics::Model3D</td><td>
  <ul>
    <li>rw::loaders::Loader3DS</li>
    <li>rw::loaders::LoaderAC3D</li>
    <li>rw::loaders::LoaderAssimp</li>
    <li>rw::loaders::LoaderOBJ</li>
    <li>rw::loaders::LoaderTRI</li>
  </ul>
</td></tr>
<tr><td>rw::loaders::WorkCellLoader::Factory</td><td>Loaders that will load a rw::models::WorkCell</td><td>
  <ul>
    <li>rw::loaders::ColladaLoader</li>
    <li>rw::loaders::TULLoader</li>
    <li>rw::loaders::XMLRWLoader</li>
  </ul>
</td></tr>
</table>

## Proximity ## {#sec_rw_plugins_extensionpoints_proximity}
It is possible to use different strategies for detecting collisions and calculating distance between geometries. Typically the Proximity Query Package (PQP) is used.
RobWork allows other algorithms to be supported by the framework, as there are extension points defined for each of the different proximity strategy types.
Notice that the ProximityStrategy is a superclass of CollisionStrategy, CollisionToleranceStrategy, DistanceStrategy and DistanceMultiStrategy.
An extension to any of the four subtypes, should therefore also be available in the ProximityStrategy extension point.

<table>
<tr><th>Extension Point</th><th>Description</th><th>Known Extensions</th></tr>
<tr><td>rw::proximity::ProximityStrategy::Factory</td><td>Proximity strategies.</td><td>
  <ul>
    <li>rw::proximity::ProximityStrategyRW</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyBullet</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyPQP</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyFCL</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyYaobi</li>
  </ul>
</td></tr>
<tr><td>rw::proximity::CollisionStrategy::Factory</td><td>Proximity strategies that supports the rw::proximity::CollisionStrategy interface.</td><td>
  <ul>
    <li>rw::proximity::ProximityStrategyRW</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyBullet</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyPQP</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyFCL</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyYaobi</li>
  </ul>
</td></tr>
<tr><td>rw::proximity::CollisionToleranceStrategy::Factory</td><td>Proximity strategies that supports the rw::proximity::CollisionToleranceStrategy interface.</td><td>
  <ul>
    <li>rwlibs::proximitystrategies::ProximityStrategyBullet</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyPQP</li>
  </ul>
</td></tr>
<tr><td>rw::proximity::DistanceMultiStrategy::Factory</td><td>Proximity strategies that supports the rw::proximity::DistanceMultiStrategy interface.</td><td>
  <ul>
    <li>rwlibs::proximitystrategies::ProximityStrategyBullet</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyPQP</li>
  </ul>
</td></tr>
<tr><td>rw::proximity::DistanceStrategy::Factory</td><td>Proximity strategies that supports the rw::proximity::DistanceStrategy interface.</td><td>
  <ul>
    <li>rwlibs::proximitystrategies::ProximityStrategyBullet</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyPQP</li>
    <li>rwlibs::proximitystrategies::ProximityStrategyFCL</li>
  </ul>
</td></tr>
</table>

## Assembly Strategies ## {#sec_rw_plugins_extensionpoints_assembly}
RobWork has a framework for defining assembly operations and the result of performing assembly operations.
It is the intention that a library of different assembly strategies will be built with time. The framework can easily be extended with new user-specified control strategies using this extension point.

| Extension Point                                    | Description                                                       |
| :------------------------------------------------- | :---------------------------------------------------------------- |
| rwlibs::assembly::AssemblyRegistry                 | Strategies for assembly rwlibs::assembly::AssemblyControlStrategy |

## SWIG ## {#sec_rw_plugins_extensionpoints_swig}
The script interface can be extended with custom LUA commands, using the extension point for LUA.

| Extension Point                                    | Known Extensions                                                                                      |
| :------------------------------------------------- | :---------------------------------------------------------------------------------------------------- |
| rwlibs::swig::LuaState::Factory                    | RobWork, RobWorkStudio and RobWorkSim all extends this extension point with their own LUA interfaces. |

## Dynamic Simulation and Physics Engines ## {#sec_rw_plugins_extensionpoints_simulation}
RobWorkSim defines an extension point for physics engines. This way it is possible to add new engines for dynamic simulation in a modular way.
Furthermore, a hierarchic logging structure is provided for debugging simulation engines. As different engines can have different data that is relevant to log, the logging structure is extensible. On top of the logging structure, there is also a graphical layer, that can be extended with specific Qt widgets for graphical visualisation of the log.
The engine test framework also allows extension with new custom tests.

<table>
<tr><th>Extension Point</th><th>Description</th><th>Known Extensions</th></tr>
<tr><td>rwsim::simulator::PhysicsEngine::Factory</td><td>Physics engines rwsim::simulator::PhysicsEngine.</td><td>
  <ul>
    <li>rwsim::simulator::ODESimulator</li>
    <li>rwsim::simulator::RWSimulator</li>
    <li>rwsimlibs::bullet::BtSimulator</li>
    <li>rwsimlibs::rwpe::RWPEIsland</li>
  </ul>
</td></tr>
<tr><td>rwsim::log::SimulatorLogEntry::Factory</td><td>Logging types for simulators rwsim::log::SimulatorLogEntry, making it possible for specific physics engines to add more specific log types.</td><td>
  <ul>
    <li>rwsim::log::LogCollisionResult</li>
    <li>rwsim::log::LogConstraints</li>
    <li>rwsim::log::LogContactSet</li>
    <li>rwsim::log::LogContactVelocities</li>
    <li>rwsim::log::LogEquationSystem</li>
    <li>rwsim::log::LogForceTorque</li>
    <li>rwsim::log::LogMessage</li>
    <li>rwsim::log::LogPositions</li>
    <li>rwsim::log::LogValues</li>
    <li>rwsim::log::LogVelocities</li>
  </ul>
</td></tr>
<tr><td>rwsimlibs::gui::SimulatorLogEntryWidget::Factory</td><td>rwsimlibs::gui::SimulatorLogEntryWidget::Dispatcher for a Qt Widget that is able to show a rwsim::log::SimulatorLogEntry graphically. Physics engines can make engine specific log types and graphical representation.</td><td>
  <ul>
    <li>rwsimlibs::gui::BodyMotionWidget::Dispatcher</li>
    <li>rwsimlibs::gui::CollisionResultWidget::Dispatcher</li>
    <li>rwsimlibs::gui::ConstraintWidget::Dispatcher</li>
    <li>rwsimlibs::gui::ContactSetWidget::Dispatcher</li>
    <li>rwsimlibs::gui::ContactVelocitiesWidget::Dispatcher</li>
    <li>rwsimlibs::gui::EquationSystemWidget::Dispatcher</li>
    <li>rwsimlibs::gui::ForceTorqueWidget::Dispatcher</li>
    <li>rwsimlibs::gui::LogMessageWidget::Dispatcher</li>
    <li>rwsimlibs::gui::LogValuesWidget::Dispatcher</li>
  </ul>
</td></tr>
<tr><td>rwsimlibs::test::EngineTest::Factory</td><td>Tests of physics engines rwsimlibs::test::EngineTest. Makes it possible to extend the test suite with new tests.</td><td>
  <ul>
    <li>rwsimlibs::test::CollisionTest</li>
    <li>rwsimlibs::test::IntegratorTest</li>
  </ul>
</td></tr>
</table>

# Writing a Plugin - ImageLoader Example # {#sec_rw_plugins_plugin}
Most plugins will have a very similar structure. In this example a plugin named MyPlugin is created, by implementing the rw::common::Plugin interface. The MyPlugin.hpp file will look as follows:
\code{.cpp}
#include <rw/common/Plugin.hpp>

class MyPlugin: public rw::common::Plugin {
public:
	MyPlugin();
	virtual ~MyPlugin() {};
    std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();
    rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& id);
};
\endcode
By default, RobWork has the ability to load images in PGM, PPM and RGB formats. If, for instance, a user needs support for JPG or JPEG formats, it is possible to implement the rw::loaders::ImageLoader interface with a specific loader for these formats. Assume that such an implementation, MyImageLoader, is already developed. Now, the user could just use this loader directly in its program. Instead, the loader should be provided in a plugin, as it will then be possible to use the loader in all programs based on the RobWork framework, such as RobWorkStudio. The implementation of the Plugin interface would then look similar to the following example:
\code{.cpp}

#include "MyPlugin.hpp"
#include "MyImageLoader.hpp"

using namespace rw::common;

RW_ADD_PLUGIN(MyPlugin)

MyPlugin::MyPlugin():
	Plugin("MyPlugin", "My Lovely Plugin", "1.0")
{
}

std::vector<Extension::Descriptor> MyPlugin::getExtensionDescriptors() {
    std::vector<Extension::Descriptor> exts;
	exts.push_back(Extension::Descriptor("MyImageLoaderUniqueID","rw.loaders.ImageLoader"));
	exts.back().getProperties().set<std::string>("JPG", true);
	exts.back().getProperties().set<std::string>("JPEG", true);
    return exts;
}

Extension::Ptr MyPlugin::makeExtension(const std::string& id) {
	static const MyImageLoader::Ptr loader = ownedPtr(new MyImageLoader());
	if (id == "MyImageLoaderUniqueID") {
		const Extension::Ptr extension = ownedPtr(new Extension("MyImageLoaderUniqueID","rw.loaders.ImageLoader", this, loader));
		extension->getProperties().set<std::string>("JPG", true);
		extension->getProperties().set<std::string>("JPEG", true);
		return extension;
	}
	return NULL;
}

\endcode
The RW_ADD_PLUGIN() macro will define a standard entry point in the compiled binary, and is required to make it possible to load the plugin again after creation.
The constructor must call the parent Plugin constructor, with a unique id, a human-friendly name, and a version string.
The Plugin will be responsible for providing a list of extension descriptors and a method for actually creating a specific extension from an extension id.
The extension descriptor provides meta-information about the extensions in the plugin. The importance of the distinction between Extensions and ExtensionDescriptors, will become apparent for lazy-loading of plugins, where the meta-information can also be stored in a xml file.
Each extension and extension descriptor has a unique id, and refers to the id of the extension point that it fits into. In this case the rw::loaders::ImageLoader::Factory extension point has the id %rw.loaders.ImageLoader.
The id should be found in the documentation for the relevant extension point or on the \ref extensionpoints list.
A concrete Extension takes two additional arguments compared to the extension descriptor, namely a pointer to the plugin itself, and a pointer to the object provided by the extension.
In this case, the object provided by the extension is an instance of MyImageLoader. It is important to notice that a plugin will never be able to provide more than one instance per extension.
Sometimes objects can contain state or information that makes it unreasonable to only have one instance of an object. In such cases, the extension point will typically expect the object to be a so-called "dispatcher" that is then used to construct multiple objects.
Finally, notice that each extension has an attached PropertyMap with a list of properties. It is up to the individual extension point to define the properties necessary for the extension point. For a loader, this will typically be the names of the file extensions supported. For a proximity strategy, it will be the type of strategy, such as PQP, FCL or Bullet. 

Compilation of the plugin will typically just require a couple of lines to be added to the CMakeLists.txt file:
\code{.cmake}
ADD_LIBRARY(MyPlugin.rwplugin MODULE MyPlugin.cpp)
TARGET_LINK_LIBRARIES(MyPlugin.rwplugin ${MyImageLoader_Library} ${ROBWORK_LIBRARIES})
\endcode
Notice that the MyPlugin.cpp should never be included in other targets than the MyPlugin.rwplugin target.
If a target has source files containing more than one RW_ADD_PLUGIN definition, compilation will fail.

On Linux the plugin will have the name libMyPlugin.rwplugin.so. When this plugin is loaded dynamically, rw::loaders::ImageLoader::Factory will be able to provide a loader for the JPG and JPEG image formats. In fact, RobWorkStudio provides the rws::RWSImageLoaderPlugin that does exactly this for a wide range of image formats. The RWSImageLoaderPlugin basically uses Qt to load the images, and support the formats that Qt supports. This is an example of how the functionality of RobWork can be extended, without RobWork having to know about anything called Qt. Hence, dependencies are reduced, and code should be easier to maintain and modularize.

# Loading Plugins # {#sec_rw_plugins_loading}
There are different ways to actually load the plugins after creation. RobWork will look in various default directories where it will try to load all libraries with the correct rwplugin extension. If plugins lie in other directories, the user must point RobWork to these plugins. Currently, RobWork will automatically do the loading. <b>In the future this behaviour is likely to change</b>, so if program is written that depends on plugins being loaded, an explicit statement should be used to initialize the RobWork framework.

Every program that needs to use plugins, should initialize RobWork in the main function using rw::RobWork::init() or rw::RobWork::init(int argc, const char *const * argv) :
\code{.cpp}
#include <rw/RobWork.hpp>

using rw::RobWork;

int main(int argc, char** argv) {
    RobWork::init(argc,argv);
    
    ...
    
    return 0;
}
\endcode
RobWork will automatically search for plugins using the first of the following rules that succeeds:
-# If the RW_ROOT environment variable is set, RW_ROOT, RWS_ROOT, RWIM_ROOT and RWHW_ROOT is used to search the libs directory for each project.
-# Else, if a configuration file exists with the name robwork-BUILD_TYPE-VERSION.cfg.xml (for instance robwork-release-0.7.0.cfg.xml), the directories specified in this file is used.
 - First, the current working directory is used to look for the configuration file.
 - Second, the home folder is searched, which is specific to the operating system.
  - Windows: APPDATA/robwork/robwork-BUILD_TYPE-VERSION.cfg.xml
  - Mac: HOME/Library/Preferences/com.robwork-BUILD_TYPE-VERSION.cfg.xml
  - Linux: HOME/.config/robwork/robwork-BUILD_TYPE-VERSION.cfg.xml
-# Finally, the directory where RobWork was built is searched for libs folder for each of the projects RobWork, RobWorkStudio, RobWorkSim and RobWorkHardware.
Notice that If additional directories are given with the --rwplugin program option, RobWork will also search in these directories.

Additionally, a plugin can be loaded directly in code with:
\code{.cpp}
ExtensionRegistry::Ptr reg = ExtensionRegistry::getInstance();
Plugin::Ptr plugin = Plugin::load("/path/to/plugin.so");
if (!plugin.isNull()) {
	reg->registerExtensions(plugin);
}
\endcode

## Direct and Lazy-loading of Plugins ## {#sec_rw_plugins_loading_lazy}
When RobWork encounters finds files with the ending .rwplugin.so or .rwplugin.dll, it will try to load these plugins immediately. There can, potentially, be many plugins to load, which can take up unnecessary resources as they might not be needed by a specific application. For this reason, there is also the concept of lazy-loaded plugins. When RobWork encounters a file with the extension .rwplugin.xml, it will recognize this as a specification of a plugin that should only be loaded at the time where it is needed. To make a plugin lazy-loaded, create a file with a filename ending in .rwplugin.xml and give it a similar content as the following:
\code{.xml}
<?xml version="1.0" encoding="UTF-8" ?>
<plugin id="MyPlugin" name="My Lovely Plugin" version="1.0"
xmlns="http://www.robwork.dk/Plugin" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="http://www.robwork.dk/Plugin file:///path/to/RobWork/xml-schemas/rwxml_plugin.xsd">
        <runtime library="/path/to/libMyPlugin.rwplugin.so" />
        <extension id="MyImageLoaderUniqueID" name="MyImageLoaderUniqueID" point="rw.loaders.ImageLoader" />
</plugin>
\endcode
In this example, it is clear to see that the same meta-information is provided as in the extension descriptor discussed in \ref sec_rw_plugins_plugin .