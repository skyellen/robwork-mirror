This example shows how to use the new RobWork plugin system.

The library contains an abstract Animal class, which is inherited by
statically available Cat class. AnimalFactory class defines an extension
point, so that more types of animals can be added by plugins.

A sample plugin (src/plugin/dog) is provided, that extends the range of
available species with a Dog class. The plugin is built as a separate
shared library, and the file has to be put in the RobWork plugin
search path (e.g. RW_ROOT/libs/release). The configuration file of
RobWork specifies the set of paths that are browsed for plugins during
the initialization. The configuration can most likely be found in
~/.config/robwork/robwork-BUILDTYPE-VERSION.cfg.xml.

An executable is built (src/tools/test.cpp -> bin/test) that shows
RobWork initialization and plugin use. The code instantiates the
statically available Cat, and the Dog available through the plugin.

To build, go to the root directory of the example (where the README is),
and:

	mkdir build
	cd build
	cmake ..
	make -j 3
	
Then place the plugin where appropriate (e.g. RW_ROOT/libs/release), and
run TEST in the bin directory.

The output should read, e.g.:

	List of plugins: 
	+ DogPlugin
	+ RWSLuaPlugin
	+ RWSImageLoaderPlugin
	+ LuaPlugin
	+ ODEPlugin
	+ TNTPlugin
	meow
	woof
