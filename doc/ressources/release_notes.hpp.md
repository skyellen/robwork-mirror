Release Notes of RobWork  {#pagereleasenotes}
========================

[TOC]


# Major Changes 0.6 # {#sec_release_notes_05}

- rwlibs::drawable refactored into rwlibs::graphics and rwlibs::opengl
- swig is now used for generating script interfaces, lua and python are supported
- added priliminary support for serialization
- added kinect camera simulation

#  Major Changes 0.5 # {#sec_release_notes_05}

- New structure for rw::geometry package in RobWork. 
- Updated rwlibs::drawable to use rw::geometry and share a common representation.
- New structure for collision detection.
- The lua script interface has been updated
- Enhancement to cmake structure to enable maintaining both a debug and a release version simultanously
- Support Visual Studio 2005 and 2008 and MinGW 4.4
- Introduction of rwlibs::simulation for simulation sensors
- Plugin structure for RobWork.

## Dependencies ## {#sec_release_notes_05_dependencies}

- XercesC are now required.

# Major Changes 0.4 # {#sec_release_notes_04}

- Distributed under the Apache License Version 2.0
- New Task Format in rwlibs/task. The new task format has XML save and load functionality
- XML save and load functionality for rw::common::PropertyMap
- Refactoring of joint structure to provide support for n-dof joints.
- The responsibility of calculating Jacobians are moved from the DeviceJacobian to the individual joints. DeviceJacobian are
replaced by JacobianCalculator which combines the contributions of the individual joints.
- PassiveFrame is replaced by VirtualJoint
- Refactoring of rw::common::Log
- Update of CMake structure
- Projects template to be found in the example folder
- Support for textures in the AC3D file format
- OBJ files can be used as drawables



## Dependencies ## {#sec_release_notes_04_dependencies}

- XercesC to enable save/load of XML files. Can be downloaded from http://xerces.apache.org/xerces-c/
- Boost in no longer included in ext.

# Major Changes 0.3 # {#sec_release_notes_03_major}

- All static methods start with lower-case. Most of the old methods are still available but are
deprecated and will be removed in later releases.
- New package rw::trajectory is replacing the old rw::interpolator package.
- Drawables are splitted into a Render part and a thin Drawable class. Renders are cached and
reused to save memory and load time.
- Dynamic inserting and removing of frames, enabling users to modify the workcell at runtime.
This feature is still to be considered at a beta state.
- All device and sensor drivers are moved into a seperate project called RobWorkHardware,
which will be release soon. The version number of RobWorkHardware will follow the version number
of RobWork.
- A Task format has been introduced. This format may still be subject to change.
- A factory for loading workcells has been introduced (rw::loaders::WorkCellLoader)
- The collision detection strategy based on Opcode has been removed, because of compile issues
and problems with invalid results. Instead a collision detector called Yaobi is introduced.
- New structure for motion planners and abstraction of constraints, samplers etc. for planning
- Introduction of SBL and ARW motion planners
- Introduction of the RobWork smart-pointer rw::common::Ptr, which handles optional ownership.
This pointer is used in most of the new interfaces and several of the old once to make it
optional whether to transfer ownership of an object.
- Introduction of rw::math::MetricFactory for constructing metrics.
- Interfaces for simulated devices and sensors. This is still in a beta state.
- A directory acting as a sandbox for experimental code and code under development. Whether to
compile the sandbox is specified in the RobWork.cmake file.
- New CMakeFile structure to make it easier to configure RobWork, RobWorkStudio and other
projects with the same setup.

