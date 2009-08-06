// -*- latex -*-

/**
\page page_release_notes_04 Release Notes RobWork Version 0.4

\section sec_release_notes_04_major Major Changes

- New Task Format placed in rwlibs/task. The new task format has XML save and load functionality

- XML Save and load functionality of rw::common::PropertyMap

- Refactoring of joint structure to provide support for n-dof joints.

- The responsibility of calculating Jacobian are moved from the DeviceJacobian to the individual joints. DeviceJacobian are therefore
replaced by JacobianCalculator which combines the contributions of the individual joints.

- PassiveFrame is replaced by VirtualJoint

- Refactoring of structure around rw::common::Log

- Update of CMake structure

- Projects template to be found in the example folder

- Support for textures in the AC3D file format

- OBJ files can be used for drawables

\section sec_release_notes_04_dependencies Dependencies

- XercesC to enable save/load of XML files. Can be downloaded from http://xerces.apache.org/xerces-c/

- Boost in no longer included in ext.

*/
