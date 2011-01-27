// -*- latex -*-

/**

\page page_rws_release_notes RobWorkStudio Release Notes



\section page_rws_release_notes_05 Release Notes RobWorkStudio Version 0.4

\subsection sec_release_notes_rws_05_major Major Changes

- Updated to match RobWork version 0.5

- New CMake structure

- Added plugin for lua scripting

- Added plugin for sensor visualization

- Introduction of reusable GUI components for image viewing and opengl view.

\section page_rws_release_notes_04 Release Notes RobWorkStudio Version 0.4

\subsection sec_release_notes_rws_04_major Major Changes

- Updated to match RobWork version 0.4

- Distributed under the Apache License Version 2.0

- A shared rw::commmon::Log object is available for plugins. This enables a uniform output of messages from plugins.

- A shared rw::common::PropertyMap is available for plugins. This enables plugins to exchange of data

- A configuration file is introduced which stores basic properties of the window and the plugins for next startup

- Refactoring of CMake structure and project template in example folder

\section page_rws_release_notes_03 Release Notes RobWorkStudio Version 0.3

\subsection sec_release_notes_rws_03_major Major Changes

- Updated to match RobWork version 0.3

- Build in plugins are as default linked statically. I upgrading from an old version
of RobWorkStudio it is necessary to remove the content of RobWorkStudio.ini to avoid
trying to load them dynamically. To continue linking dynamically set "UseStaticLinkedPlugins"
to 0 in RobWorkStudio.cmake. User defined plugins should still be loaded dynamically.

- New Jog-Plugin with Cartesian jog and with support for rw::kinematics::MovableFrame. The old
plugin still exists but are not loaded at default.

*/
