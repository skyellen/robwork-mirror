// -*- latex -*-

/**
\page page_rws_release_notes_03 Release Notes RobWorkStudio Version 0.3

\section sec_release_notes_rws_03_major Major Changes

- Updated to match RobWork version 0.3

- Build in plugins are as default linked statically. I upgrading from an old version
of RobWorkStudio it is necessary to remove the content of RobWorkStudio.ini to avoid
trying to load them dynamically. To continue linking dynamically set "UseStaticLinkedPlugins"
to 0 in RobWorkStudio.cmake. User defined plugins should still be loaded dynamically.

- New Jog-Plugin with Cartesian jog and with support for rw::kinematics::MovableFrame. The old
plugin still exists but are not loaded at default.

*/
