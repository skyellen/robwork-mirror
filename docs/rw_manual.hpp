// -*- latex -*-

/**

\page page_rw_manual RobWork manual

These are the most important things to cover in this manual:

- Installation

- Libraries, their naming convention, which to use, and how to link to
  them.

- Howto for the most important and stable parts of RobWork.
.

Other things to cover as time permits include:

- Device and workcell file formats and the workcells and devices we
  provide.
.

\section sec_load_workcell Getting started: Loading a workcell

RobWork support workcells described in an XML format as well as the
.wu and .dev workcell file formats used by the TUL program.

The below program loads a workcell from the file named on the command
line. If the loading of the workcell fails, the WorkCellLoader::load()
function will throw an exception, and the program will abort with an
error message.

\include ex-load-workcell.cpp

*/

/*

Here is how you can include example code in the manual:

\include ex-load-workcell.cpp

This is equivalent to the following:

\code
#include <string>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

... and so on ...
\endcode

Jeg vil foreslå at lade alle eksempler være komplette i sig selv, dvs.
hvert eksempel placeres i en fil og man kontrollerer at alle eksempler
compilerer. Se CMakeLists.txt i dette directory.

*/
