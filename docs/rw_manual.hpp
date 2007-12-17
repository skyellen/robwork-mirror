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

Here is how you can include example code in the manual:

\include ex-load-workcell.cpp

This is equivalent to the following:

\code
#include <string>

#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

int main(int argc, char** argv)
{
    if (argc > 1) {
        const std::string file = argv[1];
        std::auto_ptr<WorkCell> workcell = WorkCellLoader::load(file);

        std::cout << "Work cell successfully loaded.\n";
        return 0;
    } else {
        std::cout << "No work cell loaded.\n";
        return 1;
    }
}
\endcode

Jeg vil foreslå at lade alle eksempler være komplette i sig selv, dvs.
hvert eksempel placeres i en fil og man kontrollerer at alle eksempler
compilerer. Se CMakeLists.txt i dette directory.

*/
