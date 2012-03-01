
/**

\page page_coding_guidelines Code guide lines

@{

This section will present the coding guidelines that are used in RobWork. <br>

We use the following guide lines:<br>
http://geosoft.no/development/cppstyle.html

With the following exceptions:
<UL>
<LI> We name the C++ header files .hpp (instead of .h)
<LI> We name the C++ source files .cpp (instead of .c++)
<LI> We prefix member variables with _ (instead of using _ as a suffix)
<LI> We use an indentation level of 4 characters
<LI> We include RobWork headers before anything else
</UL>
In-depth explanations:

a) Naming of source files:

Source files are named in UpperCamelCase (Java style) the following suffixes should be used:<br>
C++ header files: .hpp<br>
C++ source files: .cpp<br>

As a rule of thumb: There should be one .hpp and one .cpp file for each class. The .hpp and .cpp
file should have the same name as the class


b) Include guards
Use the following includeguards:
\code
#ifndef RW_PACKAGENAME_CLASSNAME_HPP
#define RW_PACKAGENAME_CLASSNAME_HPP
//...
#endif // RW_PACKAGENAME_CLASSNAME_HPP
\endcode

example:
\code
#ifndef RW_VWORKCELL_SERIALDEVICE_HPP
#define RW_VWORKCELL_SERIALDEVICE_HPP

#endif // RW_VWORKCELL_SERIALDEVICE_HPP
\endcode
c) Use of namespaces
\code
namespace rw{
    namespace packagename{

    }
}
\endcode

do not use: "using namespace" in .hpp files. This violates the principle of namespaces
"using namespace" must only be used in .cpp files.


d) Class definitions
place public members before protected members, place protected members before private members
\code
class SerialDevice{
    public:
     ...

    protected:
     ...


    private:
     ...
};
\endcode
e) Documentation<br>
We use doxygen for documentations, doxygen tags should start with a "\@" (JavaDoc style). Brief member
descriptions should be prefixed by \@brief

We use the same writing style as used in the Java API (see http://java.sun.com/j2se/1.5.0/docs/api/)

Example of good brief member descriptions:<br>
\@brief Removes all of the elements of this collection<br>
\@brief Constructs an ActionEvent object<br>
\@brief Returns a parameter string identifying this action event<br>

Example of bad brief member descriptions:<br>
\@brief This method is used for finding the square root

There should be a space between class members and the next documentation block

Right:
\code
class Test{
    public:
    // @brief Performs the first test
    void test1();

    // @brief Performs the second test
    void test2();
};
\endcode

Wrong:
\code
class Test{
    public:

    // @brief Performs the first test
    void test1();

    // @brief Performs the first test
    void test2();
};
\endcode
f) Indentation<br>
We use indentation level 4

g) Notation for math<br>
When possible use the following notation for code and documentation:<br>
<TABLE>
<TR> <TD>Documentation</TD> <TD>doxygen</TD> <TD>code </TD> <TD>description</TD></TR>
<TR> <TD>\f$\abx{a}{b}{\bf{T}}\f$</TD> <TD>\\f$\\abx{a}{b}{\\bf{T}}\\f$ </TD> <TD>aTb</TD> <TD>Transform a to b (or b wrt. a)</TD></TR>
<TR> <TD>\f$\bf{x}\f$</TD> <TD>\\f$\\b{x}\\f$</TD> <TD>X</TD> <TD>Pose</TD></TR>
<TR> <TD>\f$\bf{d}\f$</TD> <TD>\\bf{d}</TD> <TD>d</TD> <TD>Vector</TD></TR>
<TR> <TD>\f$\hat{\bf{k}}\theta\f$</TD> <TD>\\f$\\hat{\\bf{k}}\\theta\\f$</TD> <TD>k</TD> <TD>EAA, equivalent angle and axis</TD></TR>
<TR> <TD>\f$\bf{\nu}\f$</TD> <TD>\\bf{\\nu}</TD> <TD>V</TD> <TD>VelocityScrew</TD></TR>
<TR> <TD>\f$\bf{v}\f$</TD> <TD>\\bf{v}</TD> <TD>v</TD> <TD>Linear velocity</TD></TR>
<TR> <TD>\f$\bf{\omega}\f$</TD> <TD>\\bf{\\omega}</TD> <TD>w</TD> <TD>Angular velocity</TD></TR>
<TR> <TD>\f$\bf{q}\f$</TD> <TD>\\f$\\bf{q}\\f$</TD> <TD>q</TD> <TD>Joint configuration</TD></TR>
</TABLE>

h) Include files<br>
.hpp files should be included in the follwing order:
<OL>
<LI> (for .cpp files only) ClassName.hpp
<LI> .hpp files from same namespace
<LI> RobWork .hpp files
<LI> ext includes
<LI> other includes
<LI> boost includes
<LI> stl includes
</OL>

Example.: (SerialDevice.cpp)
\code
#include "SerialDevice.hpp"

#include "Joint.hpp"
#include "kinematics/Frame.hpp"
#include "math/Vector.hpp"

#include <stdlib.h>
\endcode

For source files in test, example and demo use the above rules but include the
RobWork files as library files instead of local files (use <file.hpp> instead of "file.hpp")


i) Try to reduce .hpp dependencies<br>
Try to reduce .hpp dependencies by not including more .hpp files than absolutely necessary.
Use forward declarations when possible

j) Use tests<br>
Do not remove or comment-out tests from the test directory
Write tests

k) Use the RobWork smart pointer<br>
All classes which are expected to be passed as pointers should declare a pointer typedef using the
RobWork smart pointer rw::common::Ptr.
\code
class MyClass;

// A pointer to a MyClass
typedef rw::common::Ptr<MyClass> MyClassPtr;
\endcode

Classes taking pointers to objects should likewise use the smart pointer to determine ownership
and avoid memory leaks.

Commit guidelines:

Before you commit:<br>
make (verify that there is no build errors)
make test (verify that there is no build errors and that the changes does not break any of the unit tests)
doxygen (verify that there is no doxygen warnings)
if you added any files make sure that you have "cvs add"'ed them
verify that example, demo and RobWorkStudio still compiles

When you commit:<br>
Always specify a commit message stating what was changed (this makes it possible for other people to see
why you have changes and, more importantly, why you changed it)

Right:
\verbatim
svn commit -m "fixed bug in SerialDevice causing a crash when there was only one frame in the serial chain"
\endverbatim

Wrong:
\verbatim
svn commit -m ""
\endverbatim
@}
*/
