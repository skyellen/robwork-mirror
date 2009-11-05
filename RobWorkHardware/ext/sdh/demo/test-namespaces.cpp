//======================================================================
/*!
  \file
  \section cpp_demo_test-namespaces_c_general General file information

    \author   Osswald2
    \date     02.10.2008

  \brief
    Implementation of general test of access to operator<< declared in a namespace.

  \section cpp_demo_test-namespaces_c_copyright Copyright

  Copyright (c) 2008 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection cpp_demo_test-namespaces_c_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
      \par SVN file revision:
        $Id: test-namespaces.cpp 3659 2008-10-08 08:48:38Z Osswald2 $

  \subsection cpp_demo_test-namespaces_c_changelog Changelog of this file:
      \include test-namespaces.cpp.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <iostream>
#include <vector>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function implementation (function definitions)
//----------------------------------------------------------------------

namespace myNamespace
{
    int myFunc(int p)
    {
        return p;
    }

    struct myStruct {
        int i;
    };

    std::ostream & operator<<( std::ostream &stream,  myStruct const &s )
    {
        stream << "i=" << s.i << "\n";

        return stream;
    }

    template<typename T>
    std::ostream& operator<<(std::ostream& stream, std::vector<T> v)
    {
        stream << "|v| = " << v.size() << "\n";

        return stream;
    }
}


int main( int argc, char** argv )
{
    std::cout << myNamespace::myFunc( 42 ) << "\n";

    myNamespace::myStruct s;

    std::cout << s;

    std::vector<int> v;
    v.push_back( 42 );

    // accessing the std::ostream& operator<<(std::ostream& stream, std::vector<T> v)
    // declared in the myNamespace namespace is not that easy:

    // this one would obviously fail with a "no match for >>operator<<<< in >>std::cout << v<<" as expected:
    //std::cout << v;

    // the following tries would yield syntax errors:
    //std::cout myNamespace :: << v;
    //std::cout myNamespace :: operator << v;

    // So without a 'using namespace' directive the call to
    // the operator<< defined in namespace myNamespace is somewhat ugly:
    myNamespace::operator <<( std::cout, v );

    {
        // with a 'using namespace' directive it is as expected:
        using namespace myNamespace;
        std::cout << v;
    }
}

//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C
  mode:ELSE
  End:
*/
//======================================================================
