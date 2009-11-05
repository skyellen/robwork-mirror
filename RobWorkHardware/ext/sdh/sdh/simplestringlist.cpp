//======================================================================
/*!
  \file
  \section sdhlibrary_cpp_simplestirnglist_cpp_general General file information

    \author   Dirk Osswald
    \date     2007-02-19

  \brief
    Implementation of class #SDH::cSimpleStringList.

  \section sdhlibrary_cpp_simplestirnglist_cpp_copyright Copyright

  - Copyright (c) 2007 SCHUNK GmbH & Co. KG

  <HR>
  \internal

    \subsection sdhlibrary_cpp_simplestirnglist_cpp_details SVN related, detailed file specific information:
      $LastChangedBy: Osswald2 $
      $LastChangedDate: 2008-10-08 10:48:38 +0200 (Mi, 08 Okt 2008) $
      \par SVN file revision:
        $Id: simplestringlist.cpp 3659 2008-10-08 08:48:38Z Osswald2 $

  \subsection sdhlibrary_cpp_simplestirnglist_cpp_changelog Changelog of this file:
      \include simplestringlist.cpp.log
*/
//======================================================================

//----------------------------------------------------------------------
// System Includes - include with <>
//----------------------------------------------------------------------

#include <assert.h>

//----------------------------------------------------------------------
// Project Includes - include with ""
//----------------------------------------------------------------------

#include "dbg.h"
#include "simplestringlist.h"

USING_NAMESPACE_SDH

//----------------------------------------------------------------------
// Defines, enums, unions, structs,
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Global variables
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Function definitions
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Class definitions
//----------------------------------------------------------------------


cSimpleStringList::cSimpleStringList()
{
    Reset();
}
//-----------------------------------------------------------------


char* cSimpleStringList::CurrentLine()
{
    return line[ current_line ];
}
//-----------------------------------------------------------------


char* cSimpleStringList::NextLine()
{
    current_line++;
    assert( current_line < eMAX_LINES );
    return line[ current_line ];
}
//-----------------------------------------------------------------


int cSimpleStringList::Length() const
{
    return current_line+1;
}
//-----------------------------------------------------------------


char* cSimpleStringList::operator[]( int index )
{
    int i;
    if (index < 0)
        i = Length() + index;
    else
        i = index;

    assert( 0 <= i  &&  i <= current_line );
    return line[ i ];
}
//-----------------------------------------------------------------


void cSimpleStringList::Reset()
{
    current_line = -1;
}
//-----------------------------------------------------------------


NAMESPACE_SDH_START

std::ostream &operator<<( std::ostream &stream, cSimpleStringList& ssl )
{
    for ( int i = 0; i < ssl.Length(); i++ )
        stream << "line[" << i << "]='" << ssl[i] << "'\n";

    return stream;
}

NAMESPACE_SDH_END

//-----------------------------------------------------------------



//======================================================================
/*
  Here are some settings for the emacs/xemacs editor (and can be safely ignored):
  (e.g. to explicitely set C++ mode for *.h header files)

  Local Variables:
  mode:C++
  mode:ELSE
  End:
*/
//======================================================================
