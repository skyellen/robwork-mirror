/****************************************************************************
**
** Copyright (C) 2009-2010 C.B. Barber. All rights reserved.
** $Id: //product/qhull/main/rel/cpp/QhullVertexSet.h#9 $$Change: 1164 $
** $DateTime: 2010/01/07 21:52:00 $$Author: bbarber $
**
****************************************************************************/

#ifndef QHULLVERTEXSET_H
#define QHULLVERTEXSET_H

#include "QhullSet.h"

#include <ostream>

namespace orgQhull {

#//ClassRef
    class               QhullVertex;

#//Types
    //! QhullVertexSet -- a set of Qhull Vertices, as a C++ class.
    //! See Qhull
    class               QhullVertexSet;
    typedef QhullSetIterator<QhullVertex>
                        QhullVertexSetIterator;

class QhullVertexSet : public QhullSet<QhullVertex> {

private:
#//Fields
    Qhull              *qhsettemp_qhull; //! For sets allocated with qh_settemp()
    bool                qhsettemp_defined;  //! Set was allocated with q_memalloc()

public:
#//Constructor
                        //Conversion from setT* is not type-safe.  Implicit conversion for void* to T
   explicit             QhullVertexSet(setT *s) : QhullSet<QhullVertex>(s), qhsettemp_qhull(0), qhsettemp_defined(false) {}
                        QhullVertexSet(int qhRunId, facetT *facetlist, setT *facetset, bool allfacets);
                        //Copy constructor copies pointer but not contents.  Needed for return by value.
                        QhullVertexSet(const QhullVertexSet &o) : QhullSet<QhullVertex>(o), qhsettemp_qhull(o.qhsettemp_qhull), qhsettemp_defined(o.qhsettemp_defined) {}
                       ~QhullVertexSet();

private:
                        //!Disable default constructor and copy assignment.  See QhullSetBase
                        QhullVertexSet();
    QhullVertexSet      &operator=(const QhullVertexSet &);
public:

#//Constructor, destructor
    void                freeQhSetTemp();

#//IO
    struct PrintVertexSet{
        const QhullVertexSet *Vertex_set;
        const char     *message;
        int             run_id;
                        PrintVertexSet(int qhRunId, const char *message, const QhullVertexSet *s) : Vertex_set(s), message(message), run_id(qhRunId) {}
    };//PrintVertexSet
    const PrintVertexSet       print(int qhRunId, const char *message) const { return PrintVertexSet(qhRunId, message, this); }

    struct PrintIdentifiers{
        const QhullVertexSet *Vertex_set;
        const char     *message;
                        PrintIdentifiers(const char *message, const QhullVertexSet *s) : Vertex_set(s), message(message) {}
    };//PrintIdentifiers
    PrintIdentifiers    printIdentifiers(const char *message) const { return PrintIdentifiers(message, this); }

};//class QhullVertexSet

}//namespace orgQhull

#//== Global namespace =========================================

std::ostream &operator<<(std::ostream &os, const orgQhull::QhullVertexSet::PrintVertexSet &pr);
std::ostream &operator<<(std::ostream &os, const orgQhull::QhullVertexSet::PrintIdentifiers &p);
inline std::ostream &operator<<(std::ostream &os, const orgQhull::QhullVertexSet &vs) { os << vs.print(orgQhull::UsingLibQhull::NOqhRunId, ""); return os; }

#endif // QHULLVERTEXSET_H
