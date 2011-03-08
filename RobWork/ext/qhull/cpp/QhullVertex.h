/****************************************************************************
**
** Copyright (C) 2008-2010 C.B. Barber. All rights reserved.
** $Id: //product/qhull/main/rel/cpp/QhullVertex.h#27 $$Change: 1193 $
** $DateTime: 2010/01/23 11:31:35 $$Author: bbarber $
**
****************************************************************************/

#ifndef QHULLVERTEX_H
#define QHULLVERTEX_H

#include "UsingLibQhull.h"
#include "QhullPoint.h"
#include "QhullLinkedList.h"
#include "QhullSet.h"
extern "C" {
    #include "../src/qhull_a.h"
};

#include <ostream>

namespace orgQhull {

#//ClassRef
    class QhullFacetSet;

#//Types
    //! QhullVertex -- Qhull's vertex structure, vertexT [libqhull.h], as a C++ class
    class QhullVertex;
    typedef QhullLinkedList<QhullVertex> QhullVertexList;
    typedef QhullLinkedListIterator<QhullVertex> QhullVertexListIterator;


/*********************
  topological information:
    next,previous       doubly-linked list of all vertices
    neighborFacets           set of adjacent facets (only if qh.VERTEXneighbors)

  geometric information:
    point               array of DIM coordinates
*/

class QhullVertex {

private:
#//Fields
    vertexT            *qh_vertex;

#//Class objects
    static vertexT      s_empty_vertex;  // needed for shallow copy

public:
#//Constants

#//Constructors
                        QhullVertex() : qh_vertex(&s_empty_vertex) {}
                        // Creates an alias.  Does not copy QhullVertex.  Needed for return by value and parameter passing
                        QhullVertex(const QhullVertex &o) : qh_vertex(o.qh_vertex) {}
                        // Creates an alias.  Does not copy QhullVertex.  Needed for vector<QhullVertex>
    QhullVertex        &operator=(const QhullVertex &o) { qh_vertex= o.qh_vertex; return *this; }
                       ~QhullVertex() {}

#//Conversion
                        //Implicit conversion from vertexT
                        QhullVertex(vertexT *v) : qh_vertex(v ? v : &s_empty_vertex) {}
    vertexT            *getVertexT() const { return qh_vertex; }

#//QhullSet<QhullVertex>
    vertexT            *getBaseT() const { return getVertexT(); }

#//getSet
    int                 dimension() const { return (qh_vertex->dim || !isDefined()) ? qh_vertex->dim : UsingLibQhull::globalVertexDimension(); }
    int                 id() const { return qh_vertex->id; }
    bool                isDefined() const { return qh_vertex != &s_empty_vertex; }
    QhullVertex         next() const { return qh_vertex->next; }
    bool                operator==(const QhullVertex &o) const { return qh_vertex==o.qh_vertex; }
    bool                operator!=(const QhullVertex &o) const { return !operator==(o); }
    QhullPoint          point() const { return QhullPoint(dimension(), qh_vertex->point); }
    QhullVertex         previous() const { return qh_vertex->previous; }

#//ForEach
    //See also QhullVertexList
    QhullFacetSet       neighborFacets() const;

#//IO
    struct PrintVertex{
        const QhullVertex *vertex;
        int             run_id;
                        PrintVertex(int qhRunId, const QhullVertex &v) : vertex(&v), run_id(qhRunId) {}
    };//PrintVertex
    PrintVertex         print(int qhRunId) const { return PrintVertex(qhRunId, *this); }
};//class QhullVertex

}//namespace orgQhull

#//GLobal

std::ostream &operator<<(std::ostream &os, const orgQhull::QhullVertex::PrintVertex &pr);
inline std::ostream &operator<<(std::ostream &os, const orgQhull::QhullVertex &v) { os << v.print(orgQhull::UsingLibQhull::NOqhRunId); return os; }

#endif // QHULLVERTEX_H
