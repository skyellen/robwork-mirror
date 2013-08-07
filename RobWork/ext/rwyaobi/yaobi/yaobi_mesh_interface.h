//----------------------------------------------------------------------------
//             Yaobi - Yet Another OBB-Tree Implementation
//----------------------------------------------------------------------------
//
// Copyright (c) 2006 Morten Strandberg
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//----------------------------------------------------------------------------

#ifndef YAOBI_MESH_INTERFACE_H_
#define YAOBI_MESH_INTERFACE_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi_mesh_interface.h
//! \brief Provides an interface to the mesh data.
//!
//! \note Mesh data can be shared between Yaobi and the application, or it can
//! be owned completely by Yaobi.
//!
//! \author Morten Strandberg
//! \date   6/10 2005
//!

#include <iosfwd>
#include <cassert>

#include "yaobi_settings.h"


namespace yaobi {

//! A triangle represented as three indices into a vertex array.
struct IndexedTriangle {
  VertIndex p1;
  VertIndex p2;
  VertIndex p3;
};


//! \brief An interface to the mesh data.
//!
//! Yaobi only accesses mesh data through this interface class.
class TriMeshInterface {
public:
  //! \a mode can be either SHARE_DATA, or OWN_DATA. With \a mode = OWN_DATA, the mesh interface
  //! is responsible for deleting the mesh data.
  //! The share mode will affect the value returned by MemUsage.
  //! \see MemUsage
  //! \param[in] num_verts  The number of vertices
  //! \param[in] v          A vertex array
  //! \param[in] num_tris   The number of triangles
  //! \param[in] tris       An index array for the triangles
  //! \param[in] tri_stride Tells how many elements of \a tris each triangle uses
  //! \param[in] mode       Determines if the mesh interface is responsible for deleting the mesh data
  //!
  //! \note \a tri_stride should in most cases be 3. However, many geometry formats use a -1 to end
  //! each face. In this case \a tri_stride should be 4.
  TriMeshInterface(unsigned int num_verts, const AppRealT v[],
                   unsigned int num_tris,  const int tris[],
                   unsigned char tri_stride, ShareMode mode);
  
  ~TriMeshInterface();
  
  //! \return The number of triangles in the mesh
  unsigned int NumTriangles() const;
  
  //! \return The number of vertices in the mesh
  unsigned int NumVertices()  const;
  
  //! Returns vertex number \a n in double-precision
  //! \param[in]  n Vertex index
  //! \param[out] v The vertex
  //! \pre \a n must be less than the number of vertices. This is
  //! asserted in debug mode.
  void GetVertex(unsigned int n, double v[]) const;
  
  //! Returns vertex number \a n in single-precision
  //! \param[in]  n Vertex index
  //! \param[out] v The vertex
  //! \pre \a n must be less than the number of vertices. This is
  //! asserted in debug mode.
  void GetVertex(unsigned int n, float v[])  const;
  
  //! Returns triangle number \a n in double-precision format
  //! \param[in]  n     The triangle index
  //! \param[out] verts The triangle vertices
  //! \pre \a n must be less than the number of triangles. This is
  //! asserted in debug mode.
  void GetTriangle(unsigned int n, double verts[][3]) const;
  
  //! Returns triangle number \a n in single-precision format
  //! \param[in]  n     The triangle index
  //! \param[out] verts The triangle vertices
  //! \pre \a n must be less than the number of triangles. This is
  //! asserted in debug mode.
  void GetTriangle(unsigned int n, float verts[][3])  const;
  
  //! Returns triangle number \a n as an IndexedTriangle
  //! \pre \a n must be less than the number of triangles. This is
  //! asserted in debug mode.
  const IndexedTriangle GetIndexedTriangle(unsigned int n) const;
  
  //! \param[in] f A file pointer.
  //! If \a f is not null, then a message will be written to \a f
  //! \return The number of bytes used by the mesh interface.
  //! \note The storage for the mesh data will not be included if the mesh
  //! is shared.
  unsigned int MemUsage(FILE* f = 0) const;
  
private:
  TriMeshInterface(const TriMeshInterface& src);            //!< not defined
  TriMeshInterface& operator=(const TriMeshInterface& rhs); //!< not defined
  
  const AppRealT* const verts_;
  const int* const      tris_;
  const unsigned int    num_verts_;
  const unsigned int    num_tris_;
  const unsigned char   stride_;
  const unsigned char   own_data_;
};

//////////////////////////////////////////////////////////////////////////////
///////////////////   Inline definitions below           /////////////////////


inline
TriMeshInterface::TriMeshInterface(unsigned int   num_verts,
                                   const AppRealT v[],
                                   unsigned int   num_tris,
                                   const int      tris[],
                                   unsigned char  tri_stride,
                                   ShareMode      mode):
verts_(v),
tris_(tris),
num_verts_(num_verts),
num_tris_(num_tris),
stride_(tri_stride),
own_data_(mode == OWN_DATA? 1 : 0)
{
}

//============================================================================


inline
TriMeshInterface::~TriMeshInterface()
{
  if (own_data_) {
    delete[] verts_;
    delete[] tris_;
  }
}

//============================================================================


inline unsigned int
TriMeshInterface::NumTriangles() const
{
  return num_tris_;
}

//============================================================================


inline unsigned int
TriMeshInterface::NumVertices() const
{
  return num_verts_;
}

//============================================================================


YAOBI_INLINE void
TriMeshInterface::GetVertex(unsigned int n, double v[]) const
{
  assert((verts_ != 0) && (n < num_verts_));
  
  const AppRealT* p_v = verts_ + 3 * n;
  
  v[0] = static_cast<double>(*p_v++);
  v[1] = static_cast<double>(*p_v++);
  v[2] = static_cast<double>(*p_v);
  
  return;
}


//============================================================================


YAOBI_INLINE void
TriMeshInterface::GetVertex(unsigned int n, float v[]) const
{
  assert((verts_ != 0) && (n < num_verts_));
  
  const AppRealT* p_v = verts_ + 3 * n;
  
  v[0] = static_cast<float>(*p_v++);
  v[1] = static_cast<float>(*p_v++);
  v[2] = static_cast<float>(*p_v);
  
  return;
}

//============================================================================


YAOBI_INLINE void
TriMeshInterface::GetTriangle(unsigned int n, double verts[][3]) const
{
  assert((tris_ != 0) && (n < num_tris_));
  
  n *= stride_;
  GetVertex(tris_[n],     verts[0]);
  GetVertex(tris_[n + 1], verts[1]);
  GetVertex(tris_[n + 2], verts[2]);
  
  return;
}

//============================================================================


YAOBI_INLINE void
TriMeshInterface::GetTriangle(unsigned int n, float verts[][3]) const
{
  assert((tris_ != 0) && (n < num_tris_));
  
  n *= stride_;
  GetVertex(tris_[n],     verts[0]);
  GetVertex(tris_[n + 1], verts[1]);
  GetVertex(tris_[n + 2], verts[2]);
  
  return;
}


} // namespace yaobi

#endif // YAOBI_MESH_INTERFACE_H_
