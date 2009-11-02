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


//////////////////////////////////////////////////////////////////////////////
//
// Author: Morten Strandberg
// Date:   6/11 2005
//
//

#include <ostream>
#include <stdio.h> // for mingw4.4 changed by jimmy jorgensen
#include "yaobi_mesh_interface.h"


namespace yaobi {


const IndexedTriangle
TriMeshInterface::GetIndexedTriangle(unsigned int n) const
{
  assert(n < num_tris_);
  
  IndexedTriangle tri;
  n *= stride_;
  
  tri.p1 = static_cast<yaobi::VertIndex>(tris_[n++]);
  tri.p2 = static_cast<yaobi::VertIndex>(tris_[n++]);
  tri.p3 = static_cast<yaobi::VertIndex>(tris_[n]);
  
  assert(tri.p1 >= 0 && tri.p1 < num_verts_);
  assert(tri.p2 >= 0 && tri.p2 < num_verts_);
  assert(tri.p3 >= 0 && tri.p3 < num_verts_);
  
  return tri;
}

//============================================================================


unsigned int
TriMeshInterface::MemUsage(FILE* f) const
{
  unsigned int verts_mem = 0;
  unsigned int tris_mem  = 0;
  unsigned int mem_usage = sizeof(TriMeshInterface);
  
  if (own_data_) {
    verts_mem = num_verts_ * 3 * sizeof(AppRealT);
    tris_mem  = num_tris_ * stride_ * sizeof(int);
    
    mem_usage += verts_mem;
    mem_usage += tris_mem;
  }
  
  if (f != 0) {
    fprintf(f, "Mesh interface uses %u bytes %s\n", mem_usage, own_data_? "(mesh data included)" :
                                                                          "(mesh data excluded)");
  }
  
  return mem_usage;
}


} // namespace yaobi

