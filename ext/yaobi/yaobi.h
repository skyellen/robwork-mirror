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

#ifndef YAOBI_H_
#define YAOBI_H_

//////////////////////////////////////////////////////////////////////////////
//!
//! \file yaobi.h
//! \brief The Yaobi interface for collision queries
//!
//! \author Morten Strandberg
//! \date   6/10 2005
//!

#include <cstdio>

#include "yaobi_settings.h"
#include "yaobi_vector.h"

//! The yaobi namespace
namespace yaobi {

//! Return value if operation was successful.
#define YAOBI_OK  0

//! The given collision model had no mesh. That is, the mesh pointer was null.
#define YAOBI_NULL_MESH  (-1)

//! The mesh did not contain any triangles
#define YAOBI_NO_TRIANGLES  (-2)

//! The mesh had less than 3 vertices.
#define YAOBI_TOO_FEW_VERTS  (-3)

//! The collision model was invalid for some reason
#define YAOBI_INVALID_MODEL  (-4)

//============================================================================


//! Used to determine the type of collision query.
enum QueryType {
  //! Return as soon as a contact is found
  FIRST_CONTACT_ONLY,
  
  //! Search for all contacting triangles
  ALL_CONTACTS
};


// forward declarations
class NodeStack;
class TriMeshInterface;

struct TriNode;
struct OBB_Node;
struct CollideResult;

//! \brief Represents an OBB-tree and a set of triangles.
//!
//! This is the most central Yaobi class.
class CollModel {
public:
  CollModel();
  
  //! If \a mode == OWN_DATA, then the destructor will delete the given mesh.
  CollModel(const TriMeshInterface* mesh, ShareMode mode);
  
  ~CollModel();
  
  //! If \a mode == OWN_DATA, then the destructor will delete the given mesh.
  void SetTriMesh(const TriMeshInterface* mesh, ShareMode mode);
  
  //! \return TRUE if the collision model is valid.
  Bool IsValid() const;
  
  //! \return TRUE if the collision model owns the mesh data.
  Bool OwnsMesh() const;
  
  //! \param[in] f A file pointer.
  //! If \a f is not null, then a message will be written to \a f
  //! \return The number of bytes used by the collision model
  //! \note The storage for the mesh data will not be included if the mesh
  //! is shared.
  unsigned int MemUsage(FILE* f) const;
  
  //! returns unused memory.
  //! \note Clients do not have to use this function if the model was built
  //! using the function build_obb_tree().
  void ShrinkToFit();
  
private:
  enum { OWN_MESH = (1U << 0),
         IS_VALID = (1U << 1) };
  
  friend int build_obb_tree(CollModel& m, unsigned int flags);
  
  friend int Collide(CollideResult& res,
                     const Real ta[][4], const CollModel& a,
                     const Real tb[][4], const CollModel& b,
                     QueryType qtype);
  
  CollModel(const CollModel& src);            //!< not defined
  CollModel& operator=(const CollModel& rhs); //!< not defined
  
  //! Clears everything but the mesh data.
  //!
  //! This function is used when rebuilding the OBB-tree.
  void Clear();
  
  const TriMeshInterface* tri_mesh;
  
  OBB_Node*    obb_nodes;
  unsigned int num_obbs;
  unsigned int num_obbs_alloced;
  
  TriNode*     tri_nodes;
  unsigned int num_tri_nodes;
  unsigned int num_tri_nodes_alloced;
  unsigned int state;
};

//! \brief Represents a pair of colliding triangles
struct CollisionPair
{
  int id1; //!< Index to triangle in first object
  int id2; //!< Index to triangle in second object
};


//! \brief Contains the result from a collision query.
struct CollideResult
{
  // statistics variables
  unsigned int num_bv_tests;      //!< The number of bounding-volume tests
  unsigned int num_tri_tests;     //!< The number of triangle overlap tests
  unsigned int num_tri_box_tests; //!< The number of triangle-box overlap tests
  
  //! Transform from model 1 to model 2
  Real obj2_rel_obj1[12];
  
  
  unsigned int num_pairs_alloced; //!< The number of allocated triangle pairs
  unsigned int num_pairs;         //!< The number of colliding triangles
  CollisionPair* pairs;           //!< Pointer to allocated triangle pairs
  
  //! Used internally to avoid recursion.
  NodeStack* stack;
  
  //! The ctor argument controls the intial stack size. The stack will
  //! allocate more memory if needed.
  CollideResult(unsigned int stack_size = 10);
  ~CollideResult();
  
  //! Reserve space for more colliding triangles
  void Reserve(unsigned int n);
  
  //! Add a pair of colliding triangles.
  void Add(int i1, int i2);
  
  //! Clears both the statistics variables and the list of colliding triangles.
  //!
  //! \note Clients do not have to call this as it is called inside the
  //! function \a Collide().
  void Clear();
  
  
  //! \brief The number of bounding-volume tests
  unsigned int NumBVTests()     const { return num_bv_tests;      }
  
  //! \brief The number of triangle overlap tests
  unsigned int NumTriTests()    const { return num_tri_tests;     }
  
  //! \brief The number of triangle-box overlap tests
  unsigned int NumTriBoxTests() const { return num_tri_box_tests; }
  
  //! Free the list of contact pairs; ordinarily this list is reused
  //! for each query, and only deleted in the destructor.
  void FreePairsList();
  
  
  //! \return TRUE if two models are colliding
  Bool IsColliding()      const { return (num_pairs != 0); }
  
  //! \brief The number of colliding triangle pairs
  unsigned int NumPairs() const { return num_pairs;        }
  
  //! \return id of the first triangle in collision pair \a k
  //! \pre \a k must be less than NumPairs. This is an unchecked condition.
  int Id1(unsigned int k) const { return pairs[k].id1;     }
  
  //! \return id of the second triangle in collision pair \a k
  //! \pre \a k must be less than NumPairs. This is an unchecked condition.
  int Id2(unsigned int k) const { return pairs[k].id2;     }
  
private:
  CollideResult(const CollideResult& src);            //!< not defined
  CollideResult& operator=(const CollideResult& rhs); //!< not defined
};

//! \param[out] res  The result of the collision query
//! \param[in]  ta   The rigid body transformation for model \a a
//! \param[in]  a    A collision model
//! \param[in]  tb   The rigid body transformation for model \a b
//! \param[in]  b    A collision model
//! \param[in] qtype Determines the type of query.
//! \return YAOBI_OK if the collision query went OK. The actual result is stored in \a res.
int Collide(CollideResult& res,
            const Real ta[][4], const CollModel& a,
            const Real tb[][4], const CollModel& b,
            QueryType qtype = FIRST_CONTACT_ONLY);


//////////////////////////////////////////////////////////////////////////////
///////////       Inline definitions below              //////////////////////


inline Bool
CollModel::IsValid() const
{
  return (state & IS_VALID) != 0;
}

//============================================================================


inline Bool
CollModel::OwnsMesh() const
{
  return (state & OWN_MESH) != 0;
}

//============================================================================


inline void
CollideResult::Clear()
{
  num_bv_tests      = 0;
  num_tri_tests     = 0;
  num_tri_box_tests = 0;
  num_pairs         = 0;
  
  return;
}


} // namespace yaobi

#endif /// YAOBI_H_
