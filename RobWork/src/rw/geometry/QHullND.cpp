/********************************************************************************
 * Copyright 2009-2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#include "QHullND.hpp"

#if defined(__cplusplus)
extern "C"
{
#endif
#include <stdio.h>
#include <stdlib.h>

#include <libqhull_r/libqhull_r.h>

#if defined(__cplusplus)
}
#endif

using namespace std;
using namespace rw::geometry;

void qhull::build(size_t dim,
                  double *coords,
                  size_t nrCoords,
                  std::vector<int>& vertIdxs,
                  std::vector<int>& faceIdxs,
                  std::vector<double>& faceNormals,
                  std::vector<double>& faceOffsets)
{
    vertIdxs.clear();
    faceIdxs.clear();
    faceNormals.clear();
    faceOffsets.clear();

    std::vector<int>& result = vertIdxs;

    int curlong, totlong; /* used !qh_NOmem */
    int exitcode;
    boolT ismalloc = false;
    qhT qh;
    qhT * qhT_pointer = &qh; // allocate memory for qhull

    //qh_init_A (stdin, stdout, stderr, argc, argv);  /* sets qh qhull_command */
    //char flags[] = "qhull Qt Pp Qs";

    //char flags[] = "qhull Pp n Qt Qx QJ C-0.0001";
    //char flags[] = "qhull Pp Qs QJ C-0.0001 n";
    //char flags[] = "qhull Qx Qs W1e-1 C1e-2 Qt Pp n"; //graspit
    //char flags[] = "qhull Qs Pp Qt n";

    char flags[] = "qhull Pp QJ";

    // According to doc. in libqhull_r/user_r.c, the qhull memory should be cleared:
    qh_zero(qhT_pointer, stderr);

    // Then the object is constructed.
    exitcode = qh_new_qhull(qhT_pointer, (int)dim, (int)nrCoords, coords, ismalloc, flags, NULL, stderr);


    if (!exitcode) {
        //        facetT *facet;
        //ublas::vector<double> center = ublas::zero_vector<double>(dim);
        //ublas::vector<double> vert = ublas::zero_vector<double>(dim);

        // Loop through all vertices,
        //size_t nrVerts = 0;
        vertexT *vertex;//, **vertexp;
        for (vertex=qhT_pointer->vertex_list;vertex && vertex->next;vertex= vertex->next)
        {
            int vertexIdx = qh_pointid(qhT_pointer,vertex->point);
            result.push_back(vertexIdx);

            //for(size_t i=0; i<dim; i++){
                //double val = vertex->point[i];
                //center[i] = center[i] + val;
            //}
            //nrVerts++;
            //result->push_back(contacts->at(vertexIdx));
            //contacts->at(vertexIdx)=NULL;
        }
        //center /= (double)nrVerts;

        // also find all facets, such that we can recreate the hull
        //ublas::vector<double> zerov = ublas::zero_vector<double>(dim);

        // For all facets:
        facetT *facet;
        for (facet=qhT_pointer->facet_list; facet && facet->next; facet = facet->next)
        {
            /*
            std::cout << "CENTER: ";
            for(int i=0;i<dim;i++){
                std::cout << facet->center[i] << ", ";
            }
            std::cout << std::endl;
             */
            //std::cout << facet->vertices->maxsize << std::endl;
            int vertex_n, vertex_i;
            //std::cout << "{ ";
            // if offset is positive then the center is outside
            //ublas::vector<double> n = ublas::zero_vector<double>(dim);
            //ublas::vector<double> v = ublas::zero_vector<double>(dim);
            for(size_t j=0;j<dim;j++){
                faceNormals.push_back( facet->normal[j] );
            }
            faceOffsets.push_back(facet->offset);

            //int idx = 0;
            FOREACHvertex_i_(qhT_pointer,facet->vertices)
            {
                int vertexIdx = qh_pointid(qhT_pointer,vertex->point);
                faceIdxs.push_back(vertexIdx);
                //idx = vertexIdx;
                //std::cout << vertexIdx << " ";
            }
            //std::cout << idx << std::endl;
            //for(size_t j=0;j<dim;j++){
            //    v[j] = coords[dim*idx+j];
            //}
            //n = n/norm_2(n);

            //double dist;
            //qh_distplane(&zerov[0],facet,&dist);
            //if( dist>0 ){
            //    std::cout << "GRASP IS NOT FORCE CLOSURE" << std::endl;
            //}


            //std::cout << " }\n";
        }

    } else {
        cout<<"Not Exit Code!";
    }

    if (qhT_pointer->VERIFYoutput && !qhT_pointer->FORCEoutput && !qhT_pointer->STOPpoint && !qhT_pointer->STOPcone)
        qh_check_points(qhT_pointer);

    qh_freeqhull(qhT_pointer, False);
    qh_memfreeshort (qhT_pointer,&curlong, &totlong);
    if (curlong || totlong)
        fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);
    //delete[] coords;

    #if qh_QHpointer
    qh_restore_qhull(&qht);
    #endif


}
