/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
#include <qhull/src/qhull.h>
#include <qhull/src/mem.h>
#include <qhull/src/qset.h>
#include <qhull/src/geom.h>
#include <qhull/src/merge.h>
#include <qhull/src/poly.h>
#include <qhull/src/io.h>
#include <qhull/src/stat.h>

#include <qhull/src/libqhull.h>
#if defined(__cplusplus)
}
#endif

#include <boost/numeric/ublas/vector.hpp>

using namespace std;
using namespace rw::geometry;
using namespace boost::numeric;
// Contact pos, normal, hastighed, depth, idA, idB

void qhull::build(size_t dim, double *coords, size_t nrCoords, std::vector<int>& vertIdxs, std::vector<int>& faceIdxs, std::vector<double>& faceNormals){
    vertIdxs.clear();
    faceIdxs.clear();
    faceNormals.clear();

    std::vector<int>& result = vertIdxs;

    int curlong, totlong; /* used !qh_NOmem */
    int exitcode;
    boolT ismalloc = false;
    int argc = 0;
    char** argv = NULL;
    qh_init_A(0, 0, stderr, argc, argv); /* sets qh qhull_command */
    //qh_init_A (stdin, stdout, stderr, argc, argv);  /* sets qh qhull_command */
    //char flags[] = "qhull Qt Pp Qs";
    char flags[] = "qhull Pp n Qt C-0.0001";

    exitcode = qh_new_qhull(dim, nrCoords, coords, ismalloc, flags, NULL, stderr);

    if (!exitcode) {
        //        facetT *facet;
        vertexT *vertex;//, **vertexp;
        FORALLvertices {
            int vertexIdx = qh_pointid(vertex->point);
            result.push_back(vertexIdx);
            //result->push_back(contacts->at(vertexIdx));
            //contacts->at(vertexIdx)=NULL;
        }
        // also find all facets, such that we can recreate the hull
        ublas::vector<double> zerov = ublas::zero_vector<double>(dim);
        facetT *facet;
        FORALLfacets {
            //std::cout << facet->vertices->maxsize << std::endl;
            int     vertex_n, vertex_i;
            //std::cout << "{ ";
            // if offset is positive then the center is outside
            //ublas::vector<double> n = ublas::zero_vector<double>(dim);
            //ublas::vector<double> v = ublas::zero_vector<double>(dim);
            for(size_t j=0;j<dim;j++){
                faceNormals.push_back( facet->normal[j] );
                //n[j] = facet->normal[j];
            }

            //int idx = 0;
            FOREACHvertex_i_(facet->vertices){
                int vertexIdx = qh_pointid(vertex->point);
                faceIdxs.push_back(vertexIdx);
                //idx = vertexIdx;
                //std::cout << vertexIdx << " ";
            }
            //std::cout << idx << std::endl;
            //for(size_t j=0;j<dim;j++){
            //    v[j] = coords[dim*idx+j];
            //}
            //n = n/norm_2(n);
            /*
            if( facet->offset>0 ){
                std::cout << "UNSECURE GRASP";
                std::cout << "DIST " << inner_prod(n, v) << "\n";
                //std::cout << n[0] << ";" << n[1] << "   "  << v[0] << ";" << v[1] << std::endl;
            } else {
                std::cout << "DIST " << inner_prod(n, v) << "\n";
                //std::cout << n[0] << ";" << n[1] << "   "  << v[0] << ";" << v[1] << std::endl;
            }
            */

            //std::cout << " }\n";
        }


    } else {
        cout<<"Not Exit Code!";
    }
    if (qh VERIFYoutput && !qh FORCEoutput && !qh STOPpoint && !qh STOPcone)
        qh_check_points();

    qh_freeqhull( False );
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong)
        fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);
    //delete[] coords;

}
