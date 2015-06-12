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
#include <RobWorkConfig.hpp>

#if defined(__cplusplus)
extern "C"
{
#endif
#include <stdio.h>
#include <stdlib.h>

// This distinction is necessary because qhull changed structure at some point
#ifdef HAVE_QHULL_2011
#include <libqhull/libqhull.h>
#include <libqhull/mem.h>
#include <libqhull/qset.h>
#include <libqhull/geom.h>
#include <libqhull/merge.h>
#include <libqhull/poly.h>
#include <libqhull/io.h>
#include <libqhull/stat.h>
#else
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
#endif

//#include <qhull/libqhull.h>

#if defined(__cplusplus)
}
#endif

#include <boost/numeric/ublas/vector.hpp>
#include <boost/thread.hpp>

using namespace std;
using namespace rw::geometry;
using namespace boost::numeric;
// Contact pos, normal, hastighed, depth, idA, idB

boost::mutex _qhullMutex;
static bool firstCalll = true;

void qhull::build(size_t dim,
                  double *coords,
                  size_t nrCoords,
                  std::vector<int>& vertIdxs,
                  std::vector<int>& faceIdxs,
                  std::vector<double>& faceNormals,
                  std::vector<double>& faceOffsets)
{
    boost::mutex::scoped_lock lock(_qhullMutex);

    vertIdxs.clear();
    faceIdxs.clear();
    faceNormals.clear();
    faceOffsets.clear();

    std::vector<int>& result = vertIdxs;

    int curlong, totlong; /* used !qh_NOmem */
    int exitcode;
    boolT ismalloc = false;
    int argc = 0;
    char** argv = NULL;

    #if qh_QHpointer==1
    if( firstCalll ){
    	qh_init_A(0, 0, stderr, argc, argv); /* sets qh qhull_command */
    	firstCalll = false;
    }
	#else
    	qh_init_A(0, 0, stderr, argc, argv); /* sets qh qhull_command */
	#endif

	//qh_init_A (stdin, stdout, stderr, argc, argv);  /* sets qh qhull_command */
	//char flags[] = "qhull Qt Pp Qs";

	//char flags[] = "qhull Pp n Qt Qx QJ C-0.0001";
	//char flags[] = "qhull Pp Qs QJ C-0.0001 n";
	//char flags[] = "qhull Qx Qs W1e-1 C1e-2 Qt Pp n"; //graspit
	//char flags[] = "qhull Qs Pp Qt n";

	char flags[] = "qhull Pp QJ";

	// WHEN DYNAMIC LINKING qh_QHpointer gets set to 1. Requires some fixes
	#if qh_QHpointer==1
	// for some reason i had to apply this HACK in order to get QHull working...
	qhT *qht = qh_save_qhull();
	exitcode = qh_new_qhull((int)dim, (int)nrCoords, coords, ismalloc, flags, NULL, stderr);
	#else
	exitcode = qh_new_qhull((int)dim, (int)nrCoords, coords, ismalloc, flags, NULL, stderr);
	#endif


    if (!exitcode) {
        //        facetT *facet;
        ublas::vector<double> center = ublas::zero_vector<double>(dim);
        ublas::vector<double> vert = ublas::zero_vector<double>(dim);
        size_t nrVerts = 0;
        vertexT *vertex;//, **vertexp;
        FORALLvertices {
            int vertexIdx = qh_pointid(vertex->point);
            result.push_back(vertexIdx);

            for(size_t i=0; i<dim; i++){
                double val = vertex->point[i];
                center[i] = center[i] + val;
            }
            nrVerts++;
            //result->push_back(contacts->at(vertexIdx));
            //contacts->at(vertexIdx)=NULL;
        }
        center /= (double)nrVerts;

        // also find all facets, such that we can recreate the hull
        ublas::vector<double> zerov = ublas::zero_vector<double>(dim);
        facetT *facet;
        FORALLfacets {
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
    if (qh VERIFYoutput && !qh FORCEoutput && !qh STOPpoint && !qh STOPcone)
        qh_check_points();

    qh_freeqhull( False );
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong)
        fprintf (stderr, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",totlong, curlong);
    //delete[] coords;

	#if qh_QHpointer
	qh_restore_qhull(&qht);
	#endif


}
