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

#include "ContactCluster.hpp"

#include "ContactPoint.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/MetricUtil.hpp>

using namespace rw::math;
using namespace rwsim::dynamics;

namespace {

    struct FilterState {
    public:
        //FilterState(int *iSrc, int fSize, )
        int *cIdxSrc;
        int frontSize;
        int frontSrcIdx;
        int backSize;
        int *cIdxDst;
        void print(){
             std::cout  << "FilterState: (" << frontSize << "," << backSize << ","
                      << cIdxSrc << "," << cIdxDst << ")" << std::endl;
        }
    };

    /**
     * @brief clusters together contactpoints using a simple distance
     * threshold strategy.
     * @param src
     * @param srcSize
     * @param state
     * @param maxDist
     * @return The deepest penetrating point with a normal that is the
     * average of all contact points in the cluster.
     */
    ContactPoint filterContactsRec(
                const ContactPoint src[],
                const int srcSize,
                FilterState &state,
                const double maxDist)
    {
       // locate deepest point from frontBuffer
       //// std::cout  << "fileterContactRec" << std::endl;
       int deepIdx = srcSize-1;
       //// std::cout  << itmp << std::endl;
       float deepPen = src[ state.cIdxSrc[deepIdx] ].penetration;

       for(int j=0; j<state.backSize; j++){
           const int i = srcSize-1-j;
           //// std::cout  << "i: " << i;
           RW_ASSERT( i>=0 && i<srcSize );
           if(src[state.cIdxSrc[i]].penetration>deepPen){
        	   RW_ASSERT( state.cIdxSrc[i]>=0 && state.cIdxSrc[i]<srcSize);
               deepPen = src[state.cIdxSrc[i]].penetration;
               deepIdx = i;
           }
       }

       //// std::cout  << "swap" << std::endl;
       // the deepest point
       RW_ASSERT( deepIdx>=0 && deepIdx<srcSize);
       ContactPoint dpoint = src[state.cIdxSrc[deepIdx]];

       //dContactGeom deepGeom = src[state.cIdxSrc[deepIdx]].geom;
       //// std::cout  << "normal" << std::endl;
       dpoint.n = Vector3D<>(0,0,0);
       int frontIdx=state.frontSize, backIdx = srcSize-1;


       RW_ASSERT( frontIdx>=0 && frontIdx<srcSize);
       int deepSrcIdx = state.cIdxSrc[deepIdx];
       state.cIdxDst[frontIdx] = deepSrcIdx;
       frontIdx++;
       RW_ASSERT( deepIdx>=0 && deepIdx<srcSize);
       state.cIdxSrc[deepIdx] = -1;
       //std::cout << "DeepIdx: " << deepIdx << " " << deepSrcIdx << std::endl;


       // deepest point is at conIdx[0]
       // find all points in frontBuffer and back buffer that is within maxDist
       const double maxDist2 = maxDist*2;
       //// std::cout  << "front" << std::endl;
       // don't look at the first element since its the deepest
//       for(int i=1; i<state.frontSize; i++){
       for(int j=0; j<state.backSize; j++){
           const int i = srcSize-1-j;
           //const dContactGeom &geom = src[state.cIdxSrc[i]].geom;
           if(state.cIdxSrc[i]<0)
               continue;
           const ContactPoint &point = src[state.cIdxSrc[i]];
           double dist = MetricUtil::dist2(dpoint.p, point.p);
           // std::cout << dist << "<" << maxDist << std::endl;
           if( dist < maxDist){
               // merge normal
               dpoint.n += point.n;
               RW_ASSERT( frontIdx>=0 && frontIdx<srcSize );
               state.cIdxDst[frontIdx] = state.cIdxSrc[i];
               frontIdx++;
           } else {
        	   RW_ASSERT(backIdx>=0 && backIdx<srcSize);
               state.cIdxSrc[backIdx] = state.cIdxSrc[i];
               backIdx--;
           }
       }
       state.frontSize = frontIdx;
       state.backSize = (srcSize-1)-backIdx;

       //state.cIdxSrc[++state.frontSrcIdx] = deepSrcIdx;

       normalize( dpoint.n );
       return dpoint;
    }
}

int ContactCluster::thresClustering(
                ContactPoint src[], int srcCnt,
                int *cIdxSrc, int *cIdxDst,
                ContactPoint dst[],
                double maxDist)
{
   if(srcCnt==0){
       return 0;
   }

   // first locate deepest penetrating point
   //// std::cout  << "Initialize srcindex: " << std::endl;
   for(int i=0;i<srcCnt;i++){
       cIdxSrc[i] = i;
   }

   FilterState state;
   state.cIdxSrc = cIdxSrc;
   state.cIdxDst = cIdxDst;
   state.frontSize = 0;
   state.frontSrcIdx = 0;
   state.backSize = srcCnt;

   int nrOfContacts = 0;
   int nextContactIdx = 0;
   do {
       //state.print();
       // std::cout  << "rec started" << std::endl;
       // now find deepest in src and copy all in radius thres into the
       // front of dst and everything else into back of dst.
       ContactPoint dpoint = filterContactsRec( src, srcCnt, state, maxDist);

       state.cIdxSrc[nrOfContacts] = nextContactIdx;
       nrOfContacts++;
       nextContactIdx = state.frontSize;
       //std::cout  << "rec returned" << std::endl;
   } while( state.backSize>=1 );
   state.cIdxSrc[nrOfContacts] = nextContactIdx;

   //state.print();
   //std::cout << "NR of contacts: " << nrOfContacts << std::endl;
   for(int i=0;i<nrOfContacts;i++){
       //std::cout << "src[cIdxDst[cIdxSrc[" << i<<"] ] ]" << std::endl;
       //std::cout << "src[cIdxDst["<< cIdxSrc[i] <<"] ]" << std::endl;
       //std::cout << "src["<< cIdxDst[cIdxSrc[i]] << "]" << std::endl;
       dst[i] = src[ cIdxDst[ cIdxSrc[i] ] ];
   }

   return nrOfContacts;
}
