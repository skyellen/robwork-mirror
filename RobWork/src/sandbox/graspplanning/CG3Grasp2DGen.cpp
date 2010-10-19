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

#include "CG3Grasp2DGen.hpp"

#include <rw/math/Math.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Rotation2D.hpp>
#include "Contour2DInfoMap.hpp"
#include <boost/numeric/ublas/matrix.hpp>
#include <rw/common/macros.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/Joint.hpp>

#include <boost/foreach.hpp>

#include <queue>

using namespace rw::geometry;
using namespace rw::kinematics;
using namespace rw::models;
using namespace boost::numeric;
using namespace rw::math;
using namespace rw::sensor;
using namespace rw::graspplanning;

namespace {

    double cosAngle(const Vector2D<>& v1, const Vector2D<>& v2){
        return acos( dot(v1,v2)/ (v1.norm2()*v2.norm2()));
    }

    double clampAnglePi(double angle){
        while(angle<-Pi) angle += 2*Pi;
        while(angle>Pi) angle -= 2*Pi;
        return angle;
    }

    int curvatureFilter(Contour2DInfoMap::ContactPtrList& clist,
                        double scale, double curvthres){
        int left = clist.size();
        //std::cout << "List Size: " << left << std::endl;
        for(size_t i=0; i<clist.size();i++){
            if( clist[i] == NULL ){
                left--;
            } else if(fabs(clist[i]->avgCurvature)*scale>curvthres){
                clist[i] = NULL;
                left--;
            } else {
               //std::cout << "Curvature: " << fabs(clist[i]->avgCurvature) << " " << scale << "<" << curvthres << std::endl;
            }
        }
        return left;
    }

    typedef std::pair<double,Grasp2D*> GraspItem;
    class Prioritize {
    public:
         int operator() ( const GraspItem& p1,
                          const GraspItem& p2 ) {
             return p1.second > p2.second;
         }
    };

}

CG3Grasp2DGen::CG3Grasp2DGen(const TreeDevice &hand,
                     const rw::kinematics::State& state,
                     int thetaRes, bool counterclock):
    _hand(hand), _state(state),
    _counterClock(counterclock),
    _thetaRes(thetaRes),_psiRes(100), _phiRes(30),
    _thetaStep( 2*Pi/thetaRes ),
    _psiStep( 2*Pi/_psiRes ),
    _phiStep( (Pi/2)/_phiStep ),
    //_sqrCurvThres(0.03*0.00136124),
    //_sqrCurvThres(0.1),
    _sqrCurvThres(1),
    _acceptUniform(0.005),
    _acceptPerp(8*Deg2Rad),
    _acceptDirs(0.05),
    _maxGraspWidth(0.18)

{
    // first initialize member variables
    //Frame *tBase = hand.getActiveJoint(0);
    Frame *f1Base = hand.getJoints()[3];
    Frame *f2Base = hand.getJoints()[5];
    _w = Kinematics::frameTframe(f1Base,f2Base,state).P().norm2()/2.0;
    _h = sin(Pi/6)*_w;
    _sqrL = _w*_w+_h*_h;
    _L = sqrt(_sqrL);
    _thumDir = Vector2D<>(0,-_h);
    _f1Dir = Vector2D<>(_w,0);
    _f2Dir = Vector2D<>(-_w,0);

}

std::vector<Grasp2D >
    CG3Grasp2DGen::computeGrasps(const QualityMeasure2D& measure,
                                 double minQuality, const unsigned int nrOfGrasps){

    std::priority_queue<GraspItem,std::vector<GraspItem>, Prioritize> queue;

    //std::priority_queue<double, &Grasp2D> grasps(nrOfGrasps);
    //std::cout << "Nr of candidates: " <<  _graspCandidates.size() << std::endl;
    double bestQuality = 0;
    BOOST_FOREACH(Grasp2D& candidate, _graspCandidates){
        double quality  = measure.computeQuality(candidate);
        if( minQuality<quality ){
            queue.push(GraspItem(quality,&candidate));

            if(queue.size()>nrOfGrasps)
                queue.pop();
        }
        if( bestQuality<quality)
            bestQuality = quality;
    }

    std::vector<Grasp2D > grasps(queue.size(),Grasp2D(3));
    int i = queue.size()-1;
    while(!queue.empty()){
        grasps[i] = *(queue.top().second);
        queue.pop();
        i--;
    }
    std::cout << "CG3Grasp2DGen: grasps computed " << std::endl
              << " - Found best " << grasps.size() << " out of " << _graspCandidates.size() << " candidates "<< std::endl
              << " - Best quality grasp: " << bestQuality << std::endl;

    return grasps;
}

Grasp2D CG3Grasp2DGen::getGrasp(int idx){
    return _graspCandidates[idx];
}

void CG3Grasp2DGen::init(
        const Contour2D& contour,
        int psiRes, int phiRes)
{
    const double minPhi(0);
    const double maxPhi(Pi/2);
    _psiRes = psiRes;
    _phiRes = phiRes;
    _psiStep = 2*Pi/_psiRes;
    _phiStep = (maxPhi-minPhi)/_phiRes;

    std::cout << "CG3Grasp2DGen: initializing " << std::endl
              << " - Perp angle filter: " << _acceptPerp*Rad2Deg << std::endl
              << " - Dir angle filter: " << _acceptDirs*Rad2Deg << std::endl;


    double _sqrCurvThresScale = _sqrCurvThres/_sqrL;

    _infoMap.reset(contour);
    //_infoMap.printToFile("contourMap.txt");

    // first generate grasp candidates based on normal direction
    for(int psiIdx=0; psiIdx<psiRes; psiIdx++){
        double psi = psiIdx*_psiStep;

        Contour2DInfoMap::ContactPtrList thumList =
            _infoMap.getCNormals(psi-Pi/2.0, _acceptPerp, _sqrCurvThresScale);
        if( thumList.size()==0 )
            continue;

        // remove any contacts from list that has bad curvature
        if( curvatureFilter( thumList, _sqrL, _sqrCurvThres )==0 ){
            continue;
        }

        Rotation2D<> rotPsi(psi);
        const Vector2D<> thumDir = rotPsi*Vector2D<>(0,-1);

        for(int phiIdx=0; phiIdx<phiRes; phiIdx++ ){
            double phi = minPhi+phiIdx*_phiStep;
            Rotation2D<> rotPsiPhi(clampAnglePi(psi+phi));

            // the thumb
            Contour2DInfoMap::ContactPtrList f1List = _infoMap.getCNormals(psi+phi, _acceptPerp, _sqrCurvThresScale);
            if( curvatureFilter( f1List, _sqrL, _sqrCurvThres )==0 ){
                continue;
            }

            Contour2DInfoMap::ContactPtrList f2List = _infoMap.getCNormals(psi+Pi-phi, _acceptPerp, _sqrCurvThresScale);
            if( curvatureFilter( f2List, _sqrL, _sqrCurvThres )==0 ){
                continue;
            }

            // now comes the real filtering
            BOOST_FOREACH(Contact2D *thumCon, thumList){
                if( thumCon==NULL )
                    continue;
                BOOST_FOREACH(Contact2D *f1Con, f1List){
                    if( f1Con==NULL )
                        continue;

                    // constrain the distance between f1 and f2 contacts to L/2
                    const double thumToF1dist = (thumCon->p-f1Con->p).norm2();
                    if( thumToF1dist<_L/2 || _maxGraspWidth<thumToF1dist )
                        continue;

                    const Vector2D<> f1Dir = rotPsiPhi*_f1Dir;
                    BOOST_FOREACH(Contact2D *f2Con, f2List){
                        if( f2Con==NULL )
                            continue;

                        // constrain the distance between f1 and f2 contacts to L/2
                        const double thumToF2dist = (thumCon->p-f2Con->p).norm2();
                        if( thumToF2dist<_L/2 || _maxGraspWidth<thumToF2dist )
                            continue;

                        const Vector2D<> f1Vf2 = f2Con->p-f1Con->p;
                        const double f1ToF2dist = f1Vf2.norm2();
                        if( f1ToF2dist<_w*2.0 || _maxGraspWidth<f1ToF2dist )
                            continue;

                        Rotation2D<> rotf2PsiPhi(clampAnglePi(psi-phi));
                        const Vector2D<> f2Dir = rotf2PsiPhi*_f2Dir;

                        // filter out contacts where thumb direction is not somewhat perpendicular on f1.c-f2.c
                        double dotangle = fabs( dot(thumDir,f1Vf2)/(thumDir.norm2()*f1ToF2dist));
                        if( dotangle>_acceptDirs ){
                           //std::cout << "DotAngle: " << dotangle << std::endl;
                            continue;
                        }

                        double scale = (dot(thumCon->p-f1Con->p,f1Vf2)/Math::sqr(f1ToF2dist))-0.5;
                        if( Math::sqr(scale)>_acceptUniform )
                            continue;

                        // check that the f1 dosn't point toward f2
                        const double f1scale = dot(f1Dir,f1Vf2)/(Math::sqr(f1ToF2dist));
                        if( f1scale > 0 )
                            continue;

                        const Vector2D<>& a1 = normalize(thumDir);
                        const Vector2D<>& c1 = thumCon->p;
                        const Vector2D<>& a2 = normalize(f1Dir);
                        const Vector2D<>& c2 = f1Con->p;
                        const Vector2D<>& c3 = f2Con->p;

                        Vector2D<> c2Pc1 = c1-c2;

                        double c2Pc3Len = (c2-c3).norm2()/2;

                        // TODO: check if approach and position is reachable
                        double clen = c2Pc3Len/( sin( acos( dot(-a1,a2) )));

                        Vector2D<> d1 = c2 - a2*clen;

                        double clen2 = _w/( sin( acos( dot(-a1,a2) )));
                        Vector2D<> center = -a1* (std::sqrt(clen2*clen2 - _w*_w)-_h)+d1;

                        double distc1center = (center-c1).norm2();
                        // if this
                        if( distc1center>2*_w)
                            continue;

                        double distc2center = (center-c2).norm2();
                        if(  distc2center>2*_w)
                            continue;

                        // here we add the filters and create grasp objects for the rest

                        Grasp2D grasp(3);
                        grasp.contacts[0] = *thumCon;
                        grasp.contacts[1] = *f1Con;
                        grasp.contacts[2] = *f2Con;
                        // calculate approach vectors
                        grasp.approach[0] = thumDir;
                        grasp.approach[1] = f1Dir;
                        grasp.approach[2] = f2Dir;
                        // calculate psi and phi
                        grasp.psi = psi;

                        grasp.center = center;
                        grasp.phi = phi-Pi/6;
                        //grasp.phi = Pi/6-(Pi/2-phi);

                        _graspCandidates.push_back(grasp);
                    }
                }
            }

        }
    }
    std::cout << "- CG3Grasp2DGen: found " << _graspCandidates.size()
              << " grasp candidates." << std::endl;
}

