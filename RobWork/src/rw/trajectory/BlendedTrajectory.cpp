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

#include "BlendedTrajectory.hpp"

#include <rw/math/Math.hpp>

using namespace rw::math;
using namespace rw::trajectory;

namespace {
    const double eps = 0.00001;
    const double inf = (std::numeric_limits<double>::max());

    double fastPow(double base, double exp) {
        return (exp == 2.0 ? ((base)*(base)) : std::pow(base,exp));   
    }


}


// Explicit template specification
template class BlendedTrajectory<rw::math::Q>;

/*
* Redeclerations to avoid premature instantiation (calling a templated
* function before its body has been defined)
*/
template<> bool BlendedTrajectory<>::init();
template<> bool BlendedTrajectory<>::checkPath();
template<> void BlendedTrajectory<>::updateLimits();
template<> void BlendedTrajectory<>::updateWI();
template<> void BlendedTrajectory<>::updateWF();
template<> void BlendedTrajectory<>::updateWFnext();
template<> void BlendedTrajectory<>::updateWInext();
template<> void BlendedTrajectory<>::updatedwSMax();
template<> void BlendedTrajectory<>::updateddwSMax();
template<> void BlendedTrajectory<>::updateddwSMin();

// Initialization ------------------------------------------------------
template<> BlendedTrajectory<>::BlendedTrajectory(rw::models::DevicePtr deviceIn,
                                                  const rw::trajectory::QPath& pathIn,
                                                  const std::vector<double>& betaIn,
                                                  const double vscaleIn, const double ascaleIn,
                                                  const bool verbose) 
{
    this->verbose = verbose;
    if(verbose) 
        std::cout << "Reading inputs..." << std::endl;

    // Get device data
    device = deviceIn;
    // Get the other parameters
    path = pathIn;  
    Npath = path.size();
    betas = betaIn;
    vscale = vscaleIn;
    ascale = ascaleIn;

    // Initialize various parameters and check for errors
    if(init()) {
        if(verbose) 
            std::cout << "Creating blends..." << std::endl;
        for(unsigned int i = 1; i < Npath-1; i++) {
            if(verbose) 
                std::cout << "Blend: " << i << std::endl;
            beta = betas[i-1];
            betaNext = (i < Npath-2 ? betas[i] : 0.0);
            qprev = path[i-1];
            q = path[i];
            path.size() == 2 ? qnext = q : qnext = path[i+1];
            i == Npath - 2 ? qnextnext = qnext : qnextnext = path[i+2];
            deltaq = q - qprev;
            deltaqnext = qnext - q;
            deltaqnextnext = qnextnext - qnext;
            updateLimits();
            updateWI();
            qI = qprev + (1.0 - wI) * deltaq;
            updateWF();
            updateWFnext();
            qF = q + wF * deltaqnext;
            qFnext = qnext + wFnext * deltaqnextnext;
            updateWInext();
            qInext = q + (1.0 - wInext) * deltaqnext;
            updatedwSMax();
            updateddwSMax();
            updateddwSMin();
            // Find extremal blend points
            qext = findQext(qI, q, qF);
            qextnext = findQext(qInext, qnext, qFnext);
            // Find initial blend velocity
            dwI = findDWI(deltaq, qI, qext, wI, wFprev, dwFprev, dwSMax, ddwSMax, aBMax);
            dqI = dwI * deltaq;
            // Find maximum segment velocity
            dwSreached = findDWSReached(wI, dwI, wFprev, dwFprev, dwSMax, ddwSMax, ddwSMin);
            // Find segment time
            TSdecc = findTSdecc(dwI, dwSreached, ddwSMin);
            TS = findTS(wI, dwI, wFprev, dwFprev, ddwSMax, ddwSMin, dwSreached, TSdecc);
            // Find initial blend time
            TI = TFprev + TS;
            TIList[i-1] = TI;
            // Find final blend velocity
            dwF = findDWF(deltaq, qI, qext, qextnext, qF, qInext, deltaqnext, deltaqnextnext, dqI, dqMax, aBMin, aBMax, aBMaxNext);
            dqF = dwF * deltaqnext;
            // Find blend execution time
            TB = findTB(qI, qF, qext, dqI, dqF, aBMax, aBMin, deltaq, deltaqnext);
            // Find final blend time
            TF = TI + TB;
            TFList[i-1] = TF;
            // Store linear segment
            for(unsigned int k = 0; k < K; k++) {
                sList[i-1][k] = funcLinear(TFprev, TS, TSdecc, qFprev[k], dwFprev, deltaq[k], dwSreached, ddwSMax, ddwSMin);
            }
            // For all joints
            for(unsigned int k = 0; k < K; k++) {
                if(dqI[k] * dqF[k] < 0) { // TURNING JOINT
                    eq2a = (tau(qext[k] - qI[k], dqI[k], 0.0, aBMax[k]) <= 0.5 * TB);
                    eq2b = (tau(qF[k] - qext[k], 0.0, dqF[k], aBMax[k]) <= 0.5 * TB);
                    if(eq2a && eq2b) {
                        Tmid[k] = TI + 0.5 * TB;
                        deltaTmid[k] = Tmid[k] - TI;
                    } else {
                        if(!eq2a) {
                            deltaTmid[k] = tau(qext[k] - qI[k], dqI[k], 0.0, aBMax[k]);
                            Tmid[k] = TI + deltaTmid[k];
                        } else { // if(!eq2b)
                            deltaTmid[k] = TB - tau(qF[k] - qext[k], 0.0, dqF[k], aBMax[k]);
                            Tmid[k] = TI + deltaTmid[k];
                        }
                    }
                    // Trajectory from qI to qext
                    if(deltaTmid[k] >= 2.0 * (qext[k] - qI[k]) / dqI[k]) {
                        aI[k] = fastPow(dqI[k], 2.0) / (2.0 * (qext[k] - qI[k]));
                        iExtList[i-1][k] = funcIext(TURNING1, qext[k], dqI[k], deltaTmid[k], Tmid[k], aI[k]);
                    } else {
                        double aI1 = (dqI[k] * (dqI[k] - 2.0 * abs(dqI[k])))/( 2.0 * (-qext[k] + qI[k] + deltaTmid[k] * abs(dqI[k])));
                        double aI2 = -((dqI[k] * (dqI[k] + 2.0 * abs(dqI[k])))/(2.0 * (qext[k] - qI[k] + deltaTmid[k] * abs(dqI[k]))));
                        if(Math::sign(aI1) == Math::sign(deltaq[k]) || std::fabs(aI1) > (aBMaxConst[k] + 0.000001)) {
                            aI[k] = aI2;
                        } else {
                            if(Math::sign(aI2) != Math::sign(deltaq[k]) && std::fabs(aI2) <= (aBMaxConst[k] + 0.000001)) {
                                aI[k] = abs(aI1) > abs(aI2) ? aI1 : aI2;
                            } else {
                                aI[k] = aI1;
                            }
                        }
                        iExtList[i-1][k] = funcIext(TURNING2, qext[k], dqI[k], deltaTmid[k], Tmid[k], aI[k]);
                    }
                    if(TB - deltaTmid[k] >= 2.0 * (qF[k] - qext[k]) / dqF[k]) {
                        aF[k] = fastPow(dqF[k], 2.0) / (2.0 * (qF[k] - qext[k]));
                        extFList[i-1][k] = funcExtF(TURNING1, qext[k], dqF[k], TB, deltaTmid[k], Tmid[k], aF[k]);
                    } else {
                        double aF1 = (-fastPow(dqF[k], 2.0) - 2.0 * dqF[k] * abs(dqF[k])) / (2.0 * (qext[k] - qF[k] + deltaTmid[k] * abs(dqF[k]) - TB * abs(dqF[k])));
                        double aF2 = (fastPow(dqF[k], 2.0) - 2.0 * dqF[k] * abs(dqF[k])) / (2.0 * (-qext[k] + qF[k] + deltaTmid[k] * abs(dqF[k]) - TB * abs(dqF[k])));
                        if(Math::sign(aF1) == Math::sign(deltaq[k]) || abs(aF1) > (aBMaxConst[k] + 0.000001)) {
                            aF[k] = aF2;
                        } else {
                            if(Math::sign(aF2) != Math::sign(deltaq[k]) && abs(aF2) <= (aBMaxConst[k] + 0.000001)) {
                                aF[k] = abs(aF1) > abs(aF2) ? aF1 : aF2;
                            } else {
                                aF[k] = aF1;
                            }
                        }
                        extFList[i-1][k] = funcExtF(TURNING2, qext[k], dqF[k], TB, deltaTmid[k], Tmid[k], aF[k]);
                    }
                } else { // NON-TURNING JOINT
                    if(abs(deltaq[k]) < eps && abs(deltaqnext[k]) < eps) {
                        // Joint is stopped twice
                        a[k] = inf;
                    } else {
                        a[k] = (dqF[k] - dqI[k]) * (qnext[k] - qprev[k]) > 0.0 ? aBMin[k] : aBMax[k];
                    }
                    // Average velocity
                    dqav[k] = (qF[k] - qI[k]) / TB;
                    // Minimal velocity ramp time
                    delta[k] = (dqF[k] - dqI[k]) / a[k];
                    // TODO: The test should be TB * abs(dqav[k]) >= (TB - 0.5 * delta[k]) * min(abs(dqI[k]), abs(dqF[k])) + 0.5 * delta[k] * max(abs(dqI[k]), abs(dqF[k]))
                    if(TB * abs(dqav[k]) - ((TB - 0.5 * delta[k]) * std::min(abs(dqI[k]), abs(dqF[k])) + 0.5 * delta[k] * std::max(abs(dqI[k]), abs(dqF[k]))) >= -0.000001) { // The very small negative number is to make sure that the if statement is evaluated to true in case the difference is very small in either direction, which is then assumed to be insignificant
                        deltaTmid[k] = abs(dqF[k] - dqI[k]) < eps ? 0.0 : TB * (dqav[k]-  dqF[k]) / (dqI[k] - dqF[k]);
                        Tmid[k] = TI + deltaTmid[k];
                        qext[k] = qI[k] + dqI[k] * deltaTmid[k] + 0.125 * ((dqF[k] - dqI[k]) * abs(dqF[k] - dqI[k])) / abs(a[k]);
                        dqIF[k] = 0.5 * (dqI[k] + dqF[k]);
                        if((dqF[k] - dqI[k]) * (qF[k] - qI[k]) >= 0.0) { // Change in velocity has same sign as change in angle
                            // Trajectory from qI to qext
                            iExtList[i-1][k] = funcIext(NONTURNING1, qext[k], dqI[k], deltaTmid[k], Tmid[k], 0.0, dqIF[k], aBMin[k], aBMax[k]);
                            extFList[i-1][k] = funcExtF(NONTURNING1, qext[k], dqF[k], TB, deltaTmid[k], Tmid[k], 0.0, dqIF[k], aBMin[k], aBMax[k]);
                        } else {
                            iExtList[i-1][k] = funcIext(NONTURNING2, qext[k], dqI[k], deltaTmid[k], Tmid[k], 0.0, dqIF[k], aBMin[k], aBMax[k]);
                            extFList[i-1][k] = funcExtF(NONTURNING2, qext[k], dqF[k], TB, deltaTmid[k], Tmid[k], 0.0, dqIF[k], aBMin[k], aBMax[k]);
                        }
                    } else { // We need to deccelerate to vmin
                        deltaTmid[k] = (dqI[k] - dqF[k] + aBMin[k] * TB) / (aBMin[k] - aBMax[k]);
                        Tmid[k] = TI + deltaTmid[k];
                        // Try first solution
                        vmin[k] = (0.5*aBMax[k]*dqF[k] - 0.5*aBMin[k]*dqI[k] - 0.5*aBMax[k]*aBMin[k]*TB -
                            0.5*aBMax[k]*aBMin[k]*sqrt((fastPow(dqF[k],2.0) - 2.0*dqF[k]*dqI[k] + fastPow(dqI[k],2.0) + 2.0*aBMax[k]*qF[k] -
                            2.0*aBMin[k]*qF[k] - 2.0*aBMax[k]*qI[k] + 2.0*aBMin[k]*qI[k] - 2.0*aBMax[k]*dqF[k]*TB + 2.0*aBMin[k]*dqI[k]*TB +
                            aBMax[k]*aBMin[k]*fastPow(TB,2.0)) / (aBMax[k]*aBMin[k])))/(0.5*aBMax[k] - 0.5*aBMin[k]);
                        // Update solution if necessary
                        if(vmin[k] * dqI[k] < 0.0) {
                            vmin[k] = (0.5*aBMax[k]*dqF[k] - 0.5*aBMin[k]*dqI[k] - 0.5*aBMax[k]*aBMin[k]*TB + 
                                0.5*aBMax[k]*aBMin[k]*sqrt((fastPow(dqF[k],2.0) - 2.0*dqF[k]*dqI[k] + fastPow(dqI[k],2.0) + 2.0*aBMax[k]*qF[k] - 
                                2.0*aBMin[k]*qF[k] - 2.0*aBMax[k]*qI[k] + 2.0*aBMin[k]*qI[k] - 2.0*aBMax[k]*dqF[k]*TB + 2.0*aBMin[k]*dqI[k]*TB + 
                                aBMax[k]*aBMin[k]*fastPow(TB,2.0))/(aBMax[k]*aBMin[k])))/(0.5*aBMax[k] - 0.5*aBMin[k]);
                        }
                        qext[k] = qI[k] + phistarmax(vmin[k], dqI[k], -aBMax[k], deltaTmid[k], deltaTmid[k]);
                        // Trajectory from qI to qext
                        iExtList[i-1][k] = funcIext(NONTURNING3, qext[k], dqI[k], deltaTmid[k], Tmid[k], 0.0, dqIF[k], aBMin[k], aBMax[k], vmin[k]);
                        extFList[i-1][k] = funcExtF(NONTURNING3, qext[k], dqF[k], TB, deltaTmid[k], Tmid[k], 0.0, dqIF[k], aBMin[k], aBMax[k], vmin[k]);
                    }
                }
            }
            TmidList[i-1] = Tmid;
            TFprev = TF;
            qFprev = qF;
            wFprev = wF;
            dwFprev = dwF;
        }
        if(Npath==2){ //Initialization in case path only consists of two configurations
            deltaqnext = path[1] - path[0];
        }

        // Final linear segment
        deltaq = deltaqnext;
        wI = 0.0;
        dwI = 0.0;
        updateLimits();
        updatedwSMax();
        updateddwSMax();
        updateddwSMin();
        dwSreached = findDWSReached(wI, dwI, wFprev, dwFprev, dwSMax, ddwSMax, ddwSMin);
        TSdecc = findTSdecc(dwI, dwSreached, ddwSMin);
        TS = findTS(wI, dwI, wFprev, dwFprev, ddwSMax, ddwSMin, dwSreached, TSdecc);
        for(unsigned int k = 0; k < K; k++) {
            sList[Npath-2][k] = funcLinear(TFprev, TS, TSdecc, qFprev[k], dwFprev, deltaq[k], dwSreached, ddwSMax, ddwSMin);
        }
        // Store end time
        t_total = TFprev + TS;
    }
    if(verbose) std::cout << "Done, returning" << std::endl;
}

template<> bool BlendedTrajectory<>::init() {
    if(verbose) std::cout << "Initializing..." << std::endl;
    // Get DOF
    K = device->getDOF();
    // Initialize all vectors
    aSMax.m().resize(K);
    aSMin.m().resize(K);
    aSnextMin.m().resize(K);
    aBMax.m().resize(K);
    aBMaxNext.m().resize(K);
    aBMin.m().resize(K);
    Tmid.resize(K);
    deltaTmid.resize(K);
    aI.m().resize(K);
    aF.m().resize(K);
    a.m().resize(K);
    dqav.m().resize(K);
    delta.m().resize(K);
    dqIF.m().resize(K);
    vmin.m().resize(K);
    qFprev.m().resize(K);
    qprev.m().resize(K);
    q.m().resize(K);
    qnext.m().resize(K);
    qnextnext.m().resize(K);
    qext.m().resize(K);
    qextnext.m().resize(K);
    deltaq.m().resize(K);
    deltaqnext.m().resize(K);
    deltaqnextnext.m().resize(K);
    qI.m().resize(K);
    qInext.m().resize(K);
    qF.m().resize(K);
    qFnext.m().resize(K);
    dqI.m().resize(K);
    dqF.m().resize(K);
    // Get position limits
    jointBoundsMin = device->getBounds().first;
    jointBoundsMax = device->getBounds().second;
    // Check path
    if(!checkPath()) {
        return false;
    }
    // Check velocity and acceleration scale
    if(vscale < eps || vscale > 1.0 || ascale < eps || ascale > 1.0) {
        RW_THROW("Velocity/acceleration scale should be between 0 and 1!");
        return false;
    }
    // Set limits for velocities and accelerations
    dqMax = device->getVelocityLimits();
    aSMaxConst = aSMinConst = aSnextMinConst = device->getAccelerationLimits();
    aBMaxConst = aBMinConst = device->getAccelerationLimits();
    for(unsigned int k = 0; k < K; k++) {
        dqMax[k] = vscale * dqMax[k];
        aSMaxConst[k] = ascale * aSMaxConst[k];
        aSMinConst[k] = ascale * aSMinConst[k];
        aSnextMinConst[k] = ascale * aSnextMinConst[k];
        aBMaxConst[k] = ascale * aBMaxConst[k];
        aBMinConst[k] = ascale * aBMinConst[k];
    }
    // Initialize start time
    TFprev = startTime();
    wFprev = 0.0;
    dwFprev = 0.0;
    // Initialize previous final configuration to first path configuration
    qFprev = path[0];
    // Initialize trajectory lists
    sList.resize(Npath-1);
    for(unsigned int i = 0; i < Npath-1; i++) {
        sList[i].resize(K);
    }
    iExtList.resize(Npath-2);
    extFList.resize(Npath-2);
    TmidList.resize(Npath-2);
    for(unsigned int i = 0; i < Npath-2; i++) {
        iExtList[i].resize(K);
        extFList[i].resize(K);
        TmidList[i].resize(K);
    }
    TIList.resize(Npath-2);
    TFList.resize(Npath-2);

    return true;
}

template<> bool BlendedTrajectory<>::checkPath() {
    if(verbose) std::cout << "Checking path..." << std::endl;
    if(Npath < 2) {
        RW_THROW("Invalid path size < 2!");
        return false;
    }
    // All joint displacements between two configurations must be >= eps
    bool tooClose;
    for(unsigned int i = 0; i < Npath-1; i++) {
        if(path[i].size() != K) {
            RW_THROW("Configuration DOF does not match device DOF!");
            return false;
        }
        tooClose = true;
        for(unsigned int k = 0; k < K; k++) {
            // Check that configuration is within joint position range
            if(path[i][k] < jointBoundsMin[k] || path[i][k] > jointBoundsMax[k]) {
                RW_THROW("Configuration " << i << ", joint " << k << " out of range!");
                return false;
            }
            if(abs(path[i+1][k] - path[i][k]) >= eps) { // Non-stopping joint
                tooClose = false;
            }
        }
        if(tooClose) {
            RW_THROW("Configurations too close!");
            return false;
        }
    }
    // Check last configuration
    for(unsigned int k = 0; k < K; k++) {
        if(path[Npath-1][k] < jointBoundsMin[k] || path[Npath-1][k] > jointBoundsMax[k]) {
            RW_THROW("Configuration " << Npath-1 << ", joint " << k << " out of range!");
            return false;
        }
    }

    return true;
}

template<> void BlendedTrajectory<>::updateLimits() {
    for(unsigned int k = 0; k < K; k++) {
        // If joint is stopped both in initial and final segment    
        if(abs(deltaq[k]) < eps && abs(deltaqnext[k]) < eps) {
            aSMax[k] = 0.0;
            aSMin[k] = 0.0;
            aBMax[k] = 0.0;
            aBMin[k] = 0.0;
            aSnextMin[k] = 0.0;
        }
        // If joint is stopped only in initial part  
        if(abs(deltaq[k]) < eps && abs(deltaqnext[k]) >= eps) {
            aSMax[k] = 0.0;
            aSMin[k] = 0.0;
            aBMax[k] = -Math::sign(deltaqnext[k]) * aBMaxConst[k];
            aBMin[k] = Math::sign(deltaqnext[k]) * aBMinConst[k];
            aSnextMin[k] = -Math::sign(deltaqnext[k]) * aSnextMinConst[k];
        }
        // If joint is stopped only in final part  
        if(abs(deltaqnext[k]) < eps && abs(deltaq[k]) >= eps) {
            aSMax[k] = Math::sign(deltaq[k]) * aSMaxConst[k];
            aSMin[k] = -Math::sign(deltaq[k]) * aSMinConst[k];
            aBMax[k] = -Math::sign(deltaq[k]) * aBMaxConst[k];
            aBMin[k] = Math::sign(deltaq[k]) * aBMinConst[k];
            aSnextMin[k] = 0.0;
        }
        // If joint is not stopped  
        if(abs(deltaq[k]) >= eps && abs(deltaqnext[k]) >= eps) {
            aSMax[k] = Math::sign(deltaq[k]) * aSMaxConst[k];
            aSMin[k] = -Math::sign(deltaq[k]) * aSMinConst[k];
            aBMax[k] = -Math::sign(deltaq[k]) * aBMaxConst[k];
            aBMin[k] = Math::sign(deltaq[k]) * aBMinConst[k];
            aSnextMin[k] = -Math::sign(deltaqnext[k]) * aSnextMinConst[k];
        }
        // Do the same for the blend decceleration at the next blend  
        if(abs(deltaqnext[k]) < eps && abs(deltaqnextnext[k]) < eps) {
            aBMaxNext[k] = 0.0;
        }
        if(abs(deltaqnext[k]) < eps && abs(deltaqnextnext[k]) >= eps) {
            aBMaxNext[k] = -Math::sign(deltaqnextnext[k]) * aBMaxConst[k];
        }
        if(abs(deltaqnextnext[k]) < eps && abs(deltaqnext[k]) >= eps) {
            aBMaxNext[k] = -Math::sign(deltaqnext[k]) * aBMaxConst[k];
        }
        if(abs(deltaqnext[k]) >= eps && abs(deltaqnextnext[k]) >= eps) {
            aBMaxNext[k] = -Math::sign(deltaqnext[k]) * aBMaxConst[k];
        }
    }
}

template<> void BlendedTrajectory<>::updateWI() {
    double deltaqmax = 0.0;
    for(unsigned int k = 0; k < K; k++) {
        if(abs(deltaq[k]) > deltaqmax)
            deltaqmax = abs(deltaq[k]);
    }
    if(deltaqmax <= 2.0 * beta) {
        wI = 0.5 - eps;
        return;
    }

    double wIMin = inf;
    double wIMinTmp;
    for(unsigned int k = 0; k < K; k++) {
        if(abs(deltaq[k]) > eps) {
            wIMinTmp = beta / abs(deltaq[k]);
            wIMin = wIMinTmp < wIMin ? wIMinTmp : wIMin;
        }
    }
    wI = wIMin;
}

template<> void BlendedTrajectory<>::updateWF() {
    double deltaqnextmax = 0.0;
    for(unsigned int k = 0; k < K; k++) {
        if(abs(deltaqnext[k]) > deltaqnextmax)
            deltaqnextmax = abs(deltaqnext[k]);
    }
    if(deltaqnextmax <= 2.0 * beta) {
        wF = 0.5 - eps;
        return;
    }

    double wFMin = inf;
    double wFMinTmp;
    for(unsigned int k = 0; k < K; k++) {
        if(abs(deltaqnext[k]) > eps) {
            wFMinTmp = beta / abs(deltaqnext[k]);
            wFMin = wFMinTmp < wFMin ? wFMinTmp : wFMin;
        }
    }
    wF = wFMin;
}

template<> void BlendedTrajectory<>::updateWFnext() {
    if(confNum == Npath - 2) {
        wFnext = 0.0;
    } else {
        double deltaqnextnextmax = 0.0;
        for(unsigned int k = 0; k < K; k++) {
            if(abs(deltaqnextnext[k]) > deltaqnextnextmax)
                deltaqnextnextmax = abs(deltaqnextnext[k]);
        }
        if(deltaqnextnextmax <= 2.0 * betaNext) {
            wFnext = 0.5 - eps;
            return;
        }

        double wFnextMin = inf;
        double wFnextMinTmp;
        for(unsigned int k = 0; k < K; k++) {
            if(abs(deltaqnextnext[k]) > eps) {
                wFnextMinTmp = betaNext / abs(deltaqnextnext[k]);
                wFnextMin = wFnextMinTmp < wFnextMin ? wFnextMinTmp : wFnextMin;
            }
        }
        wFnext = wFnextMin;
    }
}

template<> void BlendedTrajectory<>::updateWInext() {
    if(confNum == Npath - 2) {
        wInext = 0.0;
    } else {
        double deltaqnextmax = 0.0;
        for(unsigned int k = 0; k < K; k++) {
            if(abs(deltaqnext[k]) > deltaqnextmax)
                deltaqnextmax = abs(deltaqnext[k]);
        }
        if(deltaqnextmax <= 2.0 * betaNext) {
            wInext = 0.5 - eps;
            return;
        }

        double wInextMin = inf;
        double wInextMinTmp;
        for(unsigned int k = 0; k < K; k++) {
            if(abs(deltaqnext[k]) > eps) {
                wInextMinTmp = betaNext / abs(deltaqnext[k]);
                wInextMin = wInextMinTmp < wInextMin ? wInextMinTmp : wInextMin;
            }
        }
        wInext = wInextMin;
    }
}

template<> void BlendedTrajectory<>::updatedwSMax() {
    double dwSMaxMin = inf;
    double dwSMaxMinTmp;
    for(unsigned int k = 0; k < K; k++) {
        if(abs(deltaq[k]) > eps) {
            dwSMaxMinTmp = dqMax[k] / abs(deltaq[k]);
            dwSMaxMin = dwSMaxMinTmp < dwSMaxMin ? dwSMaxMinTmp : dwSMaxMin;
        }
    }
    dwSMax = dwSMaxMin;
}

template<> void BlendedTrajectory<>::updateddwSMax() {
    double ddwSMaxMin = inf;
    double ddwSMaxMinTmp;
    for(unsigned int k = 0; k < K; k++) {
        if(abs(deltaq[k]) > eps) {
            ddwSMaxMinTmp = aSMax[k] / deltaq[k];
            ddwSMaxMin = ddwSMaxMinTmp < ddwSMaxMin ? ddwSMaxMinTmp : ddwSMaxMin;
        }
    }
    ddwSMax = ddwSMaxMin;
}

template<> void BlendedTrajectory<>::updateddwSMin() {
    double ddwSMinMin = inf;
    double ddwSMinMinTmp;
    for(unsigned int k = 0; k < K; k++) {
        if(abs(deltaq[k]) > eps) {
            ddwSMinMinTmp = -aSMin[k] / deltaq[k];
            ddwSMinMin = ddwSMinMinTmp < ddwSMinMin ? ddwSMinMinTmp : ddwSMinMin;
        }
    }
    ddwSMin = -ddwSMinMin;
}

// Interface functions -------------------------------------------------
template <class T>
BlendedTrajectory<T>::~BlendedTrajectory() {}

template <class T>
T BlendedTrajectory<T>::x(double t) const {
    if(t < 0.0 || t > t_total)
        return T::zero(K);

    T x(K);
    for(unsigned int confNumber = 0; confNumber < Npath-2; confNumber++) {
        if(t < TIList[confNumber]) { // We are in a linear segment
            for(unsigned int k = 0; k < K; k++) {
                x[k] = sList[confNumber][k].x(t);
            }
            return x;
        } else if(t < TFList[confNumber]) { // We are in a blend
            for(unsigned int k = 0; k < K; k++) {
                if(t < TmidList[confNumber][k]) { // We are in the first blend part
                    x[k] = iExtList[confNumber][k].x(t);
                } else { // We are in the last blend part
                    x[k] = extFList[confNumber][k].x(t);
                }
            }
            return x;
        } 
    }
    // We are in the final linear segment
    for(unsigned int k = 0; k < K; k++) {
        x[k] = sList[Npath-2][k].x(t);
    }
    return x;
}

template <class T>
T BlendedTrajectory<T>::dx(double t) const {
    if(t < 0.0 || t > t_total)
        return rw::math::Q::zero(K);

    rw::math::Q dx(K);
    for(unsigned int confNumber = 0; confNumber < Npath-2; confNumber++) {
        if(t < TIList[confNumber]) { // We are in a linear segment
            for(unsigned int k = 0; k < K; k++) {
                dx[k] = sList[confNumber][k].dx(t);
            }
            return dx;
        } else if(t < TFList[confNumber]) { // We are in a blend
            for(unsigned int k = 0; k < K; k++) {
                if(t < TmidList[confNumber][k]) { // We are in the first blend part
                    dx[k] = iExtList[confNumber][k].dx(t);
                } else { // We are in the last blend part
                    dx[k] = extFList[confNumber][k].dx(t);
                }
            }
            return dx;
        } 
    }
    // We are in the final linear segment
    for(unsigned int k = 0; k < K; k++) {
        dx[k] = sList[Npath-2][k].dx(t);
    }
    return dx;
}

template <class T>
T BlendedTrajectory<T>::ddx(double t) const {
    if(t < 0.0 || t > t_total)
        return rw::math::Q::zero(K);

    rw::math::Q ddx(K);
    for(unsigned int confNumber = 0; confNumber < Npath-2; confNumber++) {
        if(t < TIList[confNumber]) { // We are in a linear segment
            for(unsigned int k = 0; k < K; k++) {
                ddx[k] = sList[confNumber][k].ddx(t);
            }
            return ddx;
        } else if(t < TFList[confNumber]) { // We are in a blend
            for(unsigned int k = 0; k < K; k++) {
                if(t < TmidList[confNumber][k]) { // We are in the first blend part
                    ddx[k] = iExtList[confNumber][k].ddx(t);
                } else { // We are in the last blend part
                    ddx[k] = extFList[confNumber][k].ddx(t);
                }
            }
            return ddx;
        } 
    }
    // We are in the final linear segment
    for(unsigned int k = 0; k < K; k++) {
        ddx[k] = sList[Npath-2][k].ddx(t);
    }
    return ddx;
}

// Algorithm functions -------------------------------------------------
template <class T>
T BlendedTrajectory<T>::findQext(T& qI, T& q, T& qF) {
    T qext(K);
    for(unsigned int k=0; k<K; k++){
        if(abs(qI[k]-q[k]) < eps && abs(qF[k]-q[k]) < eps){
            qext[k] = q[k];
        } else{
            if((q[k]-qI[k])*(qF[k]-q[k]) < 0.0){
                qext[k] = qI[k]+(q[k]-qI[k])*abs(qI[k]-q[k])/(abs(qI[k]-q[k])+abs(qF[k]-q[k]));
            }else{
                qext[k] = q[k];
            }
        }
    }
    return qext;
}

template <class T>
double BlendedTrajectory<T>::findDWI(T& deltaq, T& qI, T& qext, double wI, double wFprev, double dwFprev, double dwSMax, double ddwSMax, T& aBMax) {
    T dqISMax(K);
    T dqIBMax(K);
    T dqIMax(K);
    double dwI = inf;
    double dwITmp = 0.0;
    for(unsigned int k=0; k<K; k++){
        //dqISMax[k] = std::min(dwSMax*abs(deltaq[k]),dwFprev*abs(deltaq[k]) + sqrt(2.0*ddwSMax*(1.0-wI-wFprev))*abs(deltaq[k]));
        //dqISMax[k] = std::min(dwSMax,dwFprev + ddwSMax*(1.0-wI-wFprev)) * abs(deltaq[k]);
        dqISMax[k] = std::min(dwSMax,sqrt(dwFprev*dwFprev + ddwSMax*(1.0-wI-wFprev))) * abs(deltaq[k]);
        dqIBMax[k] = sqrt(2.0*abs(aBMax[k]*(qext[k]-qI[k])));
        dqIMax[k] = std::min(dqISMax[k], dqIBMax[k]);
        dwITmp = abs(deltaq[k]) < eps ? inf : dqIMax[k]/abs(deltaq[k]);
        dwI = std::min(dwI, dwITmp);
    }
    return dwI;
}

template <class T>
double BlendedTrajectory<T>::findDWF(T& deltaq, T& qI, T& qext, T& qextnext, T& qF, T& qInext, T& deltaqnext, T& deltaqnextnext, T& dqI, T& dqMax, T& aBMin, T& aBMax, T& aBMaxNext) {
    T dqIBMaxNext(K);
    T dqFMax(K);
    double dwF = inf;
    double dwFTmp;
    for(unsigned int k=0; k<K; k++) {
        dqIBMaxNext[k] = sqrt(2.0*abs(aBMaxNext[k]*(qextnext[k]-qInext[k]))); 
        if(abs(deltaq[k]) < eps && abs(deltaqnext[k]) < eps) {
            dqFMax[k] = dqMax[k];
        } else {
            if(abs(deltaq[k]) < eps){
                dqFMax[k] = std::min(dqMax[k], sqrt(2.0*abs((qF[k]-qext[k])*aBMin[k])));
            }else {
                if(Math::sign(deltaq[k]) != Math::sign(deltaqnext[k])) { // Turning joint
                    dqFMax[k] = std::min(dqMax[k], sqrt(2.0*abs((qF[k]-qext[k])*aBMax[k])));
                } else { // Non-turning joint
                    dqFMax[k] = std::min(dqMax[k], sqrt(2.0*abs((qF[k]-(qI[k]-(fastPow(dqI[k],2.0))/(2.0*aBMax[k])))*aBMin[k])));
                }
            }
        }
        if(abs(deltaqnext[k]) >= eps) {
            dwFTmp = std::min(dqFMax[k] / abs(deltaqnext[k]), sqrt(fastPow(dqIBMaxNext[k],2.0)-2.0*aSnextMin[k]*(qInext[k]-qF[k])) / abs(deltaqnext[k]));
            dwF = std::min(dwF,dwFTmp);
        } // Else there is no joint which moves...
    }

    return dwF;
}

template <class T>
double BlendedTrajectory<T>::tau(double x, double uI, double uF, double a) {
    return (uF - uI)/a + abs(x - ( (fastPow(uF,2)-fastPow(uI,2)) / (2.0*a) )) / std::max(abs(uI), abs(uF));
}

template <class T>
double BlendedTrajectory<T>::findDWSReached(double wI, double dwI, double wFprev, double dwFprev, double dwSMax, double ddwSMax, double ddwSMin) {
    return std::min(dwSMax, sqrt(((-ddwSMin*fastPow(dwFprev,2.0) + ddwSMax*fastPow(dwI,2.0)) - 2.0*ddwSMin*ddwSMax*(1.0 - wFprev - wI))/(-ddwSMin + ddwSMax)));
}

template <class T>
double BlendedTrajectory<T>::findTSdecc(double dwI, double dwSreached, double ddwSMin) {
    return (dwI - dwSreached)/ddwSMin;
}

template <class T>
double BlendedTrajectory<T>::findTS(double wI, double dwI, double wFprev, double dwFprev, double ddwSMax, double ddwSMin, double dwSreached, double TSdecc) {
    return TSdecc + tau(1.0 - wFprev - wI - (fastPow(dwI,2.0) - fastPow(dwSreached,2.0))/(2.0*ddwSMin), dwFprev, dwSreached, ddwSMax);
}

template <class T>
double BlendedTrajectory<T>::findTB(T& qI, T& qF, T& qext, T& dqI, T& dqF, T& aBMax, T& aBMin, T& deltaq, T& deltaqnext) {
    double TB = 0.0;
    T Tk(K);
    for(unsigned int k=0; k<K; k++){
        if(abs(deltaq[k]) < eps && abs(deltaqnext[k]) < eps){
            //Stopping joint
            Tk[k] = 0.0;
        }else{
            if(dqI[k]*dqF[k] < 0.0){
                //Turning joint
                Tk[k] = tau(qext[k]-qI[k], dqI[k], 0.0, aBMax[k]) + tau(qF[k]-qext[k], 0.0, dqF[k], aBMax[k]);
            }else{
                //Non-turning joint
                if((dqF[k]-dqI[k])*(qF[k]-qI[k]) >= 0.0){
                    //Initial velocity smaller than final velocity
                    Tk[k] = tau(qF[k]-qI[k], dqI[k], dqF[k], aBMin[k]);
                }else{
                    //Initial velocity larger than final velocity
                    Tk[k] = tau(qF[k]-qI[k], dqI[k], dqF[k], aBMax[k]);
                }
            }
        }
        if(TB < Tk[k]){
            TB = Tk[k];
        }
    }
    return TB;
}

// Static functions ----------------------------------------------------
template <class T>
double BlendedTrajectory<T>::phimin(double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= v/a)
            return 0.5 * a * fastPow(t, 2.0);
        else if(t > v/a && t <= tau)
            return 0.5 * fastPow(v, 2.0) / a + v * (t - v/a);
        else // t > tau
            return 0.0;      
    }
}

template <class T>
double BlendedTrajectory<T>::dphimin(double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= v/a)
            return a * t;
        else if(t > v/a && t <= tau)
            return v;
        else // t > tau
            return 0.0;      
    }
}

template <class T>
double BlendedTrajectory<T>::ddphimin(double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= v/a)
            return a;
        else if(t > v/a && t <= tau)
            return 0.0;
        else // t > tau
            return 0.0;      
    }
}

template <class T>
double BlendedTrajectory<T>::phimax(double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t <= tau - v/a)
            return 0.0;
        else if(t > tau - v/a && t <= tau)
            return 0.5 * a * fastPow(t - tau + v/a, 2.0);
        else // t > tau
            return 0.0;      
    }
}

template <class T>
double BlendedTrajectory<T>::dphimax(double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t <= tau - v/a)
            return 0.0;
        else if(t > tau - v/a && t <= tau)
            return a * t + v - a * tau;
        else // t > tau
            return 0.0;      
    }
}

template <class T>
double BlendedTrajectory<T>::ddphimax(double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t <= tau - v/a)
            return 0.0;
        else if(t > tau - v/a && t <= tau)
            return a;
        else // t > tau
            return 0.0;      
    }
}

template <class T>
double BlendedTrajectory<T>::phistarmin(double v0, double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= tau)
            return v0 * t + (phimin(v - v0, a, tau, t));
        else // t > tau
            return 0.0;
    }
}

template <class T>
double BlendedTrajectory<T>::dphistarmin(double v0, double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= tau)
            return v0 + dphimin(v - v0, a, tau, t);
        else // t > tau
            return 0.0;
    }
}

template <class T>
double BlendedTrajectory<T>::ddphistarmin(double v0, double v, double a, double tau, double t){
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= tau)
            return ddphimin(v - v0, a, tau, t);
        else // t > tau
            return 0.0;
    }
}

template <class T>
double BlendedTrajectory<T>::phistarmax(double v0, double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= tau)
            return v0 * t + phimax(v - v0, a, tau, t);
        else // t > tau
            return 0.0;
    }
}

template <class T>
double BlendedTrajectory<T>::dphistarmax(double v0, double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= tau)
            return v0 + dphimax(v - v0, a, tau, t);
        else // t > tau
            return 0.0;
    }
}

template <class T>
double BlendedTrajectory<T>::ddphistarmax(double v0, double v, double a, double tau, double t) {
    if(abs(v) < eps && abs(a) < eps) {
        return 0.0;
    } else {
        if(t < 0.0)
            return 0.0;
        else if(t >= 0.0 && t <= tau)
            return ddphimax(v - v0, a, tau, t);
        else // t > tau
            return 0.0;
    }
}

