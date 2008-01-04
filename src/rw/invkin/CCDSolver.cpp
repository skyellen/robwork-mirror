/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#include "CCDSolver.hpp"

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Constants.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rw/common/Property.hpp>

#include <float.h>

using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::invkin;

namespace
{
    enum JointType { RevoluteType, PrismaticType, UnknownType };

    JointType getType(const Joint& joint)
    {
        if (dynamic_cast<const RevoluteJoint*>(&joint))
            return RevoluteType;
        else if (dynamic_cast<const PrismaticJoint*>(&joint))
            return PrismaticType;
        else
            return UnknownType;
    }



    double objFunc(double x, double k1, double k2, double k3 ){
        double cosx = cos(x);
        return k1*(1-cosx) + k2*cosx + k3*sin(x);
    }


    bool performLocalSearch(const SerialDevice *_device,
                            const Transform3D<> &bTed,
                            double maxError,
                            State &state,
                            unsigned int maxIter,
                            double scale, double wpos, double worin){
        int ndof = _device->getDOF();
        int niterations =0;
        int maxIterations = maxIter;
        while(maxIterations--){
            niterations++;
            //std::cout << std::endl;
            // iterate through all joints
            for(int i=ndof-1;i>=0;i--){
                const Joint *curr_j = _device->getActiveJoint(i);

                const Transform3D<>& Tb =
                    Kinematics::FrameTframe(_device->getBase(), curr_j, state);

                Transform3D<> Tid = inverse(Tb)*bTed;
                Vector3D<> Pid = Tid.P();

                Transform3D<> Tic = inverse(Tb)*(_device->baseTend(state));
                Vector3D<> Pic = Tic.P();

                double rho =
                    std::min( Pid.norm2() , Pic.norm2() ) /
                    std::max( Pid.norm2() , Pic.norm2() );

                double wpos = scale*(1+ rho);

                double orin=0,orin1=0,err_orin=0,err_pos=0,orin2=0;
                Vector3D<> ujd[3],ujc[3];
                for(int j=0;j<3;j++){
                    ujd[j] = Vector3D<>(Tid(0,j),Tid(1,j),Tid(2,j));
                    ujc[j] = Vector3D<>(Tic(0,j),Tic(1,j),Tic(2,j));
                    orin  += (ujd[j])[2]*(ujc[j])[2];
                    orin1 += dot(ujd[j],ujc[j]);
                    orin2 += cross(ujc[j],ujd[j])[2];
                    err_orin +=
                        (dot(ujd[j],ujc[j])-1)*
                        (dot(ujd[j],ujc[j])-1);
                }

                err_pos = (Pid - Pic).norm2();
                err_pos *= err_pos;
                //std::cout << " "<< err_pos << " " << err_orin;
                double error = err_pos+err_orin;
                if(error<maxError){ // precision reached
                    return true;
                }

                JointType type = getType(*curr_j);
                if (type == RevoluteType) {
                    const Joint *rJoint = curr_j;
                    double k1 = wpos*(Pid[2])*(Pic[2])+worin*orin;
                    double k2 = wpos*dot(Pid,Pic)+worin*orin1;
                    double k3 = wpos*cross(Pic,Pid)[2]+worin*orin2;
                    double dq = atan2(k3,(k2-k1));
                    double qMax = dq+Pi;
                    double qMin = dq-Pi;
                    //std::cout << "q:" << q << " k1:" << k1 << " k2:" << k2 << " k3:"<<k3<< std::endl;
                    // we need also to evaluate q+PI and q-PI and chose the value that maximize the
                    // objective function the most
                    double maxObjFunc = 0;
                    if( dq==DBL_MAX )
                        continue;
                    maxObjFunc = objFunc(dq,k1,k2,k3);
                    // Objective function = k1*(1-cos(x))+k2*cos(x)+k3*sin(x)
                    //std::cout << "Max q: " << maxObjFunc << std::endl;

                    // second deriv = (k1-k2)*cos(x) - k3*sin(x)
                    double qMinMax = objFunc(qMin,k1,k2,k3);
                    //std::cout << "MinMax q: " << qMinMax << std::endl;
                    if( (-Pi<qMin) && (qMin<Pi) /*&& (k1-k2)*cos(qMin)-k3*sin(qMin)<0*/ ){
                        if( qMinMax > maxObjFunc ){
                            dq=qMin;
                            maxObjFunc = qMinMax;
                        }
                    }

                    double qMaxMax = objFunc(qMax,k1,k2,k3);
                    //std::cout << "MaxMax q: " << qMaxMax << std::endl;
                    if( (-Pi<qMax) && (qMax<Pi) /*&& (k1-k2)*cos(qMax)-k3*sin(qMax)<0*/ ){
                        if( qMaxMax > maxObjFunc ){
                            dq=qMax;
                            maxObjFunc = qMaxMax;
                        }
                    }

                    /*if( dq>0.8 ){
                       dq *= 0.8/dq;
                    }*/

                    // calculate q
                    double q = rJoint->getQ(state)[0] + dq;

                    // check if q is inside the joint boundaries
                    std::pair<double,double> b = rJoint->getBounds();
                    if( q<b.first ){
                        q=b.first;
                    } else if( q>b.second ){
                        q=b.second;
                    }

                    rJoint->setQ(state, &q);

                } else if (type == PrismaticType) {
                    const Joint *pJoint = curr_j;
                    double q = pJoint->getQ(state)[0] + (Pid - Pic)[2];

                    std::pair<double,double> b = pJoint->getBounds();
                    if(q<b.first)
                        q = b.first;
                    if(q>b.second)
                        q = b.second;

                    pJoint->setQ(state, &q);

                } else {
                    RW_WARN("CCDSolver only support Revolute and Prismatic joint types");
                    return false;
                }
            }
        }
        return false;
    }
}

CCDSolver::CCDSolver(const SerialDevice* device) :
    _worin(1),
    _scale(30),
    _maxQuatStep(0.2),
    _device(device)
{
    setMaxIterations(20);
    setMaxError(1e-5);
}

void CCDSolver::setMaxLocalStep(double quatlength, double poslength){
    _maxQuatStep = quatlength;
}

std::vector<Q> CCDSolver::solve(const Transform3D<>& bTed, const State& initial_state) const
{
    double ep = 1000000000.0;
    double eo = 1000000000.0;
    double alpha = 1000;

    std::vector<Q> solutions;
    bool no_solution = false;

    // unsigned int maxIterations = getMaxIterations();
    double maxError = getMaxError();
    State state = initial_state;

    unsigned int i = 0;
    do {
        ++i;
        for (signed int j = _device->getDOF()-1; j >= 0; --j) {
            const Joint* joint = _device->getActiveJoint(j);
            double qi = 0;

            Transform3D<> bTe = _device->baseTend(state);
            Vector3D<> Pbc = bTe.P();   // Current position
            Vector3D<> Pbd = bTed.P();  // Desired position

            Transform3D<> bTj = _device->baseTframe(joint, state);
            Vector3D<> Pbi = bTj.P();   // Current frame's position
            Vector3D<> Pid = Pbd - Pbi;
            Vector3D<> Pic = Pbc - Pbi;
            Vector3D<> Bai(bTj(0, 2), bTj(1, 2), bTj(2, 2));

            Rotation3D<> RcT = bTe.R();  // Current rotation
            Rotation3D<> RdT = bTed.R(); // Desired rotation

            Vector3D<> u1d(RdT(0, 0), RdT(1, 0), RdT(2, 0));   // Desired x-axis
            Vector3D<> u2d(RdT(0, 1), RdT(1, 1), RdT(2, 1));   // Desired y-axis
            Vector3D<> u3d(RdT(0, 2), RdT(1, 2), RdT(2, 2));   // Desired z-axis

            Vector3D<> u1c(RcT(0, 0), RcT(1, 0), RcT(2, 0));   // Current x-axis
            Vector3D<> u2c(RcT(0, 1), RcT(1, 1), RcT(2, 1));   // Current y-axis
            Vector3D<> u3c(RcT(0, 2), RcT(1, 2), RcT(2, 2));   // Current z-axis

            JointType type = getType(*joint);
            if (type == RevoluteType) {
				double wp = alpha * (1 + std::min(Pid.norm2(), Pic.norm2()) / std::max(Pid.norm2(), Pic.norm2()));
                double wo = 1;

                double k1 = wp * (dot(Pid, Bai) * dot(Pic, Bai)) + wo * (dot(u1d, Bai) * dot(u1c, Bai) + dot(u2d, Bai) * dot(u2c, Bai) + dot(u3d, Bai) * dot(u3c, Bai));
                double k2 = wp * (dot(Pid, Pic)) + wo * (dot(u1d, u1c) + dot(u2d, u2c) + dot(u3d, u3c));
                double k3 = dot(Bai, wp * cross(Pic, Pid) + wo * (cross(u1c, u1d) + cross(u2c, u2d) + cross(u3c, u3d)));

                double phi1 = atan(k3/(k2-k1));
                double phi2 = phi1 > 0 ? phi1 - Pi : phi1 + Pi;

                double secdev1 = (k1 - k2) * cos(phi1) - k3 * sin(phi1);
                double secdev2 = (k1 - k2) * cos(phi2) - k3 * sin(phi2);

                if (secdev1 < 0 && secdev2 < 0) {
                    double val1 = k1 * (1 - cos(phi1)) + k2 * cos(phi1) + k3 * sin(phi1);
                    double val2 = k1 * (1 - cos(phi2)) + k2 * cos(phi2) + k3 * sin(phi2);

                    qi = (val1 > val2) ? phi1 : phi2;
                } else if (secdev1 < 0) {
                    qi = phi1;
                } else if (secdev2 < 0) {
                    qi = phi2;
                } else {
                    std::cout << "No maximizing value found" << std::endl;
                }
            } else if (type == PrismaticType) {
                qi = dot((Pid - Pic), Bai);
            } else {
                std::cerr << "Joint type not supported by CCDSolver" << std::endl;
                return solutions;
            }

            Q q = _device->getQ(state);
            q(j) = q(j) + qi;
            _device->setQ(q, state);
        }


        // Calculate the error
        Transform3D<> bTe = _device->baseTend(state);
        Vector3D<> Pbc = bTe.P();   // Current position
        Vector3D<> Pbd = bTed.P();  // Desired position

        Rotation3D<> RcT = bTe.R();  // Current rotation  -- inverse ???
        Rotation3D<> RdT = bTed.R(); // Desired rotation  -- inverse ???

        Vector3D<> u1d(RdT(0, 0), RdT(1, 0), RdT(2, 0));   // Desired x-axis
        Vector3D<> u2d(RdT(0, 1), RdT(1, 1), RdT(2, 1));   // Desired y-axis
        Vector3D<> u3d(RdT(0, 2), RdT(1, 2), RdT(2, 2));   // Desired z-axis

        Vector3D<> u1c(RcT(0, 0), RcT(1, 0), RcT(2, 0));   // Current x-axis
        Vector3D<> u2c(RcT(0, 1), RcT(1, 1), RcT(2, 1));   // Current y-axis
        Vector3D<> u3c(RcT(0, 2), RcT(1, 2), RcT(2, 2));   // Current z-axis

        Vector3D<> Pcd = Pbd - Pbc;     // Error of position
        ep = pow(Pcd.norm2(), 4);       // Should this really be 4 and not 2???
        eo = pow(pow(dot(u1d, u1c) - 1, 2)   +   pow(dot(u2d, u2c) - 1, 2)   +   pow(dot(u3d, u3c) - 1, 2), 2);   // Should there really be an extra pow(..., 2) there

        //std::cout << "i = " << i;
        //std::cout << ",     error pos = " << ep << ",     error orientation = " << eo << std::endl;

        if (i > 10) {
            //std::cerr << "CCD Inverse kinematics failed" << std::endl;
            no_solution = true;
        }
    } while (ep > maxError || eo > maxError);


    // Push the solution into the vector if solution good enough
    if (!no_solution)
        solutions.push_back(_device->getQ(state));

    return solutions;
/*
    unsigned int maxIterations = getMaxIterations();
    double maxError = getMaxError();
    State state = initial_state;

    // if the distance between current and end configuration is
    // too large then split it up in smaller steps
    const Transform3D<>& bTeInit = _device->baseTend(state);
    Quaternion<> q1( bTeInit.R() );
    Quaternion<> q2( bTed.R() );
    Quaternion<> qDist = q2-q1;
    double length = qDist.getLength();
    int steps = (int)ceil( length/_maxQuatStep );
    Vector3D<> posDist = bTed.P()-bTeInit.P();

    std::cout << "Steps: " << steps << std::endl;
    //std::cout << "INIT: "<< q1 << " "  << bTeInit.P() << std::endl;
    //std::cout << "GOAL: "<< q2 << " "  << bTed.P() << std::endl;
    // now perform newton iterations to each generated via point
    for(int step=1; step < steps; step++){
        // calculate
        std::cout << step;
        double nStep = ((double)step) / (double)steps;
        Quaternion<> qNext = qDist;
        qNext *= nStep;
        //qNext.normalize();
        qNext = q1 + qNext;
        qNext.normalize();
        Vector3D<> pNext = bTeInit.P() + posDist*nStep;
        Transform3D<> bTedLocal(pNext,qNext);

        // we allow a relative large error since its only via points
        bool found = performLocalSearch(_device, bTedLocal, maxError*1000, state, maxIterations,_scale,_wpos,_worin );
        if(!found)
            return std::vector<Q>();
    }
    // now we perform yet another newton search with higher precision to determine
    // the end result

    // now we perform yet another newton search with higher precision to determine
    // the end result
    if( performLocalSearch(_device, bTed, maxError, state, maxIterations,_scale,_wpos,_worin) ){
        std::vector<Q> result;
        result.push_back(_device->getQ(state));
        return result;
    }
    return std::vector<Q>();
    */
}
