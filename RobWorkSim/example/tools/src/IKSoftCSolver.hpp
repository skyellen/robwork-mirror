/*
 * IKSoftCSolver.hpp
 *
 *  Created on: 27-05-2009
 *      Author: jimali
 */

#ifndef IKSOFTCSOLVER_HPP_
#define IKSOFTCSOLVER_HPP_

/**
 * @file IKSoftCSolver.hpp
 */

#include <rw/invkin/IterativeMultiIK.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/kinematics/FKRange.hpp>

#include <map>
#include <vector>

#include <ode/ode.h>

/** \addtogroup algorithms */
/*@{*/

/**
 * @brief Inverse kinematics solver for dextrous hands. Solves inverse kinematics
 * for devices that has multiple end frames.
 *
 * This method use a traditional soft constraint solver which is often found in
 * rigid body physics engines, to solve the ik problem.
 *
 */
class IKSoftCSolver: public rw::invkin::IterativeMultiIK {
public:

    /**
     * @brief Constructs ik solver for a treedevice.
     *
     * @param device [in] Device to solve for
     * @param state [in] State of the workcell
     */
    IKSoftCSolver(rw::models::SerialDevice* arm,
                  rw::models::TreeDevice* device,
                  const std::vector<rw::kinematics::Frame*>& foi,
                  const rw::kinematics::State& state);

    /**
     * @brief Constructs ik solver for a treedevice.
     *
     * @param device [in] Device to solve for
     * @param state [in] State of the workcell
     */
    IKSoftCSolver(rw::models::TreeDevice* device,
                const rw::kinematics::State& state);

    /**
     * @brief Constructs an iksolver for a joint device with end frames
     * \b foi
     *
     * @param device [in] Device to solve for
     * @param foi [in] The end frames of the device
     * @param state [in] State of the workcell
     */
    IKSoftCSolver(rw::models::JointDevice* device,
                const std::vector<rw::kinematics::Frame*>& foi,
                const rw::kinematics::State& state);


    virtual ~IKSoftCSolver();
    /**
     * @copydoc rw::invkin::IterativeIK::solve
     */
    std::vector<rw::math::Q> solve(const std::vector<rw::math::Transform3D<> >& baseTend,
                                   const rw::kinematics::State& state) const;


    /**
     * @brief the timestep is used to step the solver toward the solution. Large
     * time steps may accelerate execution time, but instability may occour which
     * will result in no or poor solutions.
     * @param dt [in] time step in seconds. Default is 0.1s.
     */
    void setTimeStep(double dt){
        _dt = dt;
    }

    /**
     * @brief if true the configuration that best approximates the solution
     * is returned even though it does not meet minimum accuracy.
     * @param rbf [in] true to enable return of best fit, false to disable it.
     */
    void setReturnBestFit(bool rbf){
        _returnBestFit = rbf;
    }

    rw::math::Transform3D<> getBaseT3D(){ return _handBaseT3d;}

public:
    // internally used
    struct InvKinState {
        dWorldID worldId;
        dSpaceID spaceId;

        std::map<rw::kinematics::Frame*,dJointID> jointMap;
        std::map<rw::kinematics::Frame*,dJointID> motorMap;
        std::map<rw::kinematics::Frame*,dBodyID> bodyMap;
        std::map<dBodyID,rw::kinematics::Frame*> bodyToFrameMap;

        std::vector<dBodyID> endJBodies;
        std::vector<rw::kinematics::Frame*> endJoints;
        std::vector<rw::kinematics::Frame*> endeff; // the actual end effectors

        std::vector<rw::kinematics::Frame*> frames;
        std::vector<dBodyID> allbodies;
        std::vector<rw::kinematics::Frame*> joints;

        std::vector<rw::kinematics::Frame*> depJoints;

    };

private:
    rw::models::SerialDevice* _arm;
    rw::models::JointDevice* _device;
    rw::common::PropertyMap _properties;
    std::vector<rw::kinematics::Frame*> _foi; // frames of interest, end frames
    //std::vector<boost::shared_ptr<rw::kinematics::FKRange> > _fkranges;
    bool _returnBestFit;
    double _dt;
    mutable rw::math::Transform3D<> _handBaseT3d;

    mutable InvKinState _ikState;

};

#endif /* IKSOFTCSOLVER_HPP_ */
