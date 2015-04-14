/*
 * rwsim.hpp
 *
 *  Created on: 27/01/2011
 *      Author: jimali
 */

#ifndef RWSIM_RWSIM_HPP_
#define RWSIM_RWSIM_HPP_


#include "./contacts/BallBallStrategy.hpp"
#include "./contacts/Contact.hpp"
#include "./contacts/ContactDetector.hpp"
#include "./contacts/ContactDetectorData.hpp"
#include "./contacts/ContactModel.hpp"
#include "./contacts/ContactStrategy.hpp"
#include "./contacts/ContactStrategyData.hpp"

#include "./dynamics/Body.hpp"
#include "./dynamics/RigidBody.hpp"
#include "./dynamics/RigidJoint.hpp"
#include "./dynamics/KinematicBody.hpp"
#include "./dynamics/FixedBody.hpp"
#include "./dynamics/DynamicDevice.hpp"
#include "./dynamics/KinematicDevice.hpp"
#include "./dynamics/RigidDevice.hpp"
#include "./dynamics/DynamicWorkCell.hpp"
//#include "./#dynamics/Contact.hpp"
#include "./dynamics/ContactPoint.hpp"
#include "./dynamics/ContactCluster.hpp"
#include "./dynamics/ContactManifold.hpp"

#include "./dynamics/ContactDataMap.hpp"
#include "./dynamics/MaterialDataMap.hpp"
#include "./dynamics/DynamicUtil.hpp"
#include "./dynamics/SuctionCup.hpp"
#include "./dynamics/OBRManifold.hpp"

#include "./sensor/SimulatedTactileSensor.hpp"
#include "./sensor/TactileArraySensor.hpp"
#include "./sensor/BodyContactSensor.hpp"
#include "./sensor/SimulatedFTSensor.hpp"

#include "./loaders/DynamicWorkCellLoader.hpp"
#include "./loaders/ScapePoseFormat.hpp"

#include "./util/MovingAverage.hpp"
#include "./util/PointRANSACFitting.hpp"
#include "./util/DistModel.hpp"
#include "./util/PlaneModel.hpp"
#include "./util/CircleModel.hpp"
#include "./util/SupportPose.hpp"
#include "./util/PlanarSupportPoseGenerator.hpp"
//#include "./#util/LineFit.hpp"
//#include "./#util/LinePolar.hpp"

#include "./util/HughLineExtractor.hpp"

#include "./util/RestingPoseGenerator.hpp"

#include "./util/CollisionFreeSampler.hpp"
#include "./util/FiniteStateSampler.hpp"
#include "./util/SpherePoseSampler.hpp"
#include "./util/StateSampler.hpp"
#include "./util/PreshapeSampler.hpp"
#include "./util/GraspPolicy.hpp"
#include "./util/GraspStrategy.hpp"
#include "./util/GraspPolicyFactory.hpp"
#include "./util/GraspStrategyFactory.hpp"
#include "./util/TargetConfigGraspPolicy.hpp"

#include "./drawable/RenderGhost.hpp"
#include "./drawable/RenderPoints.hpp"
#include "./drawable/RenderPlanes.hpp"
#include "./drawable/RenderCircles.hpp"
#include "./drawable/RenderContacts.hpp"

#include "./control/PDController.hpp"
#include "./control/SyncPDController.hpp"
#include "./control/VelRampController.hpp"
#include "./control/SuctionCupController.hpp"
#include "./control/SerialDeviceController.hpp"

#include "./simulator/DynamicSimulator.hpp"
#include "./simulator/ThreadSimulator.hpp"
#include "./simulator/PhysicsEngineFactory.hpp"

#include "./rwphysics/RWSimulator.hpp"
#include "./rwphysics/RWDebugRender.hpp"
#include "./rwphysics/BodyIntegrator.hpp"
#include "./rwphysics/ContactModel.hpp"
#include "./rwphysics/ChatterjeeContactModel.hpp"
#include "./rwphysics/CNodePairMap.hpp"
#include "./rwphysics/CNodePool.hpp"
#include "./rwphysics/ConstantForceManipulator.hpp"
#include "./rwphysics/ConstraintNode.hpp"
#include "./rwphysics/ConstraintEdge.hpp"
#include "./rwphysics/ContactGraph.hpp"
#include "./rwphysics/ContactModelFactory.hpp"
#include "./rwphysics/EulerIntegrator.hpp"
#include "./rwphysics/ConstraintSolver.hpp"
#include "./rwphysics/SequintialImpulseSolver.hpp"
#include "./rwphysics/RWBody.hpp"
#include "./rwphysics/RWBodyPool.hpp"
#include "./rwphysics/BodyController.hpp"
//#include "./rwphysics/GuendelContactModel.hpp"
//#include "./#rwphysics/ConstraintSolver.hpp"

#define USE_ROBWORKSIM_NAMESPACE \
    namespace rwsim { namespace contacts {}} \
    namespace rwsim { namespace control {}} \
    namespace rwsim { namespace drawable {}} \
    namespace rwsim { namespace dynamics {}} \
    namespace rwsim { namespace loaders {}} \
    namespace rwsim { namespace rwphysics {}} \
    namespace rwsim { namespace sensor {}} \
    namespace rwsim { namespace simulator {}} \
    namespace rwsim { namespace util {}} \
    namespace rwsimlibs { namespace gui {}} \
    namespace rwsimlibs { namespace ode {}} \
    namespace rwsimlibs { namespace bullet {}} \
    namespace rwsimlibs { namespace lua {}} \
    namespace rwsimlibs { namespace plugins {}} \
    namespace robworksim \
    { \
        using namespace rwsim; \
        using namespace rwsim::contacts; \
        using namespace rwsim::control; \
        using namespace rwsim::drawable; \
        using namespace rwsim::dynamics; \
        using namespace rwsim::loaders; \
        using namespace rwsim::rwphysics; \
        using namespace rwsim::sensor; \
        using namespace rwsim::simulator; \
        using namespace rwsim::util; \
        using namespace rwsimlibs::gui; \
        using namespace rwsimlibs::ode; \
        using namespace rwsimlibs::bullet; \
        using namespace rwsimlibs::lua; \
        using namespace rwsimlibs::plugins; \
    }

#define RWSIM_USE_RWP_NAMESPACE \
    USE_ROBWORKSIM_NAMESPACE \
    namespace rwp \
    { \
        using namespace robworksim; \
    }


#endif /* RWSIM_HPP_ */
