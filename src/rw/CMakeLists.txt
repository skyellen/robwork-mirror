set(RwSrcFiles
  ${RW_USER_FILES}
  proximity/CollisionStrategy.cpp
  proximity/CollisionToleranceStrategy.cpp
  proximity/ProximityStrategy.cpp
  proximity/CollisionDetector.cpp
  proximity/DistanceStrategy.cpp
  proximity/DistanceToleranceStrategy.cpp
  proximity/DistanceCalculator.cpp
  common/ConcatVectorIterator.cpp
  common/IOUtil.cpp
  common/Log.cpp
  common/Message.cpp
  common/Property.cpp
  common/PropertyBase.cpp
  common/PropertyMap.cpp
  common/PropertyType.cpp
  common/StringUtil.cpp
  common/Timer.cpp
  common/TimerUtil.cpp
  common/ConvertUtil.cpp
  common/VectorIterator.cpp
  geometry/FaceArrayFactory.cpp
  geometry/Geometry.cpp
  geometry/GeometryBox.cpp
  geometry/GeometryCylinder.cpp
  geometry/GeometryFactory.cpp
  geometry/GeometrySTL.cpp
  interpolator/CubicSegment.cpp
  interpolator/CubicSplineInterpolator.cpp
  ./interpolator/Interpolator.cpp
  ./interpolator/InterpolatorIterator.cpp
  ./interpolator/LloydHaySegment.cpp
  ./interpolator/PointTimeIndex.cpp
  ./interpolator/PointTimeIndexFactory.cpp
  ./interpolator/Pose6dLloydHaySegment.cpp
  ./interpolator/Pose6dStraightSegment.cpp
  ./interpolator/StateTrajectory.cpp
  ./interpolator/StateTrajectoryFactory.cpp
  ./interpolator/StraightInterpolator.cpp
  ./interpolator/StraightSegment.cpp
  ./interpolator/Timed.cpp
  ./interpolator/TimedUtil.cpp
  ./interpolator/Trajectory.cpp
  ./interpolator/TrajectoryIterator.cpp
  ./interpolator/TimedStatePath.cpp
  ./invkin/ClosedFormIK.cpp
  ./invkin/IterativeIK.cpp
  ./invkin/IterativeMultiIK.cpp
  ./invkin/CCDSolver.cpp
  ./invkin/ParallelIKSolver.cpp
  ./invkin/PieperSolver.cpp
  ./invkin/ResolvedRateSolver.cpp
  ./invkin/SimpleSolver.cpp
  ./invkin/SimpleMultiSolver.cpp  
  ./invkin/IKMetaSolver.cpp
  ./kinematics/FixedFrame.cpp
  ./kinematics/FKRange.cpp
  ./kinematics/FKTable.cpp
  ./kinematics/Frame.cpp
  ./kinematics/FrameType.cpp
  ./kinematics/Kinematics.cpp
  ./kinematics/MovableFrame.cpp
  ./kinematics/QState.cpp
  ./kinematics/State.cpp
  ./kinematics/Tree.cpp
  ./kinematics/TreeState.cpp
  ./loaders/WorkCellLoader.cpp
  ./loaders/TaskLoader.cpp
  ./loaders/path/PathLoader.cpp
  ./loaders/tul/Tag.cpp
  ./loaders/tul/TULLoader.cpp
  ./loaders/rwxml/DependencyGraph.cpp
  ./loaders/rwxml/MultipleFileIterator.cpp
  ./loaders/rwxml/XMLParserUtil.cpp
  ./loaders/rwxml/XMLRWPreParser.cpp
  ./loaders/rwxml/XMLRWParser.cpp
  ./loaders/rwxml/XMLRWLoader.cpp
  ./loaders/colsetup/CollisionSetupLoader.cpp
  ./loaders/xml/XMLErrorHandler.cpp
  ./loaders/xml/XMLParser.cpp
  ./math/EAA.cpp
  ./math/EuclideanMetric.cpp
  ./math/Jacobian.cpp
  ./math/LinearAlgebra.cpp
  ./math/MahalanobisMetric.cpp
  ./math/ManhattenMetric.cpp
  ./math/InertiaMatrix.cpp
  ./math/InfinityMetric.cpp
  ./math/Math.cpp
  ./math/Metric.cpp
  ./math/MetricUtil.cpp
  ./math/Pose6D.cpp
  ./math/Q.cpp
  ./math/Quaternion.cpp
  ./math/Rotation3D.cpp
  ./math/Rotation3DVector.cpp
  ./math/RPY.cpp
  ./math/Transform3D.cpp
  ./math/Vector2D.cpp
  ./math/Vector3D.cpp
  ./math/VelocityScrew6D.cpp
  ./math/WeightedEuclideanMetric.cpp
  ./models/Accessor.cpp
  ./models/BasicDevice.cpp
  ./models/BasicDeviceJacobian.cpp
  ./models/Cartesian6DOFDevice.cpp
  ./models/Conveyor.cpp
  ./models/ConveyorBelt.cpp
  ./models/ConveyorItem.cpp
  ./models/ConveyorSegment.cpp
  ./models/Device.cpp
  ./models/DeviceJacobian.cpp
  ./models/DeviceModel.cpp
  ./models/JointDevice.cpp
  ./models/FixedJoint.cpp
  ./models/JacobianUtil.cpp
  ./models/Joint.cpp
  ./models/JointTransform.cpp
  ./models/MobileDevice.cpp
  ./models/TreeDevice.cpp
  ./models/CompositeDevice.cpp
  ./models/ParallelDevice.cpp
  ./models/ParallelLeg.cpp
  ./models/PassivePrismaticFrame.cpp
  ./models/PassiveRevoluteFrame.cpp
  ./models/PrismaticJoint.cpp
  ./models/RevoluteJoint.cpp
  ./models/SerialDevice.cpp
  ./models/RigidBodyInfo.cpp  
  ./models/WorkCell.cpp
  ./models/TimeMetricUtil.cpp
  ./pathplanning/PathPlanner.cpp
  ./pathplanning/PlannerCollisionDetector.cpp
  ./pathplanning/PlannerUtil.cpp
  ./pathplanning/StraightLinePathPlanner.cpp
  ./pathplanning/TrajectoryPlanner.cpp
  ./pathplanning/FixedStepTrajectoryPlanner.cpp
  ./pathplanning/PathPlannerFactory.hpp
  ./pathplanning/TrajectoryPlannerFactory.hpp
  ./sensor/Camera.cpp
  ./sensor/CameraListener.cpp
  ./sensor/Image.cpp
  ./sensor/Sensor.cpp
  ./task/Task.cpp
  ./task/Task_Trajectory.cpp
  ./task/Link.cpp
  ./task/Target.cpp
  ./task/Task_Property.cpp
  ./task/Action.cpp
  ./task/Solver.cpp
  ./task/TaskUtil.cpp
  )

#Create folder structure in Visual C++ IDE
if (DEFINED MSVC)  
  SOURCE_GROUP("Source files\\proximity" REGULAR_EXPRESSION "/proximity/.*.cpp")
  SOURCE_GROUP("Source files\\common" REGULAR_EXPRESSION "/common/.*.cpp")
  SOURCE_GROUP("Source files\\geometry" REGULAR_EXPRESSION "/geometry/.*.cpp")
  SOURCE_GROUP("Source files\\iksolvers" REGULAR_EXPRESSION "/iksolvers/.*.cpp")
  SOURCE_GROUP("Source files\\invkin" REGULAR_EXPRESSION "/invkin/.*.cpp")
  SOURCE_GROUP("Source files\\interpolator" REGULAR_EXPRESSION "/interpolator/.*.cpp")
  SOURCE_GROUP("Source files\\kinematics" REGULAR_EXPRESSION "/kinematics/.*.cpp")
  SOURCE_GROUP("Source files\\loaders" REGULAR_EXPRESSION "/loaders/.*.cpp")
  SOURCE_GROUP("Source files\\math" REGULAR_EXPRESSION "/math/.*.cpp")
  SOURCE_GROUP("Source files\\models" REGULAR_EXPRESSION "/models/.*.cpp")
  SOURCE_GROUP("Source files\\pathplanning" REGULAR_EXPRESSION "/pathplanning/.*.cpp")
  SOURCE_GROUP("Source files\\sensor" REGULAR_EXPRESSION "/sensor/.*.cpp")
  SOURCE_GROUP("Source files\\task" REGULAR_EXPRESSION "/task/.*.cpp")
endif () 

add_library(rw ${RwSrcFiles})
