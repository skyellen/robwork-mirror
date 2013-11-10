#ifndef SIMULATIONTRAJECTORY_HPP
#define SIMULATIONTRAJECTORY_HPP

#include <iostream>
#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/common/Ptr.hpp>
#include <vector>
#include <string>



/**
 * @class SimulationTrajectory
 * @brief Used to record the FalconPlugin teleoperation simulation
 */
class SimulationTrajectory
{
	public: // types
		/// Smart pointer
		typedef rw::common::Ptr<SimulationTrajectory> Ptr;
	
		/// Describes a single simulation step
		struct SimulationStep
		{
			public: // types
				typedef rw::common::Ptr<SimulationStep> Ptr;
				typedef std::pair<std::string, rw::math::Transform3D<> > ObjectPose;
			
			public: // methods
				SimulationStep() : t(-1.0) {}
				
				SimulationStep(double t, rw::math::Q robotQ, rw::math::Q gripperQ, std::vector<ObjectPose> objPoses) :
					t(t),
					robotQ(robotQ),
					gripperQ(gripperQ),
					objectPoses(objPoses)
				{}
				
				friend std::ostream& operator<<(std::ostream& stream, const SimulationStep& step)
				{
					stream << step.t << ":" << step.robotQ << ":" << step.gripperQ << ":" <<
						step.objectPoses.size();
						
					typedef std::vector<ObjectPose>::const_iterator I;
					for (I it = step.objectPoses.begin(); it != step.objectPoses.end(); ++it) {
						stream << ":" << it->first << " " << it->second.P() << " " << rw::math::RPY<>(it->second.R());
					}
					
					return stream;
				}
			
			public: // data
				double t;
				rw::math::Q robotQ;
				rw::math::Q gripperQ;
				std::vector<ObjectPose> objectPoses;
		};
		
	public: // constructors
		/// Constructor
		SimulationTrajectory() {}
		
		/// Destructor
		virtual ~SimulationTrajectory() {}
		
	public: // methods
		/// Adds another step to the trajectory
		void addStep(const SimulationStep& step)
		{
			_trajectory.push_back(step);
		}
		
		/// Returns trajectory
		std::vector<SimulationStep> getSteps() const { return _trajectory; }
		
		friend std::ostream& operator<<(std::ostream& stream, const SimulationTrajectory& traj)
		{
			typedef std::vector<SimulationStep>::const_iterator I;
			for (I it = traj._trajectory.begin(); it != traj._trajectory.end(); ++it) {
				stream << *it << std::endl;
			}
			
			return stream;
		}
	
	protected: // data
		std::vector<SimulationStep> _trajectory;
};

#endif // SIMULATIONTRAJECTORY_HPP
