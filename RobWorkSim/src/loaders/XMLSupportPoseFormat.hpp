/*
 * XMLSupportPoseFormat.hpp
 *
 *  Created on: 18-03-2009
 *      Author: jimali
 */

/**
 * @brief
 *
 * <SupportPoseCollection>
 *
 *  <SupportPose>
 *   <Frame>framename</Frame>
 *   <Degree>int[0;3]</Degree>
 *   <Quality></Quality>
 *   <Segments>
 *    <Segment from="" to=""> </Segment>
 *   </Segments>
 *  </SupportPose>
 *
 *
 * </SupportPoseCollection>
 */
class XMLSupportPoseFormat {
public:
	/**
	 * @brief save a single pose of all rigid bodies in a workcell.
	 */
	static void savePoses(const std::string& FileNameAndPath,
			const std::vector<rw::kinematics::Frame*>& bodies,
			const std::vector<std::vector<SupportPose> >& poses,
			const std::string& SimulationDescription);

	/**
	 * @brief save a multiple pose of all rigid bodies in a workcell.
	 */
	static void savePoses(const std::string& FileNameAndPath,
			const std::vector<dynamics::RigidBody*>& bodies,
			const std::vector< rw::kinematics::State> states,
			const std::string& ObjectName,
			const std::string& SimulationDescription);

};
