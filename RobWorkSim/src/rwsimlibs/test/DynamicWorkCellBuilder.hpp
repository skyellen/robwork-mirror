/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TEST_DYNAMICWORKCELLBUILDER_HPP_
#define RWSIMLIBS_TEST_DYNAMICWORKCELLBUILDER_HPP_

/**
 * @file DynamicWorkCellBuilder.hpp
 *
 * \copydoc rwsimlibs::test::DynamicWorkCellBuilder
 */

#include <rw/common/Ptr.hpp>
#include <rw/math/Vector3D.hpp>

namespace rwsim { namespace dynamics { class DynamicWorkCell; } }
namespace rwsim { namespace dynamics { struct BodyInfo; } }

namespace rwsimlibs {
namespace test {
//! @addtogroup rwsimlibs_test

//! @{
/**
 * @brief Helper for building Dynamic Workcells.
 */
class DynamicWorkCellBuilder {
public:
	//! @brief Color scheme specification.
	struct ColorScheme {
		//! @brief Constructor.
		ColorScheme();

		/**
		 * @brief Construct with same color for everything.
		 * @param color [in] the rgb color.
		 */
		ColorScheme(const rw::math::Vector3D<float>& color);

		/**
		 * @brief Construct with separate colors for fixed, kinematic and dynamic bodies.
		 * @param fixedBodies [in] color for fixed bodies.
		 * @param kinematicBodies [in] color for kinematic bodies.
		 * @param dynamicBodies [in] color for dynamic bodies.
		 */
		ColorScheme(const rw::math::Vector3D<float>& fixedBodies, const rw::math::Vector3D<float>& kinematicBodies, const rw::math::Vector3D<float>& dynamicBodies);

		//! @brief Color for fixed bodies.
		rw::math::Vector3D<float> fixedBodies;
		//! @brief Color for kinematic bodies.
		rw::math::Vector3D<float> kinematicBodies;
		//! @brief Color for dynamic bodies.
		rw::math::Vector3D<float> dynamicBodies;
	};

	//! @brief Default color scheme.
	struct PaHColors: public ColorScheme {
		//! @brief Constructor.
		PaHColors();
	};

	/**
	 * @brief Create new builder with given color scheme.
	 * @param colors [in] the colors to use.
	 */
	DynamicWorkCellBuilder(const ColorScheme& colors = PaHColors());

	//! @brief Destructor.
	virtual ~DynamicWorkCellBuilder();

	/**
	 * @brief Add a fixed floor.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param name [in] (optional) the name of the body - default is Floor.
	 * @param trimesh [in] (optional) use a trimesh to represent the geometry.
	 */
	void addFloor(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, const std::string& name = "Floor", bool trimesh = false) const;

	/**
	 * @brief Add a fixed plane.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param n [in] the normal.
	 * @param d [in] the distance.
	 * @param name [in] (optional) the name of the body - default is Plane.
	 * @param trimesh [in] (optional) use a trimesh to represent the geometry.
	 */
	void addPlane(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, const rw::math::Vector3D<>& n, double d, const std::string& name = "Plane", bool trimesh = false) const;

	/**
	 * @brief Add a dynamic ball.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param radius [in] radius of the ball.
	 * @param density [in] the mass density.
	 * @param name [in] (optional) the name of the body - default is Ball.
	 * @param parent [in] (optional) the name of the parent frame - default is WORLD.
	 */
	void addBall(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, double density, const std::string& name = "Ball", const std::string& parent = "WORLD") const;

	/**
	 * @brief Add a fixed ball.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param radius [in] radius of the ball.
	 * @param name [in] (optional) the name of the body - default is Ball.
	 * @param parent [in] (optional) the name of the parent frame - default is WORLD.
	 */
	void addBallFixed(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, const std::string& name = "Ball", const std::string& parent = "WORLD") const;

	/**
	 * @brief Add a dynamic cylinder.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param radius [in] radius of the cylinder.
	 * @param height [in] the length of the cylinder.
	 * @param density [in] the mass density.
	 * @param name [in] (optional) the name of the body - default is Cylinder.
	 * @param parent [in] (optional) the name of the parent frame - default is WORLD.
	 * @param trimesh [in] (optional) use a trimesh to represent the geometry.
	 */
	void addCylinder(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, double height, double density, const std::string& name = "Cylinder", const std::string& parent = "WORLD", bool trimesh = false) const;

	/**
	 * @brief Add a fixed cylinder.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param radius [in] radius of the cylinder.
	 * @param height [in] the length of the cylinder.
	 * @param name [in] (optional) the name of the body - default is Cylinder.
	 * @param parent [in] (optional) the name of the parent frame - default is WORLD.
	 * @param trimesh [in] (optional) use a trimesh to represent the geometry.
	 */
	void addCylinderFixed(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, double height, const std::string& name = "Cylinder", const std::string& parent = "WORLD", bool trimesh = false) const;

	/**
	 * @brief Add a dynamic tube.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param radius [in] radius of the tube.
	 * @param thickness [in] the thickness of the tube.
	 * @param height [in] the length of the tube.
	 * @param density [in] the mass density.
	 * @param name [in] (optional) the name of the body - default is Tube.
	 * @param trimesh [in] (optional) use a trimesh to represent the geometry.
	 */
	void addTube(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double radius, double thickness, double height, double density, const std::string& name = "Tube", bool trimesh = false) const;

	/**
	 * @brief Add a dynamic box.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param x [in] side length.
	 * @param y [in] side length.
	 * @param z [in] side length.
	 * @param density [in] the mass density.
	 * @param name [in] (optional) the name of the body - default is Box.
	 * @param trimesh [in] (optional) use a trimesh to represent the geometry.
	 */
	void addBox(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double x, double y, double z, double density, const std::string& name = "Box", bool trimesh = false) const;

	/**
	 * @brief Add a kinematic box.
	 * @param dwc [in] the dynamic workcell to add new body to.
	 * @param x [in] side length.
	 * @param y [in] side length.
	 * @param z [in] side length.
	 * @param name [in] (optional) the name of the body - default is Box.
	 * @param trimesh [in] (optional) use a trimesh to represent the geometry.
	 */
	void addBoxKin(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double x, double y, double z, const std::string& name = "Box", bool trimesh = false) const;

	/**
	 * @brief Construct default body info.
	 * @return default body info.
	 */
	static rwsim::dynamics::BodyInfo defaultInfo();

	/**
	 * @brief Insert default info into existing body info.
	 * @param info [in/out] the body info to update.
	 */
	static void defaultInfo(rwsim::dynamics::BodyInfo& info);

	/**
	 * @brief Add mass properties for a ball.
	 * @param info [in/out] the body info to update.
	 * @param radius [in] the radius of the ball.
	 * @param density [in] the mass density.
	 */
	static void ballInfo(rwsim::dynamics::BodyInfo& info, double radius, double density);

	/**
	 * @brief Add mass properties for a cylinder.
	 * @param info [in/out] the body info to update.
	 * @param radius [in] the radius.
	 * @param height [in] the length of the cylinder.
	 * @param density [in] the mass density.
	 */
	static void cylinderInfo(rwsim::dynamics::BodyInfo& info, double radius, double height, double density);

	/**
	 * @brief Add mass properties for a tube.
	 * @param info [in/out] the body info to update.
	 * @param radius [in] the radius.
	 * @param thickness [in] the thickness of the tube.
	 * @param height [in] the length of the tube.
	 * @param density [in] the mass density.
	 */
	static void tubeInfo(rwsim::dynamics::BodyInfo& info, double radius, double thickness, double height, double density);

	/**
	 * @brief Add mass properties for a box.
	 * @param info [in/out] the body info to update.
	 * @param x [in] side length.
	 * @param y [in] side length.
	 * @param z [in] side length.
	 * @param density [in] the mass density.
	 */
	static void boxInfo(rwsim::dynamics::BodyInfo& info, double x, double y, double z, double density);

	/**
	 * @brief Add material data to the dynamic workcell.
	 * @param dwc [in] the dynamic workcell to add data to.
	 * @param friction [in] (optional) the friction coefficient - default is 0.
	 * @param restitution [in] (optional) the restitution coefficient - default is 0.
	 */
	static void addMaterialData(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, double friction = 0, double restitution = 0);

	/**
	 * @brief Exclude pairs of bodies from contact detection.
	 * @param dwc [in] dynamic workcell to add information to.
	 * @param bodyA [in] pattern of the first name.
	 * @param bodyB [in] pattern of the second name.
	 */
	static void contactsExclude(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, const std::string& bodyA, const std::string& bodyB);

private:
	ColorScheme _colors;
};
//! @}
} /* namespace test */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TEST_DYNAMICWORKCELLBUILDER_HPP_ */
