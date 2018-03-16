/********************************************************************************
 * Copyright 2018 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>

#include <rw/graphics/WorkCellScene.hpp>
#include <rw/graphics/SceneGraph.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/MovableFrame.hpp>

using rw::common::ownedPtr;
using namespace rw::graphics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using rw::graphics::WorkCellScene;

namespace {

class DummyDrawable: public DrawableNode {
public:
	DummyDrawable(const std::string& name): DrawableNode(name) {}
	virtual ~DummyDrawable() {}
	void draw(const DrawableNode::RenderInfo& info) const {}
	void setHighlighted(bool b) {}
	bool isHighlighted() const { return false; }
	void setDrawType(DrawType drawType) {}
	void setTransparency(float alpha) {}
	float getTransparency() { return 0; }
	void setScale(float scale) {}
	float getScale() const { return 1; }
	void setVisible(bool enable) {}
	bool isVisible() { return true; }
	const Transform3D<>& getTransform() const { return Transform3D<>::identity(); }
	void setTransform(const rw::math::Transform3D<>& t3d) {}
	void setMask(unsigned int mask) {}
	unsigned int getMask() const { return DrawableNode::Physical; }
};

class DummyDrawableGeometry: public DrawableGeometryNode {
public:
	DummyDrawableGeometry(const std::string& name): DrawableGeometryNode(name) {}
	virtual ~DummyDrawableGeometry() {}

	// DrawableNode
	void draw(const DrawableNode::RenderInfo& info) const {}
	void setHighlighted(bool b) {}
	bool isHighlighted() const { return false; }
	void setDrawType(DrawType drawType) {}
	void setTransparency(float alpha) {}
	float getTransparency() { return 0; }
	void setScale(float scale) {}
	float getScale() const { return 1; }
	void setVisible(bool enable) {}
	bool isVisible() { return true; }
	const Transform3D<>& getTransform() const { return Transform3D<>::identity(); }
	void setTransform(const rw::math::Transform3D<>& t3d) {}
	void setMask(unsigned int mask) {}
	unsigned int getMask() const { return DrawableNode::Physical; }

	// DrawableGeometryNode
	void setColor(double r, double g, double b, double alpha) {}
	void setColor(const Vector3D<>& rgb) {}
	void setAlpha(double alpha) {}
	Vector3D<> getColor() { return Vector3D<>::x(); }
	double getAlpha() { return 1; }
	void addLines(const std::vector<rw::geometry::Line >& lines) {}
	void addLine(const Vector3D<>& v1, const Vector3D<>& v2) {}
	void addGeometry(rw::common::Ptr<class rw::geometry::Geometry> geom) {}
	void addFrameAxis(double size) {}
};

class DummySceneGraph: public SceneGraph {
public:
	DummySceneGraph(): SceneGraph() {}
	virtual ~DummySceneGraph() {}
	void draw(RenderInfo& info) {}
	DrawableNode::Ptr pickDrawable(RenderInfo& info, int x, int y) { return NULL; }
	Vector3D<> unproject(SceneCamera::Ptr camera, int x, int y) { return Vector3D<>::zero(); }
	void update() {}
	void clear() {}
	DrawableGeometryNode::Ptr makeDrawableFrameAxis(const std::string& name, double size, int dmask=DrawableNode::Physical) {
		return ownedPtr(new DummyDrawableGeometry("FrameAxis"));
	}
	DrawableGeometryNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<class rw::geometry::Geometry> geom, int dmask=DrawableNode::Physical) {
		return ownedPtr(new DummyDrawableGeometry(name));
	}
	DrawableGeometryNode::Ptr makeDrawable(const std::string& name, const std::vector<class rw::geometry::Line >& lines, int dmask=DrawableNode::Physical) { return NULL; }
	DrawableNode::Ptr makeDrawable(const std::string& name, const class rw::sensor::Image& img, int dmask=DrawableNode::Virtual) { return NULL; }
	DrawableNode::Ptr makeDrawable(const std::string& name, const rw::geometry::PointCloud& scan, int dmask=DrawableNode::Virtual) { return NULL; }
	DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<class Model3D> model, int dmask=DrawableNode::Physical) { return NULL; }
	DrawableNode::Ptr makeDrawable(const std::string& name, rw::common::Ptr<class Render> render, int dmask=DrawableNode::Physical) {
		return ownedPtr(new DummyDrawable(name));
	}
	DrawableNode::Ptr makeDrawable(const std::string& filename, int dmask=DrawableNode::Physical) { return NULL; }
	SceneCamera::Ptr makeCamera(const std::string& name) { return NULL; }
};
}

TEST(WorkCellScene, Test) {
	const SceneGraph::Ptr scene = ownedPtr(new DummySceneGraph());
	const WorkCellScene::Ptr WSscene = ownedPtr(new WorkCellScene(scene));
	const WorkCell::Ptr ws = ownedPtr( new WorkCell("TEST_WS"));

	WSscene->setWorkCell(ws);
	EXPECT_EQ("TEST_WS",WSscene->getWorkCell()->getName());

	const GroupNode::Ptr world = WSscene->getWorldNode();
	MovableFrame* frame1 = new MovableFrame("Frame1");
	GroupNode::Ptr frame1_GN = WSscene->getNode(frame1);

	// Checks that world frame exsist with no children
	ASSERT_FALSE(world.isNull());
	EXPECT_EQ(0,world->nrOfChildren());
	EXPECT_FALSE(world->hasChild("Frame1"));
	EXPECT_TRUE(frame1_GN.isNull());

	ws->addFrame(frame1);
	frame1_GN = WSscene->getNode(frame1);

	// Checks that world frame has frame 1 as child
	EXPECT_EQ("Frame1",frame1_GN->getName());
	EXPECT_EQ(1,world->nrOfChildren());
	EXPECT_TRUE(world->hasChild("Frame1"));
	EXPECT_TRUE(world->hasChild(frame1_GN));
	EXPECT_TRUE(frame1_GN->hasParent(world));
	EXPECT_EQ(0,frame1_GN->nrOfChildren());

	MovableFrame* frame2 = new MovableFrame("Frame2");
	GroupNode::Ptr frame2_GN = WSscene->getNode(frame2);
	// Check that frame2 dosen't exist in the workcell scene
	EXPECT_TRUE(frame2_GN.isNull());

	ws->addFrame(frame2,frame1);
	frame2_GN = WSscene->getNode(frame2);

	// Check that frame2 has been added as a child of frame one and not WORLD
	EXPECT_EQ("Frame2",frame2_GN->getName());
	EXPECT_EQ(1,frame1_GN->nrOfChildren());
	EXPECT_TRUE(frame1_GN->hasChild(frame2_GN));
	EXPECT_TRUE(frame1_GN->hasChild("Frame2"));
	EXPECT_EQ(0,frame2_GN->nrOfChildren());
	EXPECT_EQ(1,world->nrOfChildren());
	EXPECT_TRUE(frame2_GN->hasParent(frame1_GN));
	ws->remove(frame2);

	// Check that frame2 has been removed from the WorkCellScene
	EXPECT_TRUE(ws->findFrame("Frame2") == NULL);
	EXPECT_EQ(0,frame1_GN->nrOfChildren());
	EXPECT_FALSE(frame1_GN->hasChild("Frame2"));
	EXPECT_FALSE(frame1_GN->hasChild(frame2_GN));

	// Check visible
	EXPECT_FALSE(WSscene->isVisible(frame1));
	WSscene->setVisible(true,frame1);
	EXPECT_TRUE(WSscene->isVisible(frame1));

	// Check frameAxisVisible
	EXPECT_FALSE(WSscene->isFrameAxisVisible(frame1));
	WSscene->setFrameAxisVisible(true,frame1);
	EXPECT_TRUE(WSscene->isFrameAxisVisible(frame1));

	// Check drawables
	EXPECT_TRUE(WSscene->removeDrawables(frame1));
	MovableFrame* frame3 = new MovableFrame("Frame3");
	ws->addFrame(frame3,frame1);
	EXPECT_EQ(0,WSscene->getDrawables().size());
	const DummyDrawable::Ptr draw1 = ownedPtr(new DummyDrawable("draw1"));
	WSscene->addDrawable(draw1,frame3);
	EXPECT_EQ(1,WSscene->getDrawables().size());
	EXPECT_TRUE(WSscene->removeDrawables(frame3));
	EXPECT_EQ(0,WSscene->getDrawables().size());

	WSscene->setWorkCell(NULL);
}
