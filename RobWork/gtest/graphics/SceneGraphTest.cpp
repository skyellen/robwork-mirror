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

#include <rw/graphics/SceneGraph.hpp>

using rw::common::ownedPtr;
using namespace rw::graphics;
using namespace rw::math;

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
    DrawableGeometryNode::Ptr makeDrawableFrameAxis(const std::string& name, double size, int dmask=DrawableNode::Physical) { return NULL; }
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

TEST(SceneGraph, Test) {
	// Create test scene graph
	const SceneGraph::Ptr scene = ownedPtr(new DummySceneGraph());
	const GroupNode::Ptr root = scene->getRoot();
	ASSERT_FALSE(root.isNull());
	EXPECT_EQ(0,root->nrOfChildren());

	const GroupNode::Ptr group1 = scene->makeGroupNode("Group1");
	EXPECT_EQ("Group1", group1->getName());

	EXPECT_FALSE(root->hasChild("Group1"));
	EXPECT_FALSE(root->hasChild(group1));
	EXPECT_FALSE(group1->hasParent(root));
	scene->addChild(group1,root);
	EXPECT_EQ(1,root->nrOfChildren());
	EXPECT_TRUE(root->hasChild("Group1"));
	EXPECT_TRUE(root->hasChild(group1));
	EXPECT_TRUE(group1->hasParent(root));
    scene->removeChild("Group1", root);
	EXPECT_FALSE(root->hasChild("Group1"));
	EXPECT_FALSE(root->hasChild(group1));
	EXPECT_FALSE(group1->hasParent(root));

/*
    // Functions to test:
    virtual void addChild(rw::common::Ptr<SceneNode> child, GroupNode::Ptr parent);
    virtual bool removeChild(const std::string& name, GroupNode::Ptr node);

    virtual std::vector<DrawableNode::Ptr> getDrawables();
    virtual std::vector<DrawableNode::Ptr> getDrawables(rw::common::Ptr<SceneNode> node);
    virtual std::vector<DrawableNode::Ptr> getDrawablesRec(rw::common::Ptr<SceneNode> node);

    virtual DrawableNode::Ptr findDrawable(const std::string& name);
    virtual DrawableNode::Ptr findDrawable(const std::string& name, rw::common::Ptr<SceneNode> node);
    virtual std::vector<DrawableNode::Ptr> findDrawables(const std::string& name);

    virtual bool removeDrawables(GroupNode::Ptr node);
    virtual bool removeDrawables(const std::string& name);
    virtual bool removeDrawable(DrawableNode::Ptr drawable);
    virtual bool removeDrawable(DrawableNode::Ptr drawable, rw::common::Ptr<SceneNode> node);
    virtual bool removeDrawable(const std::string& name);
    */
}
