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
    void draw(const DrawableNode::RenderInfo&) const {}
    void setHighlighted(bool) {}
    bool isHighlighted() const { return false; }
    void setDrawType(DrawType) {}
    void setTransparency(float) {}
    float getTransparency() { return 0; }
    void setScale(float) {}
    float getScale() const { return 1; }
    void setVisible(bool) {}
    bool isVisible() { return true; }
    const Transform3D<>& getTransform() const { return Transform3D<>::identity(); }
    void setTransform(const rw::math::Transform3D<>&) {}
    void setMask(unsigned int) {}
    unsigned int getMask() const { return DrawableNode::Physical; }
};

class DummyDrawableGeometry: public DrawableGeometryNode {
public:
	DummyDrawableGeometry(const std::string& name): DrawableGeometryNode(name) {}
	virtual ~DummyDrawableGeometry() {}

	// DrawableNode
    void draw(const DrawableNode::RenderInfo&) const {}
    void setHighlighted(bool) {}
    bool isHighlighted() const { return false; }
    void setDrawType(DrawType) {}
    void setTransparency(float) {}
    float getTransparency() { return 0; }
    void setScale(float) {}
    float getScale() const { return 1; }
    void setVisible(bool) {}
    bool isVisible() { return true; }
    const Transform3D<>& getTransform() const { return Transform3D<>::identity(); }
    void setTransform(const rw::math::Transform3D<>&) {}
    void setMask(unsigned int) {}
    unsigned int getMask() const { return DrawableNode::Physical; }

    // DrawableGeometryNode
    void setColor(double, double, double, double) {}
    void setColor(const Vector3D<>&) {}
    void setAlpha(double) {}
    Vector3D<> getColor() { return Vector3D<>::x(); }
    double getAlpha() { return 1; }
    void addLines(const std::vector<rw::geometry::Line >&) {}
    void addLine(const Vector3D<>&, const Vector3D<>&) {}
    void addGeometry(rw::common::Ptr<class rw::geometry::Geometry>) {}
    void addFrameAxis(double) {}
};

class DummySceneGraph: public SceneGraph {
public:
	DummySceneGraph(): SceneGraph() {}
	virtual ~DummySceneGraph() {}
    void draw(RenderInfo&) {}
    DrawableNode::Ptr pickDrawable(RenderInfo&, int, int) { return NULL; }
    Vector3D<> unproject(SceneCamera::Ptr, int, int) { return Vector3D<>::zero(); }
    void update() {}
    void clear() {}
    DrawableGeometryNode::Ptr makeDrawableFrameAxis(const std::string&, double, int) { return NULL; }
    DrawableGeometryNode::Ptr makeDrawable(const std::string&, rw::common::Ptr<class rw::geometry::Geometry>, int) { return NULL; }
    DrawableGeometryNode::Ptr makeDrawable(const std::string&, const std::vector<class rw::geometry::Line >&, int) { return NULL; }
    DrawableNode::Ptr makeDrawable(const std::string&, const class rw::sensor::Image&, int) { return NULL; }
    DrawableNode::Ptr makeDrawable(const std::string&, const rw::geometry::PointCloud&, int) { return NULL; }
    DrawableNode::Ptr makeDrawable(const std::string&, rw::common::Ptr<class Model3D>, int) { return NULL; }
    DrawableNode::Ptr makeDrawable(const std::string&, rw::common::Ptr<class Render>, int) { return NULL; }
    DrawableNode::Ptr makeDrawable(const std::string&, int) { return NULL; }
    SceneCamera::Ptr makeCamera(const std::string&) { return NULL; }
};
}

TEST(SceneGraph, Test) {
	// Create test scene graph
/*
                        root
                      ___|______________________________________________________
                      |             |                         |        |       |
                     g1             g2                        g3      l5      l6
                    __|__         __|_____________       _____|______
                    |   |        |                |     |   |   |   |
                   g4   l1      g5               g6    l1  l2  l8  l11
                ____|____    ____|___________     |
                |   |   |    |       |       |   l11
               l2  l3  l4   g7      g8       g9
                           __|__   __|__   __|__
                           |   |   |   |   |   |
                          l5  l6  l7  l8  l9  l10

*/
	const SceneGraph::Ptr scene = ownedPtr(new DummySceneGraph());
	const GroupNode::Ptr root = scene->getRoot();

	const GroupNode::Ptr group1 = scene->makeGroupNode("Group1");
    const GroupNode::Ptr group2 = scene->makeGroupNode("Group2");
    const GroupNode::Ptr group3 = scene->makeGroupNode("Group3");
    const GroupNode::Ptr group4 = scene->makeGroupNode("Group4");
    const GroupNode::Ptr group5 = scene->makeGroupNode("Group5");
    const GroupNode::Ptr group6 = scene->makeGroupNode("Group6");
    const GroupNode::Ptr group7 = scene->makeGroupNode("Group7");
    const GroupNode::Ptr group8 = scene->makeGroupNode("Group8");
    const GroupNode::Ptr group9 = scene->makeGroupNode("Group9");

    scene->addChild(group1,root);
    scene->addChild(group2,root);
    scene->addChild(group3,root);
    scene->addChild(group4,group1);
    scene->addChild(group5,group2);
    scene->addChild(group6,group2);
    scene->addChild(group7,group5);
    scene->addChild(group8,group5);
    scene->addChild(group9,group5);

    const DrawableGeometryNode::Ptr leaf1 = ownedPtr(new DummyDrawableGeometry("Leaf1"));
    const DrawableNode::Ptr leaf2 = ownedPtr(new DummyDrawable("Leaf2"));
    const DrawableGeometryNode::Ptr leaf3 = ownedPtr(new DummyDrawableGeometry("Leaf3"));
    const DrawableNode::Ptr leaf4 = ownedPtr(new DummyDrawable("Leaf4"));
    const DrawableGeometryNode::Ptr leaf5 = ownedPtr(new DummyDrawableGeometry("Leaf5"));
    const DrawableNode::Ptr leaf6 = ownedPtr(new DummyDrawable("Leaf6"));
    const DrawableGeometryNode::Ptr leaf7 = ownedPtr(new DummyDrawableGeometry("Leaf7"));
    const DrawableNode::Ptr leaf8 = ownedPtr(new DummyDrawable("Leaf8"));
    const DrawableGeometryNode::Ptr leaf9 = ownedPtr(new DummyDrawableGeometry("Leaf9"));
    const DrawableNode::Ptr leaf10 = ownedPtr(new DummyDrawable("Leaf10"));
    const DrawableGeometryNode::Ptr leaf11 = ownedPtr(new DummyDrawableGeometry("Leaf11"));

    scene->addChild(leaf1,group1);
    scene->addChild(leaf2,group4);
    scene->addChild(leaf3,group4);
    scene->addChild(leaf4,group4);
    scene->addChild(leaf5,group7);
    scene->addChild(leaf6,group7);
    scene->addChild(leaf7,group8);
    scene->addChild(leaf8,group8);
    scene->addChild(leaf9,group9);
    scene->addChild(leaf10,group9);
    scene->addChild(leaf11,group6);
    scene->addChild(leaf1,group3);
    scene->addChild(leaf2,group3);
    scene->addChild(leaf8,group3);
    scene->addChild(leaf11,group3);
    scene->addChild(leaf6,root);
    scene->addChild(leaf5,root);


    //TESTING if tree is built correctly.
    //This should showcase addChild(rw::common::Ptr<SceneNode> child, GroupNode::Ptr parent);

    ASSERT_FALSE(root.isNull());    //Assert if root exists
    EXPECT_EQ(5,root->nrOfChildren()); //root has 5 children
    EXPECT_EQ(2,group1->nrOfChildren()); //check if nodes has right amount of children
    EXPECT_EQ(2,group2->nrOfChildren());
    EXPECT_EQ(4,group3->nrOfChildren());
    EXPECT_EQ(3,group4->nrOfChildren());
    EXPECT_EQ(3,group5->nrOfChildren());
    EXPECT_EQ(1,group6->nrOfChildren());
    EXPECT_EQ(2,group7->nrOfChildren());
    EXPECT_EQ(2,group8->nrOfChildren());
    EXPECT_EQ(2,group9->nrOfChildren());

    EXPECT_TRUE(group1->hasChild("Leaf1")); // check if leafs match
    EXPECT_TRUE(group1->hasChild(leaf1));
    EXPECT_TRUE(group4->hasChild("Leaf2"));
    EXPECT_TRUE(group4->hasChild(leaf2));
    EXPECT_TRUE(group4->hasChild("Leaf3"));
    EXPECT_TRUE(group4->hasChild(leaf3));
    EXPECT_TRUE(group4->hasChild("Leaf4"));
    EXPECT_TRUE(group4->hasChild(leaf4));
    EXPECT_TRUE(group7->hasChild("Leaf5"));
    EXPECT_TRUE(group7->hasChild(leaf5));
    EXPECT_TRUE(group7->hasChild("Leaf6"));
    EXPECT_TRUE(group7->hasChild(leaf6));
    EXPECT_TRUE(group8->hasChild("Leaf7"));
    EXPECT_TRUE(group8->hasChild(leaf7));
    EXPECT_TRUE(group8->hasChild("Leaf8"));
    EXPECT_TRUE(group8->hasChild(leaf8));
    EXPECT_TRUE(group9->hasChild("Leaf9"));
    EXPECT_TRUE(group9->hasChild(leaf9));
    EXPECT_TRUE(group9->hasChild("Leaf10"));
    EXPECT_TRUE(group9->hasChild(leaf10));
    EXPECT_TRUE(group6->hasChild("Leaf11"));
    EXPECT_TRUE(group6->hasChild(leaf11));
    EXPECT_TRUE(group3->hasChild("Leaf1"));
    EXPECT_TRUE(group3->hasChild(leaf1));
    EXPECT_TRUE(group3->hasChild("Leaf2"));
    EXPECT_TRUE(group3->hasChild(leaf2));
    EXPECT_TRUE(group3->hasChild("Leaf8"));
    EXPECT_TRUE(group3->hasChild(leaf8));
    EXPECT_TRUE(group3->hasChild("Leaf11"));
    EXPECT_TRUE(group3->hasChild(leaf11));
    EXPECT_TRUE(root->hasChild("Leaf5"));
    EXPECT_TRUE(root->hasChild(leaf5));
    EXPECT_TRUE(root->hasChild("Leaf6"));
    EXPECT_TRUE(root->hasChild(leaf6));

    EXPECT_TRUE(root->hasChild(group1)); //check is children nodes are matching
    EXPECT_TRUE(root->hasChild(group2));
    EXPECT_TRUE(root->hasChild(group3));
    EXPECT_TRUE(group1->hasChild(group4));
    EXPECT_TRUE(group2->hasChild(group5));
    EXPECT_TRUE(group2->hasChild(group6));
    EXPECT_TRUE(group5->hasChild(group7));
    EXPECT_TRUE(group5->hasChild(group8));
    EXPECT_TRUE(group5->hasChild(group9));

    EXPECT_TRUE(leaf1->hasParent(group1)); // check if parents are correct, leafs
    EXPECT_TRUE(leaf1->hasParent(group3));
    EXPECT_TRUE(leaf2->hasParent(group4));
    EXPECT_TRUE(leaf2->hasParent(group3));
    EXPECT_TRUE(leaf3->hasParent(group4));
    EXPECT_TRUE(leaf4->hasParent(group4));
    EXPECT_TRUE(leaf5->hasParent(group7));
    EXPECT_TRUE(leaf5->hasParent(root));
    EXPECT_TRUE(leaf6->hasParent(group7));
    EXPECT_TRUE(leaf6->hasParent(root));
    EXPECT_TRUE(leaf7->hasParent(group8));
    EXPECT_TRUE(leaf8->hasParent(group8));
    EXPECT_TRUE(leaf8->hasParent(group3));
    EXPECT_TRUE(leaf9->hasParent(group9));
    EXPECT_TRUE(leaf10->hasParent(group9));
    EXPECT_TRUE(leaf11->hasParent(group6));
    EXPECT_TRUE(leaf11->hasParent(group3));

    EXPECT_TRUE(group1->hasParent(root)); //check if parents are correct, nodes
    EXPECT_TRUE(group2->hasParent(root));
    EXPECT_TRUE(group3->hasParent(root));
    EXPECT_TRUE(group4->hasParent(group1));
    EXPECT_TRUE(group5->hasParent(group2));
    EXPECT_TRUE(group6->hasParent(group2));
    EXPECT_TRUE(group7->hasParent(group5));
    EXPECT_TRUE(group8->hasParent(group5));
    EXPECT_TRUE(group9->hasParent(group5));

    // TESTING virtual std::vector<DrawableNode::Ptr> getDrawables();
    // test for uniqueness ?
    std::vector<DrawableNode::Ptr> drawables = scene->getDrawables();
    EXPECT_EQ(11, drawables.size()); //There should be 11 leafs/drawables


    // TESTING virtual std::vector<DrawableNode::Ptr> getDrawables(rw::common::Ptr<SceneNode> node);
    // test for uniqueness ?
    std::vector<DrawableNode::Ptr> drawables0 = scene->getDrawables(root);
    std::vector<DrawableNode::Ptr> drawables1 = scene->getDrawables(group1);
    std::vector<DrawableNode::Ptr> drawables2 = scene->getDrawables(group2);
    std::vector<DrawableNode::Ptr> drawables3 = scene->getDrawables(group3);
    std::vector<DrawableNode::Ptr> drawables4 = scene->getDrawables(group4);
    std::vector<DrawableNode::Ptr> drawables5 = scene->getDrawables(group5);
    std::vector<DrawableNode::Ptr> drawables6 = scene->getDrawables(group6);
    std::vector<DrawableNode::Ptr> drawables7 = scene->getDrawables(group7);
    std::vector<DrawableNode::Ptr> drawables8 = scene->getDrawables(group8);
    std::vector<DrawableNode::Ptr> drawables9 = scene->getDrawables(group9);
    EXPECT_EQ(2, drawables0.size()); //There should be 2 leafs/drawables
    EXPECT_EQ(1, drawables1.size()); //There should be 1 leafs/drawables
    EXPECT_EQ(0, drawables2.size()); //There should be 0 leafs/drawables
    EXPECT_EQ(4, drawables3.size()); //There should be 4 leafs/drawables
    EXPECT_EQ(3, drawables4.size()); //There should be 3 leafs/drawables
    EXPECT_EQ(0, drawables5.size()); //There should be 0 leafs/drawables
    EXPECT_EQ(1, drawables6.size()); //There should be 1 leafs/drawables
    EXPECT_EQ(2, drawables7.size()); //There should be 2 leafs/drawables
    EXPECT_EQ(2, drawables8.size()); //There should be 2 leafs/drawables
    EXPECT_EQ(2, drawables9.size()); //There should be 2 leafs/drawables


    // TESTING virtual DrawableNode::Ptr findDrawable(const std::string& name);
    DrawableNode::Ptr l1 = scene->findDrawable("Leaf1");
    DrawableNode::Ptr l2 = scene->findDrawable("Leaf2");
    DrawableNode::Ptr l3 = scene->findDrawable("Leaf3");
    DrawableNode::Ptr l4 = scene->findDrawable("Leaf4");
    DrawableNode::Ptr l5 = scene->findDrawable("Leaf5");
    DrawableNode::Ptr l6 = scene->findDrawable("Leaf6");
    DrawableNode::Ptr l7 = scene->findDrawable("Leaf7");
    DrawableNode::Ptr l8 = scene->findDrawable("Leaf8");
    DrawableNode::Ptr l9 = scene->findDrawable("Leaf9");
    DrawableNode::Ptr l10 = scene->findDrawable("Leaf10");
    DrawableNode::Ptr l11 = scene->findDrawable("Leaf11");

    EXPECT_EQ(l1->getName(),"Leaf1");
    EXPECT_EQ(l2->getName(),"Leaf2");
    EXPECT_EQ(l3->getName(),"Leaf3");
    EXPECT_EQ(l4->getName(),"Leaf4");
    EXPECT_EQ(l5->getName(),"Leaf5");
    EXPECT_EQ(l6->getName(),"Leaf6");
    EXPECT_EQ(l7->getName(),"Leaf7");
    EXPECT_EQ(l8->getName(),"Leaf8");
    EXPECT_EQ(l9->getName(),"Leaf9");
    EXPECT_EQ(l10->getName(),"Leaf10");
    EXPECT_EQ(l11->getName(),"Leaf11");


    // TESTING virtual DrawableNode::Ptr findDrawable(const std::string& name, rw::common::Ptr<SceneNode> node)
    DrawableNode::Ptr rt = scene->findDrawable("Leaf5",root);
    l1 = scene->findDrawable("Leaf1",group1);
    l2 = scene->findDrawable("Leaf2",group4);
    l3 = scene->findDrawable("Leaf3",group4);
    l4 = scene->findDrawable("Leaf4",group4);
    l5 = scene->findDrawable("Leaf5",group7);
    l6 = scene->findDrawable("Leaf6",group7);
    l7 = scene->findDrawable("Leaf7",group8);
    l8 = scene->findDrawable("Leaf8",group8);
    l9 = scene->findDrawable("Leaf9",group9);
    l10 = scene->findDrawable("Leaf10",group9);
    l11 = scene->findDrawable("Leaf11",group6);

    EXPECT_EQ(rt->getName(),"Leaf5");
    EXPECT_EQ(l1->getName(),"Leaf1");
    EXPECT_EQ(l2->getName(),"Leaf2");
    EXPECT_EQ(l3->getName(),"Leaf3");
    EXPECT_EQ(l4->getName(),"Leaf4");
    EXPECT_EQ(l5->getName(),"Leaf5");
    EXPECT_EQ(l6->getName(),"Leaf6");
    EXPECT_EQ(l7->getName(),"Leaf7");
    EXPECT_EQ(l8->getName(),"Leaf8");
    EXPECT_EQ(l9->getName(),"Leaf9");
    EXPECT_EQ(l10->getName(),"Leaf10");
    EXPECT_EQ(l11->getName(),"Leaf11");



    // TESTING virtual std::vector<DrawableNode::Ptr> findDrawables(const std::string& name);
    drawables = scene->findDrawables("Leaf1");
    drawables0 = scene->findDrawables("Leaf2");
    drawables1 = scene->findDrawables("Leaf3");
    drawables2 = scene->findDrawables("Leaf4");
    drawables3 = scene->findDrawables("Leaf5");
    drawables4 = scene->findDrawables("Leaf6");
    drawables5 = scene->findDrawables("Leaf7");
    drawables6 = scene->findDrawables("Leaf8");
    drawables7 = scene->findDrawables("Leaf9");
    drawables8 = scene->findDrawables("Leaf10");
    drawables9 = scene->findDrawables("Leaf11");

    EXPECT_EQ(2, drawables.size());
    EXPECT_EQ(drawables[0]->getName(),"Leaf1");
    EXPECT_EQ(drawables[1]->getName(),"Leaf1");
    EXPECT_EQ(2, drawables0.size());
    EXPECT_EQ(drawables0[0]->getName(),"Leaf2");
    EXPECT_EQ(drawables0[1]->getName(),"Leaf2");
    EXPECT_EQ(1, drawables1.size());
    EXPECT_EQ(drawables1[0]->getName(),"Leaf3");
    EXPECT_EQ(1, drawables2.size());
    EXPECT_EQ(drawables2[0]->getName(),"Leaf4");
    EXPECT_EQ(2, drawables3.size());
    EXPECT_EQ(drawables3[0]->getName(),"Leaf5");
    EXPECT_EQ(drawables3[1]->getName(),"Leaf5");
    EXPECT_EQ(2, drawables4.size());
    EXPECT_EQ(drawables4[0]->getName(),"Leaf6");
    EXPECT_EQ(drawables4[1]->getName(),"Leaf6");
    EXPECT_EQ(1, drawables5.size());
    EXPECT_EQ(drawables5[0]->getName(),"Leaf7");
    EXPECT_EQ(2, drawables6.size());
    EXPECT_EQ(drawables6[0]->getName(),"Leaf8");
    EXPECT_EQ(drawables6[1]->getName(),"Leaf8");
    EXPECT_EQ(1, drawables7.size());
    EXPECT_EQ(drawables7[0]->getName(),"Leaf9");
    EXPECT_EQ(1, drawables8.size());
    EXPECT_EQ(drawables8[0]->getName(),"Leaf10");
    EXPECT_EQ(2, drawables9.size());
    EXPECT_EQ(drawables9[0]->getName(),"Leaf11");
    EXPECT_EQ(drawables9[1]->getName(),"Leaf11");


    // TESTING virtual bool removeDrawables(GroupNode::Ptr node);
    scene->removeDrawables(group4);
    EXPECT_FALSE(group4->hasChild(leaf2));
    EXPECT_FALSE(group4->hasChild(leaf3));
    EXPECT_FALSE(group4->hasChild(leaf4));


    // TESTING virtual bool removeDrawables(const std::string& name);
    scene->removeDrawables("Leaf5");
    EXPECT_FALSE(group7->hasChild(leaf5));
    EXPECT_FALSE(root->hasChild(leaf5));


    // TESTING virtual bool removeDrawable(DrawableNode::Ptr drawable);
    scene->removeDrawable(leaf6);
    EXPECT_FALSE(group7->hasChild(leaf6));
    EXPECT_FALSE(root->hasChild(leaf6));

    // TESTING virtual bool removeDrawable(DrawableNode::Ptr drawable, rw::common::Ptr<SceneNode> node);
    scene->removeDrawable(leaf1,group3);
    EXPECT_TRUE(group1->hasChild(leaf1));
    EXPECT_FALSE(group3->hasChild(leaf1));


    // TESTING virtual bool removeDrawable(const std::string& name);
    scene->removeDrawable("Leaf11");
    scene->removeDrawable("Group2"); // IS THIS DESIREABLE?
    EXPECT_FALSE(group1->hasChild("Group2"));
    EXPECT_FALSE(root->hasChild("Leaf11"));
    EXPECT_TRUE(group3->hasChild("Leaf11"));

    // TESTING virtual bool removeChild(const std::string& name, GroupNode::Ptr node);
    //scene->removeChild("Group3",root);
    scene->removeChild("Group1",root);
    //EXPECT_FALSE(root->hasChild("Group3"));
    EXPECT_FALSE(root->hasChild("Group1"));

    EXPECT_TRUE(group1->hasChild("Leaf1"));
    scene->removeChild("Leaf11",group3);
    EXPECT_FALSE(group3->hasChild("Leaf11"));
}
