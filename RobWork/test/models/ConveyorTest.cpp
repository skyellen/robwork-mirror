/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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

#include "../TestSuiteConfig.hpp"

#include <rw/math/Q.hpp>
#include <rw/interpolator/StraightInterpolator.hpp>
#include <rw/interpolator/CubicSplineInterpolator.hpp>
#include <rw/interpolator/Trajectory.hpp>
#include <rw/models/ConveyorBelt.hpp>
#include <rw/models/Conveyor.hpp>
#include <rw/kinematics/FixedFrame.hpp>

using namespace rw::math;
using namespace rw::interpolator;
using namespace rw::models;
using namespace rw::kinematics;


Q getPosition(Trajectory& traj1, Trajectory& traj2, double t) {
	double totallength = traj1.getLength() + traj2.getLength();

	int n = (int)floor(t/totallength);

	t -= n*totallength;

	if (t <= traj1.getLength())
		return traj1.getX(t);
	else
		return traj2.getX(t-traj1.getLength());
}

BOOST_AUTO_TEST_CASE( ConveyorTest ) {
	FixedFrame* world = new FixedFrame(NULL, "world", Transform3D<>());

	Q qstart(Q::ZeroBase(7));
	Q via1(Q::ZeroBase(7));
	Q via2(Q::ZeroBase(7));
	Q via3(Q::ZeroBase(7));
	Q via4(Q::ZeroBase(7));
	qstart(0) = qstart(1) = 0;
	via1(0) = 3; via1(1) = 0;
	via2(0) = 4; via2(1) = -0.5;
	via3(0) = 4.5; via3(1) = -1.5;
	via4(0) = 4.5; via4(1) = -3;

	//Construct the first straight line segment
	StraightInterpolator straight1(qstart, via1, 1);

	//Construct the cubic spline based segment
	std::vector<std::pair<Q, double> > vias;
	vias.push_back(std::pair<Q, double>(via2, 0.5));
	vias.push_back(std::pair<Q, double>(via3, 1));
	CubicSplineInterpolator spline(via1, vias);


	StraightInterpolator straight2(via3, via4, 1);
	StraightInterpolator straight3(via4, qstart, 2);

	Trajectory trajectory1;
	trajectory1.add(&straight1);
	trajectory1.add(&spline);


	Trajectory trajectory2;
	trajectory2.add(&straight2);
	trajectory2.add(&straight3);


	FixedFrame* parent1 = new FixedFrame(world, "parent1", Transform3D<>());
	FixedFrame* parent2 = new FixedFrame(world, "parent2", Transform3D<>());


	//Construct a couple of segments
	ConveyorBelt conveyorbelt1(trajectory1, parent1, NULL, NULL);
	ConveyorBelt conveyorbelt2(trajectory2, parent2, &conveyorbelt1, &conveyorbelt1);

	//Connect these to seach other
	conveyorbelt1.setPreviousSegment(&conveyorbelt2);
	conveyorbelt1.setNextSegment(&conveyorbelt2);


	//Construct the Conveyor
	std::vector<ConveyorSegment*> segments;
	segments.push_back(&conveyorbelt1);
	segments.push_back(&conveyorbelt2);
	FixedJoint* joint = new FixedJoint(world, "Conveyor", Transform3D<>());
	Conveyor conveyor("conveyor", joint, segments);

	//Construct some items
	ConveyorItem* object1 = new ConveyorItem("object1");
	ConveyorItem* object2 = new ConveyorItem("object2");
	ConveyorItem* object3 = new ConveyorItem("object3");

	//Construct the tree and state
	boost::shared_ptr<Tree> tree = boost::shared_ptr<Tree>(new Tree());
	tree->addFrame(world);
	tree->addFrame(parent1);
	tree->addFrame(parent2);
	tree->addFrame(joint);
	tree->addFrame(object1);
	tree->addFrame(object2);
	tree->addFrame(object3);
	tree->setDafParent(*object1, *world);
	tree->setDafParent(*object2, *world);
	tree->setDafParent(*object3, *world);

	State state(tree);


	const double OFF1 = 0.0;
	const double OFF2 = 0.5;
	const double OFF3 = 1.0;

	conveyor.addItem(object1, OFF1, state);
	conveyor.addItem(object2, OFF2, state);
	conveyor.addItem(object3, OFF3, state);

	const int CNT = 9;
	double positions[] = {0.0, 0.5, 1.4, 3.0, 4.0, 5.5, 3.5, -1, -3};

	Q q(1);
	Q pos1, pos2, pos3;
	for (int i = 0; i<CNT; i++) {
		q(0) = positions[i];
		conveyor.setQ(q, state);
		pos1 = getPosition(trajectory1, trajectory2, q(0)+OFF1);
		pos2 = getPosition(trajectory1, trajectory2, q(0)+OFF2);
		pos3 = getPosition(trajectory1, trajectory2, q(0)+OFF3);

		BOOST_CHECK(object1->getTransform(state).P()(0) == pos1(0) && object1->getTransform(state).P()(1) == pos1(1));
		BOOST_CHECK(object2->getTransform(state).P()(0) == pos2(0) && object2->getTransform(state).P()(1) == pos2(1));
		BOOST_CHECK(object3->getTransform(state).P()(0) == pos3(0) && object3->getTransform(state).P()(1) == pos3(1));

	}




	for (q(0) = 0; q(0)<11; q(0) += 0.5) {
		conveyor.setQ(q, state);
		pos1 = getPosition(trajectory1, trajectory2, q(0)+OFF1);
		pos2 = getPosition(trajectory1, trajectory2, q(0)+OFF2);
		pos3 = getPosition(trajectory1, trajectory2, q(0)+OFF3);

		BOOST_CHECK(object1->getTransform(state).P()(0) == pos1(0) && object1->getTransform(state).P()(1) == pos1(1));
		BOOST_CHECK(object2->getTransform(state).P()(0) == pos2(0) && object2->getTransform(state).P()(1) == pos2(1));
		BOOST_CHECK(object3->getTransform(state).P()(0) == pos3(0) && object3->getTransform(state).P()(1) == pos3(1));


	}


	for (q(0) = 10; q(0)>0; q(0) -= 0.5) {
		conveyor.setQ(q, state);
		pos1 = getPosition(trajectory1, trajectory2, q(0)+OFF1);
		pos2 = getPosition(trajectory1, trajectory2, q(0)+OFF2);
		pos3 = getPosition(trajectory1, trajectory2, q(0)+OFF3);

		BOOST_CHECK(object1->getTransform(state).P()(0) == pos1(0) && object1->getTransform(state).P()(1) == pos1(1));
		BOOST_CHECK(object2->getTransform(state).P()(0) == pos2(0) && object2->getTransform(state).P()(1) == pos2(1));
		BOOST_CHECK(object3->getTransform(state).P()(0) == pos3(0) && object3->getTransform(state).P()(1) == pos3(1));
	}






	//TODO Test removing an object from the conveyor

	//TODO Test setting up different end frames of the conveyor.


}
