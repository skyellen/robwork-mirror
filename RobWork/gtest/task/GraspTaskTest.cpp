/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "GraspTask.xml.hpp"

#include <rwlibs/task/GraspTask.hpp>

#include <RobWorkConfig.hpp>

#include <rwlibs/task/loader/DOMTaskLoader.hpp>
#include <rwlibs/task/loader/DOMTaskSaver.hpp>
#ifdef RW_HAVE_XERCES
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#endif

using rw::common::ownedPtr;
using rw::common::Ptr;
using namespace rw::math;
using namespace rwlibs::task;

namespace {
static const Transform3D<> Tref1 = Transform3D<>(Vector3D<>(0.128173, -0.297606, 0.946046), Rotation3D<>(-0.74816, -0.220389, 0.625848, -0.445398, 0.865945, -0.227507, -0.49181, -0.448963, -0.746026));
static const Transform3D<> Tref2 = Transform3D<>(Vector3D<>(-0.599348, 0.144885, 0.787268), Rotation3D<>(0.021466, 0.478732, -0.877699, 0.823371, 0.489505, 0.287133, 0.567098, -0.728835, -0.383666));
static const Transform3D<> Tref3 = Transform3D<>(Vector3D<>(-0.0373863, 0.206556, -0.97772), Rotation3D<>(0.823355, 0.407011, 0.395511, -0.410789, 0.908257, -0.0795062, -0.391585, -0.0970095, 0.915014));
static const Transform3D<> Tref4 = Transform3D<>(Vector3D<>(-0.841704, 0.528854, -0.108845), Rotation3D<>(0.449831, 0.0716641, 0.890234, -0.269712, 0.961137, 0.0589122, -0.851415, -0.266607, 0.451678));
static const Transform3D<> Tref5 = Transform3D<>(Vector3D<>(0.25201, -0.0695093, -0.965225), Rotation3D<>(-0.00270293, -0.89361, 0.448835, 0.67105, -0.334393, -0.66172, 0.741407, 0.299402, 0.600561));
static const Transform3D<> Tref6 = Transform3D<>(Vector3D<>(0.154929, 0.987479, 0.0296881), Rotation3D<>(-0.455489, -0.884303, -0.102657, -0.663233, 0.413999, -0.623479, 0.593844, -0.215902, -0.775071));
static const Transform3D<> Tref7 = Transform3D<>(Vector3D<>(0.2288791, -0.8427269, 0.4872634), Rotation3D<>(0.8744951, -0.4795291, 0.07287096, 0.08946266, 0.3071274, 0.947454, -0.4767124, -0.8220247, 0.3114814));

static const Transform3D<> Tref11 = Transform3D<>(Vector3D<>(-0.2459544, -0.8678307, 0.4317132), Rotation3D<>(0.2549632, -0.8745079, 0.4125889, -0.5708913, 0.2082455, 0.7941769, -0.7804338, -0.4380293, -0.446154));
static const Transform3D<> Tref12 = Transform3D<>(Vector3D<>(0.3580329, 0.5530171, 0.7523194), Rotation3D<>(-0.6115657, 0.7761471, 0.1535677, -0.7856076, -0.6187233, -0.001500034, 0.09385167, -0.1215613, 0.988137));
static const Transform3D<> Tref13 = Transform3D<>(Vector3D<>(-0.202799, -0.5613984, -0.8023119), Rotation3D<>(0.8131517, -0.4044635, -0.4185614, 0.3927172, 0.9120122, -0.1183505, 0.4296015, -0.06813937, 0.9004441));

static const Q Qref1 = Q(4, 0.1081486, -0.2511118, 0.7982471, -0.5367013);
static const Q Qref2 = Q(4, 0.6743709, 0.6914217, 0.2217055, -0.1341887);
static const Q Qref3 = Q(4, -0.3795243, -0.4163515, 0.6758378, 0.4752431);
static const Q Qref4 = Q(4, -0.03584265, 0.1980277, -0.9373505, -0.2843841);
static const Q Qref5 = Q(4, 0.7606183, -0.2080465, 0.6000562, -0.1345698);
static const Q Qref6 = Q(4, 0.2719389, 0.4146291, 0.853626, -0.1595451);
static const Q Qref7 = Q(4, 0.2510191, -0.06923598, -0.9614297, 0.08859286);

GraspTask::Ptr makeReferenceTask() {
	const GraspTask::Ptr task = ownedPtr(new GraspTask);
	task->setGripperID("GripperId");
	task->setTCPID("TCPId");
	task->setGraspControllerID("GraspControllerId");

	GraspSubTask stask1;
	stask1.refframe = "refframe1";
	stask1.objectID = "objectId1";
	stask1.offset = Tref1;
	stask1.approach = Tref2;
	stask1.retract = Tref3;
	stask1.openQ = Qref1;
	stask1.closeQ = Qref2;
	stask1.tauMax = Qref3;
	stask1.taskID = "taskId1";

	const GraspResult::Ptr res1 = ownedPtr(new GraspResult());
	res1->testStatus = GraspResult::Success;
	res1->liftresult = 0.123;
	res1->gripperConfigurationGrasp = Qref4;
	res1->gripperConfigurationLift = Qref5;
	res1->qualityBeforeLifting = Qref6;
	res1->qualityAfterLifting = Qref7;
	res1->objectTtcpTarget = Tref4;
	res1->objectTtcpApproach = Tref5;
	res1->objectTtcpGrasp = Tref6;
	res1->objectTtcpLift = Tref7;
	//res1->gripperTobjects.push_back(Tref8);
	//res1->gripperTobjects.push_back(Tref9);
	res1->contactsGrasp.resize(4);
	res1->contactsLift.resize(2);
	//res1->interferenceTs.push_back(Tref10);
	res1->interferenceDistances.push_back(0.1);
	res1->interferenceDistances.push_back(0.2);
	res1->interferenceDistances.push_back(0.3);
	res1->interferenceDistances.push_back(0.4);
	res1->interferenceAngles.push_back(0.4);
	res1->interferenceAngles.push_back(0.5);
	res1->interferenceAngles.push_back(0.6);
	res1->interferences.push_back(0.7);
	res1->interferences.push_back(0.8);
	res1->interference = 1.123;

	GraspTarget target1;
	target1.pose = Tref11;
	target1.result = res1;

	stask1.addTarget(target1);
	stask1.addTarget(Tref12);
	stask1.addTarget(Tref13);

	task->addSubTask(stask1);

	return task;
}

void checkTask(GraspTask::Ptr task) {
	EXPECT_EQ("GripperId",task->getGripperID());
	EXPECT_EQ("TCPId",task->getTCPID());
	EXPECT_EQ("GraspControllerId",task->getGraspControllerID());

	const GraspSubTask& stask1 = task->getSubTasks()[0];
	EXPECT_EQ("refframe1",stask1.refframe);
	EXPECT_EQ("objectId1",stask1.objectID);
	EXPECT_TRUE(stask1.offset.equal(Tref1,1e-6));
	EXPECT_TRUE(stask1.approach.equal(Tref2,1e-6));
	EXPECT_TRUE(stask1.retract.equal(Tref3,1e-6));
	EXPECT_NEAR((stask1.openQ-Qref1).normInf(),0,std::numeric_limits<double>::epsilon());
	EXPECT_NEAR((stask1.closeQ-Qref2).normInf(),0,std::numeric_limits<double>::epsilon());
	EXPECT_NEAR((stask1.tauMax-Qref3).normInf(),0,std::numeric_limits<double>::epsilon());
	EXPECT_EQ(3,stask1.targets.size());
	if (stask1.targets.size() >= 3) {
		EXPECT_TRUE(stask1.targets[0].pose.equal(Tref11,1e-6));
		EXPECT_TRUE(stask1.targets[1].pose.equal(Tref12,1e-6));
		EXPECT_TRUE(stask1.targets[2].pose.equal(Tref13,1e-6));

		const GraspResult& res1 = *stask1.targets[0].result;
		EXPECT_EQ(GraspResult::Success,res1.testStatus);
		EXPECT_DOUBLE_EQ(0.123,res1.liftresult);
		EXPECT_NEAR((res1.gripperConfigurationGrasp-Qref4).normInf(),0,std::numeric_limits<double>::epsilon());
		EXPECT_NEAR((res1.gripperConfigurationLift-Qref5).normInf(),0,std::numeric_limits<double>::epsilon());
		EXPECT_NEAR((res1.qualityBeforeLifting-Qref6).normInf(),0,std::numeric_limits<double>::epsilon());
		EXPECT_NEAR((res1.qualityAfterLifting-Qref7).normInf(),0,std::numeric_limits<double>::epsilon());
		EXPECT_TRUE(res1.objectTtcpTarget.equal(Tref4,1e-6));
		EXPECT_TRUE(res1.objectTtcpApproach.equal(Tref5,1e-6));
		EXPECT_TRUE(res1.objectTtcpGrasp.equal(Tref6,1e-6));
		EXPECT_TRUE(res1.objectTtcpLift.equal(Tref7,1e-6));
		//EXPECT_EQ(2,res1.gripperTobjects.size());
		//if (res1.gripperTobjects.size() >= 2) {
		//	EXPECT_TRUE(res1.gripperTobjects[0].equal(Tref8,1e-6));
		//	EXPECT_TRUE(res1.gripperTobjects[1].equal(Tref9,1e-6));
		//}
		EXPECT_EQ(4,res1.contactsGrasp.size());
		EXPECT_EQ(2,res1.contactsLift.size());
		//EXPECT_EQ(1,res1.interferenceTs.size());
		//if (res1.interferenceTs.size() >= 1) {
		//	EXPECT_TRUE(res1.interferenceTs[0].equal(Tref10,1e-6));
		//}
		EXPECT_EQ(4,res1.interferenceDistances.size());
		if (res1.interferenceDistances.size() >= 4) {
			EXPECT_DOUBLE_EQ(0.1,res1.interferenceDistances[0]);
			EXPECT_DOUBLE_EQ(0.2,res1.interferenceDistances[1]);
			EXPECT_DOUBLE_EQ(0.3,res1.interferenceDistances[2]);
			EXPECT_DOUBLE_EQ(0.4,res1.interferenceDistances[3]);
		}
		EXPECT_EQ(3,res1.interferenceAngles.size());
		if (res1.interferenceAngles.size() >= 3) {
			EXPECT_DOUBLE_EQ(0.4,res1.interferenceAngles[0]);
			EXPECT_DOUBLE_EQ(0.5,res1.interferenceAngles[1]);
			EXPECT_DOUBLE_EQ(0.6,res1.interferenceAngles[2]);
		}
		EXPECT_EQ(2,res1.interferences.size());
		if (res1.interferences.size() >= 2) {
			EXPECT_DOUBLE_EQ(0.7,res1.interferences[0]);
			EXPECT_DOUBLE_EQ(0.8,res1.interferences[1]);
		}
		EXPECT_DOUBLE_EQ(1.123,res1.interference);
	}
	EXPECT_EQ("taskId1",stask1.taskID);
}
}

TEST(GraspTask, saveAndLoadXML) {
	std::ostringstream stream;
	{
		const GraspTask::Ptr task = makeReferenceTask();
		GraspTask::saveRWTask(task,stream);
	}
	{
		std::istringstream istream(stream.str());
		const GraspTask::Ptr task = GraspTask::load(istream);
		SCOPED_TRACE("Checking restore of XML previously saved.");
		checkTask(task);
	}
	{
		std::istringstream istream(getGraspTaskXML());
		const GraspTask::Ptr task = GraspTask::load(istream);
		SCOPED_TRACE("Checking loading of compiled XML reference file.");
		checkTask(task);
	}
}

TEST(TaskLoaderSaver, DOMParser) {
	TaskLoader::Ptr loader;
	TaskSaver::Ptr saver;

	loader = TaskLoader::Factory::getTaskLoader("xml","");
	saver = TaskSaver::Factory::getTaskSaver("xml","");
	EXPECT_FALSE(loader.cast<DOMTaskLoader>().isNull());
	EXPECT_FALSE(saver.cast<DOMTaskSaver>().isNull());

	loader = TaskLoader::Factory::getTaskLoader("xml","DOM");
	saver = TaskSaver::Factory::getTaskSaver("xml","DOM");
	ASSERT_FALSE(loader.cast<DOMTaskLoader>().isNull());
	ASSERT_FALSE(saver.cast<DOMTaskSaver>().isNull());

	std::ostringstream stream;
	{

		const GraspTask::Ptr task = makeReferenceTask();
		const CartesianTask::Ptr ctask = task->toCartesianTask();
		EXPECT_TRUE(saver->save(ctask,stream));
	}
	{
		std::istringstream istream(stream.str());
		loader->load(istream);
		const CartesianTask::Ptr ctask = loader->getCartesianTask();
		ASSERT_FALSE(ctask.isNull());
		const GraspTask::Ptr task = ownedPtr(new GraspTask(ctask));
		SCOPED_TRACE("Checking restore of XML previously saved.");
		checkTask(task);
	}
	{
		std::istringstream istream(getGraspTaskXML());
		loader->load(istream);
		const CartesianTask::Ptr ctask = loader->getCartesianTask();
		ASSERT_FALSE(ctask.isNull());
		const GraspTask::Ptr task = ownedPtr(new GraspTask(ctask));
		SCOPED_TRACE("Checking loading of compiled XML reference file.");
		checkTask(task);
	}
}

#ifdef RW_HAVE_XERCES
TEST(TaskLoader, Xerces) {
	TaskLoader::Ptr loader;
	TaskSaver::Ptr saver;

	loader = TaskLoader::Factory::getTaskLoader("xml","Xerces");
	saver = TaskSaver::Factory::getTaskSaver("xml","Xerces");
	ASSERT_FALSE(loader.cast<DOMTaskLoader>().isNull());
	ASSERT_FALSE(saver.cast<DOMTaskSaver>().isNull());

	std::ostringstream stream;
	{

		const GraspTask::Ptr task = makeReferenceTask();
		const CartesianTask::Ptr ctask = task->toCartesianTask();
		EXPECT_TRUE(saver->save(ctask,stream));
	}
	{
		std::istringstream istream(stream.str());
		loader->load(istream);
		const CartesianTask::Ptr ctask = loader->getCartesianTask();
		ASSERT_FALSE(ctask.isNull());
		const GraspTask::Ptr task = ownedPtr(new GraspTask(ctask));
		SCOPED_TRACE("Checking restore of XML previously saved.");
		checkTask(task);
	}
	{
		std::istringstream istream(getGraspTaskXML());
		loader->load(istream);
		const CartesianTask::Ptr ctask = loader->getCartesianTask();
		ASSERT_FALSE(ctask.isNull());
		const GraspTask::Ptr task = ownedPtr(new GraspTask(ctask));
		SCOPED_TRACE("Checking loading of compiled XML reference file.");
		checkTask(task);
	}
}
#endif
