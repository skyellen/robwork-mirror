/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
#include "../TestEnvironment.hpp"

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/geometry/PointCloud.hpp>

#include <rwlibs/simulation/Simulator.hpp>

#include <rwlibs/simulation/SimulatedScanner2D.hpp>
#include <rwlibs/simulation/SimulatedScanner25D.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>

#include <rwlibs/simulation/GLFrameGrabber25D.cpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

#include <rws/RobWorkStudio.hpp>

#include <QApplication>

#include <string>

using rw::geometry::PointCloud;
using rw::kinematics::Frame;
using rw::loaders::WorkCellLoader;
using namespace rwlibs::simulation;
using rws::RobWorkStudio;
using std::string;

namespace
{
class SimulatedSensors_test : public ::testing::Test
{
public:
    const string workcellFile = TestEnvironment::testfilesDir() + "/SensorTest.wc.xml";

    rw::models::WorkCell::Ptr wc;

    // Scanner
    const string scannerName = "TestScanner";
    Frame::Ptr sensorFrame;

    // Def state:
    rw::kinematics::State defaultState;

    rwlibs::simulation::Simulator::UpdateInfo updateInfo;

    QApplication* app;
    RobWorkStudio* rwstudio;

    SceneViewer::Ptr sceneViewer;

    virtual void SetUp()
    {
        // Find and workcell, default state and sensor frame
        wc = WorkCellLoader::Factory::load(workcellFile);
        if (wc == NULL)
            RW_THROW("workcell not found");
        sensorFrame = wc->findFrame("Sensor1");
        if (sensorFrame == NULL)
            RW_THROW("sensor frame not found");
        defaultState = wc->getDefaultState();

        updateInfo.time = 0;
        updateInfo.dt = 0.1001;

        // Open an instance of RWS
        int argc = 1;
        char name[14] = "RobWorkStudio";
        char* argv[1] = {name};
        PropertyMap map;
        app = new QApplication(argc,argv);
        rwstudio = new RobWorkStudio(map);
        rwstudio->setWorkcell(wc);
        rwstudio->show();
        app->processEvents();

        sceneViewer = rwstudio->getView()->getSceneViewer();
    }

    virtual void TearDown() {
    	delete rwstudio;
    	delete app;
    }
};

TEST_F(SimulatedSensors_test, SimulatedScanner2D_test) {
    static const int x = 101;
    static const int y = 101;

    GLFrameGrabber25D* const frameGrabber = new GLFrameGrabber25D(x, y, 90, 0.1, 5);
    if (!frameGrabber->init(sceneViewer)) {
    	delete frameGrabber;
    	RW_WARN("Not performing this test because it does not seem to be supported.");
    	return;
    }

    SimulatedScanner2D* const scanner2d = new SimulatedScanner2D(scannerName,sensorFrame.get(),frameGrabber);
    scanner2d->open();
    TimerUtil::sleepMs(50);

    //scanner->acquire();
    EXPECT_TRUE(scanner2d->isOpen());
    scanner2d->setFrameRate(10);
    EXPECT_NEAR(scanner2d->getFrameRate(),10,0.0001);
    EXPECT_FALSE(scanner2d->isScanReady());
    scanner2d->acquire();  // not necessary for simulated

    PointCloud lastScan = scanner2d->getScan();

    scanner2d->update(updateInfo,defaultState);
    EXPECT_TRUE(scanner2d->isScanReady());

    lastScan = scanner2d->getScan();

    //Middle point of scan should be in X = 0
    EXPECT_NEAR(lastScan(50,50)(0),0,0.01);

    //First and last point in scan should be at X = -0.95 and +0.95
    EXPECT_NEAR(lastScan(0,50)(0),-0.95,0.01);
    EXPECT_NEAR(lastScan(100,50)(0),0.95,0.05);

    // We expect z coordinate to be = -0.95 at all points:
    for (int i = 0; i < x; i++)
    {
        EXPECT_NEAR(lastScan(i,50)(2),-0.95,0.01);
    }

    delete scanner2d;
    delete frameGrabber;
}

TEST_F(SimulatedSensors_test, SimulatedScanner25D_test) {
    static const int x = 101;
    static const int y = 101;

    GLFrameGrabber25D* const frameGrabber = new GLFrameGrabber25D(x, y, 90, 0.1, 5);
    if (!frameGrabber->init(sceneViewer)) {
    	RW_WARN("Not performing this test because it does not seem to be supported.");
    	delete frameGrabber;
    	return;
    }

    SimulatedScanner25D* const scanner25d = new SimulatedScanner25D(scannerName,sensorFrame.get(),frameGrabber);
    scanner25d->open();
    TimerUtil::sleepMs(50);
    EXPECT_TRUE(scanner25d->isOpen());

    // In case we dont want to implement setFrameRate, delete the following lines
    scanner25d->setFrameRate(10);
    EXPECT_NEAR(scanner25d->getFrameRate(),10,0.0001);

    EXPECT_FALSE(scanner25d->isScanReady());

    scanner25d->acquire();  // not necessary for simulated sensors as the function does nothing

    PointCloud lastScan = scanner25d->getScan();

    scanner25d->update(updateInfo,defaultState);
    EXPECT_TRUE(scanner25d->isScanReady());

    lastScan = scanner25d->getScan();

    //Middle point of scan should be in X = 0
    EXPECT_NEAR(lastScan(50,50)(0),0,0.01);

    //First and last point in scan should be at X = -0.95 and +0.95
    EXPECT_NEAR(lastScan(0,50)(0),-0.95,0.01);
    EXPECT_NEAR(lastScan(100,50)(0),0.95,0.05);

    // We expect z coordinate to be = -0.95 at all points:
    for (int i = 0; i < x; i++)
    {
        EXPECT_NEAR(lastScan(i,50)(2),-0.95,0.01);
    }

    delete scanner25d;
    delete frameGrabber;
}

TEST_F(SimulatedSensors_test, SimulatedCamera_test)
{
	static const int x = 128;
	static const int y = 128;

    GLFrameGrabber* const frameGrabber2d = new GLFrameGrabber(x, y, 100, 0.1, 5);

    if (!frameGrabber2d->init(sceneViewer)) {
    	RW_WARN("Not performing this test because it does not seem to be supported.");
    	delete frameGrabber2d;
    	return;
    }
    TimerUtil::sleepMs(100);

    SimulatedCamera* const camera = new SimulatedCamera(scannerName,90,sensorFrame.get(),frameGrabber2d);

    // initialize and start camera
    EXPECT_TRUE(camera->initialize());
    EXPECT_TRUE(camera->start());
    TimerUtil::sleepMs(100);

    // Check framerate
    EXPECT_EQ(camera->getFrameRate(),30);
    camera->setFrameRate(20);
    EXPECT_EQ(camera->getFrameRate(),20);
    // Check image dimensions:
    EXPECT_EQ(camera->getWidth(),128);
    EXPECT_EQ(camera->getHeight(),128);

    // We expect no image, since no time steps has been completed
    EXPECT_FALSE(camera->isImageReady());

    updateInfo.dt = 0.0501;

    // Update to get an image
    camera->update(updateInfo,defaultState);

    ASSERT_TRUE(camera->isImageReady());

    const rw::sensor::Image* lastImage = camera->getImage();

    EXPECT_EQ(lastImage->getWidth(),128);
    EXPECT_EQ(lastImage->getHeight(),128);

    // save image for reference
    lastImage->saveAsPPM("cameraimage.ppm");

    // check some pixel values: (white first)
    EXPECT_EQ(lastImage->getPixelValuei(0,0,0),255);
    EXPECT_EQ(lastImage->getPixelValuei(0,0,1),255);
    EXPECT_EQ(lastImage->getPixelValuei(0,0,2),255);

    EXPECT_EQ(lastImage->getPixelValuei(2,57,0),255);
    EXPECT_EQ(lastImage->getPixelValuei(2,57,1),255);
    EXPECT_EQ(lastImage->getPixelValuei(2,57,2),255);

    // red pixel(collision)
    EXPECT_EQ(lastImage->getPixelValuei(1,57,0),184);
    EXPECT_EQ(lastImage->getPixelValuei(1,57,1),31);
    EXPECT_EQ(lastImage->getPixelValuei(1,57,2),31);

    // gray pixel (box with no texture)
    EXPECT_EQ(lastImage->getPixelValuei(124,58,0),46);
    EXPECT_EQ(lastImage->getPixelValuei(124,58,1),46);
    EXPECT_EQ(lastImage->getPixelValuei(124,58,2),46);

	delete camera;
	delete frameGrabber2d;
}
}
