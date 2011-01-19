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

#include <rw/proximity.hpp>
#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/math.hpp>
#include <rw/geometry.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/common/TimerUtil.hpp>

#include <string>
#include <boost/test/unit_test.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

using namespace boost::unit_test;
using namespace rwlibs::proximitystrategies;


typedef std::pair<int,int> ModelPair;
struct CollisionTestSetup {
    CollisionStrategy::Ptr strategy;
    std::vector<ProximityModel::Ptr> models;
    std::vector<std::vector<Transform3D<> > > modelsConfigurations;
    std::string strategyName;
    std::vector< ModelPair > modelPairs;
    CollisionQueryType qtype;
    std::string modelDetail, queryTypeStr;

};

void testPerConfiguration( CollisionTestSetup& setup ){
    // This function iterates over all configurations and for each configuration
    // it tests collision between all model pairs
    int nrOfQueries = setup.modelsConfigurations.size()*setup.modelPairs.size();
    std::cout << "--------- Performancetest - testPerConfiguration ----------" << std::endl;
    std::cout << "- Collision Strategy ID: " << setup.strategyName << std::endl;
    std::cout << "- Performing " << nrOfQueries << " collision tests" << std::endl;
    std::cout << "- Model mesh detail: " << setup.modelDetail << std::endl;
    std::cout << "- Collision queries: " << setup.queryTypeStr << std::endl;

    ProximityStrategyData data;
    data.setCollisionQueryType(setup.qtype);
    Timer time;
    int nrCollisions = 0;
    BOOST_FOREACH(std::vector<Transform3D<> >& config, setup.modelsConfigurations){
        BOOST_FOREACH( ModelPair mpair, setup.modelPairs){
            ProximityModel::Ptr& modelA = setup.models[mpair.first];
            Transform3D<> Ta = config[mpair.first];
            ProximityModel::Ptr& modelB = setup.models[mpair.second];
            Transform3D<> Tb = config[mpair.second];

            if(setup.strategy->inCollision(modelA, Ta, modelB, Tb, data))
                nrCollisions++;
        }
    }
    time.pause();
    std::cout << " - nrCollissions: " << nrCollisions/((double)nrOfQueries) << std::endl;
    std::cout << " - time: " << time.getTime() << "s" << std::endl;
    std::cout << " - avg time per query: " << time.getTime()/nrOfQueries << "s" << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
}

void testPerObjectPair(CollisionTestSetup& setup ){
    // This function iterates over all configurations and for each configuration
    // it tests collision between all model pairs
    int nrOfQueries = setup.modelsConfigurations.size()*setup.modelPairs.size();
    std::cout << "--------- Performancetest - testPerObjectPair ----------" << std::endl;
    std::cout << "- Collision Strategy ID: " << setup.strategyName << std::endl;
    std::cout << "- Performing " << setup.modelsConfigurations.size()*setup.modelPairs.size() << " collision tests" << std::endl;
    std::cout << "- Model mesh detail: " << setup.modelDetail << std::endl;
    std::cout << "- Collision queries: " << setup.queryTypeStr << std::endl;

    ProximityStrategyData data;
    data.setCollisionQueryType(setup.qtype);
    Timer time;
    int nrCollisions = 0;
    BOOST_FOREACH( ModelPair mpair, setup.modelPairs){
        ProximityModel::Ptr& modelA = setup.models[mpair.first];
        ProximityModel::Ptr& modelB = setup.models[mpair.second];
        BOOST_FOREACH(std::vector<Transform3D<> >& config, setup.modelsConfigurations){
            Transform3D<> Ta = config[mpair.first];
            Transform3D<> Tb = config[mpair.second];

            if(setup.strategy->inCollision(modelA, Ta, modelB, Tb, data))
                nrCollisions++;
        }
    }
    time.pause();
    std::cout << " - nrCollissions: " << nrCollisions/((double)nrOfQueries) << std::endl;
    std::cout << " - time: " << time.getTime() << "s" << std::endl;
    std::cout << " - avg time per query: " << time.getTime()/nrOfQueries << "s" << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
}

void intializeTestSetupData(CollisionTestSetup& setup, int nrConfigurations){
    setup.modelsConfigurations.clear();
    // this initialize a number of configurations for each model all bound within a 1x1x1 cube
    for(int i=0;i<nrConfigurations;i++){
        std::vector<Transform3D<> > config;
        for(int j=0; j<setup.models.size();j++){
            Vector3D<> v(Math::ran(0,1.0),Math::ran(0,0.3),Math::ran(0,0.3));
            RPY<> r(Math::ran(-Pi,Pi),Math::ran(-Pi,Pi),Math::ran(-Pi,Pi));

            config.push_back( Transform3D<>(v,r.toRotation3D() ));
        }
        setup.modelsConfigurations.push_back(config);
    }



}



void intializeBuildingSetup(CollisionTestSetup& setup ){
    std::cout << "--------- Performancetest - Building/initializing ----------" << std::endl;
    std::cout << "- Collision Strategy ID: " << setup.strategyName << std::endl;
    std::cout << "- Performing " << setup.modelPairs.size() << " collision tests" << std::endl;
    std::cout << "- Model mesh detail: " << setup.modelDetail << std::endl;
    std::cout << "- Collision queries: " << setup.queryTypeStr << std::endl;
    ProximityStrategyData data;
    data.setCollisionQueryType(setup.qtype);
    Timer time;
    std::vector<Transform3D<> >& config = setup.modelsConfigurations[0];
    BOOST_FOREACH( ModelPair mpair, setup.modelPairs){
        ProximityModel::Ptr& modelA = setup.models[mpair.first];
        Transform3D<> Ta = config[mpair.first];
        ProximityModel::Ptr& modelB = setup.models[mpair.second];
        Transform3D<> Tb = config[mpair.second];

        setup.strategy->inCollision(modelA, Ta, modelB, Tb, data);
    }
    time.pause();
    std::cout << " - time: " << time.getTime() << "s" << std::endl;

    std::cout << "-------------------------------------------------------------" << std::endl;


}

void loadModelsData(CollisionTestSetup& setup, int nrModels, const std::string& filename, const std::string& geomDetail){
    std::cout << "------------------ Loading: " << std::endl;
    std::cout << "- File: " << filename << std::endl;
    RW_ASSERT(setup.strategy!=NULL);
    setup.modelPairs.clear();
    setup.models.clear();


    setup.modelDetail = geomDetail;
    Geometry::Ptr geom = GeometryFactory::load( testFilePath().append( filename ) );
    char istr[20];
    Timer time;
    for(int i=0;i<nrModels;i++){
        std::string gstr("geom");
        itoa(i, istr, 10);
        std::string id = gstr.append( istr );
        geom->setId( id );
        ProximityModel::Ptr model = setup.strategy->createModel();
        setup.strategy->addGeometry( model.get(), *geom );
        setup.models.push_back(model);
        std::cout << "- nr models loaded: " << i << "\r";
    }
    time.pause();

    // generate model pairs
    for(int i=0;i<nrModels-1;i++){
        for(int j=i+1;j<nrModels;j++){
            setup.modelPairs.push_back( std::make_pair(i,j) );
        }
    }

    std::cout << "Build time: " << time.getTime() << "s" << std::endl;
    std::cout << "-------------------------------------------------------------\n";

}

void setFirstContact(CollisionTestSetup& setup){
    setup.qtype = FirstContact;
    setup.queryTypeStr = "FirstContact";
}

void setAllContact(CollisionTestSetup& setup){
    setup.qtype = AllContacts;
    setup.queryTypeStr = "AllContacts";
}

void testStrategy(CollisionStrategy::Ptr strategy, const std::string& strategyname){
    /* TODO:
     * Motion Planning (First Contact) Scene - Coarse Geometry
     * Motion Planning (First Contact) Scene - Fine Geometry
     * Close Proximity (First Contact) Scene - Coarse Geometry
     * Close Proximity (First Contact) Scene - Fine Geometry
     * Random sampling (First Contact) Scene - Coarse Geometry
     * Random sampling (First Contact) Scene - Fine Geometry
     * Random sampling (Full Contact) Scene - Coarse Geometry
     * Random sampling (Full Contact) Scene - Fine Geometry
     * Simulation (Full Contact Info) Scene - Coarse Geometry
     * Simulation (Full Contact Info) Scene - Fine Geometry
     *
     * 1000 configurations, 10 objects, Per configuration - Fine Geometry
     * 1000 configurations, 10 objects, Per object pair (Cache optimized) - Fine Geometry
     *
     * 1000 configurations, 10 objects, Per configuration - Fine Geometry
     * 1000 configurations, 10 objects, Per object pair (Cache optimized) - Fine Geometry
     *
     */

    CollisionTestSetup coarsesetup, finesetup;
    coarsesetup.strategy = strategy;
    coarsesetup.strategyName = strategyname;
    finesetup.strategy = strategy;
    finesetup.strategyName = strategyname;

    loadModelsData(coarsesetup, 20, "geoms/performance/CoarseModel.stl", "Coarse");
    //setFirstContact(coarsesetup);
    setAllContact(coarsesetup);
    intializeTestSetupData( coarsesetup, 1);
    intializeBuildingSetup( coarsesetup );
    intializeTestSetupData( coarsesetup, 1000);
    testPerConfiguration( coarsesetup );
    testPerObjectPair( coarsesetup );


    loadModelsData(finesetup, 20, "geoms/performance/FineModel.stl", "Fine");
    setFirstContact(finesetup);
    intializeTestSetupData( finesetup, 1);
    intializeBuildingSetup( finesetup );
    intializeTestSetupData( finesetup, 1000);
    testPerConfiguration( finesetup );
    testPerObjectPair( finesetup );

}

BOOST_AUTO_TEST_CASE( testCollisionQueryPerformance )
{
    BOOST_MESSAGE("Collission Query Performance Tests.");
    // We seed the random number generator so that we get reproducible results.
    Math::seed(0);

    // now for each strategy we perform the series of tests
    BOOST_FOREACH(std::string strategyname, ProximityStrategyFactory::getCollisionStrategyIDs()){
        //if(strategyname=="PQP")
        //    continue;
        CollisionStrategy::Ptr strategy = ProximityStrategyFactory::makeCollisionStrategy(strategyname);
        testStrategy(strategy, strategyname);
    }


}
