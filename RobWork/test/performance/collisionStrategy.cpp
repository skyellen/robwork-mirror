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

#include <rw/loaders.hpp>
#include <rw/common/TimerUtil.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iomanip>


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
    CollisionStrategy::QueryType qtype;
    std::string modelDetail, queryTypeStr;
};

struct TestResult {
    std::string testid;
    std::string strategyID;
    std::string modelDetail;
    std::string queryType;

    int nrOfQueries;
    long nrOfBVTests;
    long nrOfPrimTests;

    double time;
    double timePerQuery;


};

void testPerConfiguration( CollisionTestSetup& setup , std::vector<std::pair<std::string, double> > &timings){
    // This function iterates over all configurations and for each configuration
    // it tests collision between all model pairs
    int nrOfQueries = (int)(setup.modelsConfigurations.size()*setup.modelPairs.size());
    std::cout << "--------- Performancetest - testPerConfiguration ----------" << std::endl;
    std::cout << "- Collision Strategy ID: " << setup.strategyName << std::endl;
    std::cout << "- Performing " << nrOfQueries << " collision tests" << std::endl;
    std::cout << "- Model mesh detail: " << setup.modelDetail << std::endl;
    std::cout << "- Collision queries: " << setup.queryTypeStr << std::endl;

    ProximityStrategyData data;
    data.setCollisionQueryType(setup.qtype);
    Timer time;
    int nrCollisions = 0;
    long nrOfBVTests = 0;
    long nrOfPrimTests = 0;
    BOOST_FOREACH(std::vector<Transform3D<> >& config, setup.modelsConfigurations){
        BOOST_FOREACH( ModelPair mpair, setup.modelPairs){
            ProximityModel::Ptr& modelA = setup.models[mpair.first];
            Transform3D<> Ta = config[mpair.first];
            ProximityModel::Ptr& modelB = setup.models[mpair.second];
            Transform3D<> Tb = config[mpair.second];

            if(setup.strategy->inCollision(modelA, Ta, modelB, Tb, data))
                nrCollisions++;

            nrOfBVTests += data.getCollisionData().getNrBVTests();
            nrOfPrimTests += data.getCollisionData().getNrPrimTests();
        }
    }
    //double bvTestsPerCollision = nrOfBVTests/(1.0*nrCollisions);
    //double primTestsPerCollision = nrOfPrimTests/(1.0*nrCollisions);

    time.pause();
    std::cout << " - nrCollissions: " << nrCollisions/((double)nrOfQueries) << std::endl;
    std::cout << " - time: " << time.getTime() << "s" << std::endl;
    std::cout << " - avg time per query: " << time.getTime()/nrOfQueries << "s" << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    timings.push_back(std::make_pair(std::string("PerQ_check_t;")+setup.modelDetail+";"+setup.queryTypeStr, time.getTime()));
    timings.push_back(std::make_pair(std::string("PerQ_check_%;")+setup.modelDetail+";"+setup.queryTypeStr, nrCollisions/((double)nrOfQueries)));
}

void testPerObjectPair(CollisionTestSetup& setup, std::vector<std::pair<std::string, double> > &timings){
    // This function iterates over all configurations and for each configuration
    // it tests collision between all model pairs
    int nrOfQueries = (int)(setup.modelsConfigurations.size()*setup.modelPairs.size());
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

    timings.push_back(std::make_pair(std::string("PerO_check_t;")+setup.modelDetail+";"+setup.queryTypeStr, time.getTime()));
    timings.push_back(std::make_pair(std::string("PerO_check_%;")+setup.modelDetail+";"+setup.queryTypeStr, nrCollisions/((double)nrOfQueries)));
}

void intializeTestSetupData(CollisionTestSetup& setup, int nrConfigurations){
    setup.modelsConfigurations.clear();
    // this initialize a number of configurations for each model all bound within a 1x1x1 cube
    for(int i=0;i<nrConfigurations;i++){
        std::vector<Transform3D<> > config;
        for(unsigned int j=0; j<setup.models.size();j++){
            Vector3D<> v(Math::ran(0,1.0),Math::ran(0,0.3),Math::ran(0,0.3));
            RPY<> r(Math::ran(-Pi,Pi),Math::ran(-Pi,Pi),Math::ran(-Pi,Pi));

            config.push_back( Transform3D<>(v,r.toRotation3D() ));
        }
        setup.modelsConfigurations.push_back(config);
    }
}



void intializeBuildingSetup(CollisionTestSetup& setup, std::vector<std::pair<std::string, double> > &timings){
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

    timings.push_back(std::make_pair(std::string("build;")+setup.modelDetail+";"+setup.queryTypeStr, time.getTime()));
}

void loadModelsData(CollisionTestSetup& setup, int nrModels, const std::string& filename, const std::string& geomDetail){
    std::cout << "------------------ Loading: " << std::endl;
    std::cout << "- File: " << filename << std::endl;
    RW_ASSERT(setup.strategy!=NULL);
    setup.modelPairs.clear();
    setup.models.clear();


    setup.modelDetail = geomDetail;
    Geometry::Ptr geom = GeometryFactory::load( testFilePath().append( filename ) );
    //Geometry::Ptr geom = Geometry::makeCylinder(0.1, 0.3);
    //char istr[20];
    std::string istr;
    Timer time;
    for(int i=0;i<nrModels;i++){
        std::string gstr("geom");
        
        //std::itoa(i, istr, 10);
        std::ostringstream os;
        os << std::dec << i;
        istr = os.str();
        
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
    setup.qtype = CollisionStrategy::FirstContact;
    setup.queryTypeStr = "FirstContact";
}

void setAllContact(CollisionTestSetup& setup){
    setup.qtype = CollisionStrategy::AllContacts;
    setup.queryTypeStr = "AllContacts";
}

std::vector<std::pair<std::string, double> > testStrategy(CollisionStrategy::Ptr strategy, const std::string& strategyname){
    std::vector<std::pair<std::string, double> > timings;
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
    setAllContact(coarsesetup);
    intializeTestSetupData( coarsesetup, 1);
    intializeBuildingSetup( coarsesetup , timings);
    intializeTestSetupData( coarsesetup, 100);

    testPerConfiguration( coarsesetup , timings);
    testPerObjectPair( coarsesetup , timings);

    //setFirstContact(coarsesetup);

    loadModelsData(finesetup, 20, "geoms/performance/CoarseModel.stl", "Fine");
    setFirstContact(finesetup);
    intializeTestSetupData( finesetup, 1);
    intializeBuildingSetup( finesetup , timings);
    intializeTestSetupData( finesetup, 200);
    testPerConfiguration( finesetup, timings );
    testPerObjectPair( finesetup , timings);
    return timings;
}

BOOST_AUTO_TEST_CASE( testCollisionQueryPerformance )
{
    BOOST_TEST_MESSAGE("Collission Query Performance Tests.");
    // We seed the random number generator so that we get reproducible results.
    Math::seed(0);
    std::vector<std::string> colids;
    std::vector<std::vector<std::pair<std::string, double> > > timings;

    // now for each strategy we perform the series of tests
    BOOST_FOREACH(std::string strategyname, ProximityStrategyFactory::getCollisionStrategyIDs()){
        //if(strategyname=="PQP")
        //    continue;
        colids.push_back(strategyname);
        CollisionStrategy::Ptr strategy = ProximityStrategyFactory::makeCollisionStrategy(strategyname);
        timings.push_back( testStrategy(strategy, strategyname) );
    }

    // print results
    // first find the longest of the strings
    size_t length = 0;
    for(size_t i=0;i<timings[0].size();i++){
        length = std::max(timings[0][i].first.size(), length);
    }

    std::cout << std::setw(length) <<"." << "\t";
    for(size_t j=0;j<colids.size();j++){
        std::cout << colids[j] << "\t";
    }
    std::cout << std::endl;

    for(size_t i=0;i<timings[0].size();i++){
        std::cout << std::setw(length) << timings[0][i].first << "\t";
        for(size_t j=0;j<colids.size();j++){
            std::cout << std::setprecision(5) << timings[j][i].second << "\t";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;


}
