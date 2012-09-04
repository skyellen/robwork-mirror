#include "Experiments.hpp"

#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace std;
using namespace boost::numeric;
using namespace boost::property_tree;
using namespace rwlibs::task;

rwlibs::task::CartesianTask::Ptr Experiments::toCartesianTask(){
    rwlibs::task::CartesianTask::Ptr root= ownedPtr( new rwlibs::task::CartesianTask() );

    BOOST_FOREACH(Experiment &exp, _experiments ){
        rwlibs::task::CartesianTask::Ptr experiment = ownedPtr( new rwlibs::task::CartesianTask() );
        root->addTask( experiment );

        experiment->getPropertyMap().set<int>("ID",exp.id);
        experiment->getPropertyMap().set<int>("TestStatus",exp.testStatus);

        if(!(exp.worldTobjectReal.equal( Transform3D<>::identity() ) ) )
        	experiment->getPropertyMap().set<Transform3D<> >("WorldTObjectReal",exp.worldTobjectReal);
        if(!(exp.worldTobjectDetected.equal( Transform3D<>::identity() ) ) )
        	experiment->getPropertyMap().set<Transform3D<> >("WorldTObjectDetected",exp.worldTobjectDetected);

        if(exp.gripperConfigurationTarget.size()>0)
        	experiment->getPropertyMap().set<Q>("GripperConfigurationTarget",	exp.gripperConfigurationTarget);
        if(exp.gripperConfigurationGrasp.size()>0)
        	experiment->getPropertyMap().set<Q>("GripperConfigurationGrasp",	exp.gripperConfigurationGrasp);
        if(exp.gripperConfigurationLift.size()>0)
        	experiment->getPropertyMap().set<Q>("GripperConfigurationLift",		exp.gripperConfigurationLift);
        if(exp.gripperConfigurationRelease.size()>0)
        	experiment->getPropertyMap().set<Q>("GripperConfigurationRelease",	exp.gripperConfigurationRelease);

        if(!(exp.objectTtcpTarget.equal( Transform3D<>::identity() ) ) )
        	experiment->getPropertyMap().set<Transform3D<> >("ObjectTtcpTarget",	exp.objectTtcpTarget);
        if(!(exp.objectTtcpGrasp.equal( Transform3D<>::identity() ) ) )
        	experiment->getPropertyMap().set<Transform3D<> >("ObjectTtcpGrasp",		exp.objectTtcpGrasp);
        if(!(exp.objectTtcpLift.equal( Transform3D<>::identity() ) ) )
        	experiment->getPropertyMap().set<Transform3D<> >("ObjectTtcpLift",		exp.objectTtcpLift);
        if(!(exp.objectTtcpRelease.equal( Transform3D<>::identity() ) ) )
        	experiment->getPropertyMap().set<Transform3D<> >("ObjectTtcpRelease",	exp.objectTtcpRelease);

        if(exp.mark!=-1)
        	experiment->getPropertyMap().set<int>("Mark", exp.mark);
        if(exp.poseEstimateError!=-1)
        	experiment->getPropertyMap().set<double>("PoseEstimateError", exp.poseEstimateError);
        if(exp.inlierFraction!=-1)
        	experiment->getPropertyMap().set<double>("InlierFraction", exp.inlierFraction);
        if(exp.temperature.size() > 0)
        	experiment->getPropertyMap().set<std::vector<int> >("Temperature", exp.temperature);
    }

    return root;
}

void Experiments::saveRWTask(Experiments::Ptr experiments, const std::string& name ){
    std::ofstream outfile(name.c_str());
    rwlibs::task::CartesianTask::Ptr ctask = experiments->toCartesianTask();
    try {
        XMLTaskSaver saver;
        saver.save(ctask, outfile );
    } catch (const Exception& exp) {
        RW_THROW("Unable to save task: " << exp.what());
    }

    outfile.close();
}

Experiments::Ptr Experiments::load(const std::string& filename){
    std::string file = IOUtil::getAbsoluteFileName(filename);
    std::string firstelem = IOUtil::getFirstXMLElement(file);
    //std::cout << "FIRST ELEMENT: " << firstelem << std::endl;

    rwlibs::task::CartesianTask::Ptr grasptask;

    if(firstelem=="CartesianTask"){
        XMLTaskLoader loader;
        loader.load( file );
        grasptask = loader.getCartesianTask();
    }

    Experiments::Ptr gtask = ownedPtr( new Experiments(grasptask) );
    return gtask;
}

Experiments::Experiments(rwlibs::task::CartesianTask::Ptr experiments){
    _experiments.resize( experiments->getTasks().size() );
    for(size_t i=0;i<experiments->getTasks().size();i++){
        rwlibs::task::CartesianTask::Ptr experiment = experiments->getTasks()[i];
        _experiments[i].id = experiment->getPropertyMap().get<int>("ID",-1);
        _experiments[i].testStatus = (GraspTask::TestStatus) experiment->getPropertyMap().get<int>("TestStatus",GraspTask::UnInitialized);

        _experiments[i].worldTobjectReal		= experiment->getPropertyMap().get<Transform3D<> >("WorldTObjectReal",Transform3D<>::identity());
        _experiments[i].worldTobjectDetected	= experiment->getPropertyMap().get<Transform3D<> >("WorldTObjectDetected",Transform3D<>::identity());

        _experiments[i].gripperConfigurationTarget	= experiment->getPropertyMap().get<Q>("GripperConfigurationTarget",Q());
        _experiments[i].gripperConfigurationGrasp	= experiment->getPropertyMap().get<Q>("GripperConfigurationGrasp",Q());
        _experiments[i].gripperConfigurationLift	= experiment->getPropertyMap().get<Q>("GripperConfigurationLift",Q());
        _experiments[i].gripperConfigurationRelease	= experiment->getPropertyMap().get<Q>("GripperConfigurationRelease",Q());

        _experiments[i].objectTtcpTarget	= experiment->getPropertyMap().get<Transform3D<> >("ObjectTtcpTarget",Transform3D<>::identity());
        _experiments[i].objectTtcpGrasp		= experiment->getPropertyMap().get<Transform3D<> >("ObjectTtcpGrasp",Transform3D<>::identity());
        _experiments[i].objectTtcpLift		= experiment->getPropertyMap().get<Transform3D<> >("ObjectTtcpLift",Transform3D<>::identity());
        _experiments[i].objectTtcpRelease	= experiment->getPropertyMap().get<Transform3D<> >("ObjectTtcpRelease",Transform3D<>::identity());

        _experiments[i].mark				= experiment->getPropertyMap().get<int>("Mark",-1);
        _experiments[i].poseEstimateError	= experiment->getPropertyMap().get<double>("PoseEstimateError",-1);
        _experiments[i].inlierFraction		= experiment->getPropertyMap().get<double>("InlierFraction",-1);
        _experiments[i].temperature			= experiment->getPropertyMap().get<std::vector<int> >("Temperature",std::vector<int>());
    }
}
