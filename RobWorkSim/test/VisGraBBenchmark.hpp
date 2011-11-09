#ifndef VISGRABBENCHMARK_HPP_
#define VISGRABBENCHMARK_HPP_

#include <rw/common/Ptr.hpp>
#include <rwlibs/task/Task.hpp>
#include <rwsim/simulator/GraspTask.hpp>
#include <boost/tuple/tuple.hpp>

/**
 * @brief a container for describing one or multiple grasping tasks. It is based on the rwlibs::tasks library
 *
 * Definition of VisGraBGraspTask xml format
 *
 */

class VisGraBBenchmark {
public:
    typedef rw::common::Ptr<VisGraBBenchmark> Ptr;

    VisGraBBenchmark(){}

    virtual ~VisGraBBenchmark(){};

    static void save(VisGraBBenchmark::Ptr task, const std::string& name );

    static VisGraBBenchmark::Ptr load(const std::string& name);


    virtual size_t getNrOfScenes() = 0;
    virtual std::string getSceneWCData(size_t sceneNr) = 0;
    virtual std::string getSceneDWCData(size_t sceneNr) = 0;
    /**
     * @brief this should point to a workcell with collision information that
     * may be used prior to the simulation of grasps. That is as a filtering
     * of grasp candidates. If the string is empty then there are no such scene
     */
    virtual std::string getSceneCollisionData(size_t sceneNr) = 0;

    /**
     * @brief returns the index of the scene with sceneID \b sid
     * @param sid [in] id of the scene to locate
     * @return index of the scene or -1 if it does not exist
     */
    virtual size_t findSceneFromID(const std::string& sid) = 0;

};

/**
 * @brief describe a benchmark setup where the objects are unknown
 * and stereo images and derived point clouds are available.
 */
struct GraspUnknownObjectStereoBM: public VisGraBBenchmark {
    typedef rw::common::Ptr<GraspUnknownObjectStereoBM> Ptr;

    GraspUnknownObjectStereoBM(){}
    virtual ~GraspUnknownObjectStereoBM(){};

    size_t getNrOfScenes(){ return _sceneInfos.size(); };

    std::string getSceneIDInfo(size_t sceneNr){ return _sceneInfos[sceneNr].get<0>(); };
    std::string getSceneLocatorInfo(size_t sceneNr){ return _sceneInfos[sceneNr].get<1>(); };
    std::string getSceneSetInfo(size_t sceneNr){ return _sceneInfos[sceneNr].get<2>(); };
    std::string getSceneSetIDInfo(size_t sceneNr){ return _sceneInfos[sceneNr].get<3>(); };
    std::string getSceneTextureInfo(size_t sceneNr){ return _sceneInfos[sceneNr].get<4>(); };
    std::vector<std::string> getSceneObjectsInfo(size_t sceneNr){ return _sceneInfos[sceneNr].get<5>(); };

    std::string getSceneLeftImage(size_t sceneNr){ return fullpathBenchmark+"/"+_sceneInput[sceneNr].get<0>(); };
    std::string getSceneRightImage(size_t sceneNr){ return fullpathBenchmark+"/"+_sceneInput[sceneNr].get<1>(); };
    std::string getSceneDispData(size_t sceneNr){ return fullpathBenchmark+"/"+_sceneInput[sceneNr].get<2>(); };
    std::string getScenePointCloud(size_t sceneNr){ return fullpathBenchmark+"/"+_sceneInput[sceneNr].get<3>(); };

    std::string getSceneWCData(size_t sceneNr){ return fullpathBenchmark+"/"+_sceneSimData[sceneNr].get<0>(); };
    std::string getSceneDWCData(size_t sceneNr){ return fullpathBenchmark+"/"+_sceneSimData[sceneNr].get<1>(); };
    std::string getSceneCollisionData(size_t sceneNr){ return fullpathBenchmark+"/"+_sceneSimData[sceneNr].get<2>(); };

    size_t findSceneFromID(const std::string& sid){
        if(_sidToIndex.find(sid)==_sidToIndex.end())
            return -1;
        return _sidToIndex[sid];
    }

    //! the info, encoded as SceneID, SceneLocator, SceneSet, SceneSetID, scenetextureinfo, sceneObjects
    std::vector<boost::tuple<std::string, std::string, std::string, std::string, std::string, std::vector<std::string> > > _sceneInfos;
    //! the input data, encoded as left, right, dispdata, point cloud
    std::vector<boost::tuple<std::string, std::string, std::string, std::string > > _sceneInput;
    //! the simulated data, encoded as wc, dwc, collisiondata
    std::vector<boost::tuple<std::string, std::string, std::string> > _sceneSimData;

    std::string fullpathBenchmark;

    std::map<std::string, size_t> _sidToIndex;
};


#endif /* GRASPTASK_HPP_ */
