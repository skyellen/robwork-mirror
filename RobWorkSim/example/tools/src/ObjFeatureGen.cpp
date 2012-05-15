
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>

#include <vector>

#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rwsim/simulator/GraspTaskSimulator.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rwsim/loaders/DynamicWorkCellLoader.hpp>

#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>


#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>

#include <rwlibs/task/GraspTask.hpp>
#include <rwsim/util/SurfacePoseSampler.hpp>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;
using namespace rwsim::simulator;
using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwlibs::task;

GraspTask::Ptr generateTasks(int nrTasks, DynamicWorkCell::Ptr dwc, string objectName, string type);


class InputArchive {
public:
    virtual void open(const std::string& filename) = 0;
    virtual void open(std::istream& ifs) = 0;

};

class OutputArchive {
public:
    virtual void open(const std::string& filename) = 0;
    virtual void open(std::ostream& ifs) = 0;

};

class Serializer {
public:

};

class ClassFactory {
public:
    virtual std::vector<std::string> supportedClassTypes() = 0;
    virtual boost::any load(Archive& archive, const std::string& type, int version) = 0;

    template<class T>
    T* load(Archive& archive){
        boost::any data = load(archive, typeid(T));
        return boost::any_cast<T>(data);
    }

};

class Serializable {
public:
    virtual void serialize(Archive& archive) = 0;
    rw::common::Ptr<Deserializer> makeDeserializerFactory();
};


void saveKDTree(KDTreeQ* tree){
    std::vector<KDTreeQ::Node>
}

KDTreeQ* loadTree(const std::string& filename){
    XMLArchive archive(filename);
    return ClassFactory::load<KDTreeQ>(archive);
}

int main(int argc, char** argv)
{
    // we need
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input,i", value<string>(), "The object geometry that is to be sampled.")
        ("points", value<int>()->default_value(1000), "The nr of points to represent the object.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    rw::math::Math::seed( TimerUtil::currentTimeMs() );
    // write standard welcome, status
    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] -o<outfile> <expFile1> <expFile2> <...> <expFileN> \n"
                  << "\n";
        cout << desc << "\n";
        return 1;
    }

    using namespace boost::filesystem;


    typedef boost::tuple<int,int,int> KD_VALUE_TYPE;

    // geet geometry file
    std::string geomfile = vm["input"].as<std::string>();
    // get nr of samples
    int nrPoints = vm["points"].as<int>();
    Timer timer;
    timer.resetAndResume();

    std::cout << "Loading geometry." << std::endl;
    Geometry::Ptr geom = GeometryFactory::load( geomfile );
    //TriMesh::Ptr mesh = geom->getGeometryData()->getTriMesh();

    std::cout << "--duration: " << timer.getTime() << "s" << std::endl; timer.resetAndResume();
    std::cout << "Computing random points on surface of geometry."<< std::endl;


    typedef std::pair<Vector3D<>,Vector3D<> > VALUE;
    std::vector<std::pair<Vector3D<>,Vector3D<> > > pointsOnSurf(nrPoints); // pos,normal
    SurfacePoseSampler ssurf( geom );
    ssurf.setRandomRotationEnabled(false);
    ssurf.setRandomPositionEnabled(false); // the z-axis of rot is aligned with tri normal
    for(int i=0;i<nrPoints;i++){
        Transform3D<> t3d = ssurf.sample();
        pointsOnSurf[i].first = t3d.P();
        pointsOnSurf[i].second = t3d.R() * Vector3D<>::z();
    }

    std::cout << "--duration: " << timer.getTime() << "s" << std::endl; timer.resetAndResume();
    std::cout << "Computing features of point combinations. nrPoints:" << nrPoints << std::endl;

    // now we create the database of features
    std::vector<KDTreeQ::KDNode> nodes;
    const double minDist = 0.02, maxDist = 0.5;
    int nrOffeatures_initial = 0;
    // we need to traverse all combinations of points and calculate features between them
    for(int i=0;i<nrPoints;i++){
        for(int j=i+1;j<nrPoints;j++){
            // calculate features between the two points

            double dist = MetricUtil::dist2(pointsOnSurf[i].first, pointsOnSurf[j].first);
            //std::cout << dist << std::endl;
            // we throw away close points
            if(dist<minDist || maxDist<dist)
                continue;

            Vector3D<> p2p = pointsOnSurf[i].first - pointsOnSurf[j].first;
            double ang1 = angle( pointsOnSurf[i].second, p2p);
            double ang2 = angle( pointsOnSurf[j].second, -p2p);

            Q key(3, dist, ang1, ang2);

            // now add it to the node list,
            nodes.push_back( KDTreeQ::KDNode(key, KD_VALUE_TYPE(i,j,0) ) );

            nrOffeatures_initial++;
        }
    }

    std::cout << "--duration: " << timer.getTime() << "s" << std::endl; timer.resetAndResume();
    std::cout << "Building KDTree on data of size: " << nrOffeatures_initial << std::endl;

    KDTreeQ *nntree = KDTreeQ::buildTree(nodes);

    std::cout << "--duration: " << timer.getTime() << "s" << std::endl; timer.resetAndResume();
    std::cout << "Computing quality per feature,"<< std::endl;

    int maxNeigh = 0, minNeigh=1000000, currentNode=0;
    long avgNeighSum = 0;
    std::list<const KDTreeQ::KDNode*> result;
    Q diff(3, 0.005, 10*Deg2Rad, 10*Deg2Rad);
    BOOST_FOREACH( KDTreeQ::KDNode& node , nodes){
        result.clear();
        //nntree->
        Q key = node.key;
        nntree->nnSearchRect(key-diff,key+diff, result);
        size_t nrNeighbors = result.size();
        if(maxNeigh<nrNeighbors)
            maxNeigh = nrNeighbors;
        if(minNeigh>nrNeighbors)
            minNeigh = nrNeighbors;
        avgNeighSum += nrNeighbors;

        KD_VALUE_TYPE value = boost::any_cast<KD_VALUE_TYPE>(node.value);
        value.get<2>() = nrNeighbors;
        node.value = value;

        std::cout << "\t" << (int)((currentNode/(1.0*nrOffeatures_initial))*100.0) << "\%\t" << nrNeighbors << "\r";
        currentNode++;
    }
    double avgNeigh = avgNeighSum/(1.0*nrOffeatures_initial);
    std::cout << "--duration: " << timer.getTime() << "s\t" << std::endl; timer.resetAndResume();

    std::cout << "\tmaxNeigh: " << maxNeigh
              << "\n\tminNeigh: " << minNeigh
              << "\n\tavgNeigh: " << avgNeigh << std::endl;


    std::vector<int> buckets(51,0);
    double scaleVal = maxNeigh/50.0;
    // create 50 buckets for the data
    BOOST_FOREACH( KDTreeQ::KDNode& node , nodes){
        KD_VALUE_TYPE value = boost::any_cast<KD_VALUE_TYPE>(node.value);
        int bucket_idx = 0;


        bucket_idx = std::floor( value.get<2>()/scaleVal);

        if(bucket_idx>50)
            bucket_idx = 50;
        buckets[bucket_idx]++;
    }

    for(int i=0;i<51; i++){
        std::cout << i << " ; " << buckets[i] << std::endl;
    }


    // next we reduce the features by thresholding the max neigh count at maxNeigh_init/4
    std::vector<KDTreeQ::KDNode> nodes_reduced;
    BOOST_FOREACH( KDTreeQ::KDNode& node , nodes){
        KD_VALUE_TYPE value = boost::any_cast<KD_VALUE_TYPE>(node.value);
        if( value.get<2>() < 200 ){
            nodes_reduced.push_back( node );
        } else if( value.get<2>() > 500 ) {
            // these are the redundant nodes, we should do something with these
        }
    }

    KDTreeQ *nntree_reduced = KDTreeQ::buildTree(nodes_reduced);
    int intree = 0, intree_reduced = 0, nrtofar = 0;
    // do random sampling of features and check if the features can
    // be found in the complete set and in the reduced set
    for(int i=0;i<20000;i++){

        Transform3D<> t3d = ssurf.sample();
        Vector3D<> p1 = t3d.P();
        Vector3D<> n1 = t3d.R() * Vector3D<>::z();

        t3d = ssurf.sample();
        Vector3D<> p2 = t3d.P();
        Vector3D<> n2 = t3d.R() * Vector3D<>::z();

        Vector3D<> p2p = p2 - p1;
        double ang1 = angle( n1, p2p);
        double ang2 = angle( n2, -p2p);

        double dist = MetricUtil::dist2(p1,p2);
        //std::cout << dist << std::endl;
        // we throw away close points
        if(dist<minDist || maxDist<dist){
            nrtofar++;
            continue;
        }


        Q key(3, dist, ang1, ang2);
        KDTreeQ::KDNode& nnode =  nntree->nnSearch(key);

        // check if the keys are close
        Q diff = nnode.key-key;
        if( fabs(diff(0))<0.01 && fabs(diff(1))<0.01 && fabs(diff(2))<0.01){
            intree++;
        }

        KDTreeQ::KDNode& nnode_reduced =  nntree_reduced->nnSearch(key);

        // check if the keys are close
        diff = nnode_reduced.key-key;
        if( fabs(diff(0))<0.01 && fabs(diff(1))<0.01 && fabs(diff(2))<0.01){
            intree_reduced++;
        }
    }
    std::cout << "Hits: " << 20000 << " : " << intree << " : " << intree_reduced << " : " << nrtofar << std::endl;

    // save the complete and reduced sets
    nnode_reduced->save("nntree_reduced.txt");
    nntree->save("nntree");

    std::cout << "Done" << std::endl;
    return 0;
}

