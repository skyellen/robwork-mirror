#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>

#include <vector>
#include <rwlibs/task/GraspTask.hpp>
#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/math/Vector3D.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>
#include <rw/math/LinearAlgebra.hpp>

#include <rwsim/dynamics/DynamicUtil.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rw/geometry/GeometryFactory.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>
#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <iterator>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;
using namespace boost::program_options;

variables_map init(int argc, char** argv){
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("angle", value<double>()->default_value(8.0), "Standard deviation of angle in input in degree.")
        ("dist", value<double>()->default_value(0.003), "Standard deviation of distance in input.")
        ("input", value<string>(), "The input grasp task file that should be filtered.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

    // write standard welcome, status
    if (vm.count("help")) {
        cout << "Usage:\n\n"
                  << "\t" << argv[0] <<" [options] -o<outfile> <expFile1> <expFile2> <...> <expFileN> \n"
                  << "\n";
        cout << desc << "\n";
        abort();
    }
    return vm;
}

KDTreeQ* buildKDTree_normal(GraspTask::Ptr gtask);
KDTreeQ* buildKDTree(GraspTask::Ptr gtask, std::vector<KDTreeQ::KDNode>& simnodes);

int main_jaj(int argc, char** argv);
int main_lpe(int argc, char** argv);



rw::common::Ptr< std::map<int,std::vector<int> > > calculateRegions(const std::vector<Transform3D<> >& data, double dist, double angle){

    // now build a kdtree with all end configurations
    typedef boost::tuple<int,int> KDTreeValue; // (transform index, region index)
    std::vector<KDTreeQ::KDNode> nodes;
    for(size_t i=0;i<data.size();i++){
        Transform3D<> k = data[i];
        EAA<> r( k.R());
        //Vector3D<> r = k.R()*Vector3D<>::z();
        Q key(6, k.P()[0],k.P()[1],k.P()[2], r[0],r[1],r[2]);
        nodes.push_back(KDTreeQ::KDNode(key, KDTreeValue(i, -1)));
    }

    std::cout << "Nodes created, building tree.. " << std::endl;
    KDTreeQ* nntree = KDTreeQ::buildTree(nodes);
    // todo estimate the average distance between neighbors
    std::cout << "Tree build, finding regions" << std::endl;
    std::map<int, bool > regions;
    int freeRegion = 0;
    std::list<const KDTreeQ::KDNode*> result;
    Q diff(6, dist, dist, dist, angle, angle, angle);
    // find neighbors and connect them
    BOOST_FOREACH(KDTreeQ::KDNode &n, nodes){
        KDTreeValue &val = n.valueAs<KDTreeValue&>();
        // check if the node is allready part of a region
        if(val.get<1>() >=0)
            continue;

        result.clear();
        nntree->nnSearchRect(n.key-diff, n.key+diff, result);
        int currentIndex = -1;
        // first see if any has an id
        BOOST_FOREACH(const KDTreeQ::KDNode* nn, result){
            KDTreeValue nnval = nn->valueAs<KDTreeValue>();
            if(nnval.get<1>() >=0){
                currentIndex = nnval.get<1>();
                break;
            }
        }
        if( currentIndex<0 ){
            currentIndex=freeRegion;
            //std::cout << "Adding " << freeRegion << std::endl;
            regions[freeRegion] = true;
            freeRegion++;
        }
        val.get<1>() = currentIndex;
        BOOST_FOREACH(const KDTreeQ::KDNode* nn, result){
            KDTreeValue nnval = nn->valueAs<KDTreeValue>();
            if(nnval.get<1>() >=0 && nnval.get<1>()!=currentIndex){

                //std::cout << "Merging regions " << currentIndex << "<--" << nnval.get<1>() << std::endl;

                // merge all previously defined nnval.get<1>() into freeRegion
                regions[nnval.get<1>()] = false;
                BOOST_FOREACH(KDTreeQ::KDNode &npro, nodes){
                    KDTreeValue &npval = npro.valueAs<KDTreeValue&>();
                    // check if the node is allready part of a region
                    if(npval.get<1>() == nnval.get<1>())
                        npval.get<1>()=currentIndex;
                }
            }
        }

    }

    // now print region information
    std::vector<int> validRegions;
    typedef std::map<int,bool>::value_type mapType;
    BOOST_FOREACH(mapType val , regions){
        if(val.second==true){
            validRegions.push_back(val.first);
        }
    }


    std::cout << "Nr of detected regions: " << validRegions.size() << std::endl;
    std::map<int,std::vector<int> >* statMap = new std::map<int,std::vector<int> >();

    BOOST_FOREACH(KDTreeQ::KDNode &n, nodes){
        KDTreeValue &val = n.valueAs<KDTreeValue&>();
        // check if the node is allready part of a region
        (*statMap)[val.get<1>()].push_back( val.get<0>() );
    }
    return ownedPtr( statMap );
}



int main(int argc, char** argv){

    Math::seed(time(NULL));
    srand ( time(NULL) );
    variables_map vm = init(argc, argv);


    double angle = vm["angle"].as<double>()*Deg2Rad;
    double dist = vm["dist"].as<double>();

    std::string input = vm["input"].as<std::string>();
    std::string output = vm["output"].as<std::string>();
    char line[1000];
    std::vector<Transform3D<> > inputStartCfgs, inputEndCfgs;
    std::ifstream in(input.c_str());
    in.getline(line,1000);
    in.getline(line,1000);
    char tmpc;
    while(!in.eof() ){
        in.getline(line,400);
        std::istringstream istr(line);
        Vector3D<float> pos_s, pos_e;
        EAA<float> rot_s, rot_e;

        sscanf(line,"%f %c %f %c %f %c %f %c %f %c %f %c %f %c %f %c %f %c %f %c %f %c %f",
               &pos_s[0], &tmpc, &pos_s[1], &tmpc, &pos_s[2], &tmpc, &rot_s[0], &tmpc, &rot_s[1], &tmpc, &rot_s[2], &tmpc,
               &pos_e[0], &tmpc, &pos_e[1], &tmpc, &pos_e[2], &tmpc, &rot_e[0], &tmpc, &rot_e[1], &tmpc, &rot_e[2]);

        if(pos_e[2]<0)
            continue;

        //std::cout << pos_e << std::endl;
        inputStartCfgs.push_back( cast<double>(Transform3D<float>(pos_s, rot_s)));
        inputEndCfgs.push_back( cast<double>(Transform3D<float>(pos_e, rot_e)));
        if( rot_e.angle() > 180*Deg2Rad )
            std::cout << "larger" << std::endl;
    }
    std::cout << "Size of intput: " << inputStartCfgs.size() << std::endl;

    Ptr< std::map<int, std::vector<int> > > statMap = calculateRegions(inputEndCfgs, dist, angle);

    std::cout << "Map stats: " << std::endl;
    std::ofstream fstr(output.c_str());
    fstr.precision(16);
    fstr << " added region, first int in each line represents the region in witch it belong\n";
    fstr << " \n";
    typedef std::map<int,std::vector<int> >::value_type mapType2;
    BOOST_FOREACH(mapType2 val, *statMap){
        if(val.second.size() > 4)
            std::cout << "Region stat: " << val.first << ":" << val.second.size() ;
        Vector3D<> dir(0,0,0);
        double angle = 0;
        int count = 0;

        BOOST_FOREACH(int idx, val.second){
            count ++;
            Transform3D<> s = inputStartCfgs[ idx ];
            EAA<> sr( s.R() );
            Vector3D<> srz = s.R()*Vector3D<>::z();
            Transform3D<> e = inputEndCfgs[ idx ];
            EAA<> er( e.R() );
            Vector3D<> erz = e.R()*Vector3D<>::z();
            dir += er.axis();
            angle += er.angle();

            Transform3D<> aTb = inverse( s )*e;
            Vector3D<> abp = aTb.P();
            EAA<> abe(aTb.R() );
            RPY<> abr(aTb.R() );
            Vector3D<> aby = aTb.R()*Vector3D<>::z();

            fstr << val.first << " ; "
                    << s.P()[0] << " ; " << s.P()[1] << " ; " << s.P()[2] << ";"
                    << sr[0] << " ; " << sr[1] << " ; " << sr[2] << " ; "
                    //<< sr.axis()[0] << " ; " << sr.axis()[1] << " ; " << sr.axis()[2] << " ; " << sr.angle() << ";"

                    //<< srz[0] << " ; " << srz[1] << " ; " << srz[2] << " ; " << sr.angle() << ";"
                    << e.P()[0] << " ; " << e.P()[1] << " ; " << e.P()[2] << ";"
                    << er[0] << " ; " << er[1] << " ; " << er[2] << ";"
                    //<< er.axis()[0] << " ; " << er.axis()[1] << " ; " << er.axis()[2] << ";" << er.angle() << ";"
                    << abp[0] << " ; " << abp[1] << " ; " << abp[2] << ";"
                    << abe[0] << " ; " << abe[1] << " ; " << abe[2] << ";"
                    << abr(0) << " ; " << abr(1) << " ; " << abr(2) << ";"
                    << aby[0] << " ; " << aby[1] << " ; " << aby[2] << ";"
                    << "\n";
                    //<< erz[0] << " ; " << erz[1] << " ; " << erz[2] << ";" << er.angle() << "\n";
        }
        if(val.second.size() > 4){
            EAA<> rot( normalize(dir), angle/count);

            std::cout << "\t" << rot.toRotation3D()*Vector3D<>::z() << std::endl;

        }
    }
    fstr.close();

    return 0;
}

/**
 * We need to choose a number of grasps
 */
int main_lpe(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );
    variables_map vm = init(argc, argv);

    std::string grasptask_file_out = vm["output"].as<std::string>();
    double angleThres = vm["angle"].as<double>()*Deg2Rad;
    std::string input = vm["input"].as<string>();
    std::string output = vm["output"].as<string>();
    int count = vm["samples"].as<int>();
    GraspTask::Ptr gtask = GraspTask::load( input );

    // first we build a search tree to efficiently search for targets in 6d
    std::vector<KDTreeQ::KDNode> simnodes;
    KDTreeQ *nntree = buildKDTree(gtask,simnodes);


    Q diff(7, 0.01, 0.01, 0.01, angleThres, angleThres, angleThres,angleThres);
    typedef std::pair<GraspSubTask*, GraspTarget*> Value;
    std::vector< Value > selGrasps;
    std::list<const KDTreeQ::KDNode*> result;

    for(int i=0;i<simnodes.size(); i++){
        // find the node with the highest quality
        double q_max = -100;
        KDTreeQ::KDNode *n_max = &simnodes[0];
        BOOST_FOREACH(KDTreeQ::KDNode& n, simnodes){
            Value &val = n.valueAs<Value&>();
            if(val.second==NULL)
                continue;
            if(val.second->getResult()==NULL)
                continue;
            Q qual = val.second->getResult()->qualityAfterLifting;
            if(qual.size()==0)
                continue;
            if(q_max<qual[0]){
                q_max = qual[0];
                n_max = &n;
            }
        }
        if(q_max<-90)
            break;
        // we stop the sampling at a certain threshold 30% success
        if(q_max<0.3)
            break;

        // add the max node
        Value &v = n_max->valueAs<Value&>();
        selGrasps.push_back( v );
        v.second = NULL;

        // use the max node as a sample point and remove all neighboring nodes
        result.clear();
        nntree->nnSearchRect( n_max->key-diff, n_max->key+diff, result);
        BOOST_FOREACH(const KDTreeQ::KDNode* res_n, result){
            KDTreeQ::KDNode* r_n = (KDTreeQ::KDNode*) res_n;
            r_n->valueAs<Value&>().second = NULL;
        }
    }
    std::cout << "GENRATED: "<< selGrasps.size() << " from " << simnodes.size() << std::endl;

    // convert selectGrasps to a graspTask
    GraspTask res = *gtask;
    res.getSubTasks().clear();
    BOOST_FOREACH(Value& taskpair, selGrasps){
        GraspSubTask stask = *taskpair.first;
        stask.targets.clear();
        stask.targets.push_back( *taskpair.second );
        res.addSubTask(stask);
    }

    GraspTask::saveRWTask(&res, output + ".rwtask.xml");
    //std::cout << "COVERAGE: " << (((double)samples_t)/(double)samples_f)*100.0 << "%" << std::endl;
    return 0;
}

/**
 * We need to choose a number of grasps
 */
int main_jaj(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );
    variables_map vm = init(argc, argv);

	std::string grasptask_file_out = vm["output"].as<std::string>();
	double angleThres = vm["angle"].as<double>()*Deg2Rad;
    std::string input = vm["input"].as<string>();
    std::string output = vm["output"].as<string>();
    int count = vm["samples"].as<int>();
	GraspTask::Ptr gtask = GraspTask::load( input );

	// first we build a search tree to efficiently search for targets orientet in a certain direction
	KDTreeQ *nntree_normal = buildKDTree_normal(gtask);

	// now we sample the normals with random orientations
	Q diff(4, angleThres, angleThres, angleThres, angleThres);
	typedef std::pair<GraspSubTask*, GraspTarget*> Value;

	std::vector< Value > selGrasps;
	std::list<const KDTreeQ::KDNode*> result;
	int samples_f = 0, samples_t = 0;
	while(selGrasps.size()<count){
        result.clear();
        EAA<> eaa( Math::ranRotation3D<double>() );
        //Vector3D<> k = Math::ranRotation3D<double>()*Vector3D<>::z();
        Vector3D<> k = eaa.axis();
        Q key(3, k[0], k[1], k[2], eaa.angle());
        nntree_normal->nnSearchRect(key-diff,key+diff, result);
        samples_f++;
        if(result.size()==0)
            continue;
        samples_t++;
        int idx = Math::ranI(0,result.size());
        if(idx == result.size())
            idx--;
        std::list<const KDTreeQ::KDNode*>::iterator i = result.begin();
        //std::cout << idx << "==" <<  result.size() << std::endl;
        if(idx>0)
            std::advance(i, idx);
        const KDTreeQ::KDNode* node = *i;
        Value v = node->valueAs<Value>();
        selGrasps.push_back( v );
        std::cout << result.size() << "    \r" << std::flush;
	}
	// print all failures

	// convert selectGrasps to a graspTask
	GraspTask res = *gtask;
	res.getSubTasks().clear();
	BOOST_FOREACH(Value& taskpair, selGrasps){
	    GraspSubTask stask = *taskpair.first;
	    stask.targets.clear();
	    stask.targets.push_back( *taskpair.second );
	    res.addSubTask(stask);
	}

	GraspTask::saveRWTask(&res, output + ".rwtask.xml");
	std::cout << "COVERAGE: " << (((double)samples_t)/(double)samples_f)*100.0 << "%" << std::endl;
	return 0;
}


KDTreeQ* buildKDTree_normal(GraspTask::Ptr gtask) {
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    std::vector<KDTreeQ::KDNode> *simnodes = new std::vector<KDTreeQ::KDNode>();
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;
            //Vector3D<> n = t3d.R()*Vector3D<>::z();
            EAA<> eaa( t3d.R() );
            Vector3D<> n = eaa.axis();
            Q key(4, n[0], n[1], n[2], eaa.angle());
            simnodes->push_back( KDTreeQ::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return KDTreeQ::buildTree(*simnodes);
}

KDTreeQ* buildKDTree(GraspTask::Ptr gtask, std::vector<KDTreeQ::KDNode>& simnodes) {
    // we need the simulated grasps to attach a quality to the experiments
    // first we build a NN-structure for efficient nearest neighbor search
    BOOST_FOREACH(GraspSubTask& stask, gtask->getSubTasks()){
        BOOST_FOREACH(GraspTarget& target,stask.targets ){
            Transform3D<> t3d = target.pose;
            Vector3D<> p = t3d.P();
            //Vector3D<> n = t3d.R()*Vector3D<>::z();
            EAA<> eaa( t3d.R() );
            Vector3D<> n = eaa.axis();

            Q key(7, p[0], p[1], p[2], n[0], n[1], n[2], eaa.angle() );
            simnodes.push_back( KDTreeQ::KDNode(key, std::pair<GraspSubTask*,GraspTarget*>(&stask,&target)) );
        }
    }
    return KDTreeQ::buildTree(simnodes);
}

