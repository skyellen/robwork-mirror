#include <fstream>
#include <vector>
#include <string>
#include <stdio.h>

#include <rw/math/Vector3D.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/option.hpp>
#include <boost/program_options/parsers.hpp>

#include "util.hpp"

using namespace std;
using namespace rw::math;
using namespace rwlibs::algorithms;
using namespace rwlibs::task;
using namespace boost::program_options;

int calcPerturbedQuality(GraspTask::Ptr gtask, std::string outfile, int pertubations ){
    int count = 0, succCnt=0, failCnt=0;
    std::vector<std::pair<GraspSubTask*,GraspTarget*> > tasks = gtask->getAllTargets();
    GraspTask::Ptr ngtask = gtask->clone();
    for(size_t i = 0; i<tasks.size();i++){
        // save the target that the quality should be calculated for
        if(count==0){
            GraspSubTask nstask = tasks[i].first->clone();
            ngtask->addSubTask( nstask );
            ngtask->getSubTasks().back().addTarget( *tasks[i].second );
            count++;
            continue;
        }

        GraspResult::Ptr tres = tasks[i].second->getResult();
        if(tres->testStatus == GraspResult::Success || tres->testStatus == GraspResult::ObjectSlipped){
            succCnt++;
        } else {
            failCnt++;
        }

        if(count==pertubations){
            // set the quality of the target
            GraspResult::Ptr result = ngtask->getSubTasks().back().getTargets().back().getResult();

            double successProbability = ((double)succCnt)/((double)(succCnt+failCnt));
            result->qualityAfterLifting = Q(1, successProbability );
            succCnt = 0;
            failCnt = 0;
            count = 0;
        } else {
            count++;
        }
    }
    GraspTask::saveRWTask(ngtask, outfile);
    return 0;


}

std::vector<std::pair<Transform3D<>, RPY<> > > readPoses(std::string file){

    std::vector<std::pair<Transform3D<>, RPY<> > > data;
    char line[1000];
    std::ifstream in(file.c_str());
    in.getline(line,1000);
    in.getline(line,1000);
    in.getline(line,1000);

    //Unused: char tmpc;
    while(!in.eof() ){
        in.getline(line,1000);
        std::istringstream istr(line);
        Vector3D<float> pos_s, pos_e;
        EAA<float> rot_s, rot_e;
        RPY<float> rpy;

        sscanf(line,"%f %f %f %f %f %f %f  %f %f ",
               &pos_s[0], &pos_s[1], &pos_s[2], &rpy(0), &rpy(1), &rpy(2), &rot_s[0], &rot_s[1], &rot_s[2]);


        //std::cout << pos_e << std::endl;
        data.push_back( std::make_pair(Transform3D<>(cast<double>(pos_s),cast<double>(rot_s)), cast<double>(rpy) ) );

    }
    std::cout << "Size of intput: " << data.size() << std::endl;
    return data;
}

int main(int argc, char** argv)
{
    Math::seed(time(NULL));
    srand ( time(NULL) );

    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", value<string>()->default_value("out.xml"), "the output file.")
        ("stable", value<string>(), "Name of grasp task file.")
        ("misses", value<string>(), "Add small random pertubations to all targets.")
        ("angle", value<double>(), "angular standard deviation.")
        //("dist", value<double>(), "distance standard deviation.")
    ;
    positional_options_description optionDesc;
    optionDesc.add("input",-1);

    variables_map vm;
    //store(parse_command_line(argc, argv, desc), vm);
    store(command_line_parser(argc, argv).
              options(desc).positional(optionDesc).run(), vm);
    notify(vm);

	std::string stable_file = vm["stable"].as<std::string>();
	std::string misses_file = vm["misses"].as<std::string>();
    std::string output_file = vm["output"].as<std::string>();
    double angle_sd = vm["angle"].as<double>()*Deg2Rad;


    std::vector<std::pair<Transform3D<>,RPY<> > > stable = readPoses(stable_file);
    std::vector<std::pair<Transform3D<>,RPY<> > > misses = readPoses(misses_file);
    std::vector<double> qualityestimates( stable.size() );
    std::vector<double> qualityestimates_misses( misses.size() );
    // build the nodes
    typedef std::pair<int, bool> Value;
    typedef KDTreeQ<Value> NNSearch;
    std::vector<NNSearch::KDNode> nodes;
    for(std::size_t i=0;i<stable.size();i++){
        //Unused: Transform3D<> t3d = stable[i].first;
        //stable[i].second(1) = stable[i].second(1)+13*Deg2Rad;
        RPY<> rpy = stable[i].second;
        Q key(2, rpy(1), rpy(2));
        nodes.push_back( NNSearch::KDNode(key, Value(i, true)) );
    }
    for(std::size_t i=0;i<misses.size();i++){
    	//Unused: Transform3D<> t3d = misses[i].first;
        //misses[i].second(1) = misses[i].second(1)+13*Deg2Rad;
        RPY<> rpy = misses[i].second;
        Q key(2, rpy(1), rpy(2));
        nodes.push_back( NNSearch::KDNode(key, Value(i, false)) );
    }

    // create nodes for all successes
    //std::vector<KDTree<Pose6D<>, 6 >::KDNode> nodes;
    NNSearch *nntree = NNSearch::buildTree( nodes );


    // define the truncation
    Q diff(2, 2*angle_sd, 2*angle_sd);

    // now for each stable node predict its quality
    std::list<const NNSearch::KDNode*> result;
    result.clear();

    BOOST_FOREACH(NNSearch::KDNode &nn, nodes){
    //for(int i=0;i<stable.size();i++){
        result.clear();
        Transform3D<> t3d;
        RPY<> rpy ;
        Value nv = nn.value;
        if(nv.second){
            t3d = stable[nv.first].first;
            rpy = stable[nv.first].second;
        } else {
            t3d = misses[nv.first].first;
            rpy = misses[nv.first].second;
        }
        Q key(2, rpy(1), rpy(2));
        nntree->nnSearchRect(key-diff,key+diff, result);

        double sum = 0, sum1=0;
        double xo = rpy(1);
        double yo = rpy(2);

        //Unused: int N = result.size();
        BOOST_FOREACH(const NNSearch::KDNode* n, result ){
            Value v = n->value;
            RPY<> rpyo;
            if(v.second)
                rpyo = stable[v.first].second;
            else
                rpyo = misses[v.first].second;

            double x = rpyo(1);
            double y = rpyo(2);

            // calculate the
            double tmp_x = Math::sqr(x-xo)/(2*Math::sqr(angle_sd));
            double tmp_y = Math::sqr(y-yo)/(2*Math::sqr(angle_sd));


            double val = std::pow(2.718281828, -(tmp_x+tmp_y));
            //std::cout << val << " " <<  N << " y-yo" << y-yo << std::endl;
            //std::cout << val << " " <<  N << " x-xo" << x-xo << std::endl;
            if(v.second) // if o=1
                sum += val;
            sum1 += val;
        }


        double quality = sum/sum1;

        //std::cout << sum << " " << sum1 << " "<< quality <<  std::endl;
        if(nv.second){
            qualityestimates[nv.first] = quality;
        } else {
            qualityestimates_misses[nv.first] = quality;
        }
    }

    std::ofstream outf(output_file.c_str());

    for(std::size_t i=0;i<stable.size();i++){
        Transform3D<> t3d = stable[i].first;
        EAA<> eaa(t3d.R());
        RPY<> rpy = stable[i].second;
        outf << t3d.P()[0] << "\t" << t3d.P()[1] << "\t" << t3d.P()[2] << "\t";
        outf << rpy(0)*Rad2Deg << "\t" << rpy(1)*Rad2Deg << "\t" << rpy(2)*Rad2Deg << "\t";
        outf << qualityestimates[i] << "\n";
    }
    RW_WARN("1");
    for(std::size_t i=0;i<misses.size();i++){
        Transform3D<> t3d = misses[i].first;
        EAA<> eaa(t3d.R());
        RPY<> rpy = misses[i].second;
        outf << t3d.P()[0] << "\t" << t3d.P()[1] << "\t" << t3d.P()[2] << "\t";
        outf << rpy(0)*Rad2Deg << "\t" << rpy(1)*Rad2Deg << "\t" << rpy(2)*Rad2Deg << "\t";
        outf << qualityestimates_misses[i] << "\n";
    }

    outf.close();

	return 0;
}


