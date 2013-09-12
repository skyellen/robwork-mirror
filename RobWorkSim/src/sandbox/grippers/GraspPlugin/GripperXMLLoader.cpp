#include "GripperXMLLoader.hpp"

#include <string>
#include <sstream>
#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include "Gripper.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace boost::numeric;
using namespace boost::property_tree;



typedef boost::property_tree::ptree PTree;
typedef PTree::iterator CI;


std::pair<bool, double> toDouble(const std::string& str)
{
	std::pair<bool, double> nothing(false, 0);
	istringstream buf(str);
	double x;
	buf >> x;
	if (!buf) return nothing;
	string rest;
	buf >> rest;
	if (buf) return nothing;
	else return make_pair(true, x);
}



std::pair<bool, int> toInt(const std::string& str)
{
	std::pair<bool, int> nothing(false, 0);
	istringstream buf(str);
	int x;
	buf >> x;
	if (!buf) return nothing;
	string rest;
	buf >> rest;
	if (buf) return nothing;
	else return make_pair(true, x);
}



std::vector<double> readArray(PTree& tree){
	//RW_DEBUGS( "ReadArray: " << tree.get_value<string>() );
	istringstream buf(tree.get_value<string>());
	std::vector<double> values;

	std::string str;
	while( buf >> str ){
		//cout << str << endl;
		const pair<bool, double> okNum = toDouble(str);
		if (!okNum.first)
			RW_THROW("Number expected. Got \"" << str << "\" ");
		values.push_back(okNum.second);
	}
	return values;
}



double readInt(PTree& tree)
{
	string str = tree.get_value<string>();
	pair<bool, int> okNum = toInt(str);
	
	if (!okNum.first)
		RW_THROW("Number expected. Got \"" << str << "\" ");
		
	return okNum.second;
}



double readDouble(PTree& tree)
{
	string str = tree.get_value<string>();
	pair<bool, double> okNum = toDouble(str);
	
	if (!okNum.first)
		RW_THROW("Number expected. Got \"" << str << "\" ");
		
	return okNum.second;
}



Q readQ(PTree& tree){
	//Log::debugLog()<< "ReadQ" << std::endl;
	std::vector<double> arr = readArray(tree);
	Q q(arr.size());
	for(size_t i=0;i<q.size();i++){
		q[i] = arr[i];
	}
	return q;
}



JawPrimitive::Ptr readJaw(PTree& tree)
{
	Q params = readQ(tree);
	
	params[4] *= Deg2Rad;
	params[7] *= Deg2Rad;
	
	//cout << params << endl;
	
	return ownedPtr(new JawPrimitive(params));
}



void readGeometry(PTree& tree, Gripper::Ptr gripper)
{
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Jaw") {
			gripper->setGeometry(readJaw(p->second));
			
			//cout << "Reading jaw..." << p->second.get_value<string>() << endl;
		}
		
		else if (p->first == "File") {
			// reading in geometry from file HERE
			RW_THROW("Reading geometry from file is not supported yet.");
		}
	}
}



void readParameters(PTree& tree, Gripper::Ptr gripper)
{
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Geometry") {
			readGeometry(p->second, gripper);
		}
		
		else if (p->first == "Offset") {
			double offset = readDouble(p->second);
			gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, offset)));
		}
		
		else if (p->first == "Force") {
			double force = readDouble(p->second);
			gripper->setForce(force);
		}
	}
}



void readResult(PTree& tree, Gripper::Ptr gripper)
{
	GripperQuality::Ptr result = gripper->getQuality();
	
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Experiments") {
			result->nOfExperiments = readInt(p->second);
		}
		
		else if (p->first == "Shape") {
			result->shapeQ = readDouble(p->second);
		}
		
		else if (p->first == "Coverage") {
			result->coverageQ = readDouble(p->second);
		}
		
		else if (p->first == "SuccessRatio") {
			result->successQ = readDouble(p->second);
		}
		
		else if (p->first == "WSM") {
			result->wrenchQ = readDouble(p->second);
		}
		
		else if (p->first == "Interference") {
			result->interferenceQ = readDouble(p->second);
		}
		
		else if (p->first == "Quality") {
			result->finalQ = readDouble(p->second);
		}
	}
}



Gripper::Ptr readGripper(PTree& tree)
{
	Gripper::Ptr gripper = ownedPtr(new Gripper);
	
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Parameters") {
			readParameters(p->second, gripper);
		}
		
		else if (p->first == "Result") {
			readResult(p->second, gripper);
		}
	}
	
	return gripper;
}



rw::models::Gripper::Ptr GripperXMLLoader::load(const std::string& filename)
{
	Gripper::Ptr gripper;
	
    try {
        PTree tree;
        read_xml(filename, tree);

        gripper = readGripper(tree.get_child("Gripper"));
        string name = tree.get_child("Gripper").get_child("<xmlattr>.name").get_value<string>();
        gripper->setName(name);        
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
    
    return gripper;
}



void GripperXMLLoader::save(rw::models::Gripper::Ptr gripper, const std::string& filename)
{
	PTree tree;
	
	tree.put("Gripper.<xmlattr>.name", gripper->getName());
	
	tree.put("Gripper.Parameters.Geometry.Jaw", gripper->getGeometry()->toString());
	tree.put("Gripper.Parameters.Offset", gripper->getTCP().P()[2]);
	tree.put("Gripper.Parameters.Jawdist", gripper->getJawdist());
	tree.put("Gripper.Parameters.Opening", gripper->getOpening());
	tree.put("Gripper.Parameters.Force", gripper->getForce());
	
	GripperQuality::Ptr q = gripper->getQuality();
	tree.put("Gripper.Result.Experiments", q->nOfExperiments);
	tree.put("Gripper.Result.Shape", q->shapeQ);
	tree.put("Gripper.Result.Coverage", q->coverageQ);
	tree.put("Gripper.Result.SuccessRatio", q->successQ);
	tree.put("Gripper.Result.WSM", q->wrenchQ);
	tree.put("Gripper.Result.Interference", q->interferenceQ);
	tree.put("Gripper.Result.Quality", q->finalQ);
	
	try {
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
        write_xml(filename, tree, std::locale(), settings);
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}
