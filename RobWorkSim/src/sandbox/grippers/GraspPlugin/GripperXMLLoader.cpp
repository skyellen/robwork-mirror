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
#include "XMLHelpers.hpp"



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace boost::numeric;
using namespace boost::property_tree;
using namespace rwlibs::xml;



JawPrimitive::Ptr readJaw(PTree& tree)
{
	Q params = XMLHelpers::readQ(tree);
	
	params[5] *= Deg2Rad;
	if (params[0] == JawPrimitive::Prismatic) params[8] *= Deg2Rad;
	
	cout << params << endl;
	
	return ownedPtr(new JawPrimitive(params));
}



void readGeometry(PTree& tree, Gripper::Ptr gripper)
{
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Jaw") {
			//gripper->setGeometry(readJaw(p->second));
			
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
			double offset = XMLHelpers::readDouble(p->second);
			gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, offset)));
		}
		
		else if (p->first == "Force") {
			double force = XMLHelpers::readDouble(p->second);
			gripper->setForce(force);
		}
	}
}



void readTasks(PTree& tree, Gripper::Ptr gripper)
{
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "File") {
			//gripper->setDataFilename(p->second.get_value<string>());
			//gripper->loadTasks(gripper->getDataFilename());
		}
	}
}



void readResult(PTree& tree, Gripper::Ptr gripper)
{
	GripperQuality::Ptr result = gripper->getQuality();
	
	for (CI p = tree.begin(); p != tree.end(); ++p) {
		if (p->first == "Experiments") {
			//result->nOfExperiments = XMLHelpers::readInt(p->second);
		}
		
		else if (p->first == "Shape") {
			//result->shapeQ = XMLHelpers::readDouble(p->second);
		}
		
		else if (p->first == "Coverage") {
			//result->coverageQ = XMLHelpers::readDouble(p->second);
		}
		
		else if (p->first == "SuccessRatio") {
			//result->successQ = XMLHelpers::readDouble(p->second);
		}
		
		else if (p->first == "WSM") {
			//result->wrenchQ = XMLHelpers::readDouble(p->second);
		}
		
		else if (p->first == "Interference") {
			//result->interferenceQ = XMLHelpers::readDouble(p->second);
		}
		
		else if (p->first == "Quality") {
			//result->finalQ = XMLHelpers::readDouble(p->second);
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
		
		else if (p->first == "Tasks") {
			readTasks(p->second, gripper);
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



void GripperXMLLoader::save(rw::models::Gripper::Ptr gripper, const std::string& dir, const std::string& filename)
{
	PTree tree;
	
	tree.put("Gripper.<xmlattr>.name", gripper->getName());
	
	tree.put("Gripper.Parameters.Geometry.Jaw", gripper->getGeometry()->toString());
	tree.put("Gripper.Parameters.Offset", gripper->getTCP().P()[2]);
	tree.put("Gripper.Parameters.Jawdist", gripper->getJawdist());
	tree.put("Gripper.Parameters.Opening", gripper->getOpening());
	tree.put("Gripper.Parameters.Force", gripper->getForce());
	
	//tree.put("Gripper.Tasks.File", dir+'/'+gripper->getDataFilename());
	
	GripperQuality::Ptr q = gripper->getQuality();
	/*tree.put("Gripper.Result.Experiments", q->nOfExperiments);
	tree.put("Gripper.Result.Shape", q->shapeQ);
	tree.put("Gripper.Result.Coverage", q->coverageQ);
	tree.put("Gripper.Result.SuccessRatio", q->successQ);
	tree.put("Gripper.Result.WSM", q->wrenchQ);
	tree.put("Gripper.Result.Interference", q->interferenceQ);
	tree.put("Gripper.Result.Quality", q->finalQ);*/
	
	try {
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
        write_xml(dir+'/'+filename, tree, std::locale(), settings);
        //cout << "Saving tasks to: " << dir+'/'+gripper->getDataFilename() << endl;
        //gripper->saveTasks(dir+'/'+gripper->getDataFilename());
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}
