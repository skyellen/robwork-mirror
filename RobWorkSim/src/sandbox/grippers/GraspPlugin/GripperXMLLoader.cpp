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
#include <boost/filesystem/path.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include "Gripper.hpp"
#include "XMLHelpers.hpp"

#define DEBUG cout



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rw::loaders;
using namespace boost::numeric;
using namespace boost::property_tree;
using namespace rwlibs::xml;



/*JawPrimitive::Ptr readJaw(PTree& tree)
{
	Q params = XMLHelpers::readQ(tree);
	
	params[5] *= Deg2Rad;
	if (params[0] == JawPrimitive::Prismatic) params[8] *= Deg2Rad;
	
	cout << params << endl;
	
	return ownedPtr(new JawPrimitive(params));
}*/



void readJaws(PTree& tree, Gripper::Ptr gripper, const std::string& path)
{
	boost::optional<PTree&> fileNode = tree.get_child_optional("File");
	if (fileNode) {
		// read jaw geometry from STL file
		string filename = path + (*fileNode).get_value<string>();
		DEBUG << "Jaw geometry from file: " << filename << endl;
		
		TriMesh::Ptr mesh = STLFile::load(filename);
		gripper->setJawGeometry(ownedPtr(new Geometry(mesh)));
		
		return;
	}
	
	// else use parametrization
	Q params = XMLHelpers::readQ(tree.get_child("Q"));
	params(5) *= Deg2Rad;
	params(8) *= Deg2Rad;
	if (params.size() == 11) { // to accomodate for cut tilt
		params(10) *= Deg2Rad;
	}
	
	DEBUG << "Jaw geometry from parameters: " << params << endl;
	gripper->setJawGeometry(params);
}



void readBase(PTree& tree, Gripper::Ptr gripper, const std::string& path)
{
	boost::optional<PTree&> fileNode = tree.get_child_optional("File");
	if (fileNode) {
		// read base geometry from STL file
		string filename = (*fileNode).get_value<string>();
		DEBUG << "Base geometry from file: " << filename << endl;
		
		TriMesh::Ptr mesh = STLFile::load(filename);
		gripper->setBaseGeometry(ownedPtr(new Geometry(mesh)));
		
		return;
	}
	
	// else use parametrization
	Q params = XMLHelpers::readQ(tree.get_child("Q"));
	DEBUG << "Base geometry from parameters: " << params << endl;
	gripper->setBaseGeometry(params);
}



void readGeometry(PTree& tree, Gripper::Ptr gripper, const std::string& path)
{
	readJaws(tree.get_child("Jaws"), gripper, path);
	readBase(tree.get_child("Base"), gripper, path);
}



void readParameters(PTree& tree, Gripper::Ptr gripper, const std::string& path)
{
	readGeometry(tree.get_child("Geometry"), gripper, path);
	
	double offset = XMLHelpers::readDouble(tree.get_child("Offset"));
	//gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, gripper->getJawParameters()[1]-offset)));
	gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, offset)));
	
	double jawdist = XMLHelpers::readDouble(tree.get_child("Jawdist"));
	double opening = XMLHelpers::readDouble(tree.get_child("Opening"));
	double force = XMLHelpers::readDouble(tree.get_child("Force"));
	
	gripper->setJawdist(jawdist);
	gripper->setOpening(opening);
	gripper->setForce(force);
	
	// it is also possible to dispense with Jawdist, and use Stroke, which overwrites it instead
	boost::optional<PTree&> strokeNode = tree.get_child_optional("Stroke");
	if (strokeNode) {
		double stroke = XMLHelpers::readDouble(strokeNode.get());
		gripper->setJawdist(opening - stroke);
	}
	
	DEBUG << "Offset: " << offset << endl;
	DEBUG << "Opening: " << opening << endl;
	DEBUG << "Force: " << force << endl;
}



void readResult(PTree& tree, Gripper::Ptr gripper, const std::string& path)
{
	GripperQuality& result = gripper->getQuality();
	
	result.nOfExperiments = XMLHelpers::readInt(tree.get_child("Experiments"));
	result.nOfSuccesses = XMLHelpers::readInt(tree.get_child("Successes"));
	result.nOfSamples = XMLHelpers::readInt(tree.get_child("Samples"));
	result.coverage = XMLHelpers::readDouble(tree.get_child("Coverage"));
	result.success = XMLHelpers::readDouble(tree.get_child("SuccessRatio"));
	result.wrench = XMLHelpers::readDouble(tree.get_child("Wrench"));
	result.topwrench = XMLHelpers::readDouble(tree.get_child("TopWrench"));
	result.quality = XMLHelpers::readDouble(tree.get_child("Quality"));
	
	// robustness is optional, because it was introduced recently,
	// and we want to maintain compatibility
	boost::optional<PTree&> robustnessNode = tree.get_child_optional("Robustness");
	if (robustnessNode) {
		result.robustness = XMLHelpers::readDouble(robustnessNode.get());
	} else {
		result.robustness = 0.0;
	}
	
	// maxstress is optional, because it was introduced recently,
	// and we want to maintain compatibility
	boost::optional<PTree&> stressNode = tree.get_child_optional("MaxStress");
	if (stressNode) {
		result.maxstress = XMLHelpers::readDouble(stressNode.get());
	} else {
		result.maxstress = 0.0;
	}
	
	// volume is optional
	boost::optional<PTree&> volumeNode = tree.get_child_optional("Volume");
	if (volumeNode) {
		result.volume = XMLHelpers::readDouble(volumeNode.get());
	} else {
		result.volume = 0.0;
	}
	
	// alignment is optional
	boost::optional<PTree&> alignNode = tree.get_child_optional("Alignment");
	if (alignNode) {
		result.alignment = XMLHelpers::readDouble(alignNode.get());
	} else {
		result.alignment = 0.0;
	}
	
	DEBUG << "Read gripper quality:" << endl;
	DEBUG << result << endl;
}



Gripper::Ptr readGripper(PTree& tree, const std::string& path)
{
	Gripper::Ptr gripper = ownedPtr(new Gripper);
	
	readParameters(tree.get_child("Parameters"), gripper, path);
	readResult(tree.get_child("Result"), gripper, path);
	
	return gripper;
}



rw::models::Gripper::Ptr GripperXMLLoader::load(const std::string& filename)
{
	Gripper::Ptr gripper;
	
    try {
		// get path
		boost::filesystem::path p(filename);
		string gripperPath = p.parent_path().string()+"/";
		
        PTree tree;
        read_xml(filename, tree);

        gripper = readGripper(tree.get_child("Gripper"), gripperPath);
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
	
	// save jaw geometry
	if (gripper->isJawParametrized()) {
		Q params = gripper->getJawParameters();
		params(5) *= Rad2Deg;
		params(8) *= Rad2Deg;
		if (params.size() == 11) { // when using cut tilt
			params(10) *= Rad2Deg;
		}
		tree.put("Gripper.Parameters.Geometry.Jaws.Q", XMLHelpers::QToString(params));
	} else {
		// save STL file
		boost::filesystem::path p(filename);
		string stlfile = p.parent_path().string()+"/jaw.stl";
		STLFile::save(*gripper->_leftGeometry->getGeometryData()->getTriMesh(), stlfile);
		tree.put("Gripper.Parameters.Geometry.Jaws.File", filename);
	}
	
	// save base geometry
	if (gripper->isBaseParametrized()) {
		tree.put("Gripper.Parameters.Geometry.Base.Q", XMLHelpers::QToString(gripper->getBaseParameters()));
	} else {
		// save STL file
		boost::filesystem::path p(filename);
		string stlfile = p.parent_path().string()+"/base.stl";
		STLFile::save(*gripper->_baseGeometry->getGeometryData()->getTriMesh(), stlfile);
		tree.put("Gripper.Parameters.Geometry.Base.File", filename);
	}
	
	//tree.put("Gripper.Parameters.Offset", gripper->getJawParameters()[1]-gripper->getTCP().P()[2]);
	tree.put("Gripper.Parameters.Offset", gripper->getTCP().P()[2]);
	tree.put("Gripper.Parameters.Jawdist", gripper->getJawdist());
	tree.put("Gripper.Parameters.Opening", gripper->getOpening());
	tree.put("Gripper.Parameters.Force", gripper->getForce());
	tree.put("Gripper.Parameters.Stroke", gripper->getOpening()-gripper->getJawdist());
	
	GripperQuality& q = gripper->getQuality();
	tree.put("Gripper.Result.Experiments", q.nOfExperiments);
	tree.put("Gripper.Result.Successes", q.nOfSuccesses);
	tree.put("Gripper.Result.Samples", q.nOfSamples);
	//tree.put("Gripper.Result.Shape", q->shape);
	tree.put("Gripper.Result.Coverage", q.coverage);
	tree.put("Gripper.Result.SuccessRatio", q.success);
	tree.put("Gripper.Result.Wrench", q.wrench);
	tree.put("Gripper.Result.TopWrench", q.topwrench);
	tree.put("Gripper.Result.Quality", q.quality);
	tree.put("Gripper.Result.Robustness", q.robustness);
	tree.put("Gripper.Result.MaxStress", q.maxstress);
	tree.put("Gripper.Result.Alignment", q.alignment);
	tree.put("Gripper.Result.Volume", q.volume);
	
	try {
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
        write_xml(filename, tree, std::locale(), settings);
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}
