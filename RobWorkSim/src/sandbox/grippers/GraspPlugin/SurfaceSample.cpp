/**
 * @file SurfaceSample.hpp
 * @author Adam Wolniakowski
 */
 
#include "SurfaceSample.hpp"

#include <iostream>
#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "XMLHelpers.hpp"



using namespace std;

USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rwsim::dynamics;

using namespace boost::numeric;
using namespace boost::property_tree;
using boost::algorithm::trim;

using namespace rwlibs::xml;

 


vector<SurfaceSample> SurfaceSample::loadFromXML(const std::string& filename)
{
	vector<SurfaceSample> samples;
	
    try {
        PTree tree;
        read_xml(filename, tree);

        PTree root = tree.get_child("SurfaceSamples");
        
        for (CI p = root.begin(); p != root.end(); ++p) {
			if (p->first == "Sample") {
				Q posq = XMLHelpers::readQ(p->second.get_child("Pos"));
				Q rpyq = XMLHelpers::readQ(p->second.get_child("RPY"));
				double graspW = XMLHelpers::readDouble(p->second.get_child("GraspW"));
				//cout << "pos=" << posq << " rot=" << rpyq << " graspW=" << graspW << endl;
	
				Vector3D<> pos(posq[0], posq[1], posq[2]);
				RPY<> rpy(rpyq[0], rpyq[1], rpyq[2]);
				samples.push_back(SurfaceSample(Transform3D<>(pos, rpy.toRotation3D()), graspW));
			}
		}
    } catch (const ptree_error& e) {
        RW_THROW(e.what());
    }
    
	return samples;
}



void SurfaceSample::saveToXML(const std::string& filename, const std::vector<SurfaceSample>& samples)
{
	PTree tree;
	
	// save all the samples!
	BOOST_FOREACH (const SurfaceSample& sample, samples) {
		PTree node;
		
		node.put("Pos", XMLHelpers::QToString(Q(3, sample.transform.P()[0], sample.transform.P()[1],sample.transform.P()[2])));
		RPY<> rpy(sample.transform.R());
		node.put("RPY", XMLHelpers::QToString(Q(3, rpy[0], rpy[1], rpy[2])));
		node.put("GraspW", boost::lexical_cast<std::string>(sample.graspW));
		
		tree.add_child("SurfaceSamples.Sample", node);
	}
	
	// save to XML
	try {
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
        write_xml(filename, tree, std::locale(), settings);
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}
