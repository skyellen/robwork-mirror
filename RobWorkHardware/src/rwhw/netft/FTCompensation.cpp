#include "FTCompensation.hpp"

// STL
#include <exception>
#include <cmath>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

// RW
#include <rw/math/Rotation3D.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Vector3D.hpp>

using namespace rwhw;
using namespace rw::models;
using namespace rw::math;
using namespace rw::kinematics;
using namespace boost::property_tree;


FTCompensation::FTCompensation(Device::Ptr dev,
                               State state,
                               const std::string& calibfile,
                               const Wrench3D& thres) : _dev(dev), _state(state), _thres(thres) {
   // Start timer
   _prevT = boost::posix_time::microsec_clock::universal_time();
   
   // Load calibration
   if(!LoadCalib(calibfile, _calib, _eTft))
      RW_THROW("Failed to load F/T calibration file!");
   
   // Bootstrap previous measurement vectors
   _qP = _dqP = Q::zero(dev->getDOF());
}

void FTCompensation::update(const Wrench3D& ft, const Q& q, const Q& dq, const Q& ddq) {
   // Take a copy for compensation
   _ft = ft;
   
   // Bias the measurement
   bias();
   
   // Calculate FK
   fk(q);
   
   // Compensate for gravity
   gravitate();
   
   // Set the collision flag
   _status = std::abs(_ft.first[0]) > _thres.first[0] ||
             std::abs(_ft.first[1]) > _thres.first[1] ||
             std::abs(_ft.first[2]) > _thres.first[2] ||
             std::abs(_ft.second[0]) > _thres.second[0] ||
             std::abs(_ft.second[1]) > _thres.second[1] ||
             std::abs(_ft.second[2]) > _thres.second[2];
}

void FTCompensation::bias() {
   _ft.first -= _calib.bias.first;
   _ft.second -= _calib.bias.second;   
}

void FTCompensation::gravitate() {
   // Gravitational force
   const double Fg = -9.80665*_calib.m;
   
   // Base-relative F/T tool rotation
   Rotation3D<> bRft = _bTft.R();
   
   // Base-relative gravitational force
   const Vector3D<> Fgb(0, 0, Fg);
   
   // Gravitational force in F/T frame
   const Vector3D<> Fgft = bRft.inverse() * Fgb;
   
   // Compensate for gravitational force
   _ft.first -= Fgft;
   
   // Torque in F/T frame
   const Vector3D<> tau = cross(_calib.r, Fgft);
   
   // Compensate for gravity induced torque
   _ft.second -= tau;
}

bool FTCompensation::LoadCalib(const std::string& filename, FTCalib& calib, Transform3D<>& eTft) {
   try {
      
      // Open the file
      ptree tree;
      read_xml(filename.c_str(), tree);
      tree = tree.get_child("ftcalib");
      
      // Get the identifier label
      calib.label = tree.get_child("<xmlattr>").get<std::string>("label");
      
      // Get calibrated mass
      calib.m = tree.get<double>("m");
      
      // Get calibrated tool roll angle and distance to COG and set transformation from robot tool to F/T tool
      calib.a = tree.get<double>("a");
      calib.r[0] = tree.get<double>("r.x");
      calib.r[1] = tree.get<double>("r.y");
      calib.r[2] = tree.get<double>("r.z");
      eTft.P() = calib.r;
      eTft.R() = RPY<>(calib.a, 0, 0).toRotation3D();
      
      // Get calibrated bias
      std::istringstream iss(tree.get<std::string>("bias"));
      iss >> calib.bias.first[0] >> calib.bias.first[1] >> calib.bias.first[2];
      iss >> calib.bias.second[0] >> calib.bias.second[1] >> calib.bias.second[2];
      
      return true;
      
   } catch(const xml_parser_error& e) {
      std::cerr << "Parsing of the XML file failed:" << std::endl << "\t" << e.what() << std::endl;
   } catch(const std::exception& e) {
      std::cerr << "Reading of the XML file failed:" << std::endl << "\t" << e.what() << std::endl;
   } catch(...) {
      std::cerr << "Parsing of the XML file failed due to an unknown exception!" << std::endl;
   }

   return false;
}
