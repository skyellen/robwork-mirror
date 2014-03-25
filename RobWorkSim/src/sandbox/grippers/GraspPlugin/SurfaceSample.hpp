/**
 * @file SurfaceSample.hpp
 * @author Adam Wolniakowski
 */
 
#pragma once

#include <string>
#include <vector>
#include <rw/math/Transform3D.hpp>
 


class SurfaceSample
{
	public:
		/* constructors */
		/// Constructor
		SurfaceSample(rw::math::Transform3D<> transform=rw::math::Transform3D<>(), double graspW=0.0) :
			transform(transform),
			graspW(graspW)
		{}
		
		/// Destructor
		~SurfaceSample() {}
		
		/* static methods */
		/**
		 * @brief Loads a vector of SurfaceSample from XML file
		 */
		static std::vector<SurfaceSample> loadFromXML(const std::string& filename);
		
		/**
		 * @brief Saves a vector of SurfaceSample to XML file
		 */
		static void saveToXML(const std::string& filename, const std::vector<SurfaceSample>& samples);
		
		/* data */
		rw::math::Transform3D<> transform;
		double graspW;
};
