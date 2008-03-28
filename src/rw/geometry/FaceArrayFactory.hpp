/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef rw_geometry_FaceArrayFactory_HPP
#define rw_geometry_FaceArrayFactory_HPP

/**
 * @file FaceArrayFactory.hpp
 */

#include <string>
#include <istream>
#include <vector>
#include <rw/common/Cache.hpp>

#include "Face.hpp"

namespace rw { namespace geometry {
    /** @addtogroup geometry */
    /*@{*/

    /**
     * @brief Factory for construction of face arrays based on
     * their type
     */

    class FaceArrayFactory
    {
    public:
        /**
         * @brief Factory method constructing triangles associated with string
         *
         * The factory method probes the string to see it it represents
         * a geometric primitive or a filename. In case of a primitive
         * it call ConstructFromGeometry, otherwise LoadFaceArrayFile is called.
         *
         * Notice that an exception may be thrown if the string cannot be parsed.
         *
         * @param str [in] string containing information about face array
         * @param result [out] vector into which the faces are stored
         * @return bool indicating whether a face array was successfully obtained
         */
        static bool GetFaceArray(const std::string& str,
                                 std::vector<Face<float> >& result);


        /**
         * @brief Factory method extracting all triangle faces from a file.
         *
         * @param filename [in] path and name of file to load
         *
         * @param result [out] vector into which the faces are stored
         *
         * @return bool indicating whether the load was successful
         *
         * The factory determines which type of geometry loader to use based on
         * the filename extension. In case no extension exists if test whether a
         * file with the same name or a .stl, .stla or .stlb
         */
        static bool LoadFaceArrayFile(const std::string &filename,
                                      std::vector<Face<float> >& result);

        /**
         * @brief Factory method extracting from a geometric primitive
         *
         * An exception will be thrown in the string cannot be parsed.
         *
         * @param str [in] string specifying the geometric primitive
         * @param result [out] vector into which the faces are stored
         * @return bool indicating success
         */
        static bool ConstructFromGeometry(const std::string& str,
                                          std::vector<Face<float> >& result);

        
        static rw::common::Cache<std::string, std::vector<Face<float> > >& getCache();
    private:
    	static rw::common::Cache< std::string , std::vector<Face<float> > > cache;
        
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
