/*
 * XMLPathWriter.hpp
 *
 *  Created on: Nov 26, 2008
 *      Author: lpe
 */

#ifndef XMLPATHWRITER_HPP_
#define XMLPATHWRITER_HPP_


#include <rw/trajectory/Path.hpp>

#include <string>

class XMLPathSaver
{
public:
    XMLPathSaver();
    virtual ~XMLPathSaver();

    static bool savePath(const rw::trajectory::QPath& path, const std::string& filename);
    static bool savePath(const rw::trajectory::Vector3DPath& path, const std::string& filename);
    static bool savePath(const rw::trajectory::Rotation3DPath& path, const std::string& filename);
    static bool savePath(const rw::trajectory::Transform3DPath& path, const std::string& filename);

};

#endif /* XMLPATHWRITER_HPP_ */
