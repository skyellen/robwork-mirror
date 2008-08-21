/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
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

#include "XML.hpp"

#include <rw/math/Q.hpp>
#include <rw/common/PropertyMap.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <string>
#include <sstream>

typedef boost::property_tree::ptree PTree;

using namespace std;
using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;
using namespace boost::property_tree;

namespace
{
    string quote(const string& str) { return StringUtil::quote(str); }

    typedef PTree::const_iterator CI;

    Q readNArray(const PTree& tree)
    {
        // If <N> tags are present:
        if (tree.find("N") != tree.end()) {
            Q q(tree.size());
            int i = 0;
            for (CI p = tree.begin(); p != tree.end(); ++p, ++i) {

                if (p->first != "N") {
                    RW_THROW(
                        "Unexpected XML tag "
                        << quote(p->first)
                        << " where numbers <N> were expected.");
                }

                q[i] = p->second.get_own<double>();
            }
            return q;
        }

        // Otherwise extract a space separated string of numbers.
        else {
            /*
              This is very nice C++ code, but I have seen it fail for files with
              Windows line endings being read on Linux.

              istringstream buf(tree.get_own<string>());
              std::string str;
              while (buf >> str) {
                  const pair<bool, double> okNum = StringUtil::toDouble(str);
                  if (!okNum.first)
                      RW_THROW("Number expected. Got " << quote(str));
                  values.push_back(okNum.second);
              }

              Therefore we do this instead:
            */
			const std::vector<std::string> words = StringUtil::words(tree.get_own<std::string>());
            std::vector<double> values;
			BOOST_FOREACH(const std::string& str, words) {
                const pair<bool, double> okNum = StringUtil::toDouble(str);
                if (!okNum.first)
                    RW_THROW("Number expected. Got " << quote(str));
                values.push_back(okNum.second);
            }

            Q q(values.size());
            for (size_t i = 0; i < q.size(); i++)
                q[i] = values[i];
            return q;
        }
    }

    RPY<> readRPY(const PTree& tree)
    {
        const Q vals = readNArray(tree);
        if (vals.size() != 3)
            RW_THROW(
                "Unexpected number of RPY values "
                << (int)vals.size());

        return RPY<>(vals[0], vals[1], vals[2]);
    }

    Rotation3D<> readRotation3DMatrix(const PTree& tree)
    {
        const Q vals = readNArray(tree);
        if (vals.size() != 9) {
            RW_THROW(
                "Unexpected number of values "
                << (int)vals.size()
                << " for Rotation3D.");
        }

        return Rotation3D<>(
            vals[0], vals[1], vals[2],
            vals[3], vals[4], vals[5],
            vals[6], vals[7], vals[8]);
    }

    void readProperty(
        const PTree& tree, PropertyMap& properties)
    {
        const string key = tree.get<string>("Key");
        const string desc = tree.get<string>("Description", "");

        // Strings
        {
            boost::optional<string> val = tree.get_optional<string>("S");
            if (val) {
                properties.add(key, desc, *val);
                return;
            }
        }

        // Numbers
        {
            boost::optional<double> val = tree.get_optional<double>("N");
            if (val) {
                properties.add(key, desc, *val);
                return;
            }
        }

        // Vector3D
        if (tree.find("Vector3D") != tree.end()) {
            properties.add(
                key,
                desc,
                XML::readVector3D(tree.get_child("Vector3D")));
            return;
        }

        // RPY
        if (tree.find("RPY") != tree.end()) {
            properties.add(
                key,
                desc,
                readRPY(tree.get_child("RPY")));
            return;
        }

        // Rotation matrix
        if (tree.find("Rotation3D") != tree.end()) {
            properties.add(
                key,
                desc,
                readRotation3DMatrix(tree.get_child("Rotation3D")));
            return;
        }

        // Transform3D
        if (tree.find("Transform3D") != tree.end()) {
            properties.add(
                key,
                desc,
                XML::readTransform3D(tree.get_child("Transform3D")));
            return;
        }

        // Q
        if (tree.find("Q") != tree.end()) {
            properties.add(
                key,
                desc,
                readNArray(tree.get_child("Q")));
            return;
        }

        RW_THROW("No value for property " << quote(key) << " given.");
    }
}

Vector3D<> XML::readVector3D(const PTree& tree)
{
    const Q vals = readNArray(tree);
    if (vals.size() != 3)
        RW_THROW(
            "Unexpected number of values "
            << (int)vals.size()
            << " for Vector3D.");

    return Vector3D<>(vals[0], vals[1], vals[2]);
}

Rotation3D<> XML::readRotation3D(const PTree& tree)
{
    if (tree.find("Rotation3D") != tree.end()) {
        return readRotation3DMatrix(tree.get_child("Rotation3D"));
    } else if (tree.find("RPY") != tree.end()) {
        return readRPY(tree.get_child("RPY")).toRotation3D();
    } else {
        RW_THROW("No rotation specified. <Rotation3D> or <RPY> expected.");
    }
}

Transform3D<> XML::readTransform3D(const PTree& tree)
{
    return Transform3D<>(
        XML::readVector3D(tree.get_child("Vector3D")),
        XML::readRotation3D(tree));
}

Q XML::readQ(const PTree& tree)
{
    return readNArray(tree.get_child("Q"));
}

void XML::readPropertyMap(
    const PTree& tree, PropertyMap& properties)
{
    int i = 0;
    for (CI p = tree.begin(); p != tree.end(); ++p, ++i) {
        if (p->first != "Property") {
            RW_THROW(
                "Unexpected XML tag "
                << quote(p->first)
                << " where property <Property> was expected.");
        }

        readProperty(p->second, properties);
    }
}

namespace
{
    // This is just a utility that is useful when figuring out how property
    // trees are structured. Please let it stay here even though it isn't being
    // called in production code.
    void printTreeHelper(const PTree& tree, std::ostream& out, int level)
    {
        string indent(2 * level, ' ');

        if (tree.size() == 0) {
            out << indent << "Leaf: '" << tree.get_own<string>() << "'\n";
        }
        else {
            for (CI p = tree.begin(); p != tree.end(); ++p) {
                out
                    << indent << p->first << "\n"
                    << indent << "{\n";
                printTreeHelper(p->second, out, level + 1);
                out
                    << indent << "}\n";
            }
        }
    }
}

void XML::printTree(const PTree& tree, std::ostream& out)
{
    printTreeHelper(tree, out, 0);
}
