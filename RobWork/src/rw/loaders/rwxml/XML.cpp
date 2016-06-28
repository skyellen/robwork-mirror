/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "XML.hpp"

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

typedef boost::property_tree::ptree PTree;

using namespace std;
using namespace rw::math;
using namespace rw::common;
using namespace rw::loaders;
using namespace boost::property_tree;


/*

 in boost version 1.41 following changes made this file not compile

 end() --> not_found() (only when used with the find function

 get_own() --> get_value()

 */

namespace
{
    string quote(const string& str) { return StringUtil::quote(str); }

    typedef PTree::const_iterator CI;

    Q readNArray(const PTree& tree)
    {
        // If <N> tags are present:
        if (tree.find("N") != tree.not_found()) {
            Q q(tree.size());
            int i = 0;
            for (CI p = tree.begin(); p != tree.end(); ++p, ++i) {

                if (p->first != "N") {
                    RW_THROW(
                        "Unexpected XML tag "
                        << quote(p->first)
                        << " where numbers <N> were expected.");
                }

                q[i] = p->second.get_value<double>();
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
			const std::vector<std::string> words = StringUtil::words(tree.get_value<std::string>());
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
        if (tree.find("Vector3D") != tree.not_found()) {
            properties.add(
                key,
                desc,
                XML::readVector3D(tree.get_child("Vector3D")));
            return;
        }

        // RPY
        if (tree.find("RPY") != tree.not_found()) {
            properties.add(
                key,
                desc,
                readRPY(tree.get_child("RPY")));
            return;
        }

        // Rotation matrix
        if (tree.find("Rotation3D") != tree.not_found()) {
            properties.add(
                key,
                desc,
                readRotation3DMatrix(tree.get_child("Rotation3D")));
            return;
        }

        // Transform3D
        if (tree.find("Transform3D") != tree.not_found()) {
            properties.add(
                key,
                desc,
                XML::readTransform3D(tree.get_child("Transform3D")));
            return;
        }

        // Q
        if (tree.find("Q") != tree.not_found()) {
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
    if (tree.find("Rotation3D") != tree.not_found()) {
        return readRotation3DMatrix(tree.get_child("Rotation3D"));
    } else if (tree.find("RPY") != tree.not_found()) {
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
            out << indent << "Leaf: '" << tree.get_value<string>() << "'\n";
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
