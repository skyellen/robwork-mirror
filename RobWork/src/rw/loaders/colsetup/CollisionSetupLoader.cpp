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


#include "CollisionSetupLoader.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <rw/common/macros.hpp>
#include <set>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/common/StringUtil.hpp>
#include <fstream>
typedef boost::property_tree::ptree PTree;
using namespace std;
using namespace rw;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::proximity;

namespace
{
    std::string quote(const std::string& str)
    { return StringUtil::quote(str); }

    std::string infoHeader(const std::string& file)
    {
        return "File: " + quote(file) + ": ";
    }

    typedef std::pair<std::string, std::string> Pair;

    struct Result
    {
        std::set<std::string> volatileFrames;
        std::vector<Pair> excludePairs;
    };

    typedef PTree::const_iterator CI;

    void readFramePairAttributes(
        const std::string& prefix,
        const std::string& file,
        const PTree& tree,
        Result& result)
    {
        result.excludePairs.push_back(
            std::make_pair(
                prefix + tree.get<string>("first"),
                prefix + tree.get<string>("second")));
    }

    void readExclude(
        const std::string& prefix,
        const std::string& file,
        const PTree& tree,
        Result& result)
    {
        // Now traverse the FramePair tags.
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "FramePair") {
                boost::optional<const PTree&> child =
                    p->second.get_child_optional("<xmlattr>");
                if (child) {
                    readFramePairAttributes(prefix, file, *child, result);
                } else {
                    RW_THROW(
                        infoHeader(file)
                        << "Attributes expected for FramePair tag.");
                }
            } else if (p->first == "<xmlcomment>") {
            } else {
                RW_THROW(
                    infoHeader(file)
                    << "Non-supported CollisionSetup tag "
                    << quote(p->first));
            }
        }
    }

    void readVolatile(
        const std::string& prefix,
        const PTree& tree,
        Result& result)
    {
        const std::string frameName = tree.get_value<std::string>();
        result.volatileFrames.insert(prefix + frameName);
    }

    CollisionSetup readCollisionSetup(
        const std::string& prefix,
        const std::string& file,
        const PTree& tree)
    {
        Result result;

        bool excludeStaticPairs = false;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Exclude") {
                readExclude(prefix, file, p->second, result);
            } else if (p->first == "Volatile") {
                readVolatile(prefix, p->second, result);
            } else if (p->first == "ExcludeStaticPairs") {
                excludeStaticPairs = true;
            } else if (p->first == "<xmlcomment>") {
            } else {
                RW_THROW(
                    infoHeader(file)
                    << "Non-supported XML tag " << p->first);
            }
        }

        return CollisionSetup(
            result.excludePairs,
            result.volatileFrames,
            excludeStaticPairs);
    }

    CollisionSetup readCollisionSetup(
        const std::string& prefix,
        const std::string& file)
    {
        using namespace boost::property_tree;

        try {
            PTree tree;
            std::ifstream istr(file.c_str());
            read_xml(istr, tree);

            boost::optional<PTree&> child = tree.get_child_optional("CollisionSetup");
            if (child) {
                return readCollisionSetup(prefix, file, *child);
            } else {
                RW_THROW(
                    infoHeader(file)
                    << "CollisionSetup tag expected at top-level.");
                return CollisionSetup(); // To avoid a compiler warning.
            }
        } catch (const ptree_bad_path& e) {
            RW_THROW("The file specified does not exist. Pleace specify a valid path\n "
                    << file << "\n"
                    << e.what());
        } catch (const ptree_error& e) {
            RW_THROW("Error occoured when parsing the CollisionSetup\n "
                    << file << " \n"
                    << e.what());
        }

        // To avoid a compiler warning.
        return readCollisionSetup(prefix, file);
    }
}

rw::proximity::CollisionSetup CollisionSetupLoader::load(
    const std::string& prefix,
    const std::string& file)
{
    return readCollisionSetup(prefix, file);
}
