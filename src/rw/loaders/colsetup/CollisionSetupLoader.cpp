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

#include "CollisionSetupLoader.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <rw/common/macros.hpp>
#include <set>
#include <rw/proximity/CollisionSetup.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/use_robwork_namespace.hpp>

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
        const std::string frameName = tree.get_own<std::string>();
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
            read_xml(file, tree);

            boost::optional<PTree&> child = tree.get_child_optional("CollisionSetup");
            if (child) {
                return readCollisionSetup(prefix, file, *child);
            } else {
                RW_THROW(
                    infoHeader(file)
                    << "CollisionSetup tag expected at top-level.");
                return CollisionSetup(); // To avoid a compiler warning.
            }
        } catch (const ptree_error& e) {
            // Convert from parse errors to RobWork errors.
            RW_THROW(e.what());
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
