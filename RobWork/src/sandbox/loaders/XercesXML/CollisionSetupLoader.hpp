/*
 * CollisionSetupLoader.hpp
 *
 *  Created on: Nov 5, 2008
 *      Author: lpe
 */

#ifndef COLLISIONSETUPLOADER_HPP_
#define COLLISIONSETUPLOADER_HPP_

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMNodeIterator.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMText.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>

#include <vector>
#include <string>

namespace sandbox {

typedef std::pair<std::string, std::string> ProximityPair;
typedef std::vector<ProximityPair > ProximityPairList;

class CollisionSetupLoader
{
public:
    static ProximityPairList load(const std::string& filename);

private:
    CollisionSetupLoader() {};
};

}

#endif /* COLLISIONSETUPLOADER_HPP_ */
