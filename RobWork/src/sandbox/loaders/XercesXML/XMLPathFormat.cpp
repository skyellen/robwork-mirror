#include "XMLPathFormat.hpp"

using namespace rw::loaders;
using namespace xercesc;

//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
const XercesInitializer XMLPathFormat::initializer;

const XMLCh* XMLPathFormat::QPathId = XMLString::transcode("QPath");
const XMLCh* XMLPathFormat::V3DPathId = XMLString::transcode("V3DPath");

const XMLCh* XMLPathFormat::R3DPathId = XMLString::transcode("R3DPath");
const XMLCh* XMLPathFormat::T3DPathId = XMLString::transcode("T3DPath");

