#include "XMLPathFormat.hpp"

using namespace rw::loaders;
using namespace xercesc;

//Small trick to make sure Xerces is initialized before we start using XMLString::transcode
const XercesInitializer XMLPathFormat::initializer;

const XMLCh* XMLPathFormat::QPathId = XMLString::transcode("QPath");
const XMLCh* XMLPathFormat::V3DPathId = XMLString::transcode("V3DPath");

const XMLCh* XMLPathFormat::R3DPathId = XMLString::transcode("R3DPath");
const XMLCh* XMLPathFormat::T3DPathId = XMLString::transcode("T3DPath");

const XMLCh* XMLPathFormat::StatePathId = XMLString::transcode("StatePath");

/** @brief Identifier for rw::trajectory::TimedQPath in the XML format  */
const XMLCh* XMLPathFormat::TimedQPathId = XMLString::transcode("TimedQPath");

/** @brief Identifier for rw::trajectory::TimedStatePath in the XML format  */
const XMLCh* XMLPathFormat::TimedStatePathId = XMLString::transcode("TimedStatePath");

/** @brief Identifier for rw::trajectory::TimedState in the XML format  */
const XMLCh* XMLPathFormat::TimedStateId = XMLString::transcode("TimedState");

/** @brief Identifier for rw::trajectory::TimedQ in the XML format  */
const XMLCh* XMLPathFormat::TimedQId = XMLString::transcode("TimedQ");

/** @brief Identifier for time attribute used for rw::trajectory::TimedQPath and rw::trajectory::TimedStatePath in the XML format  */
const XMLCh* XMLPathFormat::TimeId = XMLString::transcode("Time");
