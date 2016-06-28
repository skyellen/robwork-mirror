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

#include "DOMPathSaver.hpp"

#include <rw/common/DOMParser.hpp>
#include <rw/loaders/dom/DOMPathLoader.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>

using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::math;

using namespace rw::common;
using namespace rw::kinematics;


namespace {
     template <class T>
     class ElementCreator {
     public:
         static DOMElem::Ptr createElement(const T& element, DOMElem::Ptr parent);
     };

     template<> DOMElem::Ptr ElementCreator<Q>::createElement(const Q& element, DOMElem::Ptr parent) {
         return DOMBasisTypes::createQ(element, parent);
     }

     template<> DOMElem::Ptr ElementCreator<Vector3D<> >::createElement(const Vector3D<>& element, DOMElem::Ptr parent) {
         return DOMBasisTypes::createVector3D(element, parent);
     }

     template<> DOMElem::Ptr ElementCreator<Rotation3D<> >::createElement(const Rotation3D<>& element, DOMElem::Ptr parent) {
         return DOMBasisTypes::createRotation3D(element, parent);
     }

     template<> DOMElem::Ptr ElementCreator<Transform3D<> >::createElement(const Transform3D<>& element, DOMElem::Ptr parent) {
         return DOMBasisTypes::createTransform3D(element, parent);
     }

     template<> DOMElem::Ptr ElementCreator<State>::createElement(const State& element, DOMElem::Ptr parent) {
         return DOMBasisTypes::createState(element, parent);
     }


     template<> DOMElem::Ptr ElementCreator<TimedQ>::createElement(const TimedQ& timedQ, DOMElem::Ptr parent) {
         DOMElem::Ptr element = parent->addChild(DOMPathLoader::TimedQId);
         element->addChild(DOMPathLoader::TimeId)->setValue(timedQ.getTime());
         DOMBasisTypes::createQ(timedQ.getValue(), element);
         return element;
     }

     template<> DOMElem::Ptr ElementCreator<TimedState>::createElement(
    		 const TimedState& timedState, DOMElem::Ptr parent) {
    	 DOMElem::Ptr element = parent->addChild(DOMPathLoader::TimedStateId);
    	 element->addChild(DOMPathLoader::TimeId)->setValue(timedState.getTime());
         DOMBasisTypes::createState(timedState.getValue(), element);
         return element;
     }


     template<class T, class PATH>
     static void insertElements(const PATH& path, DOMElem::Ptr parent) {
         for (typename PATH::const_iterator it = path.begin(); it != path.end(); ++it) {
             ElementCreator<T>::createElement(*it, parent);
         }
     }

     /**
      * @brief Create an element representing the path
      *
      * Create an element titles \b pathId representing \b path
      *
      * @param path [in] Path to save. Can be either QPath, Vector3DPath, Rotation3DPath or Transform3DPath
      * @param pathId [in] Id of the path
      * @param doc [in] Document for which to construct the element
      */
     template <class T, class PATH>
     static DOMElem::Ptr createElement(const PATH& path, const std::string& pathId, DOMElem::Ptr parent) {
    	 DOMElem::Ptr pathElement = parent->addChild(pathId);
    	 insertElements<T, PATH>(path, pathElement);
         return pathElement;
     }


     template <class T, class PATH>
     static DOMElem::Ptr createDOMDocument(const PATH& path, const std::string& pathId, DOMParser::Ptr parser) {

    	 try{
			 DOMElem::Ptr doc = parser->getRootElement();
			 DOMElem::Ptr root = doc->addChild(pathId);
			 insertElements<T, PATH>(path, root);
			 return doc;
		 } catch (const Exception& exp) {
			 throw exp;
		 } catch (...) {
			 RW_THROW("DOMPathWriter: Unknown Exception while creating saving path");
		 }
     }

     template <class T, class PATH>
     static void savePath(const PATH& path, const std::string& pathId, const std::string& filename) {
    	 DOMParser::Ptr parser = DOMParser::make();
         DOMElem::Ptr parent = createDOMDocument<T, PATH>(path, pathId, parser);
         parser->save(filename);
     }

     template <class T, class PATH>
     static void writePath(const PATH& path, const std::string& pathId, std::ostream& outstream) {
    	 DOMParser::Ptr parser = DOMParser::make();
         DOMElem::Ptr parent = createDOMDocument<T, PATH>(path, pathId, parser);
         parser->save(outstream);
     }



 } //end namespace


void DOMPathSaver::save(const QPath& path, const std::string& filename) {
    savePath<Q,  QPath>(path, DOMPathLoader::QPathId, filename);
}

void DOMPathSaver::save(const Vector3DPath& path, const std::string& filename) {
    savePath<Vector3D<>,  Vector3DPath>(path, DOMPathLoader::V3DPathId, filename);
}

void DOMPathSaver::save(const Rotation3DPath& path, const std::string& filename) {
    savePath<Rotation3D<>,  Rotation3DPath>(path, DOMPathLoader::R3DPathId, filename);
}

void DOMPathSaver::save(const Transform3DPath& path, const std::string& filename) {
    savePath<Transform3D<>,  Transform3DPath>(path, DOMPathLoader::T3DPathId, filename);
}

void DOMPathSaver::save(const StatePath& path, const std::string& filename) {
    savePath<State, StatePath>(path, DOMPathLoader::StatePathId, filename);
}

void DOMPathSaver::save(const TimedQPath& path, const std::string& filename) {
    savePath<TimedQ, TimedQPath>(path, DOMPathLoader::TimedQPathId, filename);
}

void DOMPathSaver::save(const TimedStatePath& path, const std::string& filename) {
    savePath<TimedState, TimedStatePath>(path, DOMPathLoader::TimedStatePathId, filename);
}

//------------------------------------------------------------------------

void DOMPathSaver::write(const QPath& path, std::ostream& outstream) {
    writePath<Q,  QPath>(path, DOMPathLoader::QPathId, outstream);
}

void DOMPathSaver::write(const Vector3DPath& path, std::ostream& outstream) {
    writePath<Vector3D<>,  Vector3DPath>(path, DOMPathLoader::V3DPathId, outstream);
}

void DOMPathSaver::write(const Rotation3DPath& path, std::ostream& outstream) {
    writePath<Rotation3D<>,  Rotation3DPath>(path, DOMPathLoader::R3DPathId, outstream);
}

void DOMPathSaver::write(const Transform3DPath& path, std::ostream& outstream) {
    writePath<Transform3D<>,  Transform3DPath>(path, DOMPathLoader::T3DPathId, outstream);
}


void DOMPathSaver::write(const StatePath& path, std::ostream& outstream) {
    writePath<State, StatePath>(path, DOMPathLoader::StatePathId, outstream);
}

void DOMPathSaver::write(const TimedQPath& path, std::ostream& outstream) {
    writePath<TimedQ, TimedQPath>(path, DOMPathLoader::TimedQPathId, outstream);
}

void DOMPathSaver::write(const TimedStatePath& path, std::ostream& outstream) {
    writePath<TimedState, TimedStatePath>(path, DOMPathLoader::TimedStatePathId, outstream);
}

//------------------------------------------------------------------------

rw::common::DOMElem::Ptr DOMPathSaver::createTransform3DPath(const rw::trajectory::Transform3DPath &path, rw::common::DOMElem::Ptr doc) {
    return createElement<Transform3D<>, Transform3DPath>(path, DOMPathLoader::T3DPathId, doc);
}

rw::common::DOMElem::Ptr DOMPathSaver::createQPath(const rw::trajectory::QPath &path, rw::common::DOMElem::Ptr doc) {
    return createElement<Q, QPath>(path, DOMPathLoader::QPathId, doc);
}
