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


#include "DOMTrajectorySaver.hpp"
#include <rw/loaders/dom/DOMTrajectoryLoader.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>

#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/Interpolator.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/CubicSplineInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>

#include <rw/common/DOMParser.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::loaders;
using namespace rw::trajectory;

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



    template <class T>
    class Identifiers {
     public:
         static const std::string& linearInterpolatorId();
         static const std::string& cubicSplineInterpolatorId();
         static const std::string& circularInterpolatorId();

     };

    template<> const std::string& Identifiers<Q>::linearInterpolatorId() {
        return DOMTrajectoryLoader::QLinearInterpolatorId;
    }


    template<> const std::string& Identifiers<Q>::cubicSplineInterpolatorId() {
        return DOMTrajectoryLoader::QCubicSplineInterpolatorId;
    }

    template<> const std::string& Identifiers<Q>::circularInterpolatorId() {
        return "";
    }



    template<> const std::string& Identifiers<Vector3D<> >::linearInterpolatorId() {
        return DOMTrajectoryLoader::V3DLinearInterpolatorId;
    }

    template<> const std::string& Identifiers<Vector3D<> >::cubicSplineInterpolatorId() {
        return DOMTrajectoryLoader::V3DCubicSplineInterpolatorId;
    }

    template<> const std::string& Identifiers<Vector3D<> >::circularInterpolatorId() {
        return DOMTrajectoryLoader::V3DCircularInterpolatorId;
    }


    template<> const std::string& Identifiers<Rotation3D<> >::linearInterpolatorId() {
        return DOMTrajectoryLoader::R3DLinearInterpolatorId;
    }


    template<> const std::string& Identifiers<Rotation3D<> >::cubicSplineInterpolatorId() {
        return DOMTrajectoryLoader::R3DCubicSplineInterpolatorId;
    }

    template<> const std::string& Identifiers<Rotation3D<> >::circularInterpolatorId() {
        return "";
    }

    template<> const std::string& Identifiers<Transform3D<> >::linearInterpolatorId() {
        return DOMTrajectoryLoader::T3DLinearInterpolatorId;
    }


    template<> const std::string& Identifiers<Transform3D<> >::cubicSplineInterpolatorId() {
        return DOMTrajectoryLoader::T3DCubicSplineInterpolatorId;
    }

    template<> const std::string& Identifiers<Transform3D<> >::circularInterpolatorId() {
        return "";
    }



    template <class T>
    DOMElem::Ptr writeInterpolator(const Ptr<Interpolator<T> > interpolator, DOMElem::Ptr parent) {
        LinearInterpolator<T>* linear = dynamic_cast<LinearInterpolator<T>*>(interpolator.get());
        if (linear != NULL) {
            T start = linear->getStart();
            T end = linear->getEnd();
            double duration = linear->duration();
            DOMElem::Ptr element = parent->addChild(Identifiers<T>::linearInterpolatorId());
            element->addAttribute(DOMTrajectoryLoader::DurationAttributeId)->setValue(duration);

            ElementCreator<T>::createElement(start, element);
            ElementCreator<T>::createElement(end, element);

            return element;
        }

         CubicSplineInterpolator<T>* cspline = dynamic_cast<CubicSplineInterpolator<T>*>(interpolator.get());
         if (cspline != NULL) {
             //TODO Once implemented
             RW_THROW("Interpolator not supported by DOMTrajectorySaver");
         }

         CircularInterpolator<T>* circular = dynamic_cast<CircularInterpolator<T>*>(interpolator.get());
         if (circular != NULL) {
             T p1 = circular->getP1();
             T p2 = circular->getP2();
             T p3 = circular->getP3();
             double duration = circular->duration();

             DOMElem::Ptr element = parent->addChild(Identifiers<T>::circularInterpolatorId());
             element->addAttribute(DOMTrajectoryLoader::DurationAttributeId)->setValue(duration);

             ElementCreator<T>::createElement(p1, parent);
             ElementCreator<T>::createElement(p2, parent);
             ElementCreator<T>::createElement(p3, parent);

             return element;
         }
        RW_THROW("The Trajectory contains an interpolator not supported by DOMTrajectorySaver");

    };


    template <class T>
    DOMElem::Ptr writeBlend(const Ptr<Blend<T> >& blend, DOMElem::Ptr parent) {
        const ParabolicBlend<T>* parabolic = dynamic_cast<const ParabolicBlend<T>*>(blend.get());
        if (parabolic != NULL) {
            double tau = parabolic->tau1();
            DOMElem::Ptr element  = parent->addChild(DOMTrajectoryLoader::ParabolicBlendId);
            element->addAttribute(DOMTrajectoryLoader::TauAttributeId)->setValue( tau );
            return element;
        }

        const LloydHaywardBlend<T>* lloydHayward = dynamic_cast<const LloydHaywardBlend<T>*>(blend.get());
        if (lloydHayward != NULL) {
            double tau = lloydHayward->tau1();
            double kappa = lloydHayward->kappa();

            DOMElem::Ptr element = parent->addChild( DOMTrajectoryLoader::LloydHaywardBlendId );
            element->addAttribute(DOMTrajectoryLoader::TauAttributeId)->setValue(tau);
            element->addAttribute(DOMTrajectoryLoader::KappaAttributeId)->setValue(kappa);
            return element;
        }
        RW_THROW("The Trajectory contains a blend not supported by DOMTrajectorySaver");
    }


    template <class T, class TRAJ>
    DOMElem::Ptr createDOMDocument(TRAJ& trajectory, const std::string& trajectoryId, DOMParser::Ptr parser) {
    	DOMElem::Ptr doc = parser->getRootElement();

		try
		{
			DOMElem::Ptr root = doc->addChild(trajectoryId);

			typedef const InterpolatorTrajectory<T> Traj;
			Traj* traj = dynamic_cast<Traj*>(&trajectory);
			if (traj == NULL) {
				RW_THROW("Unable to save trajectory which is not a InterpolatorTrajectory");
			}

			for (size_t i = 0; i<traj->getSegmentsCount(); i++) {
				typedef std::pair<const Ptr<Blend<T> >, const Ptr<Interpolator<T> > > Segment;
				Segment segment = traj->getSegment(i);

				if (segment.first != NULL) {
					writeBlend(segment.first, root);
				}
				writeInterpolator(segment.second, root);
			}
		} catch (const rw::common::Exception& exp) {
			throw exp;
		} catch (...) {
			RW_THROW("DOMTrajectoryWriter: Unknown Exception while creating saving path");
		}
        return doc;

    }


    template <class T, class TRAJ>
    bool saveTrajectoryImpl(TRAJ& trajectory, const std::string& trajectoryId, const std::string& filename) {
        DOMParser::Ptr parser = DOMParser::make();
    	createDOMDocument<T, TRAJ>(trajectory, trajectoryId, parser);
    	parser->save(filename);
        return true;
    }

    template <class T, class TRAJ>
    bool saveTrajectoryImpl(TRAJ& trajectory, const std::string& trajectoryId, std::ostream& outstream) {
        DOMParser::Ptr parser = DOMParser::make();
    	createDOMDocument<T, TRAJ>(trajectory, trajectoryId, parser);
    	parser->save(outstream);
        return true;
    }
} //end cpp file's namespace


bool DOMTrajectorySaver::save(const rw::trajectory::QTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Q, const QTrajectory>(trajectory, DOMTrajectoryLoader::QTrajectoryId, filename);
}


bool DOMTrajectorySaver::save(const rw::trajectory::Vector3DTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Vector3D<>, const Vector3DTrajectory>(trajectory, DOMTrajectoryLoader::V3DTrajectoryId, filename);
}

bool DOMTrajectorySaver::save(const rw::trajectory::Rotation3DTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Rotation3D<>, const Rotation3DTrajectory>(trajectory, DOMTrajectoryLoader::R3DTrajectoryId, filename);
}

bool DOMTrajectorySaver::save(const rw::trajectory::Transform3DTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Transform3D<>, const Transform3DTrajectory>(trajectory, DOMTrajectoryLoader::T3DTrajectoryId, filename);
}



bool DOMTrajectorySaver::write(const rw::trajectory::QTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Q, const QTrajectory>(trajectory, DOMTrajectoryLoader::QTrajectoryId, outstream);
}

bool DOMTrajectorySaver::write(const rw::trajectory::Vector3DTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Vector3D<>, const Vector3DTrajectory>(trajectory, DOMTrajectoryLoader::V3DTrajectoryId, outstream);
}

bool DOMTrajectorySaver::write(const rw::trajectory::Rotation3DTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Rotation3D<>, const Rotation3DTrajectory>(trajectory, DOMTrajectoryLoader::R3DTrajectoryId, outstream);
}

bool DOMTrajectorySaver::write(const rw::trajectory::Transform3DTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Transform3D<>, const Transform3DTrajectory>(trajectory, DOMTrajectoryLoader::T3DTrajectoryId, outstream);
}
