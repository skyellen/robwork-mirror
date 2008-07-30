#include "ConveyorBelt.hpp"

#include <rw/math/Quaternion.hpp>

using namespace rw::kinematics;
using namespace rw::trajectory;
using namespace rw::models;
using namespace rw::math;

ConveyorBelt::ConveyorBelt(const Trajectory<Q>& trajectory,
                           Frame* baseFrame,
                           ConveyorSegment* next,
                           ConveyorSegment* previous):
    _trajectory(trajectory),
    _baseFrame(baseFrame),
    _next(next),
    _previous(previous)
{}

ConveyorBelt::~ConveyorBelt()
{
}

Frame* ConveyorBelt::getBaseFrame() {
    return _baseFrame;
}


void ConveyorBelt::addItem(ConveyorItem* item, FramePosition position, State& state) {
	item->attachTo(_baseFrame, state);


	double q;
    if (position == START) {
    	q = 0;
    } else  {
    	q = _trajectory.duration();
    }

    moveTo(item, q, state);
}

void ConveyorBelt::moveTo(ConveyorItem* item, double q, State& state) {
    Q pose = _trajectory.x(q);

    Vector3D<> pos(pose(0), pose(1), pose(2));
    Quaternion<> qrot(pose(3), pose(4), pose(5), pose(6));

    item->setTransformAndConveyorPosition(Transform3D<>(pos, qrot), q, state);
}

void ConveyorBelt::move(ConveyorItem* item, double delta, State& state) {
    double qcurrent = item->getConveyorPosition(state);
    if (delta+qcurrent > _trajectory.duration()) {
        if (_next != NULL) {
            _next->addItem(item, START, state);
            _next->move(item, delta-(_trajectory.duration()-qcurrent), state);
        } else {
            moveTo(item, _trajectory.duration(), state);
        }
    } else if (delta+qcurrent < 0) {
        if (_previous != NULL) {
            _previous->addItem(item, END, state);
            _previous->move(item, delta+(qcurrent), state);
        } else {
            moveTo(item, 0, state);
        }
    } else {
        moveTo(item, qcurrent+delta, state);
    }
}


void ConveyorBelt::setPreviousSegment(ConveyorSegment* segment) {
	_next = segment;
}

void ConveyorBelt::setNextSegment(ConveyorSegment* segment) {
	_previous = segment;
}


double ConveyorBelt::length() const {
	return _trajectory.duration();
}
