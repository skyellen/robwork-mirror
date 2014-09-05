/********************************************************************************
 * Copyright 2014 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "TNTRollbackMethodRidder.hpp"

#include <rw/math/Math.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsimlibs::tntphysics;

class TNTRollbackMethodRidder::RollbackDataRidder: public RollbackData {
public:
	RollbackDataRidder(): left(false) {};
	virtual ~RollbackDataRidder() {};

public:
	bool left;
	std::pair<const Frame*, const Frame*> curPair;
};

TNTRollbackMethodRidder::TNTRollbackMethodRidder() {
}

TNTRollbackMethodRidder::~TNTRollbackMethodRidder() {
}

TNTRollbackMethod::RollbackData* TNTRollbackMethodRidder::createData() const {
	return new RollbackDataRidder();
}

double TNTRollbackMethodRidder::getTimestep(SampleSet& samples, RollbackData* data) const {
	fixSampleSet(samples);
	RollbackDataRidder* const ridderData = dynamic_cast<RollbackDataRidder*>(data);
	if (ridderData == NULL)
		RW_THROW("TNTRollbackMethodRidder (getTimestep): the rollback data given to the Ridder method was not of the type RollbackDataRidder!");
	if (samples.size() < 2)
		RW_THROW("TNTRollbackMethodRidder (getTimestep): the size of the SampleSet should be at least two!");
	else if(samples.size() == 2) {
		SampleSet::const_iterator it = samples.begin();
		const Sample& sample1 = *it;
		const Sample& sample2 = *(++it);
		const double t1 = sample1.time;
		const double t2 = sample2.time;
		return (t1+t2)/2.;
	} else if(samples.size() == 3) {
		SampleSet::const_iterator it = samples.begin();
		const Sample& sample1 = *it;
		const Sample& sample2 = *(++it);
		const Sample& sample3 = *(++it);
		return threeSamples(sample1,sample2,sample3,ridderData);
	} else if(samples.size() == 4) {
		SampleSet::iterator it = samples.begin();
		Sample sample1 = *it;
		Sample sample2;
		Sample sample4;
		if (ridderData->left) {
			sample4 = *(++it);
			sample2 = *(++it);
		} else {
			sample2 = *(++it);
			sample4 = *(++it);
		}
		Sample sample3 = *(++it);
		const double t1 = sample1.time;
		const double t2 = sample2.time;
		const double t3 = sample3.time;
		const double t4 = sample4.time;
		RW_ASSERT(t3 > t2 && t2 > t1);
		RW_ASSERT(t4 > t1 && t4 < t3);
		if (sample1.distance.has(ridderData->curPair)) {
			RW_ASSERT(sample2.distance.has(ridderData->curPair));
			RW_ASSERT(sample3.distance.has(ridderData->curPair));
			RW_ASSERT(sample4.distance.has(ridderData->curPair));
			const double d1 = sample1.distance[ridderData->curPair];
			const double d2 = sample2.distance[ridderData->curPair];
			const double d3 = sample3.distance[ridderData->curPair];
			const double d4 = sample4.distance[ridderData->curPair];
			Sample sample1new;
			Sample sample3new;
			if (d1*d2 < 0 && d1*d4 < 0) {
				sample1new = sample1;
				sample3new = sample4;
			} else if (d1*d2 < 0 && d2*d4 < 0) {
				sample1new = sample4;
				sample3new = sample2;
			} else if (d2*d3 < 0 && d2*d4 < 0) {
				sample1new = sample2;
				sample3new = sample4;
			} else if (d2*d3 < 0 && d3*d4 < 0) {
				sample1new = sample4;
				sample3new = sample3;
			} else {
				RW_THROW("TNTRollbackMethodRidder (getTimestep): could not do false-position part.");
			}
			samples.clear();
			samples.insert(sample1new);
			samples.insert(sample3new);
			return (sample1new.time+sample3new.time)/2.;
		} else {
			samples.clear();
			samples.insert(sample1);
			samples.insert(sample2);
			samples.insert(sample3);
			return threeSamples(sample1,sample2,sample3,ridderData);
		}
	} else if(samples.size() > 4) {
		RW_THROW("TNTRollbackMethodRidder (getTimestep): the size of the SampleSet should be maximum four!");
	}
	return 0;
}

double TNTRollbackMethodRidder::threeSamples(const Sample& sample1, const Sample& sample2, const Sample& sample3, RollbackDataRidder* data) {
	data->curPair = findDeepest(sample3);
	const double t1 = sample1.time;
	const double t2 = sample2.time;
	const double t3 = sample3.time;
	const double d1 = sample1.distance[data->curPair];
	const double d2 = sample2.distance[data->curPair];
	const double d3 = sample3.distance[data->curPair];
	const double t4 = t2 + (t2-t1)*Math::sign(d1-d3)*d2/std::sqrt(d2*d2-d1*d3);
	if (t4 > t1 && t4 < t3)
		data->left = true;
	else if (t4 > t3 && t4 < t2)
		data->left = false;
	else
		RW_THROW("TNTRollbackMethodRidder (threeSamples): could not place t4 in correct region.");
	return t4;
}

void TNTRollbackMethodRidder::fixSampleSet(SampleSet& samples) {
	std::set<std::pair<const Frame*, const Frame*> > allFramePairs = samples.begin()->framePairs;
	BOOST_FOREACH(const Sample& sample, samples) {
		std::set<std::pair<const Frame*, const Frame*> >::iterator it;
		for (it = allFramePairs.begin(); it != allFramePairs.end(); it++) {
			if (sample.framePairs.find(*it) == sample.framePairs.end()) {
				allFramePairs.erase(it);
				it--;
			}
		}
	}

	std::vector<Sample> newSamples(samples.size(),Sample());
	std::size_t i;
	i = 0;
	BOOST_FOREACH(const Sample& sample, samples) {
		newSamples[i].time = sample.time;
		newSamples[i].framePairs = allFramePairs;
		std::set<std::pair<const Frame*, const Frame*> >::iterator it;
		for (it = allFramePairs.begin(); it != allFramePairs.end(); it++) {
			newSamples[i].distance[*it] = sample.distance[*it];
		}
		i++;
	}

	SampleSet newSampleSet;
	BOOST_FOREACH(const Sample& sample, newSamples) {
		newSampleSet.insert(sample);
	}
	samples = newSampleSet;
}

std::pair<const Frame*, const Frame*> TNTRollbackMethodRidder::findDeepest(const Sample& sample) {
	RW_ASSERT(sample.framePairs.size() > 0);
	std::set<std::pair<const Frame*, const Frame*> >::iterator it = sample.framePairs.begin();
	std::pair<const Frame*, const Frame*> res = *it;
	double dist = sample.distance[res];
	for (it++; it != sample.framePairs.end(); it++) {
		const double curDist = sample.distance[*it];
		if (curDist < dist) {
			dist = curDist;
			res = *it;
		}
	}
	return res;
}
