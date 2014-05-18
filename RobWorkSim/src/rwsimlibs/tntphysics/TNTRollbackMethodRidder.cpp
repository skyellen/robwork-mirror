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

TNTRollbackMethodRidder::TNTRollbackMethodRidder() {
}

TNTRollbackMethodRidder::~TNTRollbackMethodRidder() {
}

TNTRollbackMethod::RollbackData* TNTRollbackMethodRidder::createData() const {
	return new RollbackData();
}

double TNTRollbackMethodRidder::getTimestep(SampleSet& samples, RollbackData* data) const {
	fixSampleSet(samples);
	RollbackData* ridderData = dynamic_cast<RollbackData*>(data);
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
		const std::pair<const Frame*, const Frame*> deepestFrames = findDeepest(sample3);
		const double t1 = sample1.time;
		const double t2 = sample2.time;
		const double d1 = sample1.distance[deepestFrames];
		const double d2 = sample2.distance[deepestFrames];
		const double d3 = sample3.distance[deepestFrames];
		return t2 + (t2-t1)*Math::sign(d1-d3)*d2/std::sqrt(d2*d2-d1*d3);
	} else if(samples.size() == 4) {
	} else if(samples.size() > 4) {
		RW_THROW("TNTRollbackMethodRidder (getTimestep): the size of the SampleSet should be maximum four!");
	}
	return 0;
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
