/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "RawArray.hpp"

using namespace rwlibs::mathematica;

std::size_t RawArrayUtil::detectDimensions(const Mathematica::FunctionBase& exp) {
	if (exp.getName() != "RawArray")
		RW_THROW("Expected function with name RawArray, not " << exp.getName() << ".");
	const std::list<rw::common::Ptr<const Mathematica::Expression> >& args = exp.getArguments();
	if (args.size() != 2)
		RW_THROW("Expected two arguments for RawArray, not " << args.size() << ".");
	rw::common::Ptr<const Mathematica::Expression> array = args.back();
	std::size_t dim = 0;
	rw::common::Ptr<const Mathematica::FunctionBase> fct;
	bool cont = true;;
	do {
		fct = array.cast<const Mathematica::FunctionBase>();
		cont = false;
		if (!fct.isNull()) {
			dim++;
			if (fct->getArguments().size() > 0) {
				array = fct->getArguments().front();
				cont = true;
			}
		}
	} while(cont);
	return dim;
}
