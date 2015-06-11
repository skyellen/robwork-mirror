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

#include "../TestSuiteConfig.hpp"

#include <rw/common/Exception.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/sensor/Image.hpp>
#include <rwlibs/mathematica.hpp>

using namespace rw::common;
using namespace rwlibs::mathematica;

BOOST_AUTO_TEST_CASE( MathematicaTest ){
	BOOST_MESSAGE("- Testing Mathematica");

	try {
		BOOST_MESSAGE("- - Setting up Mathematica kernel link");
		Mathematica m;
		BOOST_REQUIRE(m.initialize());
		const Mathematica::Link::Ptr l = m.launchKernel();
		BOOST_REQUIRE(!l.isNull());
		BOOST_REQUIRE(l->isOpen());
		Mathematica::Packet::Ptr result;
		*l >> result; // read the first In[1]:= prompt from kernel
		BOOST_MESSAGE("- - " << *result);
		BOOST_REQUIRE(result->packetType() == Mathematica::InputName);
		BOOST_MESSAGE("- - Link ready");

		{
			// Solve problem that is well-defined
			const char* const cmd = "Solve[x^2 + 2 y^3 == 3681 && x > 0 && y > 0, {x, y}, Integers]";
			BOOST_MESSAGE("- - " << cmd);
			BOOST_MESSAGE("- - - Sending command on link");
			*l << cmd;
			BOOST_MESSAGE("- - - Command sent - retrieving result");
			*l >> result;
			BOOST_MESSAGE("- - - Result received: ");
			BOOST_MESSAGE(*result);
			BOOST_REQUIRE(result->packetType() == Mathematica::Return);
			BOOST_MESSAGE("- - - Parsing result");
			const ReturnPacket::Ptr packet = result.cast<ReturnPacket>();
			BOOST_REQUIRE(!packet.isNull());
			try {
				const List::Ptr list = List::fromExpression(*packet->expression());
				BOOST_REQUIRE(!list.isNull());
				const std::list<rw::common::Ptr<const Mathematica::Expression> > sols = list->getArguments();
				BOOST_CHECK(sols.size() == 3);
				BOOST_FOREACH(const rw::common::Ptr<const Mathematica::Expression> sol, sols) {
					const rw::common::Ptr<const Mathematica::FunctionBase> fct = sol.cast<const Mathematica::FunctionBase>();
					const PropertyMap::Ptr map = Rule::toPropertyMap(fct->getArguments());
					const int x = map->get<int>("x");
					const int y = map->get<int>("y");
					BOOST_CHECK(x*x+2*y*y*y == 3681);
					BOOST_CHECK(x > 0);
					BOOST_CHECK(y > 0);
					BOOST_MESSAGE("- - - - Solution: " << x << " " << y);
				}
				BOOST_MESSAGE("- - - Result parsed");
			} catch (const Exception& e) {
				BOOST_ERROR("Could not parse result: " << e.what());
			}
		}

		{
			// Solve problem that gives a warning (using full evaluation mode)
			const char* const cmd = "Solve[Sin[x] == 0.5, x]";
			BOOST_MESSAGE("- - " << cmd << " with EnterTextPacket");
			BOOST_MESSAGE("- - - Sending command on link");
			*l << EnterTextPacket(cmd);
			BOOST_MESSAGE("- - - Command sent - retrieving results");
			*l >> result;
			BOOST_MESSAGE("- - - - " << *result);
			BOOST_REQUIRE(result->packetType() == Mathematica::Message);
			*l >> result;
			BOOST_MESSAGE("- - - - " << *result);
			BOOST_REQUIRE(result->packetType() == Mathematica::Text);
			*l >> result;
			BOOST_MESSAGE("- - - - " << *result);
			BOOST_REQUIRE(result->packetType() == Mathematica::OutputName);
			*l >> result;
			BOOST_MESSAGE("- - - - " << *result);
			BOOST_REQUIRE(result->packetType() == Mathematica::ReturnText);
			*l >> result;
			BOOST_MESSAGE("- - - - " << *result);
			BOOST_REQUIRE(result->packetType() == Mathematica::InputName);
			BOOST_MESSAGE("- - - Results received");
		}

		{
			// Construct data for a ListPlot
			BOOST_MESSAGE("- - ListPlot to Image");
			BOOST_MESSAGE("- - - Constructing data");
			std::vector<std::string> axesLabel(2);
			axesLabel[0] = "X axis";
			axesLabel[1] = "Y axis";
			PropertyMap properties;
			properties.add<std::string>("PlotLabel","","TESTPLOT");
			properties.add("AxesLabel","",axesLabel);
			std::vector<double> x,y;
			for (std::size_t i = 0; i < 100; i++) {
				x.push_back(i);
				y.push_back(i*i);
			}
			std::list<Rule::Ptr> options = Rule::toRules(properties);
			List imageSize;
			imageSize.add(400).add(250);
			options.push_back(ownedPtr(new Rule("ImageSize",imageSize)));

			long long start;
			long long end;
			// Send the command
			BOOST_MESSAGE("- - - Sending constructed expression");
			*l << Image(ListPlot(x,y,options));
			BOOST_MESSAGE("- - - Waiting");
			start = TimerUtil::currentTimeMs();
			l->wait();
			end = TimerUtil::currentTimeMs();
			BOOST_MESSAGE("- - - Waited for " << (end-start) << " ms.");
			BOOST_MESSAGE("- - - Receiving result");
			start = TimerUtil::currentTimeMs();
			*l >> result;
			BOOST_MESSAGE(*result);
			end = TimerUtil::currentTimeMs();
			BOOST_MESSAGE("- - - Result received in " << (end-start) << " ms.");
			BOOST_REQUIRE(result->packetType() == Mathematica::Return);
			const ReturnPacket::Ptr imgPacket = result.cast<ReturnPacket>();
			BOOST_REQUIRE(!imgPacket.isNull());

			// Now convert to RW image
			BOOST_MESSAGE("- - - Convert to RobWork image");
			start = TimerUtil::currentTimeMs();
			const rw::sensor::Image::Ptr image = Image::toRobWorkImage(*imgPacket->expression());
			end = TimerUtil::currentTimeMs();
			BOOST_MESSAGE("- - - Converted in " << (end-start) << " ms.");
			BOOST_REQUIRE(!image.isNull());
			BOOST_MESSAGE("- - - Save image to test.ppm");
			image->saveAsPPM("test.ppm");
			BOOST_MESSAGE("- - - Image saved");
		}
	} catch(const Exception& e) {
		BOOST_ERROR("Could not parse result: " << e.what());
	}
}

#if __cplusplus >= 201103L
BOOST_AUTO_TEST_CASE( MathematicaTestCPP11 ){
	BOOST_MESSAGE("- Testing Mathematica with C++11 Interface");
	try {
		BOOST_MESSAGE("- - Setting up Mathematica kernel link");
		Mathematica m;
		BOOST_REQUIRE(m.initialize());
		const Mathematica::Link::Ptr l = m.launchKernel();
		BOOST_REQUIRE(!l.isNull());
		BOOST_REQUIRE(l->isOpen());
		Mathematica::Packet::Ptr result;
		*l >> result; // read the first In[1]:= prompt from kernel
		BOOST_MESSAGE("- - " << *result);
		BOOST_MESSAGE("- - Link ready");

		{
			BOOST_MESSAGE("- - ListPlot to Image");
			*l << Image(ListPlot({{0,1},{2,2},{2.5,3}},Rule("PlotLabel","TESTPLOT C++11"),Rule("AxesLabel",{"X","Y"})));
			BOOST_MESSAGE("- - - Receiving result");
			*l >> result;
			BOOST_MESSAGE("- - - Result received");
			BOOST_REQUIRE(result->packetType() == Mathematica::Return);
			const ReturnPacket::Ptr imgPacket = result.cast<ReturnPacket>();
			BOOST_REQUIRE(!imgPacket.isNull());

			// Now convert to RW image
			BOOST_MESSAGE("- - - Convert to RobWork image");
			const rw::sensor::Image::Ptr image = Image::toRobWorkImage(*imgPacket->expression());
			BOOST_REQUIRE(!image.isNull());
			BOOST_MESSAGE("- - - Save image to test.ppm");
			image->saveAsPPM("test11.ppm");
			BOOST_MESSAGE("- - - Image saved");
		}
	} catch(const Exception& e) {
		BOOST_ERROR("Could not parse result: " << e.what());
	}
}
#endif
