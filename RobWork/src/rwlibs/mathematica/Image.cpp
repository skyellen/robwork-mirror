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

#include "Image.hpp"
#include "RawArray.hpp"
#include "Rule.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/sensor/Image.hpp>

#include <boost/foreach.hpp>

using namespace rw::common;
using namespace rwlibs::mathematica;

Image::Image(rw::common::Ptr<const rw::sensor::Image> image):
	Mathematica::FunctionBase("Image")
{
	std::vector<std::size_t> dims;
	dims.push_back(image->getHeight());
	dims.push_back(image->getWidth());
	std::string colorSpace = "";
	std::string type = "";
	if (image->getColorEncoding() == rw::sensor::Image::RGB) {
		dims.push_back(3);
		colorSpace = "RGB";
	} else {
		RW_THROW("Color encoding not supported.");
	}
	if (image->getPixelDepth() == rw::sensor::Image::Depth8U)
		type = "Byte";
	else
		RW_THROW("Depth not supported.");
	const RawArray<char,3>::Ptr data = ownedPtr(new RawArray<char,3>(dims));
	std::vector<std::size_t> indexes(3,0);
	for (std::size_t x = 0; x < image->getHeight(); x++) {
		indexes[0] = x;
		for (std::size_t y = 0; y < image->getWidth(); y++) {
			indexes[1] = y;
			for (std::size_t c = 0; c < 3; c++) {
				indexes[2] = c;
				data->set(indexes,(char)image->getPixelValuei(y,x,c));
			}
		}
	}
	_data = data;
	_type = ownedPtr(new Mathematica::String(type));
	PropertyMap options;
	options.add<std::string>("ColorSpace","",colorSpace);
	_rules = Rule::toRules(options);
}

Image::Image():
	Mathematica::FunctionBase("Image")
{
}

Image::Image(const Expression& data):
	Mathematica::FunctionBase("Image"),
	_data(data.clone())
{
}

Image::Image(const Expression& data, const PropertyMap& options):
	Mathematica::FunctionBase("Image"),
	_data(data.clone())
{
	_rules = Rule::toRules(options);
}

Image::~Image() {
}

std::list<rw::common::Ptr<const Mathematica::Expression> > Image::getArguments() const {
	std::list<rw::common::Ptr<const Mathematica::Expression> > res;
	res.push_back(_data);
	if (!_type.isNull())
		res.push_back(_type);
	BOOST_FOREACH(const Rule::Ptr rule, _rules) {
		res.push_back(rule);
	}
	return res;
}

Mathematica::Expression::Ptr Image::clone() const {
	const Image::Ptr img = ownedPtr(new Image(*_data));
	if (!_type.isNull())
		img->_type = _type->clone().cast<Mathematica::String>();
	BOOST_FOREACH(const Rule::Ptr rule, _rules) {
		img->_rules.push_back(rule->clone().cast<Rule>());
	}
	return img;
}

void Image::setImageSize(std::size_t width, std::size_t height) {
	List imageSize;
	imageSize.add(ownedPtr(new Mathematica::Integer(400)));
	imageSize.add(ownedPtr(new Mathematica::Integer(250)));
	Rule::Ptr search = NULL;
	BOOST_FOREACH(const Rule::Ptr rule, _rules) {
		if (rule->getId() == "ImageSize") {
			search = rule;
		}
	}
	if (search.isNull()) {
		search = ownedPtr(new Rule("ImageSize",imageSize));
		_rules.push_back(search);
	} else {
		search->setValue(imageSize);
	}
}

rw::sensor::Image::Ptr Image::toRobWorkImage(const Mathematica::Expression& expression) {
	if (expression.getType() != Mathematica::Expression::Function)
		RW_THROW("Could not create Image expression: Expected Function.");
	const Mathematica::FunctionBase& fct = dynamic_cast<const Mathematica::FunctionBase&>(expression);
	if (fct.getName() != "Image")
		RW_THROW("Could not create Image expression: Expected Image function, but got \"" << fct.getName() << "\".");

	const std::list<rw::common::Ptr<const Expression> >& args = fct.getArguments();
	if (args.size() == 0)
		RW_THROW("Expected 1 or more arguments in Image expression.");
	std::list<rw::common::Ptr<const Expression> >::const_iterator it = args.begin();
	const rw::common::Ptr<const Expression> dataArg = *it;
	if (dataArg->getType() != Mathematica::Expression::Function && dataArg->getType() != Mathematica::Expression::Array)
		RW_THROW("Expected first argument in Image expression to be a Function or an Array.");
	it++;
	std::string type = "Real";
	if (it != args.end()) {
		const rw::common::Ptr<const Expression> typeArg = *it;
		if (typeArg->getType() != Mathematica::Expression::String)
			RW_THROW("Expected second argument in Image expression to be a String.");
		type = typeArg.cast<const Mathematica::String>()->value();
	}
	std::list<rw::common::Ptr<const Expression> > rules;
	for (it++; it != args.end(); it++) {
		rules.push_back(*it);
	}
	const PropertyMap::Ptr map = Rule::toPropertyMap(rules);
	const std::string colorSpace = map->get<std::string>("ColorSpace","Automatic");

	const rw::common::Ptr<const Mathematica::FunctionBase> dataFct = dataArg.cast<const Mathematica::FunctionBase>();
	if (type == "Byte") {
		if (colorSpace == "RGB") {
			rw::common::Ptr<const Mathematica::Array<unsigned char> > array;
			if (!dataFct.isNull())
				array = RawArray<unsigned char, 3>::fromExpression(*dataFct);
			else
				array = dataArg.cast<const Mathematica::Array<unsigned char> >();
			if (array.isNull())
				RW_THROW("The data array was not of expected type.");
			const unsigned char* const rawArray = array->data();
			unsigned int height = array->size()[0];
			unsigned int width = array->size()[1];
			const rw::sensor::Image::Ptr img = ownedPtr(new rw::sensor::Image(width,height,rw::sensor::Image::RGB, rw::sensor::Image::Depth8U));
			char* const imgData = img->getImageData();
			std::copy(rawArray,rawArray+height*width*3,imgData);
			return img;
		} else {
			RW_THROW("Image with ColorSpace " << colorSpace << " not yet supported.");
		}
	} else {
		RW_THROW("Image type " << type << " not yet supported.");
	}

	return NULL;
}
