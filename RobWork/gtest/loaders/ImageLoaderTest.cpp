/********************************************************************************
 * Copyright 2016 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <gtest/gtest.h>

#include "../TestEnvironment.hpp"

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/image/PGMLoader.hpp>
#include <rw/loaders/image/PPMLoader.hpp>
#include <rw/loaders/image/RGBLoader.hpp>

using rw::common::ownedPtr;
using namespace rw::loaders;
using rw::sensor::Image;

TEST(ImageLoaderPGM, loadImage) {
	ImageLoader::Ptr loader = ownedPtr(new PGMLoader());
	const Image::Ptr raw = loader->loadImage(TestEnvironment::testfilesDir()+"images/image_raw.pgm");
	//const Image::Ptr ascii = loader->loadImage(TestEnvironment::testfilesDir()+"images/image_ascii.pgm");
	EXPECT_FALSE(raw.isNull());
	//EXPECT_FALSE(ascii.isNull());

	EXPECT_EQ(31,  raw->getWidth());
	EXPECT_EQ(5,   raw->getHeight());
	EXPECT_EQ(1,   raw->getNrOfChannels());
	EXPECT_EQ(207, raw->getPixelValuei(15,2,0));
	EXPECT_EQ(207, raw->getPixelValuei(15,2,1));
	EXPECT_EQ(207, raw->getPixelValuei(15,2,2));
	EXPECT_EQ(Image::Depth8U, raw->getPixelDepth());
	EXPECT_EQ(Image::GRAY,    raw->getColorEncoding());

	/*
	EXPECT_EQ(31,  ascii->getWidth());
	EXPECT_EQ(5,   ascii->getHeight());
	EXPECT_EQ(1,   ascii->getNrOfChannels());
	EXPECT_EQ(207, ascii->getPixelValuei(15,2,0));
	EXPECT_EQ(207, ascii->getPixelValuei(15,2,1));
	EXPECT_EQ(207, ascii->getPixelValuei(15,2,2));
	EXPECT_EQ(Image::Depth8U, ascii->getPixelDepth());
	EXPECT_EQ(Image::GRAY,    ascii->getColorEncoding());
	*/
}

TEST(ImageLoaderPPM, loadImage) {
	ImageLoader::Ptr loader = ownedPtr(new PPMLoader());
	const Image::Ptr raw = loader->loadImage(TestEnvironment::testfilesDir()+"images/image_raw.ppm");
	const Image::Ptr ascii = loader->loadImage(TestEnvironment::testfilesDir()+"images/image_ascii.ppm");
	EXPECT_FALSE(raw.isNull());
	EXPECT_FALSE(ascii.isNull());

	EXPECT_EQ(11,  raw->getWidth());
	EXPECT_EQ(15,  raw->getHeight());
	EXPECT_EQ(3,   raw->getNrOfChannels());
	EXPECT_EQ(173, raw->getPixelValuei(6,14,0));
	EXPECT_EQ(40,  raw->getPixelValuei(6,14,1));
	EXPECT_EQ(181, raw->getPixelValuei(6,14,2));
	EXPECT_EQ(Image::Depth8U, raw->getPixelDepth());
	EXPECT_EQ(Image::RGB,     raw->getColorEncoding());

	EXPECT_EQ(11,  ascii->getWidth());
	EXPECT_EQ(15,  ascii->getHeight());
	EXPECT_EQ(3,   ascii->getNrOfChannels());
	EXPECT_EQ(173, ascii->getPixelValuei(6,14,0));
	EXPECT_EQ(40,  ascii->getPixelValuei(6,14,1));
	EXPECT_EQ(181, ascii->getPixelValuei(6,14,2));
	EXPECT_EQ(Image::Depth8U, ascii->getPixelDepth());
	EXPECT_EQ(Image::RGB,     raw->getColorEncoding());
}

TEST(ImageLoaderRGB, loadImage) {
	ImageLoader::Ptr loader = ownedPtr(new RGBLoader());
	const Image::Ptr nocompress = loader->loadImage(TestEnvironment::testfilesDir()+"images/image_nocompress.rgb");
	const Image::Ptr rle = loader->loadImage(TestEnvironment::testfilesDir()+"images/image_rle.rgb");
	const Image::Ptr aggresive_rle = loader->loadImage(TestEnvironment::testfilesDir()+"images/image_aggresive_rle.rgb");
	EXPECT_FALSE(nocompress.isNull());
	EXPECT_FALSE(rle.isNull());
	EXPECT_FALSE(aggresive_rle.isNull());

	EXPECT_EQ(17,  nocompress->getWidth());
	EXPECT_EQ(12,  nocompress->getHeight());
	EXPECT_EQ(3,   nocompress->getNrOfChannels());
	EXPECT_EQ(220, nocompress->getPixelValuei(15,6,0));
	EXPECT_EQ(75,  nocompress->getPixelValuei(15,6,1));
	EXPECT_EQ(75,  nocompress->getPixelValuei(15,6,2));
	EXPECT_EQ(Image::Depth8U, nocompress->getPixelDepth());
	EXPECT_EQ(Image::RGB,     nocompress->getColorEncoding());

	EXPECT_EQ(17,  rle->getWidth());
	EXPECT_EQ(12,  rle->getHeight());
	EXPECT_EQ(3,   rle->getNrOfChannels());
	EXPECT_EQ(220, rle->getPixelValuei(15,6,0));
	EXPECT_EQ(75,  rle->getPixelValuei(15,6,1));
	EXPECT_EQ(75,  rle->getPixelValuei(15,6,2));
	EXPECT_EQ(Image::Depth8U, rle->getPixelDepth());
	EXPECT_EQ(Image::RGB,     rle->getColorEncoding());

	EXPECT_EQ(17,  aggresive_rle->getWidth());
	EXPECT_EQ(12,  aggresive_rle->getHeight());
	EXPECT_EQ(3,   aggresive_rle->getNrOfChannels());
	EXPECT_EQ(220, aggresive_rle->getPixelValuei(15,6,0));
	EXPECT_EQ(75,  aggresive_rle->getPixelValuei(15,6,1));
	EXPECT_EQ(75,  aggresive_rle->getPixelValuei(15,6,2));
	EXPECT_EQ(Image::Depth8U, aggresive_rle->getPixelDepth());
	EXPECT_EQ(Image::RGB,     aggresive_rle->getColorEncoding());
}

TEST(ImageLoaderFactoryTest, getSupportedFormats) {
	const std::vector<std::string> formats = ImageLoader::Factory::getSupportedFormats();
	bool foundPGM = false;
	bool foundPPM = false;
	bool foundRGB = false;
	for (std::size_t i = 0; i < formats.size(); i++) {
		if (formats[i] == "PGM") {
			foundPGM = true;
		} else if (formats[i] == "PPM") {
			foundPPM = true;
		} else if (formats[i] == "RGB") {
			foundRGB = true;
		}
	}
	EXPECT_TRUE(foundPGM);
	EXPECT_TRUE(foundPPM);
	EXPECT_TRUE(foundRGB);
}

TEST(ImageLoaderFactoryTest, hasImageLoaderPGM) {
	EXPECT_TRUE(ImageLoader::Factory::hasImageLoader("PGM"));
	EXPECT_TRUE(ImageLoader::Factory::hasImageLoader("Pgm"));
}

TEST(ImageLoaderFactoryTest, hasImageLoaderPPM) {
	EXPECT_TRUE(ImageLoader::Factory::hasImageLoader("PPM"));
	EXPECT_TRUE(ImageLoader::Factory::hasImageLoader("pPm"));
}

TEST(ImageLoaderFactoryTest, hasImageLoaderRGB) {
	EXPECT_TRUE(ImageLoader::Factory::hasImageLoader("RGB"));
	EXPECT_TRUE(ImageLoader::Factory::hasImageLoader("rGB"));
}

TEST(ImageLoaderFactoryTest, getImageLoaderPGM) {
	EXPECT_FALSE(ImageLoader::Factory::getImageLoader("PGM").isNull());
	EXPECT_FALSE(ImageLoader::Factory::getImageLoader("Pgm").isNull());
}

TEST(ImageLoaderFactoryTest, getImageLoaderPPM) {
	EXPECT_FALSE(ImageLoader::Factory::getImageLoader("PPM").isNull());
	EXPECT_FALSE(ImageLoader::Factory::getImageLoader("pPm").isNull());
}

TEST(ImageLoaderFactoryTest, getImageLoaderRGB) {
	EXPECT_FALSE(ImageLoader::Factory::getImageLoader("RGB").isNull());
	EXPECT_FALSE(ImageLoader::Factory::getImageLoader("rGB").isNull());
}
