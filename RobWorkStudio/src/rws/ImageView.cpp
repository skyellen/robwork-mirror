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

#include "ImageView.hpp"

#include <rw/sensor/Image.hpp>

#include <QImage>
#include <QPixmap>

using namespace rw::sensor;

ImageView::ImageView(QWidget* parent):
    QLabel(parent)
{


}

ImageView::~ImageView()
{
}



void ImageView::display(const rw::sensor::Image& image) {

        QImage qimage(image.getWidth(), image.getHeight(), QImage::Format_RGB32);
        if(image.getColorEncoding()==Image::GRAY){
            for (size_t i = 0; i<image.getWidth(); i++) {
                for (size_t j = 0; j<image.getHeight(); j++) {
                    float val = image.getPixelValue(i,j,0);
                    //std::cout << val << " -- " << (((int)(255.0*val))&0xFF) << "\n";
                    int value = 0;
                    value += 0xff000000;
                    value += (((int)(255*val))&0xFF)<<16;
                    value += (((int)(255*val))&0xFF)<<8;
                    value += (((int)(255.0*val))&0xFF);

                    qimage.setPixel((int)i,(int)j, value);
                }
            }
        } else {
            for (size_t i = 0; i<image.getWidth(); i++) {
                for (size_t j = 0; j<image.getHeight(); j++) {
                    Pixel4f pixel = image.getPixel(i,j);
                    int value = 0;
                    value += 0xff000000;
                    value += (((int)(255.0*pixel.ch[0]))&0xFF)<<16;
                    value += (((int)(255.0*pixel.ch[1]))&0xFF)<<8;
                    value +=  ((int)(255.0*pixel.ch[2])&0xFF);

                    qimage.setPixel((int)i,(int)j, value);
                }
            }
        }
        QPixmap pixmap = QPixmap::fromImage(qimage);
        this->setPixmap(pixmap);

}
