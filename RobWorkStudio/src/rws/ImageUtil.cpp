
#include "ImageUtil.hpp"

#include <QImage>

using namespace rws;
using namespace rw::sensor;
using namespace rw::common;

rw::sensor::Image::Ptr ImageUtil::toRwImage( const QImage& srcimg ){
    if(srcimg.isNull())
        return NULL;
    int w = srcimg.width();
    int h = srcimg.height();

    Image::Ptr dstimg;
    if( srcimg.isGrayscale() ){
        // for now we allways use the 16 bit per pixel for gray images
        dstimg = ownedPtr( new Image(w,h, Image::GRAY, Image::Depth16U) );
        for(int y=0;y<h;y++){
            for(int x=0;x<w;x++){
                QRgb rgb = srcimg.pixel(x, y);
                dstimg->setPixel<Image::Depth16U>(x, y, (uint16_t)qRed(rgb) );
            }
        }

    } else {
        if(srcimg.hasAlphaChannel()){
            dstimg = ownedPtr( new Image(w,h, Image::RGBA, Image::Depth8U) );
            for(int y=0;y<h;y++){
                for(int x=0;x<w;x++){
                    QRgb rgb = srcimg.pixel(x, y);
                    //dstimg->setPixel<Image::Depth16U>(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb) );
                    dstimg->setPixel8U(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb), qAlpha(rgb) );
                }
            }
        } else {
            dstimg = ownedPtr( new Image(w,h, Image::RGB, Image::Depth8U) );
            for(int y=0;y<h;y++){
                for(int x=0;x<w;x++){
                    QRgb rgb = srcimg.pixel(x, y);
                    //dstimg->setPixel<Image::Depth16U>(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb) );
                    dstimg->setPixel8U(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb) );
                }
            }
        }

    }

    return dstimg;
}

void ImageUtil::toRwImage( const QImage& srcimg, rw::sensor::Image& dstimg){
    RW_THROW("Not impl yet!!!");
}
