
#include "RWSImageLoaderPlugin.hpp"
#include "ImageUtil.hpp"
#include <QImage>
#include <QImageReader>
#include <boost/foreach.hpp>
#include <rw/common/Ptr.hpp>

using namespace rws;
using namespace rw::sensor;

namespace {

    class QImageLoader: public rw::loaders::ImageLoader {
    public:

        virtual ~QImageLoader(){}

        rw::sensor::Image::Ptr loadImage(const std::string& filename){
            // load the image
            QImage img;
            if(!img.load( filename.c_str() ) ){
                RW_WARN("Cannot load image: " << filename);
                return NULL;
            }
            //std::cout << "image size: " <<  img.width() << " " << img.height() << std::endl;
            //std::cout << "filename: " << filename << std::endl;
            // convert to robwork Image
            Image::Ptr rwImg = ImageUtil::toRwImage(img);
            //rwImg->saveAsPPM(filename + ".ppm");
            return rwImg;
        }

    };

}

RWSImageLoaderPlugin::RWSImageLoaderPlugin():
    rw::plugin::PluginFactory<rw::loaders::ImageLoader>("RWSImageLoaderPlugin")
{
    // add all extension that is supported by this imageloader
    //QList<QByteArray> formats = QImageReader::supportedImageFormats();
    //BOOST_FOREACH(QByteArray& format, formats){
    //    getProperties().set(std::string(".")+format.toUpper().data(), true);
    //}
}


rw::loaders::ImageLoader::Ptr RWSImageLoaderPlugin::make(){
    return rw::common::ownedPtr( new QImageLoader() );
}

rw::loaders::ImageLoader::Ptr RWSImageLoaderPlugin::make(const std::string&){
    return make();
}

