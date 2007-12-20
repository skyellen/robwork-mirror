#ifndef PNGLOADER_HPP_
#define PNGLOADER_HPP_

class PNGLoader
{
public:

	static std::auto_ptr<rw::sensor::Image> LoadImage(const std::string& filename);
	
};

#endif /*PNGLOADER_HPP_*/
