#include <rw/common/ThreadPool.hpp>
#include <rw/loaders/ImageLoader.hpp>

using namespace rw::common;
using rw::loaders::ImageLoader;
using rw::sensor::Image;

static boost::mutex outMutex;
static boost::mutex imageMutex;
static std::vector<Image::Ptr> images;

void loadFile(ThreadPool* pool, const std::string &file) {
	{
		boost::mutex::scoped_lock lock(outMutex);
		std::cout << "Loading " << file << std::endl;
	}
	Image::Ptr image;
	try {
		image = ImageLoader::Factory::load(file);
	} catch (Exception &e) {
		image = NULL;
	}
	if (pool->isStopping())
		return;
	{
		boost::mutex::scoped_lock lock(imageMutex);
		images.push_back(image);
	}
}

int main(int argc, const char* argv[]) {
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(std::atoi(argv[1])));
	for (int i = 2; i < argc; i++) {
		ThreadPool::WorkFunction work = boost::bind(&loadFile,_1,argv[i]);
		pool->addWork(work);
	}
	pool->waitForEmptyQueue();
	return 0;
}
