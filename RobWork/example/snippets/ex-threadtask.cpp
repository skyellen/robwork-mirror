#include <rw/common/ThreadPool.hpp>
#include <rw/common/ThreadTask.hpp>
#include <rw/loaders/ImageLoader.hpp>

using namespace rw::common;
using rw::loaders::ImageLoader;
using rw::sensor::Image;

static boost::mutex outMutex;

class LoadTask: public ThreadTask {
public:
	LoadTask(ThreadTask::Ptr parent, const std::string &file): ThreadTask(parent), _file(file) {};

	Image::Ptr getImage() const { return _image; }

	void run() {
		{
			boost::mutex::scoped_lock lock(outMutex);
			std::cout << "Loading " << _file << std::endl;
		}
		try {
			_image = ImageLoader::Factory::load(_file);
		} catch (Exception &e) {
			_image = NULL;
		}
	}

private:
	const std::string _file;
	Image::Ptr _image;
};

class MainTask: public ThreadTask {
public:
	MainTask(ThreadPool::Ptr pool, const std::vector<std::string> &files): ThreadTask(pool), _files(files) {};

	std::vector<Image::Ptr> getImages() {
		boost::mutex::scoped_lock lock(_imageMutex);
		return _images;
	}

	void run() {
		BOOST_FOREACH(const std::string &file, _files) {
			LoadTask::Ptr task = ownedPtr(new LoadTask(this,file));
			addSubTask(task);
		}
	}

	void subTaskDone(ThreadTask* subtask) {
		LoadTask* task = static_cast<LoadTask*>(subtask);
		boost::mutex::scoped_lock lock(_imageMutex);
		_images.push_back(task->getImage());
		// More subtasks could be added at this point
	}

	void idle() {
		boost::mutex::scoped_lock lock(outMutex);
		std::cout << "All images loaded!" << std::endl;
		// More subtasks could be added at this point
	}

	void done() {
		boost::mutex::scoped_lock lock(outMutex);
		std::cout << "Main Task ended!" << std::endl;
		// No subtasks can be added now
	}

private:
	const std::vector<std::string> _files;
	boost::mutex _imageMutex;
	std::vector<Image::Ptr> _images;
};

int main(int argc, const char* argv[]) {
	ThreadPool::Ptr pool = ownedPtr(new ThreadPool(std::atoi(argv[1])));
	std::vector<std::string> files;
	for (int i = 2; i < argc; i++)
		files.push_back(argv[i]);
	MainTask task(pool, files);
	task.execute();
	task.waitUntilDone();
	std::cout << "Images loaded: " << task.getImages().size() << std::endl;
	return 0;
}
