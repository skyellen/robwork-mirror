#ifndef RW_LOADERS_PGMLOADER_HPP_
#define RW_LOADERS_PGMLOADER_HPP_

#include <rw/sensor/Image.hpp> 

namespace rw { namespace loaders {
	/**
	 * @brief
	 */
	class PGMLoader
	{
	public:
		static std::auto_ptr<rw::sensor::Image> load(const std::string& filename);
	};
}}

#endif /*PGMLOADER_HPP_*/
