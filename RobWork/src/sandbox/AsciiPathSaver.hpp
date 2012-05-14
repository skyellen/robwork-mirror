#ifndef RW_LOADERS_ASCIIPATHSAVER_HPP
#define RW_LOADERS_ASCIIPATHSAVER_HPP


#include <rw/common/macros.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/Timed.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>

#include <boost/foreach.hpp>
#include <vector>
#include <fstream>

namespace rw {
namespace loaders {

/**
 * @brief Provides a set of methods for saving Path to ascii files
 */
class AsciiPathSaver {
public:

	/**
	 * @brief Specifies the format to use when saving
	 */
	struct Format {
		std::string entryPreToken;
		std::string entryPostToken;

		std::string numberSeparator;
		std::string entrySeparator;


		Format(std::string pre, std::string post, std::string numberSeparator, std::string entrySeparator):
			entryPreToken(pre),
			entryPostToken(post),
			numberSeparator(numberSeparator),
			entrySeparator(entrySeparator)
		{		
		}
			
	};


	/**
	 * @brief Format predefined to match standard comma separated data
	 */
	static Format CommaSeparated;

	/**
	 * @brief Format predefined to match how Mathematica requires data in files
	 */
	static Format Mathematica;

	/**
	 * @brief Writes path to ostream
	 *
	 * @param outstream [in] Output stream to use
	 * @param path [in] Path to write
	 * @param format [in] Data format to use.
	 */
	template <class T>
	static void save(std::ostream& outstream, const rw::trajectory::Path<T>& path, const Format& format = AsciiPathSaver::CommaSeparated) {	
		for (typename rw::trajectory::Path<T>::const_iterator it = path.begin(); it != path.end(); ) {
			const T& t = (*it);
			++it;
			write(outstream, t, format, it == path.end());
		}
	}

	/**
	 * @brief Writes path to file
	 *
	 * This methods throws a rw::common::Exception if the file could not be opened.
	 *
	 * @param filename [in] File to write to
	 * @param path [in] Path to write
	 * @param format [in] Data format to use.
	 */
	template <class T>
	static void save(const std::string& filename, const rw::trajectory::Path<T>& path, const Format& format = AsciiPathSaver::CommaSeparated) {
		std::ofstream outstream(filename.c_str());
		if (!outstream.is_open())
			RW_THROW("Unable to open file "<<filename);
		save(outstream, path, format);
	}



private:

	static void write(std::ostream& outstream, const rw::math::Rotation3D<>& r, const Format& format, bool lastEntry) {
		outstream<<format.entryPreToken<<format.entryPreToken;
		for (size_t i = 0; i<3; i++) {
			for (size_t j = 0; j<3; j++) {
				outstream<<r(j,i);
				if (j != 2 || i != 2)
					outstream<<format.numberSeparator;
			}
			outstream<<format.entryPostToken;
			
		}
		outstream<<format.entryPostToken;
		if (!lastEntry)
			outstream<<format.entrySeparator;
	}


	static void write(std::ostream& outstream, const rw::math::Transform3D<>& t, const Format& format, bool lastEntry) {
		outstream<<format.entryPreToken<<format.entryPreToken;
		for (size_t i = 0; i<4; i++) {
			for (size_t j = 0; j<3; j++) {
				outstream<<t(j,i);
				if (j != 2 || i != 3)
					outstream<<format.numberSeparator;
			}
			outstream<<format.entryPostToken;
		}
		outstream<<format.entryPostToken;
		if (!lastEntry)
			outstream<<format.entrySeparator;
	}



	template <class T>
	static void write(std::ostream& outstream, const rw::trajectory::Timed<T>& q, const Format& format, bool lastEntry) {
		outstream<<format.entryPreToken;
		for (size_t i = 0; i<q.getValue().size(); i++) {
			outstream<<q.getValue()(i)<<format.numberSeparator;
		}
		outstream<<q.getTime();
		outstream<<format.entryPostToken;
		if (!lastEntry)
			outstream<<format.entrySeparator;
	}


	template <class T>
	static void write(std::ostream& outstream, const T& q, const Format& format, bool lastEntry) {
		outstream<<format.entryPreToken;
		for (size_t i = 0; i<q.size(); i++) {
			outstream<<q(i);
			if (i < q.size()-1)
				outstream<<format.numberSeparator;
		}
		outstream<<format.entryPostToken;
		if (!lastEntry)
			outstream<<format.entrySeparator;
	}

};


} //end namespace loaders
} //end namespace rw

#endif /* RW_LOADERS_ASCIIPATHSAVER_HPP */
