#ifndef RWLIBS_ALGORITHMS_JTAGGENERATOR_HPP_
#define RWLIBS_ALGORITHMS_JTAGGENERATOR_HPP_

#include "JTagMarker.hpp"

namespace rwlibs { namespace algorithms {

    /**
     * @brief this class draws JTagMarker's to different fileformats 
     */
    class JTagGenerator
    {
    public:
        /**
         * @brief Draws a specific JTagMarker formated as PostScript to a
         * file with filename
         * @param mark [in] The marker that is to be drawn
         * @param filename [in] The filename where the drawn marker is to be saved
         * @param width [in] the width in mm of the marker 
         */
        static void drawJTagToPS(
            const JTagMarker &mark, 
            const std::string& filename,
            double width);

        /**
         * @brief Draws a specific JTagMarker formated as PostScript to a
         * file with filename
         * @param id [in] The id of the marker that is to be drawn
         * @param filename [in] The filename where the drawn marker is to be saved
         * @param width [in] the width in mm of the marker 
         */
        static void drawJTagToPS(
            int id, 
            const std::string& filename,
            double width);
    };

}} // end namespaces

#endif // end include guard
