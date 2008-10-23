#ifndef RWXMLFILE_HPP_
#define RWXMLFILE_HPP_


/**
 * @brief define methods for loading and saving workcells from and to the xml
 * file format.
 *
 */

class RWXMLFile {

	/**
	 * @brief save a workcell with some intial states "initState" to a file with name
	 * "filename".
	 * @param wc [in] pointer to a workcell
	 * @param initState [in] state that the workcell should initially be in
	 * @param filename [in] name of file to save workcell to
	 */
	static void saveWorkCell(rw::models::WorkCell *wc,
							 const rw::kinematics::State& initState,
							 const std::string& filename);

    /**
     * @brief Loads/imports robwork workcell in XML file format
     *
     * An exception is thrown if the file can't be loaded.
     *
     * @param filename [in] filename of XML file
     */
    static std::auto_ptr<rw::models::WorkCell> loadWorkCell(
        const std::string& filename);


};

#endif /*RWXMLFILE_HPP_*/
